"""
collision_avoidance_node
────────────────────────
LiDAR (LaserScan) + YOLO (BuoyArray) sensor fusion.
Vektör alanı (VFH — Vector Field Histogram) mantığıyla yönlendirme
vektörü hesaplayıp pymavlink_controller_node'a setpoint gönderir.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3Stamped
from std_msgs.msg import String, Float64

from idaws_msgs.msg import BuoyArray


# Otonom sürüşün motor komutu üretmesine izin verilen fazlar. Bunların dışında
# (özellikle IDLE'da) node hiçbir şey yayınlamaz — aksi hâlde araç ARM edilir
# edilmez, görev başlamamışken bile tam gaz ileri gider ve manuel kumanda da
# 10 Hz'lik otonomi komutlarıyla çakışır.
DEFAULT_ACTIVE_PHASES = ['PARKUR_1', 'PARKUR_2', 'PARKUR_3_KAMIKAZE']


class CollisionAvoidanceNode(Node):

    def __init__(self):
        super().__init__('collision_avoidance_node')

        # ---------- Parametreler ----------
        # safety_radius / danger_radius artık sabit değil, TABAN değerlerdir:
        # üzerine anlık durma mesafesi eklenir (bkz. _update_radii).
        self.declare_parameter('safety_radius', 2.0)         # metre — duruştaki taban
        self.declare_parameter('danger_radius', 1.0)         # metre — acil manevra tabanı
        self.declare_parameter('reaction_time_sec', 1.0)     # algı + komut gecikmesi
        self.declare_parameter('max_decel', 1.5)             # m/s² — etkin yavaşlama
        self.declare_parameter('max_safety_radius', 10.0)    # LiDAR menzilinin altında kalmalı
        # Hızı sensörün görebildiği mesafeyle sınırla. Kapalıysa güvenlik yarıçapı
        # tavana dayanır ve durma mesafesi menzilin ötesine taşar — yani araç
        # göremediği bir engele çarpacak hızda gider.
        self.declare_parameter('limit_speed_to_sensor_range', True)
        self.declare_parameter('speed_topic', 'telemetry/groundspeed')
        self.declare_parameter('max_speed', 1.0)             # normalleştirilmiş [-1, 1]
        self.declare_parameter('num_sectors', 72)             # 360/72 = 5° sektörler
        self.declare_parameter('goal_weight', 1.0)
        self.declare_parameter('obstacle_weight', 2.0)
        self.declare_parameter('camera_fov_deg', 60.0)       # kamera görüş açısı
        self.declare_parameter('active_phases', DEFAULT_ACTIVE_PHASES)

        self._base_safety_r = float(self.get_parameter('safety_radius').value)
        self._base_danger_r = float(self.get_parameter('danger_radius').value)
        self._reaction_t = float(self.get_parameter('reaction_time_sec').value)
        self._max_decel = max(0.1, float(self.get_parameter('max_decel').value))
        self._max_safety_r = float(self.get_parameter('max_safety_radius').value)
        self._limit_speed = bool(self.get_parameter('limit_speed_to_sensor_range').value)
        self._speed_topic = self.get_parameter('speed_topic').value
        self._max_speed = self.get_parameter('max_speed').value
        self._num_sectors = self.get_parameter('num_sectors').value
        self._goal_w = self.get_parameter('goal_weight').value
        self._obs_w = self.get_parameter('obstacle_weight').value
        self._cam_fov = math.radians(self.get_parameter('camera_fov_deg').value)
        self._active_phases = set(self.get_parameter('active_phases').value)

        # ---------- Durum ----------
        self._histogram = np.zeros(self._num_sectors, dtype=np.float64)
        self._buoy_obstacles = []  # (açı_rad, tahmini_mesafe, label)
        self._goal_angle = 0.0     # hedef yönü (rad, ileride setpoint'ten gelecek)
        self._phase = 'IDLE'
        self._was_active = False
        self._speed = 0.0          # m/s — yer hızı
        self._min_range = math.inf  # en yakın LiDAR dönüşü (m)

        # Hıza bağlı etkin yarıçaplar; duruşta taban değerlerle başlar.
        self._danger_ratio = (
            self._base_danger_r / self._base_safety_r if self._base_safety_r > 0 else 0.5
        )
        self._safety_r = self._base_safety_r
        self._danger_r = self._base_danger_r

        # Sensör menzilinin izin verdiği azami hız. v·t + v²/(2a) = pay
        # denkleminin pozitif kökü:  v = -a·t + sqrt((a·t)² + 2·a·pay)
        margin = max(0.0, self._max_safety_r - self._base_safety_r)
        at = self._max_decel * self._reaction_t
        self._v_max_sensor = -at + math.sqrt(at * at + 2.0 * self._max_decel * margin)
        self._speed_warned = False
        # Governor durumu: gaz komutu normalize olduğu ve m/s karşılığı bilinmediği
        # için integral etkiyle ayarlanır (orantısal kazanç tek başına yakınsamıyor).
        self._throttle_scale = 1.0
        self._danger_warned = False

        # ---------- Subscriber'lar ----------
        self.create_subscription(LaserScan, '/scan', self._cb_lidar, 10)
        self.create_subscription(BuoyArray, 'vision/buoys', self._cb_buoys, 10)
        self.create_subscription(Vector3Stamped, 'mission/goal_heading', self._cb_goal, 10)
        self.create_subscription(String, 'mission/phase', self._cb_phase, 10)
        self.create_subscription(Float64, self._speed_topic, self._cb_speed, 10)

        # ---------- Publisher'lar ----------
        self.pub_cmd = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.pub_setpoint = self.create_publisher(Vector3Stamped, 'cmd/setpoint_ned', 10)
        # Etkin yarıçaplar dışarı yayınlanır ki web arayüzü gerçek değerleri çizsin.
        self.pub_safety_r = self.create_publisher(Float64, 'collision/safety_radius', 10)
        self.pub_danger_r = self.create_publisher(Float64, 'collision/danger_radius', 10)

        # ---------- Timer: 10 Hz karar döngüsü ----------
        self.create_timer(0.1, self._decision_loop)

        self.get_logger().info(
            f'collision_avoidance_node başlatıldı — güvenlik yarıçapı hıza bağlı '
            f'(taban {self._base_safety_r:.1f} m, tavan {self._max_safety_r:.1f} m, '
            f't_tepki {self._reaction_t:.1f} s, a {self._max_decel:.1f} m/s²). '
            f'Sensör menziline göre azami hız: {self._v_max_sensor:.2f} m/s'
            f'{"" if self._limit_speed else " (sınırlama KAPALI)"}'
        )

    # ───────────────────── Hıza Bağlı Güvenlik Yarıçapı ─────────────────────

    def _cb_speed(self, msg: Float64):
        self._speed = max(0.0, float(msg.data))
        self._update_radii()

    def _update_radii(self):
        """
        Güvenlik yarıçapını anlık durma mesafesine bağlar:

            durma_mesafesi = v · t_tepki + v² / (2 · a_yavaşlama)
            güvenlik       = taban + durma_mesafesi        (max_safety_radius ile sınırlı)
            tehlike        = güvenlik · (taban_tehlike / taban_güvenlik)

        Sabit 2 m'lik yarıçap 5 m/s'de engeli ancak 0.4 sn kala fark ettiriyordu;
        bu formülde yarıçap hızla birlikte büyüdüğü için tepki süresi korunur.
        Üst sınır LiDAR menzilinin altında tutulmalı, aksi hâlde göremediğimiz
        bir mesafe için karar üretmiş oluruz.
        """
        v = self._speed
        stopping = v * self._reaction_t + (v * v) / (2.0 * self._max_decel)
        safety = min(self._base_safety_r + stopping, self._max_safety_r)
        self._safety_r = max(self._base_safety_r, safety)
        self._danger_r = max(self._base_danger_r, self._safety_r * self._danger_ratio)

    # ───────────────────── LiDAR Callback ─────────────────────

    def _cb_lidar(self, msg: LaserScan):
        """LaserScan → polar histogram güncelleme."""
        sector_size = 2 * math.pi / self._num_sectors
        hist = np.zeros(self._num_sectors, dtype=np.float64)
        min_range = math.inf

        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                if r < min_range:
                    min_range = r
                # Engel yakınlık skoru (ters mesafe)
                if r < self._safety_r:
                    certainty = 1.0 - (r / self._safety_r)
                    sector_idx = int(((angle % (2 * math.pi)) / sector_size)) % self._num_sectors
                    hist[sector_idx] = max(hist[sector_idx], certainty)
            angle += msg.angle_increment

        self._histogram = hist
        self._min_range = min_range

    # ───────────────────── YOLO (BuoyArray) Callback ─────────────────────

    def _cb_buoys(self, msg: BuoyArray):
        """
        Bounding box → kamera frame koordinatlarından açı tahmini.
        LiDAR ile birleştirmek için açı bilgisi kullanılır.
        """
        obstacles = []
        if msg.frame_width == 0:
            self._buoy_obstacles = obstacles
            return

        for buoy in msg.buoys:
            # Normalize pixel x → [-0.5, 0.5]
            norm_x = (buoy.center_x / msg.frame_width) - 0.5
            # Açıya çevir (kamera FOV içinde)
            angle = norm_x * self._cam_fov

            # Bounding box büyüklüğünden yaklaşık mesafe tahmini
            box_height = buoy.y_max - buoy.y_min
            ratio = box_height / msg.frame_height
            estimated_dist = max(0.5, self._safety_r * (1.0 - ratio))

            obstacles.append((angle, estimated_dist, buoy.label))

        self._buoy_obstacles = obstacles

    # ───────────────────── Hedef Yönü ─────────────────────

    def _cb_goal(self, msg: Vector3Stamped):
        self._goal_angle = math.atan2(msg.vector.y, msg.vector.x)

    # ───────────────────── Görev Fazı ─────────────────────

    def _cb_phase(self, msg: String):
        self._phase = msg.data

    def _publish_radii(self):
        s = Float64()
        s.data = float(self._safety_r)
        self.pub_safety_r.publish(s)
        d = Float64()
        d.data = float(self._danger_r)
        self.pub_danger_r.publish(d)

    # ───────────────────── Karar Döngüsü ─────────────────────

    def _decision_loop(self):
        # Görev aktif değilse otonomi hiçbir komut üretmez. Aktiflikten çıkarken
        # bir kez sıfır hız yayınlanır ki motorlar son komutta takılı kalmasın;
        # sonrasında topic manuel kumandaya bırakılır.
        if self._phase not in self._active_phases:
            if self._was_active:
                self._was_active = False
                self.pub_cmd.publish(Twist())
                self.get_logger().info(
                    f'Faz {self._phase} — otonomi beklemede, motor komutu üretilmiyor.'
                )
            return
        if not self._was_active:
            self._was_active = True
            self.get_logger().info(f'Faz {self._phase} — otonom sürüş devrede.')

        # Hız telemetrisi gelmese bile yarıçaplar tutarlı kalsın.
        self._update_radii()
        self._publish_radii()

        sector_size = 2 * math.pi / self._num_sectors
        combined = self._histogram.copy()

        # YOLO engellerini histograma birleştir
        for angle, dist, label in self._buoy_obstacles:
            if dist < self._safety_r:
                certainty = 1.0 - (dist / self._safety_r)
                sector_idx = int(((angle % (2 * math.pi)) / sector_size)) % self._num_sectors
                combined[sector_idx] = max(combined[sector_idx], certainty)

        # ─── Serbest sektörleri bul ───
        free_sectors = []
        for i in range(self._num_sectors):
            if combined[i] < 0.3:  # eşik
                free_sectors.append(i)

        if not free_sectors:
            # Tüm yönler engellenmiş — acil dur
            self.get_logger().warn('Tüm yönler engellenmiş! Motorlar durduruluyor.')
            self.pub_cmd.publish(Twist())
            return

        # ─── Hedefe en yakın serbest sektörü seç ───
        goal_sector = int(((self._goal_angle % (2 * math.pi)) / sector_size)) % self._num_sectors

        best_sector = min(
            free_sectors,
            key=lambda s: min(abs(s - goal_sector), self._num_sectors - abs(s - goal_sector)),
        )

        # ─── Yönlendirme komutu oluştur ───
        chosen_angle = best_sector * sector_size
        # Normalize [-pi, pi]
        steer = chosen_angle - self._goal_angle
        if steer > math.pi:
            steer -= 2 * math.pi
        elif steer < -math.pi:
            steer += 2 * math.pi

        # Tehlike seviyesi → hız azaltma
        max_obstacle = float(np.max(combined))
        speed_factor = max(0.2, 1.0 - max_obstacle)

        cmd = Twist()
        cmd.linear.x = self._max_speed * speed_factor * self._sensor_speed_scale()
        cmd.angular.z = float(np.clip(steer, -1.0, 1.0))

        # ─── Tehlike yarıçapı ihlali ───
        # Engel bu mesafedeyse hızlanmak için çok geç: gaz kesilir ama dümen
        # açık kalır, böylece araç sürüklenirken serbest sektöre doğru dönebilir.
        if self._min_range < self._danger_r:
            cmd.linear.x = 0.0
            if not self._danger_warned:
                self._danger_warned = True
                self.get_logger().warn(
                    f'Tehlike yarıçapı ihlali: engel {self._min_range:.2f} m '
                    f'(< {self._danger_r:.2f} m, hız {self._speed:.1f} m/s) — '
                    f'gaz kesildi, kaçış manevrası sürüyor.'
                )
        else:
            self._danger_warned = False

        self.pub_cmd.publish(cmd)

    # Governor integral kazancı — karar döngüsü 10 Hz olduğu için 0.1 s adım başına
    # hata (m/s) × bu kazanç kadar gaz oranı değişir. Teknenin ataleti ve dalga
    # bozucuları yüzünden yüksek kazanç gaz komutunu salındırıyor; ölü bant da
    # küçük hata etrafında sürekli düzeltmeyi engelliyor.
    GOVERNOR_GAIN = 0.03
    GOVERNOR_DEADBAND = 0.25   # m/s

    def _sensor_speed_scale(self) -> float:
        """
        Hızı sensör menzilinin izin verdiği değerde tutar. Gaz komutu normalize
        [-1, 1] olduğu ve m/s karşılığı araca göre değiştiği için sınır kapalı
        çevrimle uygulanır: ölçülen hız hedefin üstündeyse gaz oranı kademeli
        düşer, altına inince kademeli geri açılır.
        """
        if not self._limit_speed or self._v_max_sensor <= 0.0:
            return 1.0

        error = self._v_max_sensor - self._speed
        if abs(error) > self.GOVERNOR_DEADBAND:
            self._throttle_scale = float(
                np.clip(self._throttle_scale + self.GOVERNOR_GAIN * error, 0.05, 1.0)
            )

        over = self._speed > self._v_max_sensor
        if over and not self._speed_warned:
            self._speed_warned = True
            self.get_logger().warn(
                f'Hız {self._speed:.1f} m/s, sensör menzilinin izin verdiği '
                f'{self._v_max_sensor:.1f} m/s üstünde — gaz kısılıyor.'
            )
        elif not over:
            self._speed_warned = False

        return self._throttle_scale

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT'te rclpy kendi handler'ıyla context'i zaten kapatmış olabilir;
        # ikinci kez çağırmak RCLError fırlatır.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
