"""
mission_manager_node
────────────────────
3 aşamalı görev yönetimi: Parkur 1 → Parkur 2 → Bekleme (JSON) → Parkur 3 (Kamikaze).

JSON dinleyicisi node başlangıcından itibaren sürekli aktiftir (UART/USB üzerinden,
seri port açılamazsa /tmp/idaws_target.json dosyası fallback olarak izlenir).

Her JSON mesajındaki `src_user` alanı bir MOD SEÇİCİDİR (sayısal, 5'er aralıklı —
float hassasiyet sorunlarını önlemek için değerler arasında pay bırakılmıştır;
gelen değer en yakın 5'in katına yuvarlanır):

    0  = IDLE   → motorlar durur, faz IDLE'a döner
    5  = REAL   → gerçek görev: IDLE'daysa Parkur 1 başlatılır, otomatik P1→P2 akışı
                  mission/parkur_complete sinyalleriyle ilerler; Parkur 2 bitince
                  motorlar durur ve bir sonraki src_user=P3 (20) mesajı beklenir
    10 = P1     → test: doğrudan Parkur 1'e zorlar (mevcut fazdan bağımsız)
    15 = P2     → test: doğrudan Parkur 2'ye zorlar
    20 = P3     → hem gerçek akışın doğal son adımı hem de test zorlaması: JSON'daki
                  color/lat/lon/extra hedef bilgisi olarak işlenir, Parkur 3 (Kamikaze)
                  fazına geçilir

Parkur 1'in bittiğini / Parkur 2'nin ne zaman başlayacağını belirleyen şartname bazlı
algılama mantığı henüz bu node'a dahil edilmemiştir (ayrıca ele alınacak); o karar
mission/parkur_complete topic'i üzerinden dışarıdan besleniyor.
"""

import json
import os
import threading
import time
from enum import IntEnum, auto

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Twist


class MissionPhase(IntEnum):
    IDLE = 0
    PARKUR_1 = auto()
    PARKUR_2 = auto()
    WAITING_JSON = auto()
    PARKUR_3_KAMIKAZE = auto()
    COMPLETED = auto()


SRC_USER_STEP = 5.0
SRC_USER_MODES = ['IDLE', 'REAL', 'P1', 'P2', 'P3']
JSON_FALLBACK_PATH = '/tmp/idaws_target.json'


class MissionManagerNode(Node):

    def __init__(self):
        super().__init__('mission_manager_node')

        # ---------- Parametreler ----------
        self.declare_parameter('json_serial_port', '/dev/ttyUSB0')
        self.declare_parameter('json_baud_rate', 57600)
        self.declare_parameter('parkur1_timeout_sec', 300.0)
        self.declare_parameter('parkur2_timeout_sec', 300.0)
        # Otopilot üzerindeki mod seçici ve Parkur 3 renk parametresi
        self.declare_parameter('mode_param_name', 'SCR_USER1')
        self.declare_parameter('color_param_name', 'SCR_USER2')

        self._json_port = self.get_parameter('json_serial_port').value
        self._json_baud = self.get_parameter('json_baud_rate').value
        self._mode_param = self.get_parameter('mode_param_name').value
        self._color_param = self.get_parameter('color_param_name').value

        # ---------- Durum ----------
        self._phase = MissionPhase.IDLE
        self._target_info = {}
        self._last_mode_value = None
        self._phase_lock = threading.Lock()
        self._running = True

        # ---------- Publisher'lar ----------
        self.pub_phase = self.create_publisher(String, 'mission/phase', 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd/velocity', 10)
        self.pub_target = self.create_publisher(String, 'mission/target', 10)

        # ---------- Subscriber'lar ----------
        self.create_subscription(Bool, 'mission/start', self._cb_start, 10)
        self.create_subscription(Bool, 'mission/stop', self._cb_stop, 10)
        self.create_subscription(String, 'mission/parkur_complete', self._cb_parkur_complete, 10)
        self.create_subscription(String, 'telemetry/user_params', self._cb_user_params, 10)

        # ---------- Timer ----------
        self._timer = self.create_timer(1.0, self._publish_phase)

        # ---------- JSON dinleme thread'i (node ömrü boyunca sürekli aktif) ----------
        self._listener_thread = threading.Thread(target=self._wait_for_json, daemon=True)
        self._listener_thread.start()

        self.get_logger().info('mission_manager_node başlatıldı. Faz: IDLE')

    # ───────────────────── Faz Publish ─────────────────────

    def _publish_phase(self):
        msg = String()
        msg.data = self._phase.name
        self.pub_phase.publish(msg)

    # ───────────────────── Start / Stop (yerel/web UI kontrolü) ─────────────────────

    def _cb_start(self, msg: Bool):
        if msg.data and self._phase == MissionPhase.IDLE:
            self.get_logger().info('Görev başlatılıyor — Parkur 1')
            self._set_phase(MissionPhase.PARKUR_1)

    def _cb_stop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn('Görev durduruldu — motorlar kapatılıyor.')
            self._stop_motors()
            self._set_phase(MissionPhase.IDLE)

    # ───────────────────── Parkur Tamamlama (gerçek akış — otomatik ilerleme) ─────────────────────

    def _cb_parkur_complete(self, msg: String):
        completed = msg.data.upper()
        with self._phase_lock:
            if completed == 'PARKUR_1' and self._phase == MissionPhase.PARKUR_1:
                self.get_logger().info('Parkur 1 tamamlandı → Parkur 2 geçiliyor.')
                self._set_phase(MissionPhase.PARKUR_2)

            elif completed == 'PARKUR_2' and self._phase == MissionPhase.PARKUR_2:
                self.get_logger().info('Parkur 2 tamamlandı → Motorlar durduruluyor, src_user=P3 (20) bekleniyor.')
                self._stop_motors()
                self._set_phase(MissionPhase.WAITING_JSON)

    # ───────────────────── Otopilot Parametreleri (SCR_USER) ─────────────────────

    def _cb_user_params(self, msg: String):
        """
        pymavlink_controller_node'un otopilottan okuduğu kullanıcı parametreleri.
        mode_param (SCR_USER1) src_user mod seçicisidir; color_param (SCR_USER2)
        Parkur 3 hedef rengini taşır. Sadece mod değeri değiştiğinde faz uygulanır,
        aksi hâlde her yoklamada aynı faz tekrar tetiklenirdi.
        """
        try:
            params = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'user_params çözülemedi: {msg.data!r}')
            return

        if self._mode_param not in params:
            return

        value = float(params[self._mode_param])
        if value == self._last_mode_value:
            return
        self._last_mode_value = value

        data = {'src_user': value}
        if self._color_param in params:
            data['color'] = params[self._color_param]

        self.get_logger().info(f'{self._mode_param}={value} → görev komutu uygulanıyor.')
        self._apply_src_user(data)

    # ───────────────────── Motor Durdurma ─────────────────────

    def _stop_motors(self):
        stop = Twist()  # tüm alanlar 0.0
        self.pub_cmd_vel.publish(stop)
        self.get_logger().info('Motorlar durduruldu (zero velocity).')

    # ───────────────────── JSON Dinleme (UART/USB, sürekli) ─────────────────────

    def _wait_for_json(self):
        """
        UART/USB üzerinden gelen JSON verisini satır satır okur; node çalıştığı
        sürece durmadan dinlemeye devam eder (tek seferlik değil).
        """
        import serial

        self.get_logger().info(
            f'JSON dinleniyor: {self._json_port}@{self._json_baud}'
        )

        try:
            ser = serial.Serial(self._json_port, self._json_baud, timeout=1.0)
        except Exception as e:
            self.get_logger().error(f'Seri port açılamadı: {e}')
            self.get_logger().info(f'Fallback: dosya tabanlı JSON bekleniyor ({JSON_FALLBACK_PATH})')
            self._wait_for_json_file()
            return

        buffer = ''
        while self._running:
            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                buffer += line
                try:
                    data = json.loads(buffer)
                    self._process_json(data)
                    buffer = ''
                except json.JSONDecodeError:
                    # Çok satırlı JSON olabilir, biriktirmeye devam et
                    continue
            except Exception as e:
                self.get_logger().warn(f'UART okuma hatası: {e}')
                time.sleep(0.5)

        ser.close()

    def _wait_for_json_file(self):
        """Fallback: /tmp/idaws_target.json dosyasını mtime değişikliği için sürekli izler."""
        last_mtime = None
        while self._running:
            try:
                if os.path.exists(JSON_FALLBACK_PATH):
                    mtime = os.path.getmtime(JSON_FALLBACK_PATH)
                    if mtime != last_mtime:
                        last_mtime = mtime
                        with open(JSON_FALLBACK_PATH, 'r') as f:
                            data = json.load(f)
                        self._process_json(data)
            except Exception as e:
                self.get_logger().warn(f'JSON dosya okuma hatası: {e}')
            time.sleep(0.5)

    # ───────────────────── src_user Mod Çözümleme ─────────────────────

    @staticmethod
    def _resolve_src_user_mode(value: float) -> str:
        idx = int(round(value / SRC_USER_STEP))
        idx = max(0, min(idx, len(SRC_USER_MODES) - 1))
        return SRC_USER_MODES[idx]

    def _process_json(self, data: dict):
        """JSON verisinden src_user mod değerini ayrıştırıp ilgili faz geçişini uygular."""
        self.get_logger().info(f'JSON alındı: {json.dumps(data, ensure_ascii=False)}')

        if 'src_user' not in data:
            self.get_logger().warn('JSON içinde src_user bulunamadı!')
            return

        self._apply_src_user(data)

    def _apply_src_user(self, data: dict):
        """
        src_user mod seçicisini faz geçişine dönüştürür. Hem seri porttan gelen
        JSON hem de otopilottaki SCR_USER1 parametresi bu yola bağlanır.
        """
        try:
            raw_value = float(data['src_user'])
        except (TypeError, ValueError):
            self.get_logger().warn(f"src_user sayısal değil: {data['src_user']!r}")
            return

        mode = self._resolve_src_user_mode(raw_value)
        self.get_logger().info(f'src_user={raw_value} → mod={mode}')

        with self._phase_lock:
            if mode == 'IDLE':
                self._stop_motors()
                self._set_phase(MissionPhase.IDLE)

            elif mode == 'REAL':
                if self._phase == MissionPhase.IDLE:
                    self.get_logger().info('REAL sinyali → gerçek görev başlatılıyor (Parkur 1).')
                    self._set_phase(MissionPhase.PARKUR_1)
                else:
                    self.get_logger().info(
                        f'REAL sinyali alındı, görev zaten devam ediyor (faz: {self._phase.name}).'
                    )

            elif mode == 'P1':
                self.get_logger().info('Test modu: Parkur 1 zorlanıyor.')
                self._set_phase(MissionPhase.PARKUR_1)

            elif mode == 'P2':
                self.get_logger().info('Test modu: Parkur 2 zorlanıyor.')
                self._set_phase(MissionPhase.PARKUR_2)

            elif mode == 'P3':
                self._update_target(data)
                self.get_logger().info('Parkur 3 (Kamikaze) tetiklendi.')
                self._set_phase(MissionPhase.PARKUR_3_KAMIKAZE)

    def _update_target(self, data: dict):
        """P3 hedef bilgisini (renk, konum, ekstra) ayrıştırıp yayınlar."""
        self._target_info = {
            'color': data.get('color', ''),
            'lat': data.get('lat', 0.0),
            'lon': data.get('lon', 0.0),
            'extra': data.get('extra', {}),
        }
        target_msg = String()
        target_msg.data = json.dumps(self._target_info, ensure_ascii=False)
        self.pub_target.publish(target_msg)
        self.get_logger().info(f'Hedef güncellendi: {self._target_info}')

    # ───────────────────── Faz Güncelleme ─────────────────────

    def _set_phase(self, phase: MissionPhase):
        self._phase = phase
        self.get_logger().info(f'Görev fazı: {phase.name}')
        self._publish_phase()

    def destroy_node(self):
        self._running = False
        self._listener_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
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
