"""
pymavlink_controller_node
─────────────────────────
Doğrudan pymavlink ile Pixhawk'a seri port üzerinden bağlanır.
Thread-1 : Telemetri okuma  → ROS 2 topic'lerine publish
Thread-2 : ROS 2 komutları  → MAVLink mesajlarına çevirip otopilota yazma
"""

import json
import threading
import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import Twist, Vector3Stamped
from sensor_msgs.msg import NavSatFix, Imu

from pymavlink import mavutil


# Görev seçimi otopilot üzerindeki kullanıcı parametreleri ile yapılır:
#   SCR_USER1 → görev/parkur mod seçici (mission_manager_node'daki src_user)
#   SCR_USER2 → Parkur 3 (Kamikaze) için hedef renk kodu
# Yer istasyonundan ya da web arayüzünden bu parametre değiştirildiğinde
# Jetson değeri okuyup ilgili faz geçişini uygular.
DEFAULT_USER_PARAMS = ['SCR_USER1', 'SCR_USER2']


class PymavlinkControllerNode(Node):

    def __init__(self):
        super().__init__('pymavlink_controller_node')

        # ---------- Parametreler ----------
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('system_id', 1)
        self.declare_parameter('component_id', 1)
        self.declare_parameter('user_param_names', DEFAULT_USER_PARAMS)
        self.declare_parameter('user_param_poll_sec', 1.0)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self._sys_id = self.get_parameter('system_id').value
        self._comp_id = self.get_parameter('component_id').value
        self._user_param_names = list(self.get_parameter('user_param_names').value)
        self._user_param_poll = float(self.get_parameter('user_param_poll_sec').value)

        # ---------- MAVLink bağlantısı ----------
        self.get_logger().info(f'MAVLink bağlantısı açılıyor: {port}@{baud}')
        self._conn = mavutil.mavlink_connection(port, baud=baud)
        self._conn.wait_heartbeat(timeout=30)
        self.get_logger().info(
            f'Heartbeat alındı — system {self._conn.target_system}, '
            f'component {self._conn.target_component}'
        )

        # ---------- Publisher'lar ----------
        self.pub_gps = self.create_publisher(NavSatFix, 'telemetry/gps', 10)
        self.pub_imu = self.create_publisher(Imu, 'telemetry/imu', 10)
        self.pub_heading = self.create_publisher(Float64, 'telemetry/heading', 10)
        self.pub_groundspeed = self.create_publisher(Float64, 'telemetry/groundspeed', 10)
        self.pub_altitude = self.create_publisher(Float64, 'telemetry/altitude', 10)
        self.pub_heartbeat_status = self.create_publisher(String, 'telemetry/heartbeat_status', 10)
        self.pub_user_params = self.create_publisher(String, 'telemetry/user_params', 10)

        # ---------- Subscriber'lar ----------
        self.create_subscription(Twist, 'cmd/velocity', self._cb_velocity, 10)
        self.create_subscription(Vector3Stamped, 'cmd/setpoint_ned', self._cb_setpoint_ned, 10)
        self.create_subscription(Bool, 'cmd/arm', self._cb_arm, 10)
        self.create_subscription(String, 'cmd/mode', self._cb_mode, 10)
        self.create_subscription(String, 'cmd/set_user_param', self._cb_set_user_param, 10)

        # ---------- Durum ----------
        self._lock = threading.Lock()
        self._running = True
        self._user_params = {}       # {'SCR_USER1': 5.0, ...}

        # ---------- SCR_USER parametre yoklama ----------
        if self._user_param_names:
            self.create_timer(self._user_param_poll, self._poll_user_params)
            self.get_logger().info(
                f'Kullanıcı parametreleri {self._user_param_poll:.1f} sn\'de bir '
                f'okunuyor: {", ".join(self._user_param_names)}'
            )

        # ---------- Thread'ler ----------
        self._telem_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        self._telem_thread.start()

        self.get_logger().info('pymavlink_controller_node başlatıldı.')

    # ───────────────────── Telemetri Okuma Thread'i ─────────────────────

    def _telemetry_loop(self):
        """Kesintisiz olarak MAVLink mesajlarını okuyup ROS 2'ye publish eder."""
        while self._running:
            try:
                msg = self._conn.recv_match(
                    type=[
                        'HEARTBEAT',
                        'GLOBAL_POSITION_INT',
                        'ATTITUDE',
                        'VFR_HUD',
                        'PARAM_VALUE',
                    ],
                    blocking=True,
                    timeout=1.0,
                )
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'HEARTBEAT':
                    self._handle_heartbeat(msg)
                elif msg_type == 'GLOBAL_POSITION_INT':
                    self._handle_global_position(msg)
                elif msg_type == 'ATTITUDE':
                    self._handle_attitude(msg)
                elif msg_type == 'VFR_HUD':
                    self._handle_vfr_hud(msg)
                elif msg_type == 'PARAM_VALUE':
                    self._handle_param_value(msg)

            except Exception as e:
                self.get_logger().warn(f'Telemetri okuma hatası: {e}')
                time.sleep(0.1)

    def _handle_heartbeat(self, msg):
        status = String()
        mode = mavutil.mode_string_v10(msg) if hasattr(msg, 'custom_mode') else 'UNKNOWN'
        status.data = f'mode={mode} armed={msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0}'
        self.pub_heartbeat_status.publish(status)

    def _handle_global_position(self, msg):
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.latitude = msg.lat / 1e7
        fix.longitude = msg.lon / 1e7
        fix.altitude = msg.alt / 1e3
        self.pub_gps.publish(fix)

        alt_msg = Float64()
        alt_msg.data = msg.relative_alt / 1e3
        self.pub_altitude.publish(alt_msg)

    def _handle_attitude(self, msg):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = 'base_link'
        # Quaternion dönüşümü (roll, pitch, yaw → quaternion)
        cr = math.cos(msg.roll * 0.5)
        sr = math.sin(msg.roll * 0.5)
        cp = math.cos(msg.pitch * 0.5)
        sp = math.sin(msg.pitch * 0.5)
        cy = math.cos(msg.yaw * 0.5)
        sy = math.sin(msg.yaw * 0.5)
        imu.orientation.w = cr * cp * cy + sr * sp * sy
        imu.orientation.x = sr * cp * cy - cr * sp * sy
        imu.orientation.y = cr * sp * cy + sr * cp * sy
        imu.orientation.z = cr * cp * sy - sr * sp * cy
        imu.angular_velocity.x = msg.rollspeed
        imu.angular_velocity.y = msg.pitchspeed
        imu.angular_velocity.z = msg.yawspeed
        self.pub_imu.publish(imu)

    def _handle_vfr_hud(self, msg):
        heading = Float64()
        heading.data = float(msg.heading)
        self.pub_heading.publish(heading)

        gs = Float64()
        gs.data = float(msg.groundspeed)
        self.pub_groundspeed.publish(gs)

    # ───────────────────── SCR_USER Parametreleri ─────────────────────

    def _handle_param_value(self, msg):
        """Otopilottan gelen PARAM_VALUE — sadece izlediğimiz parametreler yayınlanır."""
        name = msg.param_id
        if isinstance(name, bytes):
            name = name.decode('utf-8', errors='ignore')
        name = name.strip('\x00')
        if name not in self._user_param_names:
            return

        value = float(msg.param_value)
        if self._user_params.get(name) == value:
            return  # değişmediyse tekrar yayınlama

        previous = self._user_params.get(name)
        self._user_params[name] = value
        self.get_logger().info(f'{name}: {previous} → {value}')

        out = String()
        out.data = json.dumps(self._user_params)
        self.pub_user_params.publish(out)

    def _poll_user_params(self):
        """İzlenen parametreleri otopilottan periyodik olarak ister."""
        with self._lock:
            for name in self._user_param_names:
                try:
                    self._conn.mav.param_request_read_send(
                        self._conn.target_system,
                        self._conn.target_component,
                        name.encode('utf-8'),
                        -1,
                    )
                except Exception as e:
                    self.get_logger().warn(f'{name} okunamadı: {e}')

    def _cb_set_user_param(self, msg: String):
        """
        Web arayüzünden parametre yazma: {"name": "SCR_USER1", "value": 10}
        Gerçek akışla aynı yolu kullanır — değer otopiluta yazılır, sonra
        PARAM_VALUE olarak geri okunup görev fazına dönüşür.
        """
        try:
            data = json.loads(msg.data)
            name = str(data['name'])
            value = float(data['value'])
        except (ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'Geçersiz set_user_param isteği: {msg.data!r} ({e})')
            return

        with self._lock:
            self._conn.mav.param_set_send(
                self._conn.target_system,
                self._conn.target_component,
                name.encode('utf-8'),
                value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        self.get_logger().info(f'{name} = {value} yazıldı.')

    # ───────────────────── Arm / Mod Komutları ─────────────────────

    def _cb_arm(self, msg: Bool):
        self.arm() if msg.data else self.disarm()

    def _cb_mode(self, msg: String):
        self.set_mode(msg.data)

    # ───────────────────── Komut Gönderme (ROS → MAVLink) ─────────────────────

    def _cb_velocity(self, twist: Twist):
        """RC_CHANNELS_OVERRIDE ile hız komutu gönderir.

        DÜMEN KANAL 1'DEN GİDER, KANAL 4'TEN DEĞİL. ArduPilot Rover dümen
        girdisini `RCMAP_ROLL` kanalından (varsayılan ch1) okur; ch4 (RCMAP_YAW)
        Rover'da hiç kullanılmaz. Dümen ch4'e yazıldığında otopilot komutu
        sessizce yok sayar — hata da vermez — ve araç yalnızca düz gider.
        SITL'de ölçüldü: ch4=1700 gönderildiğinde iki itki de 1500'de kalıyor,
        ch1=1700 gönderildiğinde sol 1680 / sağ 1320 oluyor.

        İşaret: `Twist.angular.z` ROS geleneğinde CCW pozitiftir, yani +z SOLA
        dönüştür (`collision_avoidance_node` hedef açısını atan2(y, x) ile
        üretirken bu geleneği kullanıyor). ArduPilot'ta dümen kanalı 1500'ün
        ÜSTÜNDE aracı SAĞA döndürdüğü için işaret ters çevriliyor.
        """
        # Gaz:   linear.x  → RCMAP_THROTTLE kanalı (ch3), 1100-1900
        # Dümen: angular.z → RCMAP_ROLL kanalı (ch1), 1100-1900
        throttle = int(self._clamp(twist.linear.x, -1.0, 1.0) * 400 + 1500)
        steer = int(self._clamp(-twist.angular.z, -1.0, 1.0) * 400 + 1500)

        with self._lock:
            self._conn.mav.rc_channels_override_send(
                self._conn.target_system,
                self._conn.target_component,
                steer,       # ch1 — dümen (RCMAP_ROLL)
                0,           # ch2
                throttle,    # ch3 — gaz (RCMAP_THROTTLE)
                0,           # ch4 — Rover'da kullanılmıyor
                0, 0, 0, 0,  # ch5-ch8
            )

    def _cb_setpoint_ned(self, msg: Vector3Stamped):
        """SET_POSITION_TARGET_LOCAL_NED ile pozisyon komutu gönderir."""
        with self._lock:
            self._conn.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self._conn.target_system,
                self._conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,  # position only
                msg.vector.x, msg.vector.y, msg.vector.z,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0,     # yaw, yaw_rate
            )

    # ───────────────────── Arm / Disarm / Mode ─────────────────────

    def arm(self):
        with self._lock:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0,
            )
        self.get_logger().info('ARM komutu gönderildi.')

    def disarm(self):
        with self._lock:
            self._conn.mav.command_long_send(
                self._conn.target_system, self._conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0,
            )
        self.get_logger().info('DISARM komutu gönderildi.')

    def set_mode(self, mode_name: str):
        mode_id = self._conn.mode_mapping().get(mode_name.upper())
        if mode_id is None:
            self.get_logger().error(f'Bilinmeyen mod: {mode_name}')
            return
        with self._lock:
            self._conn.set_mode(mode_id)
        self.get_logger().info(f'Mod değiştirildi: {mode_name}')

    # ───────────────────── Yardımcılar ─────────────────────

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))

    def destroy_node(self):
        self._running = False
        self._telem_thread.join(timeout=2.0)
        self._conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PymavlinkControllerNode()
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
