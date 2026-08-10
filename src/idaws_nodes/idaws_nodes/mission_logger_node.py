"""
mission_logger_node
───────────────────
Telemetri verilerini 1 Hz frekansla başlık satırlı CSV'ye kaydeder.
"""

import csv
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist


class MissionLoggerNode(Node):

    def __init__(self):
        super().__init__('mission_logger_node')

        # ---------- Parametreler ----------
        self.declare_parameter('log_dir', '/home/jetson/idaws_logs')
        self._log_dir = self.get_parameter('log_dir').value
        os.makedirs(self._log_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(self._log_dir, f'mission_{ts}.csv')

        # ---------- CSV dosyası ----------
        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow([
            'timestamp',
            'lat', 'lon', 'alt',
            'roll', 'pitch', 'yaw',
            'heading', 'groundspeed',
            'cmd_linear_x', 'cmd_angular_z',
            'mission_phase',
        ])

        # ---------- Telemetri cache ----------
        self._lat = 0.0
        self._lon = 0.0
        self._alt = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = 0.0
        self._heading = 0.0
        self._groundspeed = 0.0
        self._cmd_lx = 0.0
        self._cmd_az = 0.0
        self._phase = 'IDLE'

        # ---------- Subscriber'lar ----------
        self.create_subscription(NavSatFix, 'telemetry/gps', self._cb_gps, 10)
        self.create_subscription(Imu, 'telemetry/imu', self._cb_imu, 10)
        self.create_subscription(Float64, 'telemetry/heading', self._cb_heading, 10)
        self.create_subscription(Float64, 'telemetry/groundspeed', self._cb_gs, 10)
        self.create_subscription(Twist, 'cmd/velocity', self._cb_cmd, 10)
        self.create_subscription(String, 'mission/phase', self._cb_phase, 10)

        # ---------- 1 Hz loglama timer ----------
        self.create_timer(1.0, self._log_row)

        self.get_logger().info(f'mission_logger_node başlatıldı. CSV: {self._csv_path}')

    # ───────────────────── Callback'ler ─────────────────────

    def _cb_gps(self, msg: NavSatFix):
        self._lat = msg.latitude
        self._lon = msg.longitude
        self._alt = msg.altitude

    def _cb_imu(self, msg: Imu):
        import math
        q = msg.orientation
        # Quaternion → Euler
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        self._roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        self._pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

    def _cb_heading(self, msg: Float64):
        self._heading = msg.data

    def _cb_gs(self, msg: Float64):
        self._groundspeed = msg.data

    def _cb_cmd(self, msg: Twist):
        self._cmd_lx = msg.linear.x
        self._cmd_az = msg.angular.z

    def _cb_phase(self, msg: String):
        self._phase = msg.data

    # ───────────────────── CSV Satır Yazma ─────────────────────

    def _log_row(self):
        self._writer.writerow([
            datetime.now().isoformat(),
            f'{self._lat:.7f}', f'{self._lon:.7f}', f'{self._alt:.2f}',
            f'{self._roll:.4f}', f'{self._pitch:.4f}', f'{self._yaw:.4f}',
            f'{self._heading:.1f}', f'{self._groundspeed:.2f}',
            f'{self._cmd_lx:.3f}', f'{self._cmd_az:.3f}',
            self._phase,
        ])
        self._csv_file.flush()

    # ───────────────────── Temizlik ─────────────────────

    def destroy_node(self):
        self._csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionLoggerNode()
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
