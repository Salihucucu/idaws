"""
lidar_logger_node
─────────────────
LaserScan → engel kümeleme + lokal cost map üretimi, yayını ve kaydı.

Şartname karşılıkları:
  * "Diğer Otonomi Sensörleri Veri Seti" — LiDAR verisi, en az 1 Hz, her veri seti
    zaman etiketli, mp4 formatında, kümeleme/ayırma işlemi görünecek şekilde.
    → lidar_YYYYmmdd_HHMMSS.mp4 (kuş bakışı tarama + renklendirilmiş kümeler)
  * "Dosya 3: Lokal harita/cost map/engel haritası" — en az 1 Hz.
    → costmap_YYYYmmdd_HHMMSS.mp4 + nav_msgs/OccupancyGrid yayını

Yayınlar:
  perception/lidar_clusters : idaws_msgs/ClusterArray
  perception/costmap        : nav_msgs/OccupancyGrid  (araç merkezli, base_link)

Koordinatlar araç gövde çerçevesinde: +x ileri, +y sol.
"""

import math
import os
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from idaws_msgs.msg import Cluster, ClusterArray


# Kümeleri birbirinden ayırt etmek için sabit palet (BGR)
CLUSTER_COLORS = [
    (80, 220, 80), (80, 160, 255), (255, 180, 60), (200, 120, 255),
    (60, 230, 230), (255, 120, 160), (140, 255, 180), (180, 180, 255),
]


class LidarLoggerNode(Node):

    def __init__(self):
        super().__init__('lidar_logger_node')

        # ---------- Parametreler ----------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('log_dir', '/home/jetson/idaws_logs')
        self.declare_parameter('record_hz', 1.0)          # şartname: en az 1 Hz
        self.declare_parameter('record_video', True)
        self.declare_parameter('render_size_px', 600)

        # Kümeleme: iki komşu ışın arası mesafe eşiği, uzakta ışınlar açıldığı
        # için mesafeyle birlikte büyür (gap_base + gap_factor * menzil).
        self.declare_parameter('cluster_gap_base', 0.35)
        self.declare_parameter('cluster_gap_factor', 0.06)
        self.declare_parameter('cluster_min_points', 3)
        self.declare_parameter('max_cluster_range', 12.0)

        # Cost map
        self.declare_parameter('map_size_m', 24.0)
        self.declare_parameter('map_resolution', 0.1)
        self.declare_parameter('inflation_radius', 0.8)

        self._scan_topic = self.get_parameter('scan_topic').value
        self._log_dir = self.get_parameter('log_dir').value
        self._record_hz = max(1.0, float(self.get_parameter('record_hz').value))
        self._record_video = bool(self.get_parameter('record_video').value)
        self._render_px = int(self.get_parameter('render_size_px').value)
        self._gap_base = float(self.get_parameter('cluster_gap_base').value)
        self._gap_factor = float(self.get_parameter('cluster_gap_factor').value)
        self._min_points = int(self.get_parameter('cluster_min_points').value)
        self._max_range = float(self.get_parameter('max_cluster_range').value)
        self._map_size = float(self.get_parameter('map_size_m').value)
        self._map_res = float(self.get_parameter('map_resolution').value)
        self._inflation = float(self.get_parameter('inflation_radius').value)

        self._grid_n = max(1, int(round(self._map_size / self._map_res)))

        # ---------- Yayıncılar ----------
        self.pub_clusters = self.create_publisher(ClusterArray, 'perception/lidar_clusters', 10)
        self.pub_costmap = self.create_publisher(OccupancyGrid, 'perception/costmap', 1)

        # ---------- Abone ----------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, self._scan_topic, self._cb_scan, sensor_qos)

        # ---------- Durum ----------
        self._points = np.zeros((0, 2), dtype=np.float32)   # (x, y) metre
        self._clusters = []                                  # dict listesi
        self._cost_grid = None                               # uint8 [0, 100]
        self._scan_range_max = self._max_range
        self._have_scan = False

        # ---------- Kayıt ----------
        self._lidar_writer = None
        self._costmap_writer = None
        if self._record_video:
            os.makedirs(self._log_dir, exist_ok=True)
            self.create_timer(1.0 / self._record_hz, self._record_callback)

        self.get_logger().info(
            f'lidar_logger_node başlatıldı — {self._scan_topic} → kümeler + '
            f'{self._grid_n}x{self._grid_n} cost map ({self._map_res} m/hücre), '
            f'kayıt {"açık" if self._record_video else "kapalı"} @ {self._record_hz} Hz'
        )

    # ───────────────────── Tarama İşleme ─────────────────────

    def _cb_scan(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        limit = min(float(msg.range_max), self._max_range)
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= limit)

        r = ranges[valid]
        a = angles[valid]
        pts = np.column_stack((r * np.cos(a), r * np.sin(a))).astype(np.float32)

        self._scan_range_max = float(msg.range_max)
        self._points = pts
        self._clusters = self._cluster_points(pts, r)
        self._cost_grid = self._build_costmap(pts)
        self._have_scan = True

        self._publish_clusters(msg.header.stamp)
        self._publish_costmap(msg.header.stamp)

    def _cluster_points(self, pts, ranges):
        """Açı sırasına göre ardışık nokta mesafesine dayalı kümeleme."""
        if pts.shape[0] < self._min_points:
            return []

        # Ardışık noktalar arası mesafe, eşiği aşınca yeni küme başlar.
        deltas = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        thresholds = self._gap_base + self._gap_factor * np.minimum(ranges[:-1], ranges[1:])
        breaks = np.flatnonzero(deltas > thresholds) + 1

        groups = np.split(np.arange(pts.shape[0]), breaks)

        # 360° tarama: son küme ile ilk küme fiziksel olarak bitişikse birleştir.
        if len(groups) > 1:
            gap = float(np.linalg.norm(pts[0] - pts[-1]))
            thr = self._gap_base + self._gap_factor * min(float(ranges[0]), float(ranges[-1]))
            if gap <= thr:
                groups[0] = np.concatenate((groups[-1], groups[0]))
                groups.pop()

        clusters = []
        for idx in groups:
            if idx.size < self._min_points:
                continue
            member_pts = pts[idx]
            center = member_pts.mean(axis=0)
            span = float(np.linalg.norm(member_pts[0] - member_pts[-1]))
            clusters.append({
                'id': len(clusters),
                'points': member_pts,
                'center': center,
                'range': float(np.linalg.norm(center)),
                'bearing': float(math.atan2(center[1], center[0])),
                'min_range': float(np.min(ranges[idx])),
                'width': span,
                'count': int(idx.size),
            })
        return clusters

    def _build_costmap(self, pts):
        """Araç merkezli lokal cost map — engeller şişirilerek [0, 100] maliyete çevrilir."""
        import cv2

        grid = np.zeros((self._grid_n, self._grid_n), dtype=np.uint8)
        if pts.shape[0] == 0:
            return grid

        half = self._grid_n // 2
        # Hücre indeksi: satır = x (ileri), sütun = y (sol) — OccupancyGrid row-major
        cols = np.round(pts[:, 1] / self._map_res).astype(np.int32) + half
        rows = np.round(pts[:, 0] / self._map_res).astype(np.int32) + half
        inside = (rows >= 0) & (rows < self._grid_n) & (cols >= 0) & (cols < self._grid_n)
        if not np.any(inside):
            return grid

        occupied = np.zeros((self._grid_n, self._grid_n), dtype=np.uint8)
        occupied[rows[inside], cols[inside]] = 255

        # Şişirme: engelden uzaklaştıkça maliyet doğrusal azalır.
        inflation_cells = max(1.0, self._inflation / self._map_res)
        dist = cv2.distanceTransform(255 - occupied, cv2.DIST_L2, 3)
        cost = np.clip(1.0 - dist / inflation_cells, 0.0, 1.0) * 100.0
        grid = cost.astype(np.uint8)
        grid[occupied > 0] = 100
        return grid

    # ───────────────────── Yayınlar ─────────────────────

    def _publish_clusters(self, stamp):
        msg = ClusterArray()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        for c in self._clusters:
            m = Cluster()
            m.id = c['id']
            m.center_x = float(c['center'][0])
            m.center_y = float(c['center'][1])
            m.range = c['range']
            m.bearing = c['bearing']
            m.min_range = c['min_range']
            m.width = c['width']
            m.point_count = c['count']
            msg.clusters.append(m)
        self.pub_clusters.publish(msg)

    def _publish_costmap(self, stamp):
        if self._cost_grid is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = stamp
        grid.header.frame_id = 'base_link'
        grid.info.resolution = self._map_res
        grid.info.width = self._grid_n
        grid.info.height = self._grid_n
        # Origin araç merkezinden yarım harita geriye/sağa kayar.
        grid.info.origin.position.x = -self._map_size / 2.0
        grid.info.origin.position.y = -self._map_size / 2.0
        grid.info.origin.orientation.w = 1.0
        grid.data = self._cost_grid.astype(np.int8).flatten().tolist()
        self.pub_costmap.publish(grid)

    # ───────────────────── Görselleştirme & Kayıt ─────────────────────

    def _record_callback(self):
        if not self._have_scan:
            return
        import cv2

        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

        lidar_frame = self._render_scan(stamp)
        if self._lidar_writer is None:
            self._lidar_writer = self._open_writer('lidar', lidar_frame)
        self._lidar_writer.write(lidar_frame)

        costmap_frame = self._render_costmap(stamp)
        if self._costmap_writer is None:
            self._costmap_writer = self._open_writer('costmap', costmap_frame)
        self._costmap_writer.write(costmap_frame)

    def _open_writer(self, prefix, frame):
        import cv2

        os.makedirs(self._log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        path = os.path.join(self._log_dir, f'{prefix}_{ts}.mp4')
        h, w = frame.shape[:2]
        writer = cv2.VideoWriter(
            path, cv2.VideoWriter_fourcc(*'mp4v'), self._record_hz, (w, h)
        )
        self.get_logger().info(f'Kayıt: {path} ({w}x{h} @ {self._record_hz} Hz)')
        return writer

    def _render_scan(self, stamp: str):
        """Kuş bakışı tarama: ham noktalar gri, kümeler renkli ve etiketli."""
        import cv2

        size = self._render_px
        img = np.zeros((size, size, 3), dtype=np.uint8)
        img[:] = (10, 12, 16)
        cx = cy = size // 2
        max_r = max(1.0, self._scan_range_max)
        scale = (size / 2 - 24) / max_r

        def to_px(x, y):
            # Araç burnu yukarı: +x yukarı, +y sol
            return int(cx - y * scale), int(cy - x * scale)

        # Mesafe halkaları
        for i in range(1, 5):
            rr = max_r * i / 4.0
            cv2.circle(img, (cx, cy), int(rr * scale), (45, 50, 58), 1)
            cv2.putText(img, f'{rr:.0f}m', (cx + 4, cy - int(rr * scale) + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 116, 124), 1)
        cv2.line(img, (cx, 0), (cx, size), (35, 40, 48), 1)
        cv2.line(img, (0, cy), (size, cy), (35, 40, 48), 1)

        # Kümelenmemiş ham noktalar
        for x, y in self._points:
            px, py = to_px(x, y)
            if 0 <= px < size and 0 <= py < size:
                img[py, px] = (110, 110, 110)

        # Kümeler
        for c in self._clusters:
            color = CLUSTER_COLORS[c['id'] % len(CLUSTER_COLORS)]
            for x, y in c['points']:
                px, py = to_px(x, y)
                cv2.circle(img, (px, py), 2, color, -1)
            mx, my = to_px(c['center'][0], c['center'][1])
            radius = max(6, int((c['width'] / 2.0 + 0.2) * scale))
            cv2.circle(img, (mx, my), radius, color, 1)
            cv2.putText(img, f"K{c['id']} {c['range']:.1f}m", (mx + radius + 3, my),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

        self._draw_vehicle(img, cx, cy)

        cv2.putText(img, f'LiDAR | kume: {len(self._clusters)}', (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(img, stamp, (10, size - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    def _render_costmap(self, stamp: str):
        """Cost map görselleştirme — araç merkezli, burun yukarı."""
        import cv2

        grid = self._cost_grid
        if grid is None:
            grid = np.zeros((self._grid_n, self._grid_n), dtype=np.uint8)

        # Grid satırı = +x (ileri). Görüntüde ileri yukarı olsun diye satırları ters çevir,
        # +y (sol) sütun artışı olduğu için sütunları da ters çevir.
        view = np.flipud(np.fliplr(grid))
        # [0, 100] maliyeti [0, 255]'e ölçekle — uint8 üzerinde çarpma taşacağı
        # için float üzerinden hesaplanır.
        scaled = np.clip(view.astype(np.float32) * 2.55, 0, 255).astype(np.uint8)
        colored = cv2.applyColorMap(scaled, cv2.COLORMAP_INFERNO)
        colored[view == 0] = (10, 12, 16)

        size = self._render_px
        img = cv2.resize(colored, (size, size), interpolation=cv2.INTER_NEAREST)
        cx = cy = size // 2
        self._draw_vehicle(img, cx, cy)

        cv2.putText(img, f'Cost map | {self._map_size:.0f}x{self._map_size:.0f} m '
                         f'@ {self._map_res} m', (10, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
        cv2.putText(img, stamp, (10, size - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        return img

    @staticmethod
    def _draw_vehicle(img, cx, cy):
        import cv2

        pts = np.array([[cx, cy - 11], [cx - 7, cy + 8], [cx + 7, cy + 8]], dtype=np.int32)
        cv2.fillPoly(img, [pts], (255, 160, 80))

    # ───────────────────── Temizlik ─────────────────────

    def destroy_node(self):
        for writer in (self._lidar_writer, self._costmap_writer):
            if writer is not None:
                writer.release()
        self._lidar_writer = None
        self._costmap_writer = None
        super().destroy_node()


def main(args=None):
    # mp4 dosyasının moov atom'u ancak VideoWriter.release() ile yazılır; SIGTERM
    # ile öldürülürsek kayıt oynatılamaz hale gelir. Bu yüzden SIGTERM'i de
    # KeyboardInterrupt'a çevirip temiz kapanışı garantiye alıyoruz.
    import signal

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    node = LidarLoggerNode()
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
