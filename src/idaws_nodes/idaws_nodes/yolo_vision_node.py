"""
yolo_vision_node
────────────────
YOLOv8 ile duba tespiti. use_yolo=false ise model yüklenmez, boş BuoyArray
publish ederek sistemin kamerasız test edilmesini sağlar.
true ise: tespit + BuoyArray publish + çizimli görüntü yayını + 1 Hz mp4 kayıt.

Kamera kaynağı (video_source parametresi):
  * "ros"       → sensor_msgs/Image topic'i (Gazebo/SITL köprüsü veya köprülenmiş
                  gerçek kamera). Kamera bu node'da açılmaz, topic'ten okunur.
  * "device"    → cv2.VideoCapture(camera_index)
  * "gstreamer" → udpsrc port=gstreamer_port ! RTP/H264

Çizim yapılmış kareler `annotated_topic` üzerinden yayınlanır; web arayüzü bu
akışı gösterir, böylece kamera tek bir yerden okunur.
"""

import os
import time
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image

from idaws_msgs.msg import Buoy, BuoyArray


class YoloVisionNode(Node):

    def __init__(self):
        super().__init__('yolo_vision_node')

        # ---------- Parametreler ----------
        self.declare_parameter('use_yolo', False)
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('confidence_threshold', 0.45)
        self.declare_parameter('record_dir', '/home/jetson/idaws_recordings')
        self.declare_parameter('input_width', 640)
        self.declare_parameter('input_height', 480)
        # video_source: 'ros' -> sensor_msgs/Image topic
        #               'device' -> cv2.VideoCapture(camera_index)
        #               'gstreamer' -> UDP H264 pipeline (RTP)
        self.declare_parameter('video_source', 'ros')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', 'vision/image_annotated')
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('gstreamer_port', 5400)

        self._use_yolo = self.get_parameter('use_yolo').value
        self._model_path = self.get_parameter('model_path').value
        self._camera_idx = self.get_parameter('camera_index').value
        self._conf_thresh = self.get_parameter('confidence_threshold').value
        self._record_dir = self.get_parameter('record_dir').value
        self._width = self.get_parameter('input_width').value
        self._height = self.get_parameter('input_height').value
        self._video_source = self.get_parameter('video_source').value
        self._camera_topic = self.get_parameter('camera_topic').value
        self._annotated_topic = self.get_parameter('annotated_topic').value
        self._publish_annotated = self.get_parameter('publish_annotated').value
        self._gst_port = self.get_parameter('gstreamer_port').value

        # ---------- Publisher'lar ----------
        self.pub_buoys = self.create_publisher(BuoyArray, 'vision/buoys', 10)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_annotated = self.create_publisher(Image, self._annotated_topic, sensor_qos)

        # ---------- Kamera & Model ----------
        self._cap = None
        self._model = None
        self._video_writer = None
        self._ros_frame = None       # video_source='ros' — son gelen kare (BGR)
        self._last_frame = None      # kayıt için son çizimli kare

        if self._video_source == 'ros':
            self.create_subscription(Image, self._camera_topic, self._cb_image, sensor_qos)
            self.get_logger().info(f'Kamera kaynağı: ROS topic — {self._camera_topic}')

        if self._use_yolo:
            self._init_camera_and_model()
        else:
            self.get_logger().info('use_yolo=false — bypass modunda çalışılıyor.')

        # ---------- Parametre değişikliği callback ----------
        self.add_on_set_parameters_callback(self._on_param_change)

        # ---------- Timer: ~10 Hz algılama ----------
        self._detect_timer = self.create_timer(0.1, self._detect_callback)

        # ---------- Timer: 1 Hz kayıt ----------
        self._record_timer = self.create_timer(1.0, self._record_callback)

        self.get_logger().info('yolo_vision_node başlatıldı.')

    # ───────────────────── Kamera & Model init ─────────────────────

    def _init_camera_and_model(self):
        import cv2
        from ultralytics import YOLO

        if self._video_source == 'gstreamer':
            pipeline = (
                f'udpsrc port={self._gst_port} ! '
                'application/x-rtp, payload=96 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! '
                'videoconvert ! appsink drop=true sync=false'
            )
            self.get_logger().info(f'Kamera açılıyor (GStreamer, port={self._gst_port})...')
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        elif self._video_source == 'device':
            self.get_logger().info(f'Kamera açılıyor (index={self._camera_idx})...')
            self._cap = cv2.VideoCapture(self._camera_idx)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        if self._cap is not None and not self._cap.isOpened():
            self.get_logger().error('Kamera açılamadı!')
            self._use_yolo = False
            return

        self.get_logger().info(f'YOLO modeli yükleniyor: {self._model_path}')
        self._model = YOLO(self._model_path)

        os.makedirs(self._record_dir, exist_ok=True)

    def _init_writer(self, frame):
        """Kayıt dosyasını ilk karenin gerçek çözünürlüğüyle açar."""
        import cv2

        os.makedirs(self._record_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        video_path = os.path.join(self._record_dir, f'detect_{ts}.mp4')
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        h, w = frame.shape[:2]
        self._video_writer = cv2.VideoWriter(video_path, fourcc, 1.0, (w, h))
        self.get_logger().info(f'Video kayıt: {video_path} ({w}x{h} @ 1 Hz)')

    # ───────────────────── ROS kamera callback ─────────────────────

    def _cb_image(self, msg: Image):
        frame = self._imgmsg_to_bgr(msg)
        if frame is not None:
            self._ros_frame = frame

    @staticmethod
    def _imgmsg_to_bgr(msg: Image):
        """sensor_msgs/Image → BGR numpy dizisi (cv_bridge bağımlılığı olmadan)."""
        import cv2

        enc = msg.encoding.lower()
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(enc)
        if channels is None:
            return None

        buf = np.frombuffer(msg.data, dtype=np.uint8)
        expected = msg.height * msg.step
        if buf.size < expected:
            return None
        img = buf[:expected].reshape(msg.height, msg.step)
        img = img[:, : msg.width * channels].reshape(msg.height, msg.width, channels)

        if enc == 'rgb8':
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if enc == 'rgba8':
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if enc == 'bgra8':
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if enc == 'mono8':
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    def _bgr_to_imgmsg(self, frame) -> Image:
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = np.ascontiguousarray(frame).tobytes()
        return msg

    def _read_frame(self):
        """Aktif kaynaktan bir kare döndürür (yoksa None)."""
        if self._video_source == 'ros':
            return self._ros_frame
        if self._cap is None:
            return None
        ret, frame = self._cap.read()
        return frame if ret else None

    # ───────────────────── Parametre Değişikliği ─────────────────────

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'use_yolo':
                if p.value and not self._use_yolo:
                    self.get_logger().info('use_yolo → true, model başlatılıyor...')
                    self._use_yolo = True
                    self._init_camera_and_model()
                elif not p.value and self._use_yolo:
                    self.get_logger().info('use_yolo → false, bypass moduna geçiliyor.')
                    self._release_camera()
                    self._use_yolo = False
        return SetParametersResult(successful=True)

    # ───────────────────── Algılama Döngüsü ─────────────────────

    def _detect_callback(self):
        msg = BuoyArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        if not self._use_yolo or self._model is None:
            msg.frame_width = 0
            msg.frame_height = 0
            self.pub_buoys.publish(msg)
            return

        import cv2

        frame = self._read_frame()
        if frame is None:
            self.pub_buoys.publish(msg)
            return

        frame = frame.copy()
        results = self._model(frame, conf=self._conf_thresh, verbose=False)

        msg.frame_width = frame.shape[1]
        msg.frame_height = frame.shape[0]

        for r in results:
            for box in r.boxes:
                b = Buoy()
                coords = box.xyxy[0].cpu().numpy()
                b.x_min = int(coords[0])
                b.y_min = int(coords[1])
                b.x_max = int(coords[2])
                b.y_max = int(coords[3])
                b.center_x = (b.x_min + b.x_max) / 2.0
                b.center_y = (b.y_min + b.y_max) / 2.0
                b.confidence = float(box.conf[0])
                cls_id = int(box.cls[0])
                b.label = r.names.get(cls_id, str(cls_id))
                msg.buoys.append(b)

                # Görselleştirme
                cv2.rectangle(frame, (b.x_min, b.y_min), (b.x_max, b.y_max), (0, 255, 0), 2)
                cv2.putText(
                    frame, f'{b.label} {b.confidence:.2f}',
                    (b.x_min, b.y_min - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                )

        # Şartname: kaydedilen her karede zaman etiketi görünür olacak.
        stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        cv2.putText(frame, stamp, (8, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        self._last_frame = frame
        self.pub_buoys.publish(msg)

        if self._publish_annotated:
            self.pub_annotated.publish(self._bgr_to_imgmsg(frame))

    # ───────────────────── 1 Hz Video Kayıt ─────────────────────

    def _record_callback(self):
        if not self._use_yolo or self._last_frame is None:
            return
        if self._video_writer is None:
            self._init_writer(self._last_frame)
        self._video_writer.write(self._last_frame)

    # ───────────────────── Temizlik ─────────────────────

    def _release_camera(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
        self._model = None

    def destroy_node(self):
        self._release_camera()
        super().destroy_node()


def main(args=None):
    # mp4 kaydının bozulmaması için SIGTERM'de de temiz kapanış (bkz. lidar_logger_node).
    import signal

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt()

    signal.signal(signal.SIGTERM, _on_sigterm)

    rclpy.init(args=args)
    node = YoloVisionNode()
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
