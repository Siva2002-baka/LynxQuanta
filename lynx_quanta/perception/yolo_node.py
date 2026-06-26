import os
import threading
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO

# Force RTSP over TCP to avoid packet-loss-induced decode errors
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"

# Search for model in source tree and home directory
def _find_model(name: str) -> str:
    candidates = [
        Path(__file__).resolve().parents[2] / name,   # source package root
        Path.home() / "lynx_ws" / "src" / "lynx_quanta" / name,
        Path.home() / name,
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return name  # fall back to ultralytics auto-download

MODEL_PATH = _find_model("yolo26l.pt")


RTSP_URLS = [
    "rtsp://10.21.31.103:8554/video1",   # front wide-angle
    "rtsp://10.21.31.103:8554/video2",   # rear wide-angle
    "rtsp://10.21.31.60:8554/stream",    # piper arm
]

DEBUG_TOPICS = [
    "/yolo/debug/camera1",
    "/yolo/debug/camera2",
    "/yolo/debug/piper_arm",
]


class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")

        self.get_logger().info(f"Loading YOLO model from: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        self._model_lock = threading.Lock()
        self.bridge = CvBridge()

        self.pubs_debug = [
            self.create_publisher(Image, topic, 10) for topic in DEBUG_TOPICS
        ]

        # Chair detection from front camera (camera1) only.
        # Message layout: [x1, y1, x2, y2, confidence] of the best-confidence chair.
        self.pub_chair_detection = self.create_publisher(
            Float32MultiArray, "/yolo/chair_detection", 10
        )

        self.caps = []
        self.threads = []
        for i, url in enumerate(RTSP_URLS):
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.get_logger().error(f"Cannot open RTSP stream: {url}")
            self.caps.append(cap)
            t = threading.Thread(target=self._capture_loop, args=(i,), daemon=True)
            t.start()
            self.threads.append(t)

        self.get_logger().info("YOLO node started — watching 3 RTSP streams")

    def _capture_loop(self, cam_idx):
        cap = self.caps[cam_idx]
        url = RTSP_URLS[cam_idx]
        self.get_logger().info(f"Camera {cam_idx + 1}: capture thread started")
        while rclpy.ok():
            try:
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warn(f"Camera {cam_idx + 1}: read failed, reconnecting…")
                    cap.release()
                    time.sleep(2.0)
                    cap.open(url)
                    continue

                self.get_logger().debug(f"Camera {cam_idx + 1}: frame {frame.shape}")

                with self._model_lock:
                    results = self.model(frame, verbose=False)[0]
                annotated = results.plot()

                if not rclpy.ok():
                    break
                msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"camera{cam_idx + 1}"
                self.pubs_debug[cam_idx].publish(msg)

                if cam_idx == 0:
                    self._publish_chair_detection(results)

                self.get_logger().debug(f"Camera {cam_idx + 1}: published")
            except Exception as e:
                self.get_logger().error(f"Camera {cam_idx + 1}: exception — {type(e).__name__}: {e}")
        self.get_logger().warn(f"Camera {cam_idx + 1}: capture thread exiting (rclpy not ok)")

    def _publish_chair_detection(self, results):
        """Publish the highest-confidence chair bbox from camera1."""
        best_conf = 0.0
        best_box = None
        for box in results.boxes:
            cls_name = results.names[int(box.cls[0])].lower()
            conf = float(box.conf[0])
            if cls_name == "chair" and conf > best_conf:
                best_conf = conf
                best_box = box

        if best_box is None:
            return

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        det = Float32MultiArray()
        det.data = [float(x1), float(y1), float(x2), float(y2), best_conf]
        self.pub_chair_detection.publish(det)
        self.get_logger().debug(
            f"Chair detection: conf={best_conf:.2f}  bbox=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]"
        )

    def destroy_node(self):
        for cap in self.caps:
            cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
