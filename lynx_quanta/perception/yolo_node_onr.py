#!/usr/bin/env python3
"""
yolo_node_onr.py  ("onr" = on-robot)
=====================================
On-robot version of the YOLO perception node.
Subscribes to ROS Image topics published by the robot's cameras instead
of pulling RTSP streams directly.

Differences from yolo_node.py
------------------------------
  - Input:  sensor_msgs/Image subscribers  (not OpenCV VideoCapture + RTSP)
  - No background capture threads — the ROS executor drives inference
  - Topic names are ROS parameters → change them at launch without editing code

Topic placeholders  (override via ROS params or launch file)
------------------------------------------------------------
  ~cam1_topic   /camera1/color/image_raw        front wide-angle
  ~cam2_topic   /camera2/color/image_raw        rear  wide-angle
  ~cam3_topic   /piper_cam/color/image_raw      piper arm camera

  Replace the defaults above with your robot's actual camera topic names.

Published
---------
  /yolo/debug/camera1            sensor_msgs/Image   annotated front frame
  /yolo/debug/camera2            sensor_msgs/Image   annotated rear  frame
  /yolo/debug/piper_arm          sensor_msgs/Image   annotated arm  frame
  /yolo/chair_detection          std_msgs/Float32MultiArray
                                   [x1, y1, x2, y2, confidence]
                                   Best-confidence chair from front camera only.
                                   Not published when no chair is visible.
"""

import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from ultralytics import YOLO


# ── Model search ──────────────────────────────────────────────────────────────

def _find_model(name: str) -> str:
    candidates = [
        Path(__file__).resolve().parents[2] / name,
        Path.home() / "lynx_ws" / "src" / "lynx_quanta" / name,
        Path.home() / name,
    ]
    for p in candidates:
        if p.exists():
            return str(p)
    return name  # fall back to ultralytics auto-download


MODEL_PATH = _find_model("yolo26l.pt")

# ── Placeholder defaults — replace with real camera topic names ───────────────
#
#   Typical RealSense D435i:   /camera/color/image_raw
#   Typical USB webcam (v4l2): /usb_cam/image_raw
#   Typical GStreamer bridge:  /gscam/image_raw
#
#   Check your robot with:  ros2 topic list | grep image
#
DEFAULT_CAM1_TOPIC = "/camera1/color/image_raw"      # PLACEHOLDER — front wide-angle
DEFAULT_CAM2_TOPIC = "/camera2/color/image_raw"      # PLACEHOLDER — rear  wide-angle
DEFAULT_CAM3_TOPIC = "/piper_cam/color/image_raw"    # PLACEHOLDER — piper arm camera

DEBUG_TOPICS = [
    "/yolo/debug/camera1",
    "/yolo/debug/camera2",
    "/yolo/debug/piper_arm",
]


class YOLONodeOnRobot(Node):

    def __init__(self):
        super().__init__("yolo_node")

        # ── Parameters ───────────────────────────────────────────────────────────
        self.declare_parameter("cam1_topic", DEFAULT_CAM1_TOPIC)
        self.declare_parameter("cam2_topic", DEFAULT_CAM2_TOPIC)
        self.declare_parameter("cam3_topic", DEFAULT_CAM3_TOPIC)
        self.declare_parameter("detection_conf_threshold", 0.40)

        cam_topics = [
            self.get_parameter("cam1_topic").value,
            self.get_parameter("cam2_topic").value,
            self.get_parameter("cam3_topic").value,
        ]
        self.conf_threshold = float(
            self.get_parameter("detection_conf_threshold").value
        )

        # ── Model ────────────────────────────────────────────────────────────────
        self.get_logger().info(f"Loading YOLO model from: {MODEL_PATH}")
        self.model = YOLO(MODEL_PATH)
        self._model_lock = threading.Lock()  # guards concurrent callback calls
        self.bridge = CvBridge()

        # ── Publishers ───────────────────────────────────────────────────────────
        self.pubs_debug = [
            self.create_publisher(Image, topic, 10) for topic in DEBUG_TOPICS
        ]
        self.pub_chair_detection = self.create_publisher(
            Float32MultiArray, "/yolo/chair_detection", 10
        )

        # ── Subscribers ──────────────────────────────────────────────────────────
        # Camera 1 — front (also used for chair detection)
        self.create_subscription(
            Image, cam_topics[0], self._cb_cam1, 5
        )
        # Camera 2 — rear
        self.create_subscription(
            Image, cam_topics[1], self._cb_cam2, 5
        )
        # Camera 3 — piper arm
        self.create_subscription(
            Image, cam_topics[2], self._cb_cam3, 5
        )

        self.get_logger().info(
            "YOLONodeOnRobot ready.\n"
            f"  cam1 (front):     {cam_topics[0]}\n"
            f"  cam2 (rear):      {cam_topics[1]}\n"
            f"  cam3 (piper arm): {cam_topics[2]}\n"
            f"  detection_conf_threshold: {self.conf_threshold}"
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────────

    def _cb_cam1(self, msg: Image):
        """Front camera — runs YOLO + publishes debug image + chair detection."""
        results = self._infer(msg)
        if results is None:
            return
        self._publish_debug(results, msg, cam_idx=0)
        self._publish_chair_detection(results)

    def _cb_cam2(self, msg: Image):
        """Rear camera — runs YOLO + publishes debug image."""
        results = self._infer(msg)
        if results is None:
            return
        self._publish_debug(results, msg, cam_idx=1)

    def _cb_cam3(self, msg: Image):
        """Piper arm camera — runs YOLO + publishes debug image."""
        results = self._infer(msg)
        if results is None:
            return
        self._publish_debug(results, msg, cam_idx=2)

    # ── Inference ─────────────────────────────────────────────────────────────────

    def _infer(self, msg: Image):
        """Convert ROS Image → cv2 frame → YOLO result. Returns None on error."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return None

        with self._model_lock:
            try:
                return self.model(frame, verbose=False)[0]
            except Exception as e:
                self.get_logger().error(f"YOLO inference failed: {e}")
                return None

    # ── Debug image ───────────────────────────────────────────────────────────────

    def _publish_debug(self, results, original_msg: Image, cam_idx: int):
        annotated = results.plot()
        try:
            out = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge annotated conversion failed: {e}")
            return
        out.header = original_msg.header   # preserve timestamp and frame_id
        out.header.frame_id = f"camera{cam_idx + 1}"
        self.pubs_debug[cam_idx].publish(out)

    # ── Chair detection ───────────────────────────────────────────────────────────

    def _publish_chair_detection(self, results):
        """
        Scan all detections, pick the highest-confidence 'chair' box,
        and publish [x1, y1, x2, y2, confidence] to /yolo/chair_detection.
        Does nothing if no chair is found above the confidence threshold.
        """
        best_conf = 0.0
        best_box = None

        for box in results.boxes:
            cls_name = results.names[int(box.cls[0])].lower()
            conf = float(box.conf[0])
            if cls_name == "chair" and conf > best_conf:
                best_conf = conf
                best_box = box

        if best_box is None or best_conf < self.conf_threshold:
            return

        x1, y1, x2, y2 = best_box.xyxy[0].tolist()
        det = Float32MultiArray()
        det.data = [float(x1), float(y1), float(x2), float(y2), best_conf]
        self.pub_chair_detection.publish(det)

        self.get_logger().debug(
            f"Chair detection: conf={best_conf:.2f}  "
            f"bbox=[{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]"
        )


def main():
    rclpy.init()
    node = YOLONodeOnRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
