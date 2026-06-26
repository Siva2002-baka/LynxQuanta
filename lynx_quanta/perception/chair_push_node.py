#!/usr/bin/env python3
"""
chair_push_node.py
==================
Perception pipeline for chair manipulation:
  1. Receive YOLO chair bbox from /yolo/chair_detection
  2. Back-project depth → 3D centroid in camera optical frame
  3. TF-transform centroid to base_link
  4. Compute LEFT and RIGHT contact poses in base_link
     - LEFT contact  (arm side) → used to push chair RIGHT
     - RIGHT contact (arm side) → used to push chair LEFT
  5. Transform contact poses to arm_base_link for MoveIt
  6. On /chair_push/command ("left" | "right"), send the correct
     contact PoseStamped to /arm/target_pose

Coordinate convention reminder
-------------------------------
  command "right"  → push chair RIGHT  → arm contacts LEFT  side (+Y in base_link)
  command "left"   → push chair LEFT   → arm contacts RIGHT side (-Y in base_link)

Topics
------
  Subscribed:
    /yolo/chair_detection        std_msgs/Float32MultiArray  [x1,y1,x2,y2,conf]
    /camera/depth/image_raw      sensor_msgs/Image           16UC1 depth (mm)
    /chair_push/command          std_msgs/String             "left" | "right"

  Published:
    /chair_push/left_contact     geometry_msgs/PoseStamped   contact for pushing right
    /chair_push/right_contact    geometry_msgs/PoseStamped   contact for pushing left
    /arm/target_pose             geometry_msgs/PoseStamped   triggered arm goal
    /chair_push/status           std_msgs/String             feedback
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401 — required for PointStamped transform support
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False


# ── Tunable constants ──────────────────────────────────────────────────────────

# RealSense D435i @ 640×480 default intrinsics — override via ROS params if needed
DEFAULT_FX = 615.0
DEFAULT_FY = 615.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0

DEPTH_MM_TO_M   = 1000.0   # depth image unit: mm → m
DEPTH_MIN_M     = 0.20     # discard readings closer than this
DEPTH_MAX_M     = 5.0      # discard readings farther than this
DEPTH_SUBSAMPLE = 3        # sample every N-th pixel to reduce cost

MIN_VALID_PTS   = 30       # minimum depth points required for a valid detection

CHAIR_HALF_WIDTH_M  = 0.25  # half-width of a typical chair (~0.5 m)
CONTACT_HEIGHT_M    = 0.35  # contact height in base_link (Z up) — targets seat frame
MIN_DETECTION_CONF  = 0.40  # ignore detections below this YOLO confidence


class ChairPushNode(Node):

    def __init__(self):
        super().__init__("chair_push_node")

        # ── Parameters ───────────────────────────────────────────────────────────
        self.declare_parameter("camera_frame",   "camera_color_optical_frame")
        self.declare_parameter("base_frame",     "base_link")
        self.declare_parameter("arm_base_frame", "arm_base_link")
        self.declare_parameter("fx", DEFAULT_FX)
        self.declare_parameter("fy", DEFAULT_FY)
        self.declare_parameter("cx", DEFAULT_CX)
        self.declare_parameter("cy", DEFAULT_CY)
        self.declare_parameter("chair_half_width_m",  CHAIR_HALF_WIDTH_M)
        self.declare_parameter("contact_height_m",    CONTACT_HEIGHT_M)
        self.declare_parameter("min_detection_conf",  MIN_DETECTION_CONF)

        self.camera_frame    = self.get_parameter("camera_frame").value
        self.base_frame      = self.get_parameter("base_frame").value
        self.arm_base_frame  = self.get_parameter("arm_base_frame").value
        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.chair_half_width = float(self.get_parameter("chair_half_width_m").value)
        self.contact_height   = float(self.get_parameter("contact_height_m").value)
        self.min_conf         = float(self.get_parameter("min_detection_conf").value)

        # ── State ────────────────────────────────────────────────────────────────
        self.bridge              = CvBridge()
        self._latest_depth       = None
        self._latest_detection   = None   # [x1, y1, x2, y2, conf]
        self._lock               = threading.Lock()

        # ── TF2 ──────────────────────────────────────────────────────────────────
        if HAS_TF2:
            self._tf_buffer   = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self.get_logger().error("tf2_ros not found — frame transforms disabled")

        # ── Subscribers ──────────────────────────────────────────────────────────
        self.create_subscription(
            Float32MultiArray, "/yolo/chair_detection", self._detection_cb, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self._depth_cb, 10
        )
        self.create_subscription(
            String, "/chair_push/command", self._command_cb, 10
        )

        # ── Publishers ───────────────────────────────────────────────────────────
        self._pub_left_contact  = self.create_publisher(
            PoseStamped, "/chair_push/left_contact", 10
        )
        self._pub_right_contact = self.create_publisher(
            PoseStamped, "/chair_push/right_contact", 10
        )
        self._pub_arm_target    = self.create_publisher(
            PoseStamped, "/arm/target_pose", 10
        )
        self._pub_status        = self.create_publisher(
            String, "/chair_push/status", 10
        )

        # Continuously refresh contact poses at 5 Hz when a chair is visible
        self.create_timer(0.2, self._perception_tick)

        self.get_logger().info(
            "ChairPushNode ready.\n"
            "  Publish to /chair_push/command: 'right' or 'left'\n"
            "    'right' → arm goes to LEFT  contact point to push chair RIGHT\n"
            "    'left'  → arm goes to RIGHT contact point to push chair LEFT\n"
            f"  camera_frame={self.camera_frame}\n"
            f"  base_frame={self.base_frame}\n"
            f"  arm_base_frame={self.arm_base_frame}"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────────

    def _depth_cb(self, msg: Image):
        with self._lock:
            self._latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def _detection_cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            with self._lock:
                self._latest_detection = list(msg.data[:5])

    def _command_cb(self, msg: String):
        direction = msg.data.strip().lower()
        if direction not in ("left", "right"):
            self.get_logger().warn(
                f"Unknown direction '{direction}' — use 'left' or 'right'"
            )
            return

        self.get_logger().info(f"Command received: '{direction}'")

        with self._lock:
            contact = self._compute_contact_pose(direction)

        if contact is None:
            self.get_logger().warn(
                "No valid chair detected — cannot compute contact pose"
            )
            self._publish_status("NO_CHAIR_DETECTED")
            return

        self._pub_arm_target.publish(contact)
        self._publish_status(f"ARM_MOVING_{direction.upper()}")
        self.get_logger().info(
            f"Arm target sent: direction={direction}  "
            f"xyz=({contact.pose.position.x:.3f}, "
            f"{contact.pose.position.y:.3f}, "
            f"{contact.pose.position.z:.3f})  "
            f"frame={contact.header.frame_id}"
        )

    # ── Perception tick ───────────────────────────────────────────────────────────

    def _perception_tick(self):
        """Continuously publish left/right contact poses when chair is visible."""
        with self._lock:
            left  = self._compute_contact_pose("right")  # push RIGHT → LEFT contact
            right = self._compute_contact_pose("left")   # push LEFT  → RIGHT contact

        if left is not None:
            self._pub_left_contact.publish(left)
        if right is not None:
            self._pub_right_contact.publish(right)

    # ── Core perception ───────────────────────────────────────────────────────────

    def _compute_contact_pose(self, push_direction: str) -> PoseStamped | None:
        """
        Compute the arm contact PoseStamped in arm_base_link for the given push direction.

        push_direction = "right"  → contact LEFT  side of chair (+Y in base_link)
        push_direction = "left"   → contact RIGHT side of chair (-Y in base_link)

        Returns None if no valid detection or TF is unavailable.
        """
        if self._latest_depth is None or self._latest_detection is None:
            return None

        x1, y1, x2, y2, conf = self._latest_detection
        if conf < self.min_conf:
            return None

        # ── Step 1: depth bbox → 3D centroid in camera optical frame ─────────────
        centroid_cam = self._bbox_to_centroid_camera(
            self._latest_depth, int(x1), int(y1), int(x2), int(y2)
        )
        if centroid_cam is None:
            return None

        # ── Step 2: camera frame → base_link ─────────────────────────────────────
        centroid_base = self._transform_point(
            centroid_cam, self.camera_frame, self.base_frame
        )
        if centroid_base is None:
            return None

        cx_b, cy_b, _ = centroid_base

        # ── Step 3: contact side offset ───────────────────────────────────────────
        # push RIGHT → touch LEFT side of chair → offset in +Y (base_link)
        # push LEFT  → touch RIGHT side of chair → offset in -Y (base_link)
        if push_direction == "right":
            contact_y = cy_b + self.chair_half_width
        else:
            contact_y = cy_b - self.chair_half_width

        # ── Step 4: build contact PointStamped in base_link ───────────────────────
        contact_bl = PointStamped()
        contact_bl.header.frame_id = self.base_frame
        contact_bl.header.stamp    = self.get_clock().now().to_msg()
        contact_bl.point.x = float(cx_b)
        contact_bl.point.y = float(contact_y)
        contact_bl.point.z = float(self.contact_height)

        # ── Step 5: base_link → arm_base_link ────────────────────────────────────
        if not HAS_TF2:
            return None

        try:
            contact_arm = self._tf_buffer.transform(
                contact_bl, self.arm_base_frame, timeout=Duration(seconds=0.5)
            )
        except Exception as e:
            self.get_logger().warn(
                f"TF {self.base_frame}→{self.arm_base_frame} failed: {e}"
            )
            return None

        ax = contact_arm.point.x
        ay = contact_arm.point.y
        az = contact_arm.point.z

        # ── Step 6: build PoseStamped ─────────────────────────────────────────────
        # Orientation: end-effector approaches the chair side perpendicularly.
        # push RIGHT → arm approaches from LEFT (+Y base_link) pointing RIGHT (-Y = yaw -90°)
        # push LEFT  → arm approaches from RIGHT (-Y base_link) pointing LEFT (+Y = yaw +90°)
        yaw = -math.pi / 2.0 if push_direction == "right" else math.pi / 2.0
        qx, qy, qz, qw = _rpy_to_quat(0.0, 0.0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.arm_base_frame
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = float(ax)
        pose.pose.position.y = float(ay)
        pose.pose.position.z = float(az)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def _bbox_to_centroid_camera(
        self, depth: np.ndarray, x1: int, y1: int, x2: int, y2: int
    ) -> tuple[float, float, float] | None:
        """
        Back-project valid depth pixels inside [x1,y1,x2,y2] to 3D points
        in the camera optical frame and return their centroid.
        Returns (X, Y, Z) in metres, or None if too few valid points.
        """
        h, w = depth.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w - 1, x2), min(h - 1, y2)

        points = []
        for v in range(y1, y2, DEPTH_SUBSAMPLE):
            for u in range(x1, x2, DEPTH_SUBSAMPLE):
                z = depth[v, u] / DEPTH_MM_TO_M
                if z < DEPTH_MIN_M or z > DEPTH_MAX_M:
                    continue
                xc = (u - self.cx) * z / self.fx
                yc = (v - self.cy) * z / self.fy
                points.append((xc, yc, z))

        if len(points) < MIN_VALID_PTS:
            self.get_logger().debug(
                f"Insufficient depth points in chair bbox: {len(points)}"
            )
            return None

        pts = np.array(points)
        centroid = pts.mean(axis=0)
        return float(centroid[0]), float(centroid[1]), float(centroid[2])

    def _transform_point(
        self, xyz: tuple[float, float, float], src_frame: str, dst_frame: str
    ) -> tuple[float, float, float] | None:
        """Transform an XYZ point from src_frame to dst_frame via TF2."""
        if not HAS_TF2:
            return None

        pt = PointStamped()
        pt.header.frame_id = src_frame
        pt.header.stamp    = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y, pt.point.z = float(xyz[0]), float(xyz[1]), float(xyz[2])

        try:
            pt_out = self._tf_buffer.transform(
                pt, dst_frame, timeout=Duration(seconds=0.5)
            )
            return (pt_out.point.x, pt_out.point.y, pt_out.point.z)
        except Exception as e:
            self.get_logger().warn(f"TF {src_frame}→{dst_frame} failed: {e}")
            return None

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._pub_status.publish(msg)


# ── Quaternion helper (no external deps) ──────────────────────────────────────

def _rpy_to_quat(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll  / 2), math.sin(roll  / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw   / 2), math.sin(yaw   / 2)
    return (
        sr * cp * cy - cr * sp * sy,   # qx
        cr * sp * cy + sr * cp * sy,   # qy
        cr * cp * sy - sr * sp * cy,   # qz
        cr * cp * cy + sr * sp * sy,   # qw
    )


def main():
    rclpy.init()
    node = ChairPushNode()
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
