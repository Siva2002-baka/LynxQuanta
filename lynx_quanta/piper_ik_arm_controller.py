#!/usr/bin/env python3
"""
piper_ik_arm_controller.py
==========================
ROS 2 Jazzy — Inverse Kinematics + Arm Control Node
AgileX Piper 6-DOF arm mounted on M20 Lynx dog robot

Subscribed topics:
  /arm/target_pose   geometry_msgs/PoseStamped
  /arm/gripper_cmd   std_msgs/Float32
  /arm/named_pose    std_msgs/String

Action clients:
  arm_controller/follow_joint_trajectory
  gripper_controller/follow_joint_trajectory
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration as RosDuration

try:
    import tf2_ros
    import tf2_geometry_msgs  # noqa: F401
    HAS_TF2 = True
except ImportError:
    HAS_TF2 = False


# ─────────────────────────────────────────────────────────────────────────────
# Robot constants
# ─────────────────────────────────────────────────────────────────────────────

DH_PARAMS = np.array([
    [0.123,    0.0,      0.0,           0.0],
    [0.0,      0.0,     -math.pi / 2,  -172.22 * math.pi / 180.0],
    [0.0,      0.28503,  0.0,          -102.78 * math.pi / 180.0],
    [0.25075, -0.02198,  math.pi / 2,   0.0],
    [0.0,      0.0,     -math.pi / 2,   0.0],
    [0.009,    0.0,      math.pi / 2,   0.0],
], dtype=float)

JOINT_LIMITS = np.array([
    [-154.0,  154.0],
    [   0.0,  195.0],
    [-175.0,    0.0],
    [-100.0,  112.0],
    [ -75.0,   75.0],
    [-170.0,  170.0],
], dtype=float) * math.pi / 180.0

ARM_JOINT_NAMES = [
    "arm_joint1", "arm_joint2", "arm_joint3",
    "arm_joint4", "arm_joint5", "arm_joint6",
]

GRIPPER_JOINT_NAMES = ["arm_joint7", "arm_joint8"]

GRIPPER_OPEN = [0.035, 0.035]
GRIPPER_CLOSE = [0.000, 0.000]

NAMED_POSES = {
    "home":  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, math.pi / 4, -math.pi / 4, 0.0, math.pi / 4, 0.0],
    "stow":  [0.0, math.pi / 2, -math.pi * 2 / 3, 0.0, math.pi / 6, 0.0],
}

# IK tuning
IK_MAX_ITER = 700
IK_LAMBDA = 0.05
IK_ALPHA = 0.45
IK_W_POS = 1.0
IK_W_ORI = 0.35
IK_POS_TOL = 1e-4
IK_ORI_TOL = 1e-3
IK_MAX_RESTARTS = 18

# Strict acceptance after solving
STRICT_POS_ACCEPT_M = 0.006       # 6 mm
STRICT_ORI_ACCEPT_RAD = 0.25      # ~14 deg
STRICT_WRIST6_DELTA_RAD = 1.20    # ~69 deg from current pose

# Motion duration
DEFAULT_DURATION_S = 6.0

# Gripper hold
DEFAULT_GRIPPER_HOLD_ENABLED = False
DEFAULT_GRIPPER_HOLD_PERIOD_S = 1.0


# ─────────────────────────────────────────────────────────────────────────────
# Kinematics
# ─────────────────────────────────────────────────────────────────────────────

def _dh(d: float, a: float, alpha: float, theta: float) -> np.ndarray:
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(alpha), math.sin(alpha)
    return np.array([
        [ct,     -st,     0.0,   a],
        [st * ca, ct * ca, -sa,  -sa * d],
        [st * sa, ct * sa,  ca,   ca * d],
        [0.0,     0.0,     0.0,  1.0],
    ], dtype=float)


def forward_kinematics(q: np.ndarray) -> tuple[np.ndarray, list[np.ndarray]]:
    T = np.eye(4)
    T_all: list[np.ndarray] = []
    for i in range(6):
        d, a, al, off = DH_PARAMS[i]
        T = T @ _dh(d, a, al, q[i] + off)
        T_all.append(T.copy())
    return T, T_all


def geometric_jacobian(T_all: list[np.ndarray]) -> np.ndarray:
    p_e = T_all[-1][:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = T_all[i][:3, 2]
        o = T_all[i][:3, 3]
        J[:3, i] = np.cross(z, p_e - o)
        J[3:, i] = z
    return J


def _axis_angle_err(R_cur: np.ndarray, R_tgt: np.ndarray) -> np.ndarray:
    dR = R_tgt @ R_cur.T
    tr = max(-1.0, min(1.0, (np.trace(dR) - 1.0) / 2.0))
    angle = math.acos(tr)

    if abs(angle) < 1e-9:
        return np.zeros(3)

    s = math.sin(angle)
    if abs(s) < 1e-9:
        return np.zeros(3)

    ax = np.array([
        dR[2, 1] - dR[1, 2],
        dR[0, 2] - dR[2, 0],
        dR[1, 0] - dR[0, 1],
    ])

    return ax / (2.0 * s) * angle


def pose_error(T_cur: np.ndarray, T_tgt: np.ndarray) -> np.ndarray:
    dp = T_tgt[:3, 3] - T_cur[:3, 3]
    dw = _axis_angle_err(T_cur[:3, :3], T_tgt[:3, :3])
    return np.concatenate([dp, dw])


def _clamp(q: np.ndarray) -> np.ndarray:
    return np.clip(q, JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1])


def _wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _joint_delta(q: np.ndarray, q_ref: np.ndarray) -> np.ndarray:
    return np.array([_wrap_to_pi(q[i] - q_ref[i]) for i in range(6)], dtype=float)


# ─────────────────────────────────────────────────────────────────────────────
# IK solvers
# ─────────────────────────────────────────────────────────────────────────────

def _ik_dls(
    T_tgt: np.ndarray,
    q_init: np.ndarray,
    lam: float = IK_LAMBDA,
    alpha: float = IK_ALPHA,
    max_iter: int = IK_MAX_ITER,
    w_pos: float = IK_W_POS,
    w_ori: float = IK_W_ORI,
) -> tuple[np.ndarray | None, float]:
    W = np.diag([w_pos, w_pos, w_pos, w_ori, w_ori, w_ori])
    q = _clamp(q_init.copy())

    best_q = q.copy()
    best_err = float("inf")

    for _ in range(max_iter):
        T_cur, T_all = forward_kinematics(q)
        e = pose_error(T_cur, T_tgt)
        err = float(np.linalg.norm(e))

        if err < best_err:
            best_err = err
            best_q = q.copy()

        if np.linalg.norm(e[:3]) < IK_POS_TOL and np.linalg.norm(e[3:]) < IK_ORI_TOL:
            return q, err

        J = geometric_jacobian(T_all)
        Jw = W @ J
        A = Jw @ Jw.T + (lam ** 2) * np.eye(6)

        try:
            dq = alpha * Jw.T @ np.linalg.solve(A, W @ e)
        except np.linalg.LinAlgError:
            return None, best_err

        if not np.all(np.isfinite(dq)):
            return None, best_err

        q = _clamp(q + dq)

    T_best, _ = forward_kinematics(best_q)
    return None, float(np.linalg.norm(pose_error(T_best, T_tgt)))


def _ik_pos_only(
    pos: np.ndarray,
    q_init: np.ndarray,
    lam: float = IK_LAMBDA,
    alpha: float = IK_ALPHA,
    max_iter: int = 350,
) -> np.ndarray | None:
    q = _clamp(q_init.copy())

    for _ in range(max_iter):
        T_cur, T_all = forward_kinematics(q)
        e = pos - T_cur[:3, 3]

        if np.linalg.norm(e) < IK_POS_TOL:
            return q

        Jv = geometric_jacobian(T_all)[:3]
        A = Jv @ Jv.T + (lam ** 2) * np.eye(3)

        try:
            dq = alpha * Jv.T @ np.linalg.solve(A, e)
        except np.linalg.LinAlgError:
            return None

        if not np.all(np.isfinite(dq)):
            return None

        q = _clamp(q + dq)

    return None


def solve_ik(
    T_tgt: np.ndarray,
    q_seed: np.ndarray | None = None,
    max_restarts: int = IK_MAX_RESTARTS,
) -> tuple[np.ndarray | None, float]:
    """
    Strict IK solver.

    It only returns a valid full-6D solution.
    It prefers solutions close to the current arm posture.
    It rejects large wrist-6 spin because that is dangerous when carrying objects.
    """
    rng = np.random.default_rng(42)

    if q_seed is None:
        q_ref = (JOINT_LIMITS[:, 0] + JOINT_LIMITS[:, 1]) / 2.0
    else:
        q_ref = _clamp(q_seed.copy())

    seeds: list[np.ndarray] = []

    # Current posture first.
    seeds.append(q_ref)

    # Small local variations around current posture.
    for sigma in [0.08, 0.15, 0.25]:
        for _ in range(6):
            seeds.append(_clamp(q_ref + rng.normal(0.0, sigma, size=6)))

    # Mid-range fallback.
    seeds.append((JOINT_LIMITS[:, 0] + JOINT_LIMITS[:, 1]) / 2.0)

    # Random fallback.
    for _ in range(max_restarts):
        seeds.append(rng.uniform(JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]))

    best_q: np.ndarray | None = None
    best_err = float("inf")
    best_cost = float("inf")

    for seed in seeds:
        q_warm = _ik_pos_only(T_tgt[:3, 3], seed)
        q_start = q_warm if q_warm is not None else seed

        q_sol, err = _ik_dls(T_tgt, q_start)

        if err < best_err:
            best_err = err

        # Strict: failed full IK is not accepted.
        if q_sol is None:
            continue

        T_chk, _ = forward_kinematics(q_sol)
        e = pose_error(T_chk, T_tgt)

        pos_err = float(np.linalg.norm(e[:3]))
        ori_err = float(np.linalg.norm(e[3:]))

        if pos_err > STRICT_POS_ACCEPT_M:
            continue

        if ori_err > STRICT_ORI_ACCEPT_RAD:
            continue

        dq = _joint_delta(q_sol, q_ref)
        wrist4_delta = abs(dq[3])
        wrist5_delta = abs(dq[4])
        wrist6_delta = abs(dq[5])

        # Hard safety reject: do not spin the final wrist too much.
        if wrist6_delta > STRICT_WRIST6_DELTA_RAD:
            continue

        cost = (
            100.0 * pos_err
            + 8.0 * ori_err
            + 0.20 * float(np.linalg.norm(dq))
            + 0.60 * wrist4_delta
            + 0.40 * wrist5_delta
            + 2.00 * wrist6_delta
        )

        if cost < best_cost:
            best_cost = cost
            best_q = q_sol.copy()
            best_err = float(np.linalg.norm(e))

    return best_q, best_err


# ─────────────────────────────────────────────────────────────────────────────
# Geometry utilities
# ─────────────────────────────────────────────────────────────────────────────

def pose_stamped_to_matrix(msg: PoseStamped) -> np.ndarray:
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z

    qx = msg.pose.orientation.x
    qy = msg.pose.orientation.y
    qz = msg.pose.orientation.z
    qw = msg.pose.orientation.w

    n = math.sqrt(qx ** 2 + qy ** 2 + qz ** 2 + qw ** 2)

    if n < 1e-9:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    else:
        qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    R = np.array([
        [1.0 - 2.0 * (qy ** 2 + qz ** 2), 2.0 * (qx * qy - qz * qw),       2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw),       1.0 - 2.0 * (qx ** 2 + qz ** 2), 2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw),       2.0 * (qy * qz + qx * qw),       1.0 - 2.0 * (qx ** 2 + qy ** 2)],
    ], dtype=float)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def xyz_rpy_to_matrix(x, y, z, roll, pitch, yaw) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ], dtype=float)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


# ─────────────────────────────────────────────────────────────────────────────
# ROS 2 Node
# ─────────────────────────────────────────────────────────────────────────────

class PiperIKController(Node):

    def __init__(self):
        super().__init__("piper_ik_arm_controller")

        self.declare_parameter("move_duration_s", DEFAULT_DURATION_S)
        self.declare_parameter("arm_base_frame", "arm_base_link")
        self.declare_parameter("gripper_hold_enabled", DEFAULT_GRIPPER_HOLD_ENABLED)
        self.declare_parameter("gripper_hold_period_s", DEFAULT_GRIPPER_HOLD_PERIOD_S)

        self.move_duration = float(self.get_parameter("move_duration_s").value)
        self.arm_base_frame = str(self.get_parameter("arm_base_frame").value)
        self.gripper_hold_enabled = bool(self.get_parameter("gripper_hold_enabled").value)
        self.gripper_hold_period_s = float(self.get_parameter("gripper_hold_period_s").value)

        self._current_q = (JOINT_LIMITS[:, 0] + JOINT_LIMITS[:, 1]) / 2.0
        self._arm_busy = False
        self._last_gripper = GRIPPER_CLOSE.copy()
        self._grip_busy = False
        self._last_grip_command_time = 0.0
        

        if HAS_TF2:
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._arm_ac = ActionClient(
            self,
            FollowJointTrajectory,
            "arm_controller/follow_joint_trajectory",
        )

        self._grip_ac = ActionClient(
            self,
            FollowJointTrajectory,
            "gripper_controller/follow_joint_trajectory",
        )

        self.create_subscription(PoseStamped, "/arm/target_pose", self._pose_cb, 10)
        self.create_subscription(Float32, "/arm/gripper_cmd", self._grip_cb, 10)
        self.create_subscription(String, "/arm/named_pose", self._named_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        if self.gripper_hold_enabled and self.gripper_hold_period_s > 0.0:
            self.create_timer(self.gripper_hold_period_s, self._gripper_hold_timer_cb)

        self.get_logger().info(
            "PiperIKController ready.\n"
            "  /arm/target_pose  PoseStamped  -> strict IK -> arm trajectory\n"
            "  /arm/gripper_cmd  Float32[0,1] -> gripper open/close, last command held\n"
            "  /arm/named_pose   String       -> 'home' | 'ready' | 'stow'\n"
            f"  move_duration_s={self.move_duration:.2f}\n"
            f"  gripper_hold_enabled={self.gripper_hold_enabled}"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState):
        name_to_index = {name: idx for idx, name in enumerate(msg.name)}

        for i, joint_name in enumerate(ARM_JOINT_NAMES):
            idx = name_to_index.get(joint_name)
            if idx is not None and idx < len(msg.position):
                self._current_q[i] = msg.position[idx]

    def _pose_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f"Target pose: frame={msg.header.frame_id}  "
            f"xyz=({msg.pose.position.x:.3f}, "
            f"{msg.pose.position.y:.3f}, "
            f"{msg.pose.position.z:.3f})"
        )

        if msg.header.frame_id not in ("", self.arm_base_frame) and HAS_TF2:
            try:
                msg = self._tf_buffer.transform(
                    msg,
                    self.arm_base_frame,
                    timeout=Duration(seconds=1.0),
                )
            except Exception as e:
                self.get_logger().warn(f"TF failed ({e}) — using frame as-is")

        self._solve_and_send(pose_stamped_to_matrix(msg))

    def _named_cb(self, msg: String):
        name = msg.data.strip().lower()

        if name not in NAMED_POSES:
            self.get_logger().error(
                f"Unknown pose '{name}'. Valid: {list(NAMED_POSES.keys())}"
            )
            return

        self.get_logger().info(f"Named pose: '{name}'")
        self._send_arm(NAMED_POSES[name])

    def _grip_cb(self, msg: Float32):
        t = max(0.0, min(1.0, float(msg.data)))

        j7 = GRIPPER_OPEN[0] * t
        j8 = GRIPPER_OPEN[1] * t

        self._last_gripper = [j7, j8]

        self.get_logger().info(f"Gripper {t:.0%}  [{j7:.4f}, {j8:.4f}]")
        self._send_grip(self._last_gripper, force=True)

    # ── IK ────────────────────────────────────────────────────────────────────

    def _solve_and_send(self, T_tgt: np.ndarray):
        t0 = time.monotonic()

        q_ref = self._current_q.copy()
        q_sol, err = solve_ik(T_tgt, q_seed=q_ref)

        ms = (time.monotonic() - t0) * 1e3

        if q_sol is None:
            self.get_logger().error(
                f"IK failed. Best 6D error={err:.4f}  "
                f"approx={err * 1e3:.2f} scaled-units  ({ms:.1f} ms)"
            )
            return

        T_chk, _ = forward_kinematics(q_sol)
        e = pose_error(T_chk, T_tgt)

        pos_err_mm = float(np.linalg.norm(e[:3]) * 1e3)
        ori_err_rad = float(np.linalg.norm(e[3:]))

        dq = _joint_delta(q_sol, q_ref)
        wrist6_delta = abs(dq[5])

        self.get_logger().info(
            f"IK OK  {ms:.1f} ms\n"
            f"  pos_err = {pos_err_mm:.3f} mm\n"
            f"  ori_err = {ori_err_rad:.3f} rad\n"
            f"  q       = {[f'{v:.3f}' for v in q_sol.tolist()]}\n"
            f"  dq      = {[f'{v:.3f}' for v in dq.tolist()]}\n"
            f"  wrist6_delta = {wrist6_delta:.3f} rad"
        )

        self._send_arm(q_sol.tolist())

    # ── Trajectory senders ────────────────────────────────────────────────────

    def _ros_dur(self, s: float) -> RosDuration:
        s = max(0.1, float(s))
        sec = int(s)
        nanosec = int((s - sec) * 1e9)
        return RosDuration(sec=sec, nanosec=nanosec)

    def _send_arm(self, q: list[float], dur: float | None = None):
        if self._arm_busy:
            self.get_logger().warn("Arm is busy. Ignoring new arm command until current motion finishes.")
            return
        
        if not self._arm_ac.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("arm_controller not available")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.velocities = [0.0] * 6
        point.time_from_start = self._ros_dur(dur or self.move_duration)

        goal.trajectory.points = [point]

        self._arm_busy = True

        future = self._arm_ac.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_cb)

        # # Keep gripper at last value immediately after sending arm goal.
        # if self.gripper_hold_enabled:
        #     self._send_grip(self._last_gripper, force=False)



    def _send_grip(self, fingers: list[float], force: bool = False):
        now = time.monotonic()

        if self._grip_busy and not force:
            return

        # Avoid sending repeated gripper goals too fast.
        if not force and (now - self._last_grip_command_time) < 0.30:
            return

        if not self._grip_ac.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("gripper_controller not available")
            return

        self._last_grip_command_time = now
        self._grip_busy = True

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = GRIPPER_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = [float(fingers[0]), float(fingers[1])]
        point.velocities = [0.0, 0.0]
        point.time_from_start = self._ros_dur(0.8)

        goal.trajectory.points = [point]

        future = self._grip_ac.send_goal_async(goal)
        future.add_done_callback(self._grip_goal_cb)

    def _gripper_hold_timer_cb(self):
        if not self.gripper_hold_enabled:
            return

        self._send_grip(self._last_gripper, force=False)

    # ── Action callbacks ──────────────────────────────────────────────────────

    def _arm_goal_cb(self, future):
        self._arm_busy = False
        try:
            handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Arm goal failed before acceptance: {e}")
            return

        if not handle.accepted:
            self._arm_busy = False
            self.get_logger().error("Arm goal REJECTED")
            return

        self.get_logger().info("Arm goal accepted, executing…")
        handle.get_result_async().add_done_callback(self._arm_result_cb)

    def _arm_result_cb(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f"Arm result failed: {e}")
            return

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("Arm motion complete")
        else:
            self.get_logger().warn(
                f"Arm error code={result.error_code}: {result.error_string}"
            )

        # # Re-hold gripper after arm motion finishes or aborts.
        # if self.gripper_hold_enabled:
        #     self._send_grip(self._last_gripper, force=True)

    def _grip_goal_cb(self, future):
        try:
            handle = future.result()
        except Exception as e:
            self._grip_busy = False
            self.get_logger().warn(f"Gripper goal failed before acceptance: {e}")
            return

        if not handle.accepted:
            self._grip_busy = False
            self.get_logger().warn("Gripper goal REJECTED")
            return

        handle.get_result_async().add_done_callback(self._grip_result_cb)

    def _grip_result_cb(self, future):
        self._grip_busy = False

        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().warn(f"Gripper result failed: {e}")
            return

        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().warn(
                f"Gripper error code={result.error_code}: {result.error_string}"
            )


# ─────────────────────────────────────────────────────────────────────────────
# CLI one-shot
# python3 piper_ik_arm_controller.py x y z roll pitch yaw
# roll/pitch/yaw are radians in this CLI mode
# ─────────────────────────────────────────────────────────────────────────────

def _one_shot(argv):
    rclpy.init(args=argv)
    node = PiperIKController()

    try:
        x, y, z, roll, pitch, yaw = (float(v) for v in argv[1:7])
        T = xyz_rpy_to_matrix(x, y, z, roll, pitch, yaw)

        rclpy.spin_once(node, timeout_sec=1.0)
        node._solve_and_send(T)

        deadline = time.monotonic() + node.move_duration + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    import sys

    if len(sys.argv) == 7:
        _one_shot(sys.argv)
        return

    rclpy.init(args=sys.argv)
    node = PiperIKController()

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