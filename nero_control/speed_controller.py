#!/usr/bin/env python3
"""
Bebop Drone — Body-Frame Velocity Controller
=============================================
Control law: discrete PD on body-frame velocity error + acceleration feedforward.

Reference signals (from /bebop/ref_vec, indices per context):
    eta_d    [0:4]  — desired pose (used only for yaw reference, ψ_d)
    nu_d_w   [4:8]  — desired velocity in WORLD frame → converted to body frame below
    alpha_d  [8:12] — desired acceleration in WORLD frame → converted to body frame below

Body-frame velocity reference:
    nu_d   = J_inv(ψ) · nu_d_w      (world→body, linear channels; yaw-rate direct)
    alpha_d_b = J_inv(ψ) · alpha_d  (world→body feedforward acceleration)

Control law (body frame, discrete):
    e_nu   = nu_d - nu               (velocity error)
    de_nu  = (e_nu - e_nu_prev) / Ts (discrete derivative of error)
    U_raw  = f1_inv · (alpha_d_b + Kp·e_nu + Kd·de_nu + f2·nu)

where:
    f1 = diag(k1, k3, k5, k7)  (body-frame input gain, diagonal)
    f2 = diag(k2, k4, k6, k8)  (body-frame damping, diagonal)
    f1_inv = 1/f1 element-wise  (also diagonal)

f1, f2 are computed from the identified world-frame F1, F2 via:
    f1 ≈ diag(F1)   (F1 is nearly diagonal, small off-diagonal terms ignored for body ctrl)
    f2 ≈ diag(F2)

For a proper gain-scheduled version the full J·F1, J·F2 transform is done each step.
The diagonal approximation is used here for clarity; replace if full coupling is needed.

Gains Kp, Kd are set below under "TUNABLE GAINS". Units: body-frame normalised inputs.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool

# ---------------------------------------------------------------------------
# Helper utilities (mandated by context §8)
# ---------------------------------------------------------------------------

def euler_from_quaternion(q):
    """q = [x, y, z, w] → (roll, pitch, yaw)"""
    x, y, z, w = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def jacobian(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[ c, -s, 0, 0],
                     [ s,  c, 0, 0],
                     [ 0,  0, 1, 0],
                     [ 0,  0, 0, 1]])


def jacobian_inv(psi: float) -> np.ndarray:
    """Inverse of the yaw-rotation Jacobian (= its transpose for pure rotation)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[ c,  s, 0, 0],
                     [-s,  c, 0, 0],
                     [ 0,  0, 1, 0],
                     [ 0,  0, 0, 1]])


def jacobian_dot(psi: float, psi_dot: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return psi_dot * np.array([[-s, -c, 0, 0],
                                [ c, -s, 0, 0],
                                [ 0,  0, 0, 0],
                                [ 0,  0, 0, 0]])


def saturate(v: np.ndarray, vmax: np.ndarray) -> np.ndarray:
    return np.clip(v, -vmax, vmax)


# ---------------------------------------------------------------------------
# Identified world-frame parameters (context §5)
# ---------------------------------------------------------------------------

F1_WORLD = np.array([
    [ 0.988324,  0.014589,  0.000000, -0.009546],
    [-0.019977,  0.986558,  0.000000,  0.005055],
    [ 0.000000,  0.000000,  0.802580,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.853101],
])

F2_WORLD = np.array([
    [ 0.018878, -0.002861,  0.000000,  0.003410],
    [-0.002938,  0.025773,  0.000000, -0.008888],
    [ 0.000000,  0.000000,  0.122009,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.122507],
])

# ---------------------------------------------------------------------------
# TUNABLE GAINS  ← edit only this section when tuning
# ---------------------------------------------------------------------------
# Diagonal gain matrices for the 4 body-frame channels [vx, vy, vz, r].
# Start with small Kp and zero Kd; increase gradually on the real platform.

KP = np.diag([1.0, 1.0, 1.5, 1.2])   # proportional gain on velocity error
KD = np.diag([0.05, 0.05, 0.05, 0.02])  # derivative gain on velocity error

# ---------------------------------------------------------------------------
# Controller node
# ---------------------------------------------------------------------------

class VelocityController(Node):
    """
    Body-frame velocity controller for the Parrot Bebop.

    Subscribes:
        /bebop/fullodom   (nav_msgs/Odometry)
        /bebop/ref_vec    (std_msgs/Float64MultiArray)
        /bebop/is_flying  (std_msgs/Bool)

    Publishes:
        /safe_bebop/cmd_vel (geometry_msgs/Twist)
    """

    def __init__(self):
        super().__init__('velocity_controller')

        # --- timing ---
        self.Ts = 1.0 / 15.0

        # --- state (context §9) ---
        self.eta  = np.zeros(4)   # world-frame pose [x, y, z, ψ]
        self.nu   = np.zeros(4)   # body-frame velocity [vx, vy, vz, r]

        # --- reference signals ---
        self.eta_d   = np.zeros(4)   # desired pose (ψ_d used for yaw error)
        self.nu_d_w  = np.zeros(4)   # desired velocity in WORLD frame
        self.alpha_d = np.zeros(4)   # desired acceleration in WORLD frame

        # --- flags ---
        self.is_flying     = False
        self.odom_received = False
        self.ref_received  = False

        # --- memory (reset on disarm) ---
        self.e_nu_prev = np.zeros(4)  # previous velocity error (for derivative)

        # --- ROS2 interfaces ---
        self.pub_cmd = self.create_publisher(
            Twist, '/safe_bebop/cmd_vel', 10)

        self.create_subscription(
            Odometry, '/bebop/fullodom', self._odom_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self._ref_cb, 10)
        self.create_subscription(
            Bool, '/bebop/is_flying', self._flying_cb, 10)

        self.create_timer(self.Ts, self.control_loop)

        self.get_logger().info('VelocityController initialised at 15 Hz.')

    # -----------------------------------------------------------------------
    # Subscriber callbacks
    # -----------------------------------------------------------------------

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.eta = np.array([p.x, p.y, p.z, yaw])

        v = msg.twist.twist.linear
        r = msg.twist.twist.angular.z
        self.nu = np.array([v.x, v.y, v.z, r])

        self.odom_received = True

    def _ref_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 12:
            return  # discard malformed message silently (context §6)
        d = msg.data
        self.eta_d   = np.array(d[0:4])
        self.nu_d_w  = np.array(d[4:8])   # world-frame velocity reference
        self.alpha_d = np.array(d[8:12])  # world-frame acceleration feedforward
        self.ref_received = True

    def _flying_cb(self, msg: Bool):
        self.is_flying = msg.data

    # -----------------------------------------------------------------------
    # Safety helpers
    # -----------------------------------------------------------------------

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _reset_memory(self):
        """Zero all inter-step state (context §11)."""
        self.e_nu_prev = np.zeros(4)

    # -----------------------------------------------------------------------
    # Main control loop (context §10)
    # -----------------------------------------------------------------------

    def control_loop(self):

        # 1. Safety: disarmed
        if not self.is_flying:
            self._publish_zero()
            self._reset_memory()
            return

        # 2. Safety: missing data
        if not self.odom_received or not self.ref_received:
            self._publish_zero()
            return

        # 3. Snapshot state (never mutate stored arrays mid-step)
        eta = self.eta.copy()
        nu  = self.nu.copy()
        nu_d_w  = self.nu_d_w.copy()
        alpha_d = self.alpha_d.copy()

        psi     = eta[3]
        psi_dot = nu[3]   # yaw rate from body-frame state

        # 4. Jacobians (recomputed every step — ψ changes)
        J     = jacobian(psi)
        J_inv = jacobian_inv(psi)
        J_dot = jacobian_dot(psi, psi_dot)

        # 5. World → body transform for references
        #    nu_d   : desired body-frame velocity
        #    alpha_d_b : desired body-frame acceleration (feedforward)
        nu_d     = J_inv @ nu_d_w
        alpha_d_b = J_inv @ alpha_d

        # 6. Body-frame model matrices from world-frame identified parameters
        #    ν̇ = f1·U − f2·ν  with  f1 = J⁻¹·F1·J,  f2 = J⁻¹·F2·J
        #    Because J is unitary: f1 = J_inv @ F1_WORLD @ J  (applied each step)
        f1 = J_inv @ F1_WORLD @ J
        f2 = J_inv @ F2_WORLD @ J
        f1_inv = np.linalg.inv(f1)

        # 7. Velocity error (wrap yaw channel)
        e_nu    = nu_d - nu
        e_nu[3] = wrap_angle(e_nu[3])

        # 8. Discrete derivative of velocity error
        de_nu    = (e_nu - self.e_nu_prev) / self.Ts
        # Wrap derivative of yaw error too
        de_nu[3] = wrap_angle(de_nu[3])

        # 9. Control law
        #    U_raw = f1_inv · (alpha_d_b + Kp·e_nu + Kd·de_nu + f2·nu)
        #    The f2·nu term is the model-based damping compensation.
        U_raw = f1_inv @ (alpha_d_b + KP @ e_nu + KD @ de_nu + f2 @ nu)

        # 10. Saturate to ±1 per channel (context §7)
        U = np.clip(U_raw, -1.0, 1.0)

        # 11. Publish
        cmd = Twist()
        cmd.linear.x  = float(U[0])
        cmd.linear.y  = float(U[1])
        cmd.linear.z  = float(U[2])*0.2
        cmd.angular.z = float(U[3])
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        self.pub_cmd.publish(cmd)

        # 12. Update memory at END of step
        self.e_nu_prev = e_nu.copy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()