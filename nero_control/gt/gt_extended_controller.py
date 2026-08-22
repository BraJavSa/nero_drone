#!/usr/bin/env python3
"""
Ground Truth Cascade Controller for Bebop drone.
Adapted to use Ground Truth Odometry (/bebop/gt_fullodom) from OptiTrack VRPN Mocap.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool


def euler_from_quaternion(q):
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
    return np.array([
        [ c, -s, 0.0, 0.0],
        [ s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_inv(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c,  s, 0.0, 0.0],
        [-s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_dot(psi: float, psi_dot: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return psi_dot * np.array([
        [-s, -c, 0.0, 0.0],
        [ c, -s, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ])


def saturate(v: np.ndarray, vmax: np.ndarray) -> np.ndarray:
    return np.clip(v, -vmax, vmax)


class GtCascadeController(Node):

    RATE_HZ: float = 15.0

    # Identified OptiTrack parameters (system_identification_parameters_optitrack_4dof.json)
    f1 = np.array([
            [0.921527, 0.000000, 0.000000, 0.000000],
            [0.000000, 1.053286, 0.000000, 0.000000],
            [0.000000, 0.000000, 4.173221, 0.000000],
            [0.000000, 0.000000, 0.000000, 8.772786],
        ])
    f2 = np.array([
            [0.247044, 0.000000, 0.000000, 0.000000],
            [0.000000, 0.395160, 0.000000, 0.000000],
            [0.000000, 0.000000, 1.975836, 0.000000],
            [0.000000, 0.000000, 0.000000, 6.101834],
        ])
    opt = 1
    if opt == 1:  # trajectory gains (extra strong Y tracking, ultra-smooth U)
        KP  = np.diag([0.114000, 0.115000, 1.000000, 1.039000])
        KSP = np.diag([0.200000, 0.200000, 0.100000, 0.108000])
        KD  = np.diag([1.003000, 4.990000, 0.500000, 0.301000])
        KSD = np.diag([0.003000, 2.000000, 0.000000, 0.005000])

    U_MAX = np.ones(4)

    def __init__(self):
        super().__init__('gt_cascade_controller')

        self.Ts = 1.0 / self.RATE_HZ

        self.eta = np.zeros(4)
        self.nu  = np.zeros(4)

        self.eta_d   = np.zeros(4)
        self.nu_d    = np.zeros(4)
        self.alpha_d = np.zeros(4)

        self.X_dot_ref_prev = np.zeros(4)
        self.X_ddot_ref_filt = np.zeros(4)

        self.is_flying = False

        self.odom_received = False
        self.ref_received  = False

        # Safety cube limits (m)
        self.SAFE_XY = 1.5   # ±1.5 m in X and Y
        self.SAFE_Z_MAX = 2.0  # max altitude 2 m

        # First-reference initialization guard
        self.first_ref_initialized = False  # True once X_dot_ref_prev seeded
        self.ref_last_time = None           # ROS time of last ref received

        # Subscribes to Ground Truth Odometry
        self.sub_odom = self.create_subscription(
            Odometry, '/bebop/gt_fullodom', self.odom_callback, 10)
        self.sub_ref  = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_fly  = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/safe_bebop/cmd_vel', 10)

        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info("GT Cascade Controller initialized. Subscribed to /bebop/gt_fullodom")

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.eta[0] = p.x
        self.eta[1] = p.y
        self.eta[2] = p.z

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.eta[3] = yaw

        v = msg.twist.twist.linear
        self.nu[0] = v.x
        self.nu[1] = v.y
        self.nu[2] = v.z
        self.nu[3] = msg.twist.twist.angular.z

        self.odom_received = True

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 12:
            return

        now = self.get_clock().now()

        # If it's been more than 1 s since the last reference, treat the
        # next one as the very first so we re-seed X_dot_ref_prev and avoid
        # a derivative spike.
        if self.ref_last_time is not None:
            dt_since = (now - self.ref_last_time).nanoseconds * 1e-9
            if dt_since > 1.0:
                self.first_ref_initialized = False
                self.get_logger().warn(
                    f"Ref gap {dt_since:.2f}s > 1s — re-initializing first-ref guard.")

        self.ref_last_time = now

        self.eta_d   = np.array(msg.data[0:4])
        self.nu_d    = np.array(msg.data[4:8])
        self.alpha_d = np.array(msg.data[8:12])

        self.ref_received = True

    def is_flying_callback(self, msg: Bool):
        self.is_flying = msg.data

    def control_loop(self):
        if not self.is_flying:
            self._publish_zero()
            self._reset_memory()
            return

        if not self.odom_received or not self.ref_received:
            self._publish_zero()
            return

        eta = self.eta.copy()
        nu  = self.nu.copy()
        psi = eta[3]
        r   = nu[3]

        # ── Safety cube ──────────────────────────────────────────────────
        # If the drone is outside the allowed volume, kill all commands.
        x_out = abs(eta[0]) > self.SAFE_XY
        y_out = abs(eta[1]) > self.SAFE_XY
        z_out = eta[2] > self.SAFE_Z_MAX or eta[2] < 0.0
        if x_out or y_out or z_out:
            self.get_logger().warn(
                f"SAFETY: position ({eta[0]:.2f}, {eta[1]:.2f}, {eta[2]:.2f}) "
                f"outside safe cube. Sending zero cmd.",
                throttle_duration_sec=0.5)
            self._publish_zero()
            self._reset_memory()
            return
        # ─────────────────────────────────────────────────────────────────

        J        = jacobian(psi)
        J_dot    = jacobian_dot(psi, r)
        J_inv    = jacobian_inv(psi)
        F1       = J @ self.f1
        F2       = J @ self.f2 @ J_inv - J_dot @ J_inv
        F1_inv   = np.linalg.inv(F1)

        X_dot = J @ nu

        # Force Yaw and Z references to current measured values (no tracking)
        self.eta_d[3] = eta[3]  # yaw ref = current yaw
        self.eta_d[2] = eta[2]  # z   ref = current z
        X_tilde      = self.eta_d - eta
        X_tilde[3]   = wrap_angle(X_tilde[3])

        X_dot_ref = self.nu_d + self.KSP @ np.tanh(self.KP @ X_tilde)

        # ── First-reference guard ─────────────────────────────────────────
        # On the very first valid iteration, seed X_dot_ref_prev so that
        # X_ddot_ref_raw = 0 and there is no acceleration spike.
        if not self.first_ref_initialized:
            self.X_dot_ref_prev = X_dot_ref.copy()
            self.X_ddot_ref_filt = np.zeros(4)
            self.first_ref_initialized = True
            self.get_logger().info("First reference received — seeding derivative memory.")
        # ─────────────────────────────────────────────────────────────────

        X_dot_tilde    = X_dot_ref - X_dot
        X_ddot_ref_raw = (X_dot_ref - self.X_dot_ref_prev) / self.Ts
        self.X_dot_ref_prev = X_dot_ref.copy()
        self.X_ddot_ref_filt = 0.7 * self.X_ddot_ref_filt + 0.3 * X_ddot_ref_raw

        Ud = F1_inv @ (
            self.alpha_d
            + self.X_ddot_ref_filt
            + self.KSD @ np.tanh(self.KD @ X_dot_tilde)
            + F2 @ X_dot
        )

        U_body = saturate(Ud, self.U_MAX)

        self._publish_cmd(U_body)

    def _publish_cmd(self, U_body: np.ndarray):
        cmd = Twist()
        cmd.linear.x  = float(U_body[0])
        cmd.linear.y  = float(U_body[1])
        cmd.linear.z  = 0.0  # Z fixed to zero (ref = current Z, no Z tracking)
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0  # Yaw fixed to zero (ref = current Yaw, no yaw tracking)
        self.pub_cmd.publish(cmd)

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _reset_memory(self):
        self.X_dot_ref_prev = np.zeros(4)
        self.X_ddot_ref_filt = np.zeros(4)
        self.first_ref_initialized = False


def main(args=None):
    rclpy.init(args=args)
    node = GtCascadeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
