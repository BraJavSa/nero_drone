#!/usr/bin/env python3

# Main cascade controller implementing position and velocity loops for the drone.

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


class CascadeController(Node):

    RATE_HZ: float = 15.0

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
    opt=1
    if opt==1: 
        KP  = np.diag([2.000000, 3.500000, 2.200000, 2.000000])
        KSP = np.diag([0.500000, 0.700000, 0.500000, 0.400000])
        KD  = np.diag([0.500000, 0.600000, 0.500000, 0.400000])
        KSD = np.diag([0.150000, 0.220000, 0.150000, 0.150000])
    else: 
        KP  = np.diag([2.500000, 2.500000, 3.500000, 4.500000])
        KSP = np.diag([0.800000, 0.800000, 0.800000, 0.800000])
        KD  = np.diag([4.000000, 4.000000, 2.500000, 2.000000])
        KSD = np.diag([0.500000, 0.500000, 0.800000, 0.800000])

    U_MAX = np.ones(4)

    def __init__(self):
        super().__init__('cascade_controller')

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

        self.sub_odom = self.create_subscription(
            Odometry, '/bebop/fullodom', self.odom_callback, 10)
        self.sub_ref  = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_fly  = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/safe_bebop/cmd_vel', 10)

        self.timer = self.create_timer(self.Ts, self.control_loop)

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

        J        = jacobian(psi)
        J_dot    = jacobian_dot(psi, r)
        J_inv    = jacobian_inv(psi)
        F1       = J @ self.f1
        F2       = J @ self.f2 @ J_inv - J_dot @ J_inv
        F1_inv   = np.linalg.inv(F1)

        X_dot = J @ nu

        X_tilde      = self.eta_d - eta
        X_tilde[3]   = wrap_angle(X_tilde[3])

        X_dot_ref    = self.nu_d + self.KSP @ np.tanh(self.KP @ X_tilde)

        X_dot_tilde  = X_dot_ref - X_dot
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
        cmd.linear.z  = float(U_body[2])
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(U_body[3])*0
        self.pub_cmd.publish(cmd)

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())

    def _reset_memory(self):
        self.X_dot_ref_prev = np.zeros(4)
        self.X_ddot_ref_filt = np.zeros(4)


def main(args=None):
    rclpy.init(args=args)
    node = CascadeController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()