#!/usr/bin/env python3


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


def saturate(v: np.ndarray, vmax: np.ndarray) -> np.ndarray:
    return np.clip(v, -vmax, vmax)


class GtInverseDynamicsController(Node):

    RATE_HZ: float = 15.0

    f1 = np.array([
            [0.921527, 0.000000, 0.000000, 0.000000],
            [0.000000, 1.053286, 0.000000, 0.000000],
            [0.000000, 0.000000, 3.8086879470003603, 0.000000],
            [0.000000, 0.000000, 0.000000, 8.772786],
        ])
    f2 = np.array([
            [0.247044, 0.000000, 0.000000, 0.000000],
            [0.000000, 0.395160, 0.000000, 0.000000],
            [0.000000, 0.000000, 3.7414469817348253, 0.000000],
            [0.000000, 0.000000, 0.000000, 6.101834],
        ])

    use_safe_gains = False

    if use_safe_gains:
        KP  = np.diag([2.981000, 2.106000, 6.953000, 10.743000])
        KSP = np.diag([0.313000, 0.291000, 0.203000, 0.520000])
        KD  = np.diag([1.089000, 2.722000, 2.922000, 2.045000])
        KSD = np.diag([1.415000, 0.523000, 1.094000, 0.936000])

    
    else:
        KP  = np.diag([5.0, 4.692, 3.034, 8.091])
        KSP = np.diag([0.763, 0.675, 0.525, 0.735])
        KD  = np.diag([2.636, 3.618, 3.2, 2.067])
        KSD = np.diag([1.2, 0.844, 0.729, 1.344])

    U_MAX = np.ones(4)

    def __init__(self):
        super().__init__('gt_inverse_dynamics_controller')

        self.Ts = 1.0 / self.RATE_HZ

        self.f1_inv = np.linalg.inv(self.f1)

        self.eta = np.zeros(4)
        self.nu  = np.zeros(4)

        self.eta_d   = np.zeros(4)   # desired pose
        self.nu_d    = np.zeros(4)   # desired inertial velocity  (eta_dot_d)
        self.alpha_d = np.zeros(4)   # desired inertial accel     (eta_ddot_d)

        self.is_flying = False

        self.odom_received = False
        self.ref_received  = False

        # Safety cube limits (m)
        self.SAFE_XY = 1.5     
        self.SAFE_Z_MAX = 2.0   

        # Subscribes to Ground Truth Odometry
        self.sub_odom = self.create_subscription(
            Odometry, '/bebop/gt_fullodom', self.odom_callback, 10)
        self.sub_ref  = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_fly  = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/safe_bebop/cmd_vel', 10)

        self.timer = self.create_timer(self.Ts, self.control_loop)
        self.get_logger().info(
            "GT Inverse-Dynamics Controller initialized. Subscribed to /bebop/gt_fullodom")

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
            return

        if not self.odom_received or not self.ref_received:
            self._publish_zero()
            return

        eta = self.eta.copy()
        nu  = self.nu.copy()
        psi = eta[3]

        # -- Safety cube ----------------------------------------------------
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
            return
        # ---------------------------------------------------------------------

        # Kinematics and tracking errors
        J     = jacobian(psi)
        J_inv = jacobian_inv(psi)

        eta_dot = J @ nu

        eta_tilde = self.eta_d - eta
        eta_tilde[3] = wrap_angle(eta_tilde[3])

        eta_dot_tilde = self.nu_d - eta_dot

        # Virtual acceleration and inverse-dynamics control command
        alpha = (self.alpha_d
                 + self.KSD @ np.tanh(self.KD @ eta_dot_tilde)
                 + self.KSP @ np.tanh(self.KP @ eta_tilde))

        Ud = self.f1_inv @ (J_inv @ alpha + self.f2 @ nu)

        U_body = saturate(Ud, self.U_MAX)

        self._publish_cmd(U_body)

    def _publish_cmd(self, U_body: np.ndarray):
        cmd = Twist()
        cmd.linear.x  = float(U_body[0])
        cmd.linear.y  = float(U_body[1])
        cmd.linear.z  = float(U_body[2])*0.0 
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(U_body[3]) 
        self.pub_cmd.publish(cmd)

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GtInverseDynamicsController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()