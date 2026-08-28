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


def saturate(v: np.ndarray, vmax: np.ndarray) -> np.ndarray:
    return np.clip(v, -vmax, vmax)


class GtCompensadorController(Node):

    RATE_HZ: float = 15.0

    f1 = np.diag([0.921527, 1.053286, 3.8086879470003603, 8.772786])
    f2 = np.diag([0.247044, 0.395160, 3.7414469817348253, 6.101834])

    KP  = np.diag([3.880000, 3.416000, 8.000000, 12.533000])
    KSP = np.diag([0.203000, 0.203000, 0.200000, 0.301000])
    KD  = np.diag([8.835000, 10.000000, 2.376000, 1.962000])
    KSD = np.diag([2.000000, 2.000000, 0.985000, 0.800000])

    U_MAX = np.ones(4)

    def __init__(self):
        super().__init__('gt_compensador_controller')

        self.Ts = 1.0 / self.RATE_HZ

        self.X  = np.zeros(4)
        self.nu = np.zeros(4)

        self.X_d   = np.zeros(4)
        self.dX_d  = np.zeros(4)
        self.ddX_d = np.zeros(4)

        self.Ucw_prev = np.zeros(4)

        self.is_flying = False
        self.odom_received = False
        self.ref_received = False

        self.SAFE_XY = 1.5
        self.SAFE_Z_MAX = 2.0

        self.sub_odom = self.create_subscription(
            Odometry, '/bebop/gt_fullodom', self.odom_callback, 10)
        self.sub_ref = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_fly = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)

        self.pub_cmd = self.create_publisher(Twist, '/safe_bebop/cmd_vel', 10)

        self.timer = self.create_timer(self.Ts, self.control_loop)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.X[0] = p.x
        self.X[1] = p.y
        self.X[2] = p.z

        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.X[3] = yaw

        v = msg.twist.twist.linear
        self.nu[0] = v.x
        self.nu[1] = v.y
        self.nu[2] = v.z
        self.nu[3] = msg.twist.twist.angular.z

        self.odom_received = True

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 12:
            return
        self.X_d   = np.array(msg.data[0:4])
        self.dX_d  = np.array(msg.data[4:8])
        self.ref_received = True

    def is_flying_callback(self, msg: Bool):
        self.is_flying = msg.data

    def kinematic_control(self, X, J):
        Xtil = self.X_d - X
        Xtil[3] = wrap_angle(Xtil[3])

        Ucw = self.dX_d + self.KSP @ np.tanh(self.KP @ Xtil)

        dUcw = (Ucw - self.Ucw_prev) / self.Ts
        self.Ucw_prev = Ucw

        return Ucw, dUcw

    def dynamic_compensator(self, Ucw, dUcw, dX, J):
        A = J @ self.f1
        b = dUcw + self.KSD @ (Ucw - dX) + self.f2 @ dX
        return np.linalg.solve(A, b)

    def control_loop(self):
        if not self.is_flying:
            self._publish_zero()
            return

        if not self.odom_received or not self.ref_received:
            self._publish_zero()
            return

        X  = self.X.copy()
        nu = self.nu.copy()
        psi = X[3]

        x_out = abs(X[0]) > self.SAFE_XY
        y_out = abs(X[1]) > self.SAFE_XY
        z_out = X[2] > self.SAFE_Z_MAX or X[2] < 0.0
        if x_out or y_out or z_out:
            self.get_logger().warn(
                f"SAFETY: position ({X[0]:.2f}, {X[1]:.2f}, {X[2]:.2f}) "
                f"outside safe cube. Sending zero cmd.",
                throttle_duration_sec=0.5)
            self._publish_zero()
            return

        J = jacobian(psi)
        dX = J @ nu

        Ucw, dUcw = self.kinematic_control(X, J)
        Udw = self.dynamic_compensator(Ucw, dUcw, dX, J)

        U_body = saturate(Udw, self.U_MAX)
        self._publish_cmd(U_body)

    def _publish_cmd(self, U_body: np.ndarray):
        cmd = Twist()
        cmd.linear.x = float(U_body[0])
        cmd.linear.y = float(U_body[1])
        cmd.linear.z = float(U_body[2])*0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(U_body[3])
        self.pub_cmd.publish(cmd)

    def _publish_zero(self):
        self.pub_cmd.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = GtCompensadorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()