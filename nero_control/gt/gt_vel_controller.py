#!/usr/bin/env python3
"""
Ground Truth Velocity Controller for Bebop drone.
Subscribes to /bebop/gt_fullodom instead of /bebop/fullodom.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

# Identified OptiTrack parameters (system_identification_parameters_optitrack_4dof.json)
F1_DIAG = np.array([0.921527, 1.053286, 4.173221, 8.772786])
F2_DIAG = np.array([0.247044, 0.395160, 1.975836, 6.101834])

VX_MAX   = 4.5
VY_MAX   = 4.5
VZ_MAX   = 1.0
DYAW_MAX = np.deg2rad(100.0)

REF_TIMEOUT = 0.5
CONTROL_HZ  = 15.0


class GtBebopVelocityController(Node):
    def __init__(self):
        super().__init__('gt_bebop_velocity_controller')

        self.vx_b = self.vy_b = self.vz_b = self.dyaw_b = 0.0
        self.ref_vx_b = self.ref_vy_b = self.ref_vz_b = self.ref_dyaw_b = 0.0

        self.ref_filt = np.zeros(4)
        self.ref_prev = np.zeros(4)

        self.is_flying     = False
        self.odom_received = False
        self.last_ref_time = None

        self.declare_parameter('kv_x', 3.187700)
        self.declare_parameter('kv_y', 14.001798)
        self.declare_parameter('kv_z', 0.491455)
        self.declare_parameter('kv_yaw', 0.177302)

        self.declare_parameter('ksv_x', 1.419744)
        self.declare_parameter('ksv_y', 0.482469)
        self.declare_parameter('ksv_z', 0.132533)
        self.declare_parameter('ksv_yaw', 0.133258)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.sub_odom   = self.create_subscription(
            Odometry, '/bebop/gt_fullodom', self.odom_callback, 10)
        self.sub_ref    = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_flying = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)
        self.pub_cmd    = self.create_publisher(Twist, '/safe_bebop/cmd_vel', 10)

        self.dt    = 1.0 / CONTROL_HZ
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('GtBebopVelocityController started on /bebop/gt_fullodom')

    def _on_param_change(self, params):
        return SetParametersResult(successful=True)

    def odom_callback(self, msg):
        self.vx_b   = msg.twist.twist.linear.x
        self.vy_b   = msg.twist.twist.linear.y
        self.vz_b   = msg.twist.twist.linear.z
        self.dyaw_b = msg.twist.twist.angular.z
        self.odom_received = True

    def ref_callback(self, msg):
        if len(msg.data) < 8:
            return
        self.ref_vx_b   = float(np.clip(msg.data[4], -VX_MAX,   VX_MAX))
        self.ref_vy_b   = float(np.clip(msg.data[5], -VY_MAX,   VY_MAX))
        self.ref_vz_b   = float(np.clip(msg.data[6], -VZ_MAX,   VZ_MAX))
        self.ref_dyaw_b = float(np.clip(msg.data[7], -DYAW_MAX, DYAW_MAX))
        self.last_ref_time = self.get_clock().now()

    def is_flying_callback(self, msg):
        if not msg.data and self.is_flying:
            self.ref_filt = np.zeros(4)
            self.ref_prev = np.zeros(4)
        self.is_flying = msg.data

    def control_loop(self):
        if not self.odom_received or not self.is_flying:
            self.pub_cmd.publish(Twist())
            return

        if self.last_ref_time is not None:
            elapsed = (self.get_clock().now() - self.last_ref_time).nanoseconds * 1e-9
            if elapsed > REF_TIMEOUT:
                self.ref_vx_b = self.ref_vy_b = self.ref_vz_b = self.ref_dyaw_b = 0.0

        nu_d_raw = np.array([self.ref_vx_b, self.ref_vy_b, self.ref_vz_b, self.ref_dyaw_b])
        self.ref_filt = nu_d_raw.copy()

        nu_d_dot = (self.ref_filt - self.ref_prev) / self.dt
        self.ref_prev = self.ref_filt.copy()

        nu = np.array([self.vx_b, self.vy_b, self.vz_b, self.dyaw_b])
        nu_tilde = self.ref_filt - nu
        nu_tilde[3] = (nu_tilde[3] + np.pi) % (2.0 * np.pi) - np.pi

        K_V_DIAG = np.array([
            self.get_parameter('kv_x').value,
            self.get_parameter('kv_y').value,
            self.get_parameter('kv_z').value,
            self.get_parameter('kv_yaw').value
        ])
        K_SV_DIAG = np.array([
            self.get_parameter('ksv_x').value,
            self.get_parameter('ksv_y').value,
            self.get_parameter('ksv_z').value,
            self.get_parameter('ksv_yaw').value
        ])

        alpha_control = nu_d_dot + K_SV_DIAG * np.tanh(K_V_DIAG * nu_tilde)
        f1_inv = 1.0 / F1_DIAG
        Ud = f1_inv * (alpha_control + F2_DIAG * nu)

        U_body = np.clip(Ud, -1.0, 1.0)

        cmd = Twist()
        cmd.linear.x  = float(U_body[0])
        cmd.linear.y  = float(U_body[1])
        cmd.linear.z  = float(U_body[2])
        cmd.angular.z = float(U_body[3])
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GtBebopVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
