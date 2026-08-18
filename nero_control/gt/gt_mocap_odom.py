#!/usr/bin/env python3
"""
Ground Truth Odometry Node for Bebop drone using OptiTrack VRPN Mocap data.

Subscribes to:
    /vrpn_mocap/bebop/pose (geometry_msgs/msg/PoseStamped) using BEST_EFFORT QoS.

Publishes:
    /bebop/gt_fullodom (nav_msgs/msg/Odometry) at 15 Hz.
    /bebop/gt_odom (nav_msgs/msg/Odometry) at 15 Hz.
    TF Transform: world -> bebop_gt

Position & Orientation:
    Extracted from Mocap in Inertial Frame ('world').

Velocities:
    Derived via finite difference of filtered position and converted to Body Frame ('bebop_gt').
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_to_euler(q):
    """Convert geometry_msgs/Quaternion or object with x,y,z,w to (roll, pitch, yaw)."""
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def angle_diff(a: float, b: float) -> float:
    """Wrap-around aware angular difference (a - b) in (-pi, pi]."""
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class CausalMovingAverage:
    """
    Causal moving average filter for position and orientation signals.
    Prevents high-frequency noise amplification when taking finite differences.
    """

    def __init__(self, window_size: int = 5, is_angle: bool = False):
        self.W = window_size
        self.is_angle = is_angle
        self.buffer = []

    def update(self, val: float) -> float:
        self.buffer.append(val)
        if len(self.buffer) > self.W:
            self.buffer.pop(0)

        if self.is_angle:
            sin_sum = sum(math.sin(v) for v in self.buffer)
            cos_sum = sum(math.cos(v) for v in self.buffer)
            return math.atan2(sin_sum, cos_sum)

        return sum(self.buffer) / len(self.buffer)


class GtMocapOdomNode(Node):

    def __init__(self):
        super().__init__('gt_mocap_odom_node')

        # Declare parameters
        self.declare_parameter('mocap_topic', '/vrpn_mocap/bebop/pose')
        self.declare_parameter('gt_odom_topic', '/bebop/gt_fullodom')
        self.declare_parameter('alias_odom_topic', '/bebop/gt_odom')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('child_frame', 'bebop_gt')
        self.declare_parameter('filter_window', 5)
        self.declare_parameter('use_full_3d_rotation', False)

        mocap_topic = self.get_parameter('mocap_topic').get_parameter_value().string_value
        gt_odom_topic = self.get_parameter('gt_odom_topic').get_parameter_value().string_value
        alias_odom_topic = self.get_parameter('alias_odom_topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        filter_w = self.get_parameter('filter_window').get_parameter_value().integer_value
        self.use_full_3d = self.get_parameter('use_full_3d_rotation').get_parameter_value().bool_value

        self.get_logger().info(f"Initializing GT Mocap Odom Node at {rate_hz} Hz...")
        self.get_logger().info(f"Subscribing to: {mocap_topic} with BEST_EFFORT QoS")
        self.get_logger().info(f"Publishing GT Odometry to: {gt_odom_topic} and {alias_odom_topic}")

        # QoS profile matching VRPN MoCap publisher (BEST_EFFORT, VOLATILE)
        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Causal moving average filters
        self.x_filter = CausalMovingAverage(window_size=filter_w, is_angle=False)
        self.y_filter = CausalMovingAverage(window_size=filter_w, is_angle=False)
        self.z_filter = CausalMovingAverage(window_size=filter_w, is_angle=False)
        self.roll_filter = CausalMovingAverage(window_size=filter_w, is_angle=True)
        self.pitch_filter = CausalMovingAverage(window_size=filter_w, is_angle=True)
        self.yaw_filter = CausalMovingAverage(window_size=filter_w, is_angle=True)

        # Previous state tracking for finite differences
        self.last_pose_msg = None
        self.prev_stamp = None
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_roll = None
        self.prev_pitch = None
        self.prev_yaw = None

        # Subscription & Publishers
        self.sub_mocap = self.create_subscription(
            PoseStamped,
            mocap_topic,
            self.mocap_callback,
            mocap_qos
        )

        self.pub_gt_odom = self.create_publisher(Odometry, gt_odom_topic, 10)
        self.pub_alias_odom = self.create_publisher(Odometry, alias_odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for downsampling to requested rate (15 Hz)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

    def mocap_callback(self, msg: PoseStamped):
        """Callback to store incoming VRPN MoCap pose."""
        self.last_pose_msg = msg

    def timer_callback(self):
        """15 Hz control loop computing filtered body velocities & publishing GT odometry."""
        if self.last_pose_msg is None:
            return

        now = self.get_clock().now()

        # Compute dt
        if self.prev_stamp is None:
            dt = 1.0 / 15.0
        else:
            dt = (now - self.prev_stamp).nanoseconds * 1e-9
            if dt <= 1e-5:
                return

        self.prev_stamp = now

        # Extract raw pose components from OptiTrack message
        pos = self.last_pose_msg.pose.position
        orient = self.last_pose_msg.pose.orientation

        roll_raw, pitch_raw, yaw_raw = quat_to_euler(orient)

        # Filter position and orientation
        x_f = self.x_filter.update(pos.x)
        y_f = self.y_filter.update(pos.y)
        z_f = self.z_filter.update(pos.z)
        roll_f = self.roll_filter.update(roll_raw)
        pitch_f = self.pitch_filter.update(pitch_raw)
        yaw_f = self.yaw_filter.update(yaw_raw)

        # Initial sample check
        if self.prev_x is None:
            vx_body = 0.0
            vy_body = 0.0
            vz_body = 0.0
            wx_body = 0.0
            wy_body = 0.0
            wz_body = 0.0
        else:
            # Inertial linear velocities
            dx_w = (x_f - self.prev_x) / dt
            dy_w = (y_f - self.prev_y) / dt
            dz_w = (z_f - self.prev_z) / dt

            # Inertial angular rates
            droll_w = angle_diff(roll_f, self.prev_roll) / dt
            dpitch_w = angle_diff(pitch_f, self.prev_pitch) / dt
            dyaw_w = angle_diff(yaw_f, self.prev_yaw) / dt

            if self.use_full_3d:
                # 3D Rotation Matrix from World to Body: R_BW = R_WB^T
                cr, sr = math.cos(roll_f), math.sin(roll_f)
                cp, sp = math.cos(pitch_f), math.sin(pitch_f)
                cy, sy = math.cos(yaw_f), math.sin(yaw_f)

                # R_WB = Rz(yaw) * Ry(pitch) * Rx(roll)
                # Row vectors of R_WB are column vectors of R_BW
                r11 = cy * cp
                r12 = cy * sp * sr - sy * cr
                r13 = cy * sp * cr + sy * sr

                r21 = sy * cp
                r22 = sy * sp * sr + cy * cr
                r23 = sy * sp * cr - cy * sr

                r31 = -sp
                r32 = cp * sr
                r33 = cp * cr

                # v_body = R_BW * v_world = R_WB^T * v_world
                vx_body = r11 * dx_w + r21 * dy_w + r31 * dz_w
                vy_body = r12 * dx_w + r22 * dy_w + r32 * dz_w
                vz_body = r13 * dx_w + r23 * dy_w + r33 * dz_w

                wx_body = droll_w
                wy_body = dpitch_w
                wz_body = dyaw_w
            else:
                # 2D Kinematic Model (as used in extended_controller.py / Quadrotor dynamics):
                # v_body = J_inv(yaw) * v_world
                cy = math.cos(yaw_f)
                sy = math.sin(yaw_f)

                vx_body = cy * dx_w + sy * dy_w
                vy_body = -sy * dx_w + cy * dy_w
                vz_body = dz_w

                wx_body = 0.0
                wy_body = 0.0
                wz_body = dyaw_w

        # Save state for next iteration
        self.prev_x = x_f
        self.prev_y = y_f
        self.prev_z = z_f
        self.prev_roll = roll_f
        self.prev_pitch = pitch_f
        self.prev_yaw = yaw_f

        # Construct Odometry Message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.child_frame

        # Pose in Inertial Frame
        odom.pose.pose.position.x = x_f
        odom.pose.pose.position.y = y_f
        odom.pose.pose.position.z = z_f
        odom.pose.pose.orientation = orient  # Mocap raw quaternion

        # Covariance matrix for high-precision Mocap OptiTrack
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 1e-5  # x
        odom.pose.covariance[7] = 1e-5  # y
        odom.pose.covariance[14] = 1e-5 # z
        odom.pose.covariance[21] = 1e-4 # roll
        odom.pose.covariance[28] = 1e-4 # pitch
        odom.pose.covariance[35] = 1e-4 # yaw

        # Twist (Velocities) in Body Frame
        odom.twist.twist.linear.x = float(vx_body)
        odom.twist.twist.linear.y = float(vy_body)
        odom.twist.twist.linear.z = float(vz_body)

        odom.twist.twist.angular.x = float(wx_body)
        odom.twist.twist.angular.y = float(wy_body)
        odom.twist.twist.angular.z = float(wz_body)

        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0] = 1e-3
        odom.twist.covariance[7] = 1e-3
        odom.twist.covariance[14] = 1e-3
        odom.twist.covariance[35] = 1e-3

        # Publish GT Odometry
        self.pub_gt_odom.publish(odom)
        self.pub_alias_odom.publish(odom)

        # Broadcast TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = x_f
        tf_msg.transform.translation.y = y_f
        tf_msg.transform.translation.z = z_f
        tf_msg.transform.rotation = orient

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GtMocapOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
