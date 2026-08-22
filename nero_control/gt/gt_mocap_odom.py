#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_to_euler(q):
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
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class GtMocapOdomNode(Node):

    def __init__(self):
        super().__init__('gt_mocap_odom_node')

        self.declare_parameter('mocap_topic', '/vrpn_mocap/bebop/pose')
        self.declare_parameter('gt_odom_topic', '/bebop/gt_fullodom')
        self.declare_parameter('publish_rate', 15.0)
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('child_frame', 'bebop_gt')
        self.declare_parameter('use_full_3d_rotation', False)

        mocap_topic = self.get_parameter('mocap_topic').get_parameter_value().string_value
        gt_odom_topic = self.get_parameter('gt_odom_topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.use_full_3d = self.get_parameter('use_full_3d_rotation').get_parameter_value().bool_value

        self.get_logger().info(f"Initializing GT Mocap Odom Node at {rate_hz} Hz...")
        self.get_logger().info(f"Subscribing to: {mocap_topic} with BEST_EFFORT QoS")
        self.get_logger().info(f"Publishing GT Odometry to: {gt_odom_topic}")

        mocap_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.last_pose_msg = None
        self.prev_stamp = None
        self.prev_x = None
        self.prev_y = None
        self.prev_z = None
        self.prev_roll = None
        self.prev_pitch = None
        self.prev_yaw = None

        self.sub_mocap = self.create_subscription(
            PoseStamped,
            mocap_topic,
            self.mocap_callback,
            mocap_qos
        )

        self.pub_gt_odom = self.create_publisher(Odometry, gt_odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

    def mocap_callback(self, msg: PoseStamped):
        self.last_pose_msg = msg

    def timer_callback(self):
        if self.last_pose_msg is None:
            return

        now = self.get_clock().now()

        if self.prev_stamp is None:
            dt = 1.0 / 15.0
        else:
            dt = (now - self.prev_stamp).nanoseconds * 1e-9
            if dt <= 1e-5:
                return

        self.prev_stamp = now

        pos = self.last_pose_msg.pose.position
        orient = self.last_pose_msg.pose.orientation

        roll_raw, pitch_raw, yaw_raw = quat_to_euler(orient)

        x_f = pos.x
        y_f = pos.y
        z_f = pos.z
        roll_f = roll_raw
        pitch_f = pitch_raw
        yaw_f = yaw_raw

        if self.prev_x is None:
            vx_body = 0.0
            vy_body = 0.0
            vz_body = 0.0
            wx_body = 0.0
            wy_body = 0.0
            wz_body = 0.0
        else:
            dx_w = (x_f - self.prev_x) / dt
            dy_w = (y_f - self.prev_y) / dt
            dz_w = (z_f - self.prev_z) / dt

            droll_w = angle_diff(roll_f, self.prev_roll) / dt
            dpitch_w = angle_diff(pitch_f, self.prev_pitch) / dt
            dyaw_w = angle_diff(yaw_f, self.prev_yaw) / dt

            if self.use_full_3d:
                cr, sr = math.cos(roll_f), math.sin(roll_f)
                cp, sp = math.cos(pitch_f), math.sin(pitch_f)
                cy, sy = math.cos(yaw_f), math.sin(yaw_f)

                r11 = cy * cp
                r12 = cy * sp * sr - sy * cr
                r13 = cy * sp * cr + sy * sr

                r21 = sy * cp
                r22 = sy * sp * sr + cy * cr
                r23 = sy * sp * cr - cy * sr

                r31 = -sp
                r32 = cp * sr
                r33 = cp * cr

                vx_body = r11 * dx_w + r21 * dy_w + r31 * dz_w
                vy_body = r12 * dx_w + r22 * dy_w + r32 * dz_w
                vz_body = r13 * dx_w + r23 * dy_w + r33 * dz_w

                wx_body = droll_w
                wy_body = dpitch_w
                wz_body = dyaw_w
            else:
                cy = math.cos(yaw_f)
                sy = math.sin(yaw_f)

                vx_body = cy * dx_w + sy * dy_w
                vy_body = -sy * dx_w + cy * dy_w
                vz_body = dz_w

                wx_body = 0.0
                wy_body = 0.0
                wz_body = dyaw_w

        self.prev_x = x_f
        self.prev_y = y_f
        self.prev_z = z_f
        self.prev_roll = roll_f
        self.prev_pitch = pitch_f
        self.prev_yaw = yaw_f

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.child_frame

        odom.pose.pose.position.x = x_f
        odom.pose.pose.position.y = y_f
        odom.pose.pose.position.z = z_f
        odom.pose.pose.orientation = orient

        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0] = 1e-5
        odom.pose.covariance[7] = 1e-5
        odom.pose.covariance[14] = 1e-5
        odom.pose.covariance[21] = 1e-4
        odom.pose.covariance[28] = 1e-4
        odom.pose.covariance[35] = 1e-4

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

        self.pub_gt_odom.publish(odom)

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
