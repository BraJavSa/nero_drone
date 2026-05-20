#!/usr/bin/env python3

# Extended Kalman Filter (EKF) implementation fusing IMU, Odometry, and Optical Flow.

import math
import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


def quat_to_euler(q):

    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)

    sinp = max(-1.0, min(1.0, sinp))

    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def angle_diff(a, b):

    d = a - b

    while d > math.pi:
        d -= 2.0 * math.pi

    while d < -math.pi:
        d += 2.0 * math.pi

    return d


class AlphaBetaFilter:

    def __init__(self, alpha, beta, is_angle=False):

        self.alpha = alpha
        self.beta  = beta

        self.is_angle = is_angle

        self.x_est = None
        self.v_est = 0.0

    def update(self, measurement, dt):

        if self.x_est is None:

            self.x_est = measurement

            return self.x_est, self.v_est

        x_pred = self.x_est + self.v_est * dt

        if self.is_angle:
            residual = angle_diff(measurement, x_pred)
        else:
            residual = measurement - x_pred

        self.x_est = x_pred + self.alpha * residual

        if self.is_angle:

            while self.x_est > math.pi:
                self.x_est -= 2.0 * math.pi

            while self.x_est < -math.pi:
                self.x_est += 2.0 * math.pi

        self.v_est = self.v_est + (self.beta / dt) * residual

        return self.x_est, self.v_est


class SensorFusion(Node):

    def __init__(self):

        super().__init__('sensor_fusion')

        self.last_odom = None
        self.last_z    = None

        self.prev_stamp = None

        self.vz   = 0.0
        self.vyaw = 0.0

        self.z_filter = AlphaBetaFilter(
            alpha=0.5,
            beta=0.03,
            is_angle=False
        )

        self.yaw_filter = AlphaBetaFilter(
            alpha=0.5,
            beta=0.3,
            is_angle=True
        )

        self.sub_odom = self.create_subscription(
            Odometry,
            '/bebop/odom',
            self.cb_odom,
            10
        )

        self.sub_z = self.create_subscription(
            Float64,
            '/bebop/altitude',
            self.cb_z,
            10
        )

        self.pub = self.create_publisher(
            Odometry,
            '/bebop/fullodom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(
            1.0 / 15.0,
            self.publish_odom
        )

    def cb_odom(self, msg):

        self.last_odom = msg

    def cb_z(self, msg):

        self.last_z = float(msg.data)

    def publish_odom(self):

        if self.last_odom is None:
            return

        if self.last_z is None:
            return

        now = self.get_clock().now()

        q = self.last_odom.pose.pose.orientation

        _, _, yaw_meas = quat_to_euler(q)

        z_meas = self.last_z

        if self.prev_stamp is None:

            dt = 1.0 / 15.0

        else:

            dt = (
                now - self.prev_stamp
            ).nanoseconds * 1e-9

            if dt <= 0.0:
                return

        z_filt, vz_filt = self.z_filter.update(
            z_meas,
            dt
        )

        yaw_filt, vyaw_filt = self.yaw_filter.update(
            yaw_meas,
            dt
        )

        self.vz   = vz_filt
        self.vyaw = vyaw_filt

        self.prev_stamp = now

        cy = math.cos(yaw_filt * 0.5)
        sy = math.sin(yaw_filt * 0.5)

        odom = Odometry()

        odom.header.stamp = self.last_odom.header.stamp

        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'bebop_link'

        odom.pose.pose.position.x = (
            self.last_odom.pose.pose.position.x
        )

        odom.pose.pose.position.y = (
            self.last_odom.pose.pose.position.y
        )

        odom.pose.pose.position.z = float(z_filt)

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.pose.covariance = [0.0] * 36

        odom.pose.covariance[0]  = 4e-4
        odom.pose.covariance[7]  = 4e-4
        odom.pose.covariance[14] = 2.5e-3

        odom.pose.covariance[21] = 2.5e-4
        odom.pose.covariance[28] = 2.5e-4
        odom.pose.covariance[35] = 2.5e-4

        odom.twist.twist.linear.x = (
            self.last_odom.twist.twist.linear.x
        )

        odom.twist.twist.linear.y = (
            self.last_odom.twist.twist.linear.y
        )

        odom.twist.twist.linear.z = float(self.vz)

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0

        odom.twist.twist.angular.z = float(self.vyaw)

        odom.twist.covariance = [0.0] * 36

        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[14] = 0.01
        odom.twist.covariance[35] = 0.01

        self.pub.publish(odom)

        tf = TransformStamped()

        tf.header.stamp = self.last_odom.header.stamp

        tf.header.frame_id = 'odom'

        tf.child_frame_id = 'bebop_link'

        tf.transform.translation.x = (
            self.last_odom.pose.pose.position.x
        )

        tf.transform.translation.y = (
            self.last_odom.pose.pose.position.y
        )

        tf.transform.translation.z = float(z_filt)

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = sy
        tf.transform.rotation.w = cy

        self.tf_broadcaster.sendTransform(tf)


def main():

    rclpy.init()

    node = SensorFusion()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()