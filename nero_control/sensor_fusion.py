#!/usr/bin/env python3

# Sensor fusion node using a causal moving-average filter on position (Z and Yaw),
# then computing velocities via finite difference.
# Filter parameters (W=5) were optimised offline:
#   Z   → FIT 82.97 %,  RMSE 0.034 m,   lag 0.133 s
#   Yaw → FIT 98.47 %,  RMSE 0.041 rad, lag 0.133 s

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
    roll      = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw       = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def angle_diff(a, b):
    """Wrap-aware angular difference (a - b) ∈ (-π, π]."""
    d = a - b
    while d >  math.pi: d -= 2.0 * math.pi
    while d < -math.pi: d += 2.0 * math.pi
    return d


class CausalMovingAverage:
    """
    Causal moving-average filter on a scalar signal.

    For sample index k the output is the mean of the last min(k+1, W) samples,
    so no future information is ever used.

    Optionally handles angular wrap-around (is_angle=True) by computing the
    mean in the complex-exponential domain and unwrapping the result.
    """

    def __init__(self, window_size: int, is_angle: bool = False):
        self.W        = window_size
        self.is_angle = is_angle
        self.buffer   = []          # stores the last W raw measurements

    def update(self, measurement: float) -> float:

        self.buffer.append(measurement)

        if len(self.buffer) > self.W:
            self.buffer.pop(0)

        if self.is_angle:
            # Mean angle via unit-complex averaging → avoids wrap artefacts
            sin_sum = sum(math.sin(v) for v in self.buffer)
            cos_sum = sum(math.cos(v) for v in self.buffer)
            return math.atan2(sin_sum, cos_sum)

        return sum(self.buffer) / len(self.buffer)


class SensorFusion(Node):

    def __init__(self):

        super().__init__('sensor_fusion')

        # ── last raw measurements ────────────────────────────────────────────
        self.last_odom = None
        self.last_z    = None
        self.prev_stamp = None

        # ── causal position filters (W = 5, from offline optimisation) ───────
        self.z_filter   = CausalMovingAverage(window_size=5, is_angle=False)
        self.yaw_filter = CausalMovingAverage(window_size=5, is_angle=True)

        # ── previous filtered positions (needed for finite-difference vel.) ──
        self.z_prev   = None
        self.yaw_prev = None

        # ── published velocities ─────────────────────────────────────────────
        self.vz   = 0.0
        self.vyaw = 0.0

        # ── ROS I/O ──────────────────────────────────────────────────────────
        self.sub_odom = self.create_subscription(
            Odometry, '/bebop/odom', self.cb_odom, 10)

        self.sub_z = self.create_subscription(
            Float64, '/bebop/altitude', self.cb_z, 10)

        self.pub = self.create_publisher(
            Odometry, '/bebop/fullodom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / 15.0, self.publish_odom)

    # ── callbacks ────────────────────────────────────────────────────────────

    def cb_odom(self, msg):
        self.last_odom = msg

    def cb_z(self, msg):
        self.last_z = float(msg.data)

    # ── main loop ────────────────────────────────────────────────────────────

    def publish_odom(self):

        if self.last_odom is None or self.last_z is None:
            return

        now = self.get_clock().now()

        # ── dt ───────────────────────────────────────────────────────────────
        if self.prev_stamp is None:
            dt = 1.0 / 15.0
        else:
            dt = (now - self.prev_stamp).nanoseconds * 1e-9
            if dt <= 0.0:
                return

        self.prev_stamp = now

        # ── raw measurements ─────────────────────────────────────────────────
        _, _, yaw_meas = quat_to_euler(self.last_odom.pose.pose.orientation)
        z_meas = self.last_z

        # ── causal filter on position ────────────────────────────────────────
        z_filt   = self.z_filter.update(z_meas)
        yaw_filt = self.yaw_filter.update(yaw_meas)

        # ── velocity via finite difference of filtered position ───────────────
        if self.z_prev is None:
            # First sample — velocities are zero
            self.vz   = 0.0
            self.vyaw = 0.0
        else:
            self.vz   = (z_filt - self.z_prev) / dt
            self.vyaw = angle_diff(yaw_filt, self.yaw_prev) / dt

        self.z_prev   = z_filt
        self.yaw_prev = yaw_filt

        # ── build quaternion from filtered yaw only (roll = pitch = 0) ───────
        cy = math.cos(yaw_filt * 0.5)
        sy = math.sin(yaw_filt * 0.5)

        # ── publish Odometry ─────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = self.last_odom.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'bebop_link'

        # Filtered position
        odom.pose.pose.position.x = self.last_odom.pose.pose.position.x
        odom.pose.pose.position.y = self.last_odom.pose.pose.position.y
        odom.pose.pose.position.z = float(z_filt)

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy

        odom.pose.covariance     = [0.0] * 36
        odom.pose.covariance[0]  = 4e-4
        odom.pose.covariance[7]  = 4e-4
        odom.pose.covariance[14] = 2.5e-3
        odom.pose.covariance[21] = 2.5e-4
        odom.pose.covariance[28] = 2.5e-4
        odom.pose.covariance[35] = 2.5e-4

        # Velocities derived from filtered position
        odom.twist.twist.linear.x  = self.last_odom.twist.twist.linear.x
        odom.twist.twist.linear.y  = self.last_odom.twist.twist.linear.y
        odom.twist.twist.linear.z  = float(self.vz)
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(self.vyaw)

        odom.twist.covariance     = [0.0] * 36
        odom.twist.covariance[0]  = 0.01
        odom.twist.covariance[7]  = 0.01
        odom.twist.covariance[14] = 0.01
        odom.twist.covariance[35] = 0.01

        self.pub.publish(odom)

        # ── broadcast TF ─────────────────────────────────────────────────────
        tf = TransformStamped()
        tf.header.stamp      = self.last_odom.header.stamp
        tf.header.frame_id   = 'odom'
        tf.child_frame_id    = 'bebop_link'

        tf.transform.translation.x = self.last_odom.pose.pose.position.x
        tf.transform.translation.y = self.last_odom.pose.pose.position.y
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