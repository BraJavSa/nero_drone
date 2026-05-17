#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_to_euler(q):
    """Convert quaternion to (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for numerical safety
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def angle_diff(a, b):
    """
    Signed difference between two angles in radians, result in [-pi, pi].
    Avoids discontinuities at the ±pi boundary.
    """
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class PositionFusion(Node):
    def __init__(self):
        super().__init__('position_fusion')

        # ── Latest sensor data ──────────────────────────────────────────────
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_orientation = None   # geometry_msgs/Quaternion
        self.last_imu_stamp = None

        # ── Previous state for velocity computation ─────────────────────────
        self.prev_x     = None
        self.prev_y     = None
        self.prev_z     = None
        self.prev_yaw   = None
        self.prev_stamp = None         # rclpy.time.Time

        # ── Velocity Alpha-Beta Filter Parameters (Optimized from offline sysid) ──
        self.alpha_vel_x   = 0.9900
        self.beta_vel_x    = 0.0001
        self.alpha_vel_y   = 0.8963
        self.beta_vel_y    = 0.2252
        self.alpha_vel_z   = 0.3677
        self.beta_vel_z    = 0.2694
        self.alpha_vel_yaw = 0.3960
        self.beta_vel_yaw  = 0.3000

        # Filter states (xf: state, vf: derivative/acceleration rate)
        self.b_vx  = 0.0
        self.b_ax  = 0.0
        self.b_vy  = 0.0
        self.b_ay  = 0.0
        self.vz    = 0.0
        self.az    = 0.0
        self.vyaw  = 0.0
        self.ayaw  = 0.0
        self.filter_initialized = False

        self.sub_xy  = self.create_subscription(PointStamped, '/bebop/position', self.cb_xy,  10)
        self.sub_z   = self.create_subscription(Float64,      '/bebop/altitude', self.cb_z,   10)
        self.sub_imu = self.create_subscription(Imu,          '/bebop/imu',      self.cb_imu, 10)
        self.pub = self.create_publisher(Odometry, '/bebop/odom_ekf', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 10.0, self.publish_odom)


    def cb_xy(self, msg):
        self.last_x = msg.point.x
        self.last_y = msg.point.y

    def cb_z(self, msg):
        self.last_z = float(msg.data)

    def cb_imu(self, msg):
        q = msg.orientation
        corrected = type(q)()
        corrected.x = q.x
        corrected.y = q.y
        corrected.z = q.z
        corrected.w = q.w
        self.last_orientation = corrected
        self.last_imu_stamp = msg.header.stamp

    # ── Main publish loop ────────────────────────────────────────────────────

    def publish_odom(self):
        # Wait until all sources have arrived at least once
        if (self.last_x is None or self.last_y is None or self.last_z is None
                or self.last_orientation is None or self.last_imu_stamp is None):
            return

        now = self.get_clock().now()

        # ── Current yaw (rad) ────────────────────────────────────────────────
        _, _, yaw = quat_to_euler(self.last_orientation)

        # ── Raw position differentiation + velocity low-pass filter ──────────
        if self.prev_stamp is not None:
            dt = (now - self.prev_stamp).nanoseconds * 1e-9  # seconds

            if dt > 0.0:
                if self.prev_x is not None:
                    # World-frame velocity from position differentiation
                    w_vx = (self.last_x - self.prev_x) / dt
                    w_vy = (self.last_y - self.prev_y) / dt
                    vz_raw   = (self.last_z - self.prev_z) / dt

                    # Rotate world velocity to body frame: R_bw = R_wb^T
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    b_vx_raw =  cos_yaw * w_vx + sin_yaw * w_vy
                    b_vy_raw = -sin_yaw * w_vx + cos_yaw * w_vy

                    # Yaw rate with wrap-around handling
                    vyaw_raw = angle_diff(yaw, self.prev_yaw) / dt
                    # Apply 2nd-order Alpha-Beta filter in real-time
                    if not self.filter_initialized:
                        self.b_vx = b_vx_raw
                        self.b_ax = 0.0
                        self.b_vy = b_vy_raw
                        self.b_ay = 0.0
                        self.vz   = vz_raw
                        self.az   = 0.0
                        self.vyaw = vyaw_raw
                        self.ayaw = 0.0
                        self.filter_initialized = True
                    else:
                        # X-axis
                        pred_x = self.b_vx + dt * self.b_ax
                        pred_vx = self.b_ax
                        rk_x = b_vx_raw - pred_x
                        self.b_vx = pred_x + self.alpha_vel_x * rk_x
                        self.b_ax = pred_vx + (self.beta_vel_x / dt) * rk_x

                        # Y-axis
                        pred_y = self.b_vy + dt * self.b_ay
                        pred_vy = self.b_ay
                        rk_y = b_vy_raw - pred_y
                        self.b_vy = pred_y + self.alpha_vel_y * rk_y
                        self.b_ay = pred_vy + (self.beta_vel_y / dt) * rk_y

                        # Z-axis
                        pred_z = self.vz + dt * self.az
                        pred_vz = self.az
                        rk_z = vz_raw - pred_z
                        self.vz = pred_z + self.alpha_vel_z * rk_z
                        self.az = pred_vz + (self.beta_vel_z / dt) * rk_z

                        # Yaw (R) axis
                        pred_yaw = self.vyaw + dt * self.ayaw
                        pred_vyaw = self.ayaw
                        rk_yaw = vyaw_raw - pred_yaw
                        self.vyaw = pred_yaw + self.alpha_vel_yaw * rk_yaw
                        self.ayaw = pred_vyaw + (self.beta_vel_yaw / dt) * rk_yaw

        # ── Store state for next cycle ────────────────────────────────────────
        self.prev_x     = self.last_x
        self.prev_y     = self.last_y
        self.prev_z     = self.last_z
        self.prev_yaw   = yaw
        self.prev_stamp = now

        # ── Build Odometry message ────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = self.last_imu_stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link_ekf'

        # Position (world frame)
        odom.pose.pose.position.x  = self.last_x
        odom.pose.pose.position.y  = self.last_y
        odom.pose.pose.position.z  = self.last_z
        odom.pose.pose.orientation = self.last_orientation

        # Pose covariance [x, y, z, roll, pitch, yaw] diagonal
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0]  = 4e-4    # x
        odom.pose.covariance[7]  = 4e-4    # y
        odom.pose.covariance[14] = 2.5e-3  # z
        odom.pose.covariance[21] = 2.5e-4  # roll
        odom.pose.covariance[28] = 2.5e-4  # pitch
        odom.pose.covariance[35] = 2.5e-4  # yaw

        # Twist — raw body-frame velocities (no filter)
        odom.twist.twist.linear.x  = float(self.b_vx)
        odom.twist.twist.linear.y  = float(self.b_vy)
        odom.twist.twist.linear.z  = float(self.vz)
        odom.twist.twist.angular.x = 0.0   # not used
        odom.twist.twist.angular.y = 0.0   # not used
        odom.twist.twist.angular.z = float(self.vyaw)  # rad/s

        # Twist covariance — fixed values (no Kalman)
        odom.twist.covariance = [0.0] * 36
        odom.twist.covariance[0]  = 0.01   # vx
        odom.twist.covariance[7]  = 0.01   # vy
        odom.twist.covariance[14] = 0.01   # vz
        odom.twist.covariance[35] = 0.01   # vyaw

        self.pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp            = self.last_imu_stamp
        tf.header.frame_id         = 'odom'
        tf.child_frame_id          = 'base_link_ekf'
        tf.transform.translation.x = self.last_x
        tf.transform.translation.y = self.last_y
        tf.transform.translation.z = self.last_z
        tf.transform.rotation      = self.last_orientation

        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = PositionFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()