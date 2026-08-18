#!/usr/bin/env python3
"""
Ground Truth Trajectory Reference Publisher for Bebop drone in 'world' frame.
Generates and publishes continuous time-parameterized trajectories in OptiTrack 'world' coordinates.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time


class GtTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("gt_trajectory_ref_publisher")

        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.tf_br = TransformBroadcaster(self)

        self.dt = 1.0 / 15.0
        self.w = 0.1

        self.amplitude_x = 1.8
        self.amplitude_y = 1.8
        self.z_base = 2.0
        self.z_amp = 0.4

        self.start_time = time.time()
        self.first_sample = True
        self.last_raw_yaw = 0.0
        self.last_unwrapped_yaw = 0.0

        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info("Publishing continuous GT trajectory reference in 'world' frame.")

    def timer_cb(self):
        t = time.time() - self.start_time
        if t >= 120.0:
            self.get_logger().info("Experiment completed after 120 seconds. Shutting down trajectory reference publisher.")
            raise SystemExit

        # Trajectory directly in 'world' frame
        x = self.amplitude_x * np.sin(self.w * t)
        y = self.amplitude_y * np.sin(self.w * t) * np.cos(self.w * t)
        z = self.z_base + self.z_amp * np.sin(0.5 * self.w * t)
        pos_o = np.array([x, y, z])

        dx = self.amplitude_x * self.w * np.cos(self.w * t)
        dy = self.amplitude_y * self.w * (np.cos(self.w * t)**2 - np.sin(self.w * t)**2)
        dz = self.z_amp * 0.5 * self.w * np.cos(0.5 * self.w * t)
        vel_o = np.array([dx, dy, dz])

        ddx = -self.amplitude_x * (self.w**2) * np.sin(self.w * t)
        ddy = -4.0 * self.amplitude_y * (self.w**2) * np.sin(self.w * t) * np.cos(self.w * t)
        ddz = -self.z_amp * (0.5 * self.w)**2 * np.sin(0.5 * self.w * t)
        acc_o = np.array([ddx, ddy, ddz])

        current_raw_yaw = np.arctan2(dy, dx)

        if self.first_sample:
            self.last_raw_yaw = current_raw_yaw
            self.first_sample = False

        delta_yaw = current_raw_yaw - self.last_raw_yaw
        delta_yaw = (delta_yaw + np.pi) % (2 * np.pi) - np.pi
        wyaw = delta_yaw / self.dt

        self.last_raw_yaw = current_raw_yaw
        yaw_o = np.arctan2(np.sin(current_raw_yaw), np.cos(current_raw_yaw))

        msg = Float64MultiArray()
        msg.data = [
            float(pos_o[0]), float(pos_o[1]), float(pos_o[2]), float(yaw_o),
            float(vel_o[0]), float(vel_o[1]), float(vel_o[2]), float(wyaw),
            float(acc_o[0]), float(acc_o[1]), float(acc_o[2]), 0.0
        ]
        self.pub_ref.publish(msg)

        qz_ref = np.sin(yaw_o * 0.5)
        qw_ref = np.cos(yaw_o * 0.5)

        tmsg = TransformStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = "world"
        tmsg.child_frame_id = "ref"

        tmsg.transform.translation.x = float(pos_o[0])
        tmsg.transform.translation.y = float(pos_o[1])
        tmsg.transform.translation.z = float(pos_o[2])
        tmsg.transform.rotation.z = float(qz_ref)
        tmsg.transform.rotation.w = float(qw_ref)

        self.tf_br.sendTransform(tmsg)


def main():
    rclpy.init()
    node = GtTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
