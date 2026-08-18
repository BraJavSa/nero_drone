#!/usr/bin/env python3
"""
Ground Truth Position Reference Publisher for Bebop drone in 'world' frame.
Publishes static or step-based reference positions for setpoint regulation testing.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import time


class GtRefPublisher(Node):
    def __init__(self):
        super().__init__("gt_position_ref_publisher")
        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.tf_br = TransformBroadcaster(self)
        self.dt = 1.0 / 15.0
        self.hold_time = 10.0
        self.start_time = time.time()
        self.t0 = time.time()
        self.idx = 0
        L = 1.5

        # Target setpoints in OptiTrack 'world' frame within [-1.8, 1.8] m (X,Y) and [1.5, 2.3] m (Z)
        self.points = np.array([
            [0.0, 0.0, 1.8],
            [-1.5, -1.5, 1.6],
            [1.5, -1.5, 2.2],
            [-1.5, 1.5, 1.7],
            [1.5, 1.5, 2.3]
        ])
        self.yaws = np.deg2rad([0, 45, -45, 90, -90])
        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info("Publishing GT reference setpoints in 'world' frame (Z in [1.5m, 2.3m]).")

    def timer_cb(self):
        total_time = time.time() - self.start_time
        if total_time >= 120.0:
            self.get_logger().info("Experiment completed after 120 seconds. Shutting down position reference publisher.")
            raise SystemExit

        elapsed = time.time() - self.t0
        if elapsed > self.hold_time:
            self.idx += 1
            self.t0 = time.time()
            if self.idx >= len(self.points):
                self.idx = 0
                self.get_logger().info("Restarting GT reference sequence.")

        pos_o = self.points[self.idx]
        yaw_o = self.yaws[self.idx]

        msg = Float64MultiArray()
        msg.data = [
            float(pos_o[0]), float(pos_o[1]), float(pos_o[2]), float(yaw_o),
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
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
    node = GtRefPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
