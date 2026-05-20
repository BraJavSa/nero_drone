#!/usr/bin/env python3

# Logs drone odometry, control inputs, and state variables for offline system identification.

import os
import signal
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation
from scipy.io import savemat
import numpy as np
from datetime import datetime


class ManualLoggerNode(Node):
    def __init__(self):
        super().__init__("manual_logger_node")

        qos_odom = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self._cmd_sub = self.create_subscription(
            Twist, "/bebop/cmd_vel", self._cmd_cb, 10)

        self._odom_sub = self.create_subscription(
            Odometry, "/bebop/fullodom", self._odom_cb, qos_odom)

        self._lock = threading.Lock()
        self._u_current = np.zeros(4)
        self._pos_i = np.zeros(4)
        self._vel_b = np.zeros(4)
        self._odom_ready = False

        self._data = []
        self._tick_count = 0

        self._ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.path.expanduser("~/ros2_ws/src/neroControl/data")
        os.makedirs(base, exist_ok=True)
        self._out_mat = os.path.join(base, f"manual_log_{self._ts}.mat")

        self._hz = 15.0
        self._timer = self.create_timer(1.0 / self._hz, self._logger_timer_cb)

        self.get_logger().info("Manual recording mode active.")
        self.get_logger().info(f"Will save to: {self._out_mat}")

        signal.signal(signal.SIGINT, self._signal_handler)

    def _cmd_cb(self, msg):
        with self._lock:
            self._u_current = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.z
            ])

    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")[2]

        with self._lock:
            self._pos_i = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
                yaw
            ])

            self._vel_b = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
                msg.twist.twist.angular.z
            ])

            self._odom_ready = True

    def _logger_timer_cb(self):
        if not self._odom_ready:
            return

        with self._lock:
            u_now   = self._u_current.copy()
            pos_now = self._pos_i.copy()
            vel_now = self._vel_b.copy()

        self._data.append({
            "K":   self._tick_count,
            "u":   u_now,
            "pos": pos_now,
            "vel": vel_now
        })
        self._tick_count += 1

        self.get_logger().info(
            f"Recording... {len(self._data)} samples",
            throttle_duration_sec=3.0
        )

    def _save_mat(self):
        if not self._data:
            self.get_logger().warn("No data to save.")
            return

        try:
            K = np.array([d["K"] for d in self._data])
            u = np.array([d["u"] for d in self._data])
            p = np.array([d["pos"] for d in self._data])
            v = np.array([d["vel"] for d in self._data])

            savemat(self._out_mat, {
                "K":    K,
                "hz":   self._hz,
                "u":    u,
                "x_i":  p[:, 0],
                "y_i":  p[:, 1],
                "z_i":  p[:, 2],
                "psi":  p[:, 3],
                "vx_b": v[:, 0],
                "vy_b": v[:, 1],
                "vz_b": v[:, 2],
                "vpsi": v[:, 3]
            })
            self.get_logger().info(f"File saved: {self._out_mat}")
        except Exception as e:
            self.get_logger().error(f"Error saving: {e}")

    def _signal_handler(self, signum, frame):
        self.get_logger().info("SIGINT detected. Saving...")
        self._save_mat()
        sys.exit(0)


def main():
    rclpy.init()
    node = ManualLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._save_mat()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()