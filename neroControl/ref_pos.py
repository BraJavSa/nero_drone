#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import time

class RefPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_ref_publisher")

        # Publishers
        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.tf_br = TransformBroadcaster(self)

        # TF buffer/listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Parameters
        self.dt = 1.0 / 30.0     # 30 Hz
        self.hold_time = 10.0    # Reduced to 10 seconds per pose
        self.t0 = time.time()
        self.idx = 0

        L = 1.6
        self.points = np.array([
            [L/2, L/2, 1.5],        # Punto 1
            [-L/2, L/2, 1.5],       # Punto 2
            [-L/2, -L/2, 1.5],      # Punto 3
            [0.0, 0.0, 1.5]         # Retorno al Centro
        ])

        # Yaw in radians
        self.yaws = np.deg2rad([45, 135, -135, 0])

        # Timer
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info("Publishing infinite reference trajectory (10s per pose).")

    def timer_cb(self):
        elapsed = time.time() - self.t0

        # Logic for infinite sequence
        if elapsed > self.hold_time:
            self.idx += 1
            self.t0 = time.time() # Reset reference time for the next point
            
            # Reset index to 0 if sequence ends (Infinite Loop)
            if self.idx >= len(self.points):
                self.idx = 0
                self.get_logger().info("Restarting reference sequence.")

        # Current pose in INITIAL_FRAME
        pos_i = self.points[self.idx]
        yaw_i = self.yaws[self.idx]

        # Get transform: odom -> initial_frame
        try:
            tf_oi = self.tf_buffer.lookup_transform(
                "odom", "initial_frame", rclpy.time.Time()
            )
        except Exception:
            self.get_logger().warn("Waiting for TF odom -> initial_frame", throttle_duration_sec=5.0)
            return

        # Extract translation and rotation
        tx = tf_oi.transform.translation.x
        ty = tf_oi.transform.translation.y
        tz = tf_oi.transform.translation.z

        qx = tf_oi.transform.rotation.x
        qy = tf_oi.transform.rotation.y
        qz = tf_oi.transform.rotation.z
        qw = tf_oi.transform.rotation.w

        # Rotation matrix
        R = self.quaternion_to_matrix(qx, qy, qz, qw)

        # Transform pos_i → pos_o
        pos_o = R.dot(pos_i) + np.array([tx, ty, tz])

        # Transform yaw
        yaw_o = yaw_i + self.quaternion_yaw(qx, qy, qz, qw)

        # Publish ref vector
        msg = Float64MultiArray()
        msg.data = [
            float(pos_o[0]), float(pos_o[1]), float(pos_o[2]), float(yaw_o),
            0.0, 0.0, 0.0, 0.0
        ]
        self.pub_ref.publish(msg)

        # Publish TF odom -> ref
        qz_ref = np.sin(yaw_o * 0.5)
        qw_ref = np.cos(yaw_o * 0.5)

        tmsg = TransformStamped()
        tmsg.header.stamp = self.get_clock().now().to_msg()
        tmsg.header.frame_id = "odom"
        tmsg.child_frame_id = "ref"
        tmsg.transform.translation.x = float(pos_o[0])
        tmsg.transform.translation.y = float(pos_o[1])
        tmsg.transform.translation.z = float(pos_o[2])
        tmsg.transform.rotation.z = qz_ref
        tmsg.transform.rotation.w = qw_ref

        self.tf_br.sendTransform(tmsg)

    def quaternion_to_matrix(self, x, y, z, w):
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2*(y*y + z*z)
        R[0, 1] = 2*(x*y - z*w)
        R[0, 2] = 2*(x*z + y*w)
        R[1, 0] = 2*(x*y + z*w)
        R[1, 1] = 1 - 2*(x*x + z*z)
        R[1, 2] = 2*(y*z - x*w)
        R[2, 0] = 2*(x*z - y*w)
        R[2, 1] = 2*(y*z + x*w)
        R[2, 2] = 1 - 2*(x*x + y*y)
        return R

    def quaternion_yaw(self, x, y, z, w):
        return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def main():
    rclpy.init()
    node = RefPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()