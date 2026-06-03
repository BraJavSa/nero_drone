#!/usr/bin/env python3
# Generates and publishes continuous time-parameterized trajectories.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import time

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_ref_publisher")

        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)

        self.tf_br = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.dt = 1.0 / 15.0
        self.w = 0.1

        self.amplitude_x = 1.5
        self.amplitude_y = 1.5

        self.z_base = 1.5
        self.z_amp = 0.5

        self.start_time = time.time()

        self.first_sample = True
        self.last_raw_yaw = 0.0
        self.last_unwrapped_yaw = 0.0

        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        t = time.time() - self.start_time

        x = self.amplitude_x * np.sin(self.w * t)
        y = self.amplitude_y * np.sin(self.w * t) * np.cos(self.w * t)
        z = self.z_base + self.z_amp * np.sin(0.5 * self.w * t)

        pos_i = np.array([x, y, z])

        dx = self.amplitude_x * self.w * np.cos(self.w * t)

        dy = self.amplitude_y * self.w * (
            np.cos(self.w * t)**2 - np.sin(self.w * t)**2
        )

        dz = self.z_amp * 0.5 * self.w * np.cos(0.5 * self.w * t)

        vel_i = np.array([dx, dy, dz])

        ddx = -self.amplitude_x * (self.w**2) * np.sin(self.w * t)

        ddy = -4.0 * self.amplitude_y * (self.w**2) * \
              np.sin(self.w * t) * np.cos(self.w * t)

        ddz = -self.z_amp * (0.5 * self.w)**2 * np.sin(0.5 * self.w * t)

        acc_i = np.array([ddx, ddy, ddz])

        current_raw_yaw = np.arctan2(dy, dx)

        if self.first_sample:
            self.last_raw_yaw = current_raw_yaw
            self.last_unwrapped_yaw = current_raw_yaw
            self.first_sample = False

        delta_yaw = current_raw_yaw - self.last_raw_yaw
        delta_yaw = (delta_yaw + np.pi) % (2 * np.pi) - np.pi

        current_unwrapped_yaw = self.last_unwrapped_yaw + delta_yaw

        wyaw = (current_unwrapped_yaw - self.last_unwrapped_yaw) / self.dt

        self.last_raw_yaw = current_raw_yaw
        self.last_unwrapped_yaw = current_unwrapped_yaw

        try:
            tf_oi = self.tf_buffer.lookup_transform(
                "odom",
                "initial_frame",
                rclpy.time.Time()
            )
        except Exception:
            return

        tx = tf_oi.transform.translation.x
        ty = tf_oi.transform.translation.y
        tz = tf_oi.transform.translation.z

        qx = tf_oi.transform.rotation.x
        qy = tf_oi.transform.rotation.y
        qz = tf_oi.transform.rotation.z
        qw = tf_oi.transform.rotation.w

        R_mat = self.quaternion_to_matrix(qx, qy, qz, qw)

        base_yaw = self.quaternion_yaw(qx, qy, qz, qw)

        pos_o = R_mat.dot(pos_i) + np.array([tx, ty, tz])
        vel_o = R_mat.dot(vel_i)
        acc_o = R_mat.dot(acc_i)

        yaw_o = current_unwrapped_yaw + base_yaw

        msg = Float64MultiArray()

        msg.data = [
            float(pos_o[0]),
            float(pos_o[1]),
            float(pos_o[2]),
            float(yaw_o),

            float(vel_o[0]),
            float(vel_o[1]),
            float(vel_o[2]),
            float(wyaw),

            float(acc_o[0]),
            float(acc_o[1]),
            float(acc_o[2]),
            0.0
        ]

        self.pub_ref.publish(msg)

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
        R_mat = np.zeros((3, 3))

        R_mat[0, 0] = 1 - 2 * (y * y + z * z)
        R_mat[0, 1] = 2 * (x * y - z * w)
        R_mat[0, 2] = 2 * (x * z + y * w)

        R_mat[1, 0] = 2 * (x * y + z * w)
        R_mat[1, 1] = 1 - 2 * (x * x + z * z)
        R_mat[1, 2] = 2 * (y * z - x * w)

        R_mat[2, 0] = 2 * (x * z - y * w)
        R_mat[2, 1] = 2 * (y * z + x * w)
        R_mat[2, 2] = 1 - 2 * (x * x + y * y)

        return R_mat

    def quaternion_yaw(self, x, y, z, w):
        return np.arctan2(
            2 * (w * z + x * y),
            1 - 2 * (y * y + z * z)
        )

def main():
    rclpy.init()

    node = TrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()