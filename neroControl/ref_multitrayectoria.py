#!/usr/bin/env python3
# Trajectory reference publisher for Bebop drone (lemniscate, spiral, transverse)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import time

class TrajectoryBase:
    def position(self, t):
        raise NotImplementedError

    @property
    def duration(self):
        raise NotImplementedError

    def velocity(self, t, dt):
        p_next = self.position(min(t + dt, self.duration))
        p_prev = self.position(max(t - dt, 0.0))
        return (p_next - p_prev) / (2.0 * dt)

class LemniscateTrajectory(TrajectoryBase):
    def __init__(self):
        self.w           = 0.23
        self.amplitude_x = 1.5
        self.amplitude_y = 1.5
        self.z_base      = 1.5
        self.z_amp       = 0.6
        self._duration   = 2.0 * np.pi / self.w

    @property
    def duration(self):
        return self._duration

    def position(self, t):
        a = self.w * t
        x = self.amplitude_x * np.sin(a)
        y = self.amplitude_y * np.sin(a) * np.cos(a)
        z = self.z_base + self.z_amp * np.sin(2.0 * a)
        return np.array([x, y, z])

class SpiralTrajectory(TrajectoryBase):
    def __init__(self):
        self.w         = 0.41
        self.radius    = 1.2
        self.z_base    = 1.0
        self.z_top     = 2.2
        self._duration = 2.0 * np.pi / self.w

    @property
    def duration(self):
        return self._duration

    def position(self, t):
        alpha = self.w * t
        x = self.radius * np.cos(alpha)
        y = self.radius * np.sin(alpha)
        z = self.z_base + (self.z_top - self.z_base) * np.sin(alpha)
        return np.array([x, y, z])

class TransverseVelocityTrajectory(TrajectoryBase):
    def __init__(self):
        self.vx_fwd    = 0.30
        self.length    = 9.0
        self.y_amp     = 0.80
        self.y_freq    = 0.10
        self.z_base    = 1.5
        self.z_amp     = 0.40
        self.z_freq    = 0.10
        self._duration = self.length / self.vx_fwd

    @property
    def duration(self):
        return self._duration

    def position(self, t):
        x = self.vx_fwd * t
        y = self.y_amp * np.sin(2.0 * np.pi * self.y_freq * t)
        z = self.z_base + self.z_amp * np.sin(2.0 * np.pi * self.z_freq * t)
        return np.array([x, y, z])

TRAJECTORIES = {
    0: ("3D Lemniscate (figure-8)",           LemniscateTrajectory),
    1: ("Ascending/Descending Spiral",      SpiralTrajectory),
    2: ("Transverse Body Velocities", TransverseVelocityTrajectory),
}

V_MAX    = 0.5
WYAW_MAX = 0.75

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("trajectory_ref_publisher")
        self.declare_parameter("trajectory", 2)
        traj_id = self.get_parameter("trajectory").get_parameter_value().integer_value
        if traj_id not in TRAJECTORIES:
            self.get_logger().error(f"Trajectory {traj_id} does not exist. Options: {list(TRAJECTORIES.keys())}")
            raise ValueError(f"Invalid trajectory: {traj_id}")
        name, TrajClass = TRAJECTORIES[traj_id]
        self.trajectory = TrajClass()
        self.get_logger().info(f"[Trajectory {traj_id}] {name}")
        self.get_logger().info(f"  Duration: {self.trajectory.duration:.1f} s")
        self.get_logger().info(f"  Limits: |v_body| <= {V_MAX:.2f} m/s  |wyaw| <= {WYAW_MAX:.2f} rad/s")
        self.pub_ref = self.create_publisher(Float64MultiArray, "/bebop/ref_vec", 10)
        self.tf_br   = TransformBroadcaster(self)
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dt         = 1.0 / 30.0
        self.start_time = time.time()
        self.finished   = False
        self.first_sample       = True
        self.last_raw_yaw       = 0.0
        self.last_unwrapped_yaw = 0.0
        self.timer = self.create_timer(self.dt, self.timer_cb)

    def timer_cb(self):
        if self.finished:
            return
        t = time.time() - self.start_time
        if t >= self.trajectory.duration:
            self.get_logger().info("Trajectory completed.")
            self.timer.cancel()
            self.finished = True
            return
        pos_i = self.trajectory.position(t)
        vel_i = self.trajectory.velocity(t, self.dt)
        vel_i = np.clip(vel_i, -V_MAX, V_MAX)
        current_raw_yaw = np.arctan2(vel_i[1], vel_i[0])
        if self.first_sample:
            self.last_raw_yaw       = current_raw_yaw
            self.last_unwrapped_yaw = current_raw_yaw
            self.first_sample       = False
        delta_yaw             = current_raw_yaw - self.last_raw_yaw
        delta_yaw             = (delta_yaw + np.pi) % (2.0 * np.pi) - np.pi
        current_unwrapped_yaw = self.last_unwrapped_yaw + delta_yaw
        wyaw = float(np.clip(delta_yaw / self.dt, -WYAW_MAX, WYAW_MAX))
        self.last_raw_yaw       = current_raw_yaw
        self.last_unwrapped_yaw = current_unwrapped_yaw
        try:
            tf_oi = self.tf_buffer.lookup_transform("odom", "initial_frame", rclpy.time.Time())
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
        yaw_o = current_unwrapped_yaw + base_yaw
        msg      = Float64MultiArray()
        msg.data = [
            float(pos_o[0]), float(pos_o[1]), float(pos_o[2]), float(yaw_o),
            float(vel_o[0]), float(vel_o[1]), float(vel_o[2]), wyaw,
        ]
        self.pub_ref.publish(msg)
        qz_ref = np.sin(yaw_o * 0.5)
        qw_ref = np.cos(yaw_o * 0.5)
        tmsg                         = TransformStamped()
        tmsg.header.stamp            = self.get_clock().now().to_msg()
        tmsg.header.frame_id         = "odom"
        tmsg.child_frame_id          = "ref"
        tmsg.transform.translation.x = float(pos_o[0])
        tmsg.transform.translation.y = float(pos_o[1])
        tmsg.transform.translation.z = float(pos_o[2])
        tmsg.transform.rotation.z    = float(qz_ref)
        tmsg.transform.rotation.w    = float(qw_ref)
        self.tf_br.sendTransform(tmsg)

    def quaternion_to_matrix(self, x, y, z, w):
        R_mat = np.zeros((3, 3))
        R_mat[0, 0] = 1 - 2*(y*y + z*z)
        R_mat[0, 1] = 2*(x*y - z*w)
        R_mat[0, 2] = 2*(x*z + y*w)
        R_mat[1, 0] = 2*(x*y + z*w)
        R_mat[1, 1] = 1 - 2*(x*x + z*z)
        R_mat[1, 2] = 2*(y*z - x*w)
        R_mat[2, 0] = 2*(x*z - y*w)
        R_mat[2, 1] = 2*(y*z + x*w)
        R_mat[2, 2] = 1 - 2*(x*x + y*y)
        return R_mat

    def quaternion_yaw(self, x, y, z, w):
        return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

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