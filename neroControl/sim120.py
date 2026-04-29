#!/usr/bin/env python3
# UAV flight simulator running at 120 Hz with basic dynamics and state machine
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PointStamped
from std_msgs.msg import Empty, Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class DroneSimulator(Node):
    def __init__(self):
        super().__init__("bebop_sim")
        self.dt = 1.0 / 120.0
        self.tick = 0
        self.x = np.zeros(4)
        self.xdot = np.zeros(4)
        self.u = np.zeros(4)
        self.model_simp = np.array([
            0.8417, 0.18227,
            0.8354, 0.17095,
            3.966,  4.001,
            9.8524, 4.7295
        ])
        self.Ku = np.diag([
            self.model_simp[0],
            self.model_simp[2],
            self.model_simp[4],
            self.model_simp[6],
        ])
        self.Kv = np.diag([
            self.model_simp[1],
            self.model_simp[3],
            self.model_simp[5],
            self.model_simp[7],
        ])
        self.dt_dyn = 1.0 / 120.0
        self.mode = "IDLE"
        self.z_target_takeoff = 1.2
        self.v_takeoff = 0.5
        self.v_land = -0.5
        self.v_emergency = -1.0
        self.last_cmd = None
        self.cmd_counter = 0
        self.max_tilt = math.radians(5.0)
        self.pub_xy = self.create_publisher(PointStamped, "/bebop/position", 10)
        self.pub_z = self.create_publisher(Float64, "/bebop/altitude", 10)
        self.pub_imu = self.create_publisher(Imu, "/bebop/imu", 10)
        self.pub_odom = self.create_publisher(Odometry, "/bebop/odom", 10)
        self.sub_cmd = self.create_subscription(Twist, "/bebop/cmd_vel", self.cmd_cb, 10)
        self.sub_takeoff = self.create_subscription(Empty, "/bebop/takeoff", self.takeoff_cb, 10)
        self.sub_land = self.create_subscription(Empty, "/bebop/land", self.land_cb, 10)
        self.sub_emergency = self.create_subscription(Empty, "/bebop/emergency", self.emergency_cb, 10)
        self.sub_reset = self.create_subscription(Empty, "/bebop/reset", self.reset_cb, 10)
        self.tf_br = TransformBroadcaster(self)
        self.timer = self.create_timer(self.dt, self.timer_cb)
        self.get_logger().info("Bebop simulator running at 120 Hz")

    def cmd_cb(self, msg: Twist):
        self.last_cmd = msg
        self.cmd_counter = 0

    def takeoff_cb(self, _):
        if self.mode in ["IDLE", "LANDING"]:
            self.get_logger().info("Takeoff received — Switching to TAKING_OFF")
            self.mode = "TAKING_OFF"
            self.last_cmd = None
            self.u[:] = 0.0

    def land_cb(self, _):
        if self.mode in ["FLYING", "TAKING_OFF"]:
            self.get_logger().info("Land received — Switching to LANDING")
            self.mode = "LANDING"
            self.last_cmd = None
            self.u[:] = 0.0

    def emergency_cb(self, _):
        self.get_logger().warn("Emergency received — Switching to EMERGENCY")
        self.mode = "EMERGENCY"
        self.last_cmd = None
        self.u[:] = 0.0

    def reset_cb(self, _):
        self.get_logger().warn("Reset received — Emergency-like descent")
        self.mode = "EMERGENCY"
        self.last_cmd = None
        self.u[:] = 0.0

    def update_vertical_control(self):
        if self.mode == "TAKING_OFF":
            self.x[2] += self.v_takeoff * self.dt
            if self.x[2] >= self.z_target_takeoff:
                self.x[2] = self.z_target_takeoff
                self.xdot[:] = 0.0
                self.mode = "FLYING"
                self.get_logger().info("Reached takeoff altitude — FLYING")
        elif self.mode == "LANDING":
            self.x[2] += self.v_land * self.dt
            if self.x[2] <= 0.0:
                self.x[2] = 0.0
                self.xdot[:] = 0.0
                self.mode = "IDLE"
                self.get_logger().info("Landed — IDLE")
        elif self.mode == "EMERGENCY":
            self.x[2] += self.v_emergency * self.dt
            if self.x[2] <= 0.0:
                self.x[2] = 0.0
                self.xdot[:] = 0.0
                self.mode = "IDLE"
                self.get_logger().warn("Emergency descent complete — IDLE")
        elif self.mode == "IDLE":
            if self.x[2] < 0.0:
                self.x[2] = 0.0
                self.xdot[:] = 0.0

    def dynamics_step(self):
        yaw = self.x[3]
        c, s = math.cos(yaw), math.sin(yaw)
        F_mat = np.array([[c, -s, 0, 0], [s,  c, 0, 0], [0,  0, 1, 0], [0,  0, 0, 1]])
        xddot = F_mat @ (self.Ku @ self.u) - self.Kv @ self.xdot
        self.xdot += xddot * self.dt_dyn
        self.x += self.xdot * self.dt_dyn
        if self.x[2] < 0.0:
            self.x[2] = 0.0
            self.xdot[2] = 0.0

    def publish_xy(self):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.point.x = float(self.x[0])
        msg.point.y = float(self.x[1])
        self.pub_xy.publish(msg)

    def publish_z(self):
        msg = Float64()
        msg.data = float(self.x[2])
        self.pub_z.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        roll = float(self.u[1]) * self.max_tilt
        pitch = float(self.u[0]) * self.max_tilt
        yaw = self.x[3]
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = qx
        msg.orientation.y = -qy
        msg.orientation.z = -qz
        msg.orientation.w = qw
        self.pub_imu.publish(msg)

    def publish_tf(self):
        x, y, z, yaw = self.x
        roll = float(self.u[1]) * self.max_tilt
        pitch = float(self.u[0]) * self.max_tilt
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)

    def publish_odom(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        x, y, z, yaw = self.x
        vx, vy, vz, wz = self.xdot
        roll = float(self.u[1]) * self.max_tilt
        pitch = float(self.u[0]) * self.max_tilt
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.linear.z = float(vz)
        msg.twist.twist.angular.z = float(wz)
        self.pub_odom.publish(msg)

    def timer_cb(self):
        self.tick += 1
        self.cmd_counter += 1
        self.update_vertical_control()
        if self.mode == "FLYING" and self.last_cmd is not None:
            self.u[0] = self.last_cmd.linear.x
            self.u[1] = self.last_cmd.linear.y
            self.u[2] = self.last_cmd.linear.z
            self.u[3] = self.last_cmd.angular.z
        else:
            self.u[0] = 0.0
            self.u[1] = 0.0
            self.u[3] = 0.0
            if self.mode != "FLYING":
                self.u[2] = 0.0
        if self.mode == "FLYING":
            self.dynamics_step()
        if self.tick % 4 == 0:
            self.publish_imu()
            self.publish_tf()
        if self.tick % 8 == 0:
            self.publish_xy()
        if self.tick % 12 == 0:
            self.publish_z()
        if self.tick % 24 == 0:
            self.publish_odom()

def main(args=None):
    rclpy.init(args=args)
    node = DroneSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == "__main__":
    main()
