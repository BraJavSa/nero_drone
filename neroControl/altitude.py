#!/usr/bin/env python3
# ROS2 node for fusing XY position, altitude, and IMU data into a PoseWithCovarianceStamped message
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu

class PositionFusion(Node):
    def __init__(self):
        super().__init__('position_fusion')
        self.last_x = None
        self.last_y = None
        self.last_z = None
        self.last_orientation = None
        self.last_imu_stamp = None

        self.sub_xy = self.create_subscription(PointStamped, '/bebop/position', self.cb_xy, 10)
        self.sub_z = self.create_subscription(Float64, '/bebop/altitude', self.cb_z, 10)
        self.sub_imu = self.create_subscription(Imu, '/bebop/imu', self.cb_imu, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/bebop/position_pose', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_fused_pose)

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

    def publish_fused_pose(self):
        if (self.last_x is None or self.last_y is None or self.last_z is None or
            self.last_orientation is None or self.last_imu_stamp is None):
            return

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.last_imu_stamp
        out.header.frame_id = 'odom'
        out.pose.pose.position.x = self.last_x
        out.pose.pose.position.y = self.last_y
        out.pose.pose.position.z = self.last_z
        out.pose.pose.orientation = self.last_orientation
        out.pose.covariance = [0.0] * 36
        out.pose.covariance[0]  = 0.005
        out.pose.covariance[7]  = 0.005
        out.pose.covariance[14] = 0.005
        out.pose.covariance[21] = 0.0003
        out.pose.covariance[28] = 0.0003
        out.pose.covariance[35] = 0.0003
        self.pub.publish(out)

def main():
    rclpy.init()
    node = PositionFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
