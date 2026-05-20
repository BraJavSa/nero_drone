#!/usr/bin/env python3

# Captures and broadcasts the initial starting reference frame of the drone upon takeoff.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

class InitialFramePublisher(Node):
    def __init__(self):
        super().__init__('initial_frame_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.initial_transform = None
        self.initialized = False
        self.timer = self.create_timer(1.0 / 5.0, self.publish_tf)
        self.start_time = self.get_clock().now()

    def publish_tf(self):
        now = self.get_clock().now()
        if not self.initialized:
            if (now - self.start_time).nanoseconds < 2e9:
                return
            try:
                tf_bl = self.tf_buffer.lookup_transform("odom", "bebop_link", rclpy.time.Time())
            except Exception:
                self.get_logger().warn("Waiting for odom -> base_link TF...")
                return
            self.initial_transform = tf_bl
            self.initialized = True
            self.get_logger().info("Initial frame fixed.")
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now.to_msg()
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "initial_frame"
        tf_msg.transform = self.initial_transform.transform
        self.br.sendTransform(tf_msg)

def main():
    rclpy.init()
    node = InitialFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
