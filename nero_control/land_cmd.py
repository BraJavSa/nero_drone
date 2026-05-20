# Utility script to rapidly publish a landing command to the drone.
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Land(Node):
    def __init__(self):
        super().__init__('bebop_land')
        self.publisher_ = self.create_publisher(Empty, '/bebop/land', 10)

    def send(self):
        msg = Empty()
        self.publisher_.publish(msg)
        self.get_logger().info("Land command sent.")
