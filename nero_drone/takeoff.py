import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

class Takeoff(Node):
    def __init__(self):
        super().__init__('bebop_takeoff')
        self.publisher_ = self.create_publisher(Empty, '/bebop/takeoff', 10)

    def send(self):
        msg = Empty()
        self.publisher_.publish(msg)
        self.get_logger().info("Takeoff command sent")
