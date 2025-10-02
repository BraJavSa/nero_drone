#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloPy(Node):
    def __init__(self):
        super().__init__('hello_py')
        self.get_logger().info('Hola mundo desde Python en nero_drone!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloPy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
