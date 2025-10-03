#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from nero_drone.takeoff import Takeoff
from nero_drone.land import Land

class DemoTakeoffLand(Node):
    def __init__(self):
        super().__init__('demo_takeoff_land')
        self.takeoff = Takeoff(self)
        self.land = Land(self)
        self.run_demo()

    def run_demo(self):
        self.get_logger().info("Sending TAKEOFF...")
        self.takeoff.send()

        self.get_logger().info("Hovering for 30s...")
        time.sleep(30)

        self.get_logger().info("Sending LAND...")
        self.land.send()

        self.get_logger().info("Demo finished.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DemoTakeoffLand()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
