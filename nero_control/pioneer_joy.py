#!/usr/bin/env python3

# Maps keyboard or joystick inputs to velocity commands for a Pioneer ground robot.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardJoystick(Node):
    def __init__(self):
        super().__init__('keyboard_joystick')
        self.pub_cmd = self.create_publisher(Twist, '/pioneer/cmd_vel', 10)
        self.linear_speed = 0.9
        self.angular_speed = 0.8
        self.keys_pressed = {
            keyboard.Key.up: False,
            keyboard.Key.down: False,
            keyboard.Key.left: False,
            keyboard.Key.right: False
        }
        self.timer = self.create_timer(1.0/30.0, self.publish_callback)
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        self.get_logger().info("Keyboard joystick started.")
        self.get_logger().info("Use arrow keys to move the Pioneer. 'Esc' to exit.")

    def on_press(self, key):
        if key in self.keys_pressed:
            self.keys_pressed[key] = True
        if key == keyboard.Key.esc:
            return False

    def on_release(self, key):
        if key in self.keys_pressed:
            self.keys_pressed[key] = False

    def publish_callback(self):
        msg = Twist()
        linear_val = 0.0
        angular_val = 0.0
        if self.keys_pressed[keyboard.Key.up]:
            linear_val += self.linear_speed
        if self.keys_pressed[keyboard.Key.down]:
            linear_val -= self.linear_speed
        if self.keys_pressed[keyboard.Key.left]:
            angular_val += self.angular_speed
        if self.keys_pressed[keyboard.Key.right]:
            angular_val -= self.angular_speed
        msg.linear.x = float(linear_val)
        msg.angular.z = float(angular_val)
        self.pub_cmd.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.pub_cmd.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()