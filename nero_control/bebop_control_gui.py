#!/usr/bin/env python3

# Graphical User Interface (GUI) for manual Bebop control, takeoff, and landing.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
import tkinter as tk

class BebopControlGUI(Node):
    def __init__(self):
        super().__init__('bebop_control_gui_minimal')
        self.pub_takeoff = self.create_publisher(Empty, '/bebop/takeoff', 10)
        self.pub_land = self.create_publisher(Empty, '/bebop/land', 10)
        self.pub_cam = self.create_publisher(Vector3, '/bebop/move_camera', 10)
        self.pub_reset = self.create_publisher(Empty, '/bebop/reset', 10)
        self.root = tk.Tk()
        self.root.title("Bebop Minimal Control Panel")
        self.root.configure(bg="#2C2C2C")
        self.root.attributes('-topmost', True)
        main_frame = tk.Frame(self.root, bg="#2C2C2C")
        main_frame.pack(padx=15, pady=15)
        left_frame = tk.Frame(main_frame, bg="#2C2C2C")
        left_frame.grid(row=0, column=0, padx=10)
        right_frame = tk.Frame(main_frame, bg="#2C2C2C")
        right_frame.grid(row=0, column=1, padx=10)
        self.status_label = tk.Label(left_frame, text="", font=("Arial", 12), bg="#2C2C2C", fg="yellow")
        self.status_label.pack(pady=5)
        button_cfg = {"font": ("Arial", 14), "width": 20, "height": 2, "bg": "#3E3E3E", "fg": "white"}
        self.btn_takeoff = tk.Button(left_frame, text="Takeoff", command=self.takeoff, **button_cfg)
        self.btn_takeoff.pack(pady=10)
        self.btn_land = tk.Button(left_frame, text="Land", command=self.land, **button_cfg)
        self.btn_land.pack(pady=10)
        self.btn_reset = tk.Button(left_frame, text="RESET (EMERGENCY)", command=self.reset, 
                                   font=("Arial", 14), width=20, height=2, bg="#AA0000", fg="white")
        self.btn_reset.pack(pady=10)
        tk.Label(right_frame, text="Camera Pitch (°)", font=("Arial", 12), bg="#2C2C2C", fg="white").pack(pady=5)
        self.slider = tk.Scale(right_frame, from_=15, to=-90, orient=tk.VERTICAL, length=300, 
                               resolution=15, tickinterval=15, font=("Arial", 10), 
                               bg="#2C2C2C", fg="white", highlightthickness=0, command=self.update_camera_angle)
        self.slider.set(-15)
        self.slider.pack(pady=10)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.after(100, self.spin_once)
        self.send_camera_angle(-15.0)

    def takeoff(self):
        self.status_label.config(text="Sending Takeoff...")
        self.get_logger().info("Takeoff command published.")
        self.pub_takeoff.publish(Empty())

    def land(self):
        self.status_label.config(text="Sending Land...")
        self.get_logger().info("Land command published.")
        self.pub_land.publish(Empty())

    def reset(self):
        self.status_label.config(text="Sending Reset...")
        self.get_logger().warn("Reset/Emergency command published.")
        self.pub_reset.publish(Empty())

    def send_camera_angle(self, angle):
        msg = Vector3()
        msg.x = float(angle)
        msg.y = msg.z = 0.0
        self.pub_cam.publish(msg)

    def update_camera_angle(self, val):
        self.send_camera_angle(float(val))

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        self.root.after(100, self.spin_once)

    def on_close(self):
        self.root.destroy()
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    gui = BebopControlGUI()
    gui.root.mainloop()

if __name__ == "__main__":
    main()
