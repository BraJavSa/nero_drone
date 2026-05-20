#!/usr/bin/env python3

# Real-time plotter for monitoring live drone telemetry and tracking errors.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np
from math import cos, sin, atan2

class RealTimePlot(Node):
    def __init__(self):
        super().__init__('realtime_plot_node')
        self.sub_odom = self.create_subscription(Odometry, '/bebop/fullodom', self.odom_callback, 10)
        self.sub_ref = self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.points = 150
        self.x_axis = np.linspace(0, 1, self.points)
        self.lock = threading.Lock()
        self.data_buf = np.zeros((8, self.points))
        self.ref_buf = np.zeros((8, self.points))
        self.fig, self.axs = plt.subplots(4, 2, figsize=(11, 9))
        self.fig.canvas.manager.set_window_title('Bebop 2: Global State Tracking (Inertial Frame)')
        labels = ['x [m]', 'y [m]', 'z [m]', 'yaw [rad]', 
                  'vx_global [m/s]', 'vy_global [m/s]', 'vz [m/s]', 'v_yaw [rad/s]']
        self.lines_pos, self.lines_ref = [], []
        for i in range(8):
            ax = self.axs[i % 4, i // 4]
            ax.set_ylabel(labels[i])
            ax.set_xlim(0, 1)
            if i < 2: ax.set_ylim(-2.0, 2.0)
            elif i == 2: ax.set_ylim(0, 2.5)
            elif i == 3: ax.set_ylim(-3.15, 3.15)
            else: ax.set_ylim(-1.5, 1.5)
            lp, = ax.plot(self.x_axis, self.data_buf[i], lw=1.6, color='#1f77b4', animated=True, label='Global Measured')
            lr, = ax.plot(self.x_axis, self.ref_buf[i], lw=1.3, color='#d62728', ls='--', animated=True, label='Global Ref')
            self.lines_pos.append(lp)
            self.lines_ref.append(lr)
            if i == 0: ax.legend(loc='upper right', fontsize='x-small')
        self.fig.tight_layout()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=33, blit=True, cache_frame_data=False)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        raw_yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        yaw = atan2(sin(raw_yaw), cos(raw_yaw))
        v_body = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
            msg.twist.twist.angular.z
        ])
        F_mat = np.array([
            [cos(yaw), -sin(yaw), 0, 0],
            [sin(yaw),  cos(yaw), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        v_global = F_mat @ v_body
        with self.lock:
            curr = [
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y, 
                msg.pose.pose.position.z,
                yaw,
                v_global[0], 
                v_global[1], 
                v_global[2], 
                v_global[3]
            ]
            self.data_buf = np.roll(self.data_buf, -1, axis=1)
            self.data_buf[:, -1] = curr

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 8: return
        ref_arr = np.array(msg.data[:8])
        ref_arr[3] = atan2(sin(ref_arr[3]), cos(ref_arr[3]))
        with self.lock:
            self.ref_buf = np.roll(self.ref_buf, -1, axis=1)
            self.ref_buf[:, -1] = ref_arr

    def update_plot(self, frame):
        with self.lock:
            for i in range(8):
                self.lines_pos[i].set_ydata(self.data_buf[i])
                self.lines_ref[i].set_ydata(self.ref_buf[i])
        return self.lines_pos + self.lines_ref

def main(args=None):
    rclpy.init(args=args)
    node = RealTimePlot()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    plt.show(block=True) 

if __name__ == '__main__':
    main()