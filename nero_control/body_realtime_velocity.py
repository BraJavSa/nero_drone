#!/usr/bin/env python3

# Real-time plotter for monitoring live drone body-frame velocity tracking.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import numpy as np

class BodyRealTimeVelocityPlot(Node):
    def __init__(self):
        super().__init__('body_realtime_velocity_plot_node')
        
        # Subscribe to topics as in extended_controller.py
        self.sub_odom = self.create_subscription(Odometry, '/bebop/fullodom', self.odom_callback, 10)
        self.sub_ref = self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        
        self.points = 150
        self.x_axis = np.linspace(0, 1, self.points)
        self.lock = threading.Lock()
        
        # Buffer for 4 axes: vx, vy, vz, vyaw
        self.data_buf = np.zeros((4, self.points))
        self.ref_buf = np.zeros((4, self.points))
        
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8))
        self.axs_flat = self.axs.flatten()
        self.fig.canvas.manager.set_window_title('Bebop 2: Body-Frame Velocity Tracking')
        
        # Use subplots_adjust to set fixed spacing so subplots never move or shake
        self.fig.subplots_adjust(left=0.1, right=0.95, top=0.92, bottom=0.08, hspace=0.35, wspace=0.3)
        
        labels = [
            'vx (Forward) [m/s]', 
            'vy (Lateral) [m/s]', 
            'vz (Vertical) [m/s]', 
            'vyaw (Yaw Rate) [rad/s]'
        ]
        
        # Default y-limits to avoid continuous scaling jitter
        self.default_limits = [
            [-1.5, 1.5], # vx
            [-1.5, 1.5], # vy
            [-1.0, 1.0], # vz
            [-2.0, 2.0]  # vyaw
        ]
        
        self.lines_pos = []
        self.lines_ref = []
        
        for i in range(4):
            ax = self.axs_flat[i]
            ax.set_ylabel(labels[i])
            ax.set_xlim(0, 1)
            ax.set_ylim(self.default_limits[i][0], self.default_limits[i][1])
            ax.grid(True, linestyle=':', alpha=0.6)
            
            # Format ticks with a constant precision to prevent width changes
            ax.yaxis.set_major_formatter(plt.FormatStrFormatter('%.2f'))
            
            lp, = ax.plot(self.x_axis, self.data_buf[i], lw=1.8, color='#1f77b4', label='Measured (Body)')
            lr, = ax.plot(self.x_axis, self.ref_buf[i], lw=1.5, color='#d62728', ls='--', label='Reference (Body)')
            
            self.lines_pos.append(lp)
            self.lines_ref.append(lr)
            ax.legend(loc='upper right', fontsize='x-small')
        
        # Animate at ~30 Hz. Using blit=False to allow dynamic y-limits.
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=33, blit=False, cache_frame_data=False)

    def odom_callback(self, msg: Odometry):
        # Extract body velocities directly from twist
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        vyaw = msg.twist.twist.angular.z
        
        with self.lock:
            curr = [vx, vy, vz, vyaw]
            self.data_buf = np.roll(self.data_buf, -1, axis=1)
            self.data_buf[:, -1] = curr

    def ref_callback(self, msg: Float64MultiArray):
        # Standard reference message has at least 8 elements, where 4-7 are vx_ref, vy_ref, vz_ref, vyaw_ref
        if len(msg.data) < 8: 
            return
        
        vx_ref = msg.data[4]
        vy_ref = msg.data[5]
        vz_ref = msg.data[6]
        vyaw_ref = msg.data[7]
        
        with self.lock:
            ref_curr = [vx_ref, vy_ref, vz_ref, vyaw_ref]
            self.ref_buf = np.roll(self.ref_buf, -1, axis=1)
            self.ref_buf[:, -1] = ref_curr

    def update_plot(self, frame):
        with self.lock:
            for i in range(4):
                self.lines_pos[i].set_ydata(self.data_buf[i])
                self.lines_ref[i].set_ydata(self.ref_buf[i])
                
                # Check if data exceeds default limits, otherwise keep default limits
                d_min = min(np.min(self.data_buf[i]), np.min(self.ref_buf[i]))
                d_max = max(np.max(self.data_buf[i]), np.max(self.ref_buf[i]))
                
                lim_min, lim_max = self.default_limits[i]
                if d_min < lim_min or d_max > lim_max:
                    # Expand limits with a margin, rounded up to nearest 0.5
                    margin = max(abs(d_min), abs(d_max)) + 0.2
                    margin = np.ceil(margin * 2.0) / 2.0  # Round to nearest 0.5
                    self.axs_flat[i].set_ylim(-margin, margin)
                else:
                    self.axs_flat[i].set_ylim(lim_min, lim_max)
        return self.lines_pos + self.lines_ref

def main(args=None):
    rclpy.init(args=args)
    node = BodyRealTimeVelocityPlot()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    plt.show(block=True) 

if __name__ == '__main__':
    main()
