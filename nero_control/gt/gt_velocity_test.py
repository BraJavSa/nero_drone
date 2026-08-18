#!/usr/bin/env python3
"""
Ground Truth Velocity Test script for Bebop drone.
Subscribes to /bebop/gt_fullodom instead of /bebop/fullodom.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from nav_msgs.msg import Odometry
import numpy as np
import time
import os

V_XY    = 0.3
V_Z     = 0.5
V_YAW   = np.deg2rad(45.0)
T_RAMP   = 3.0
T_CRUISE = 2.0
T_HOVER  = 1.5
T_WAIT   = 5.0
PUBLISH_HZ = 30.0

def cosine_ramp(t: float, t0: float, t1: float) -> float:
    if t <= t0: return 0.0
    if t >= t1: return 1.0
    x = (t - t0) / (t1 - t0)
    return 0.5 * (1.0 - np.cos(np.pi * x))

def velocity_profile(t: float, v_peak: float, t_ramp: float, t_cruise: float) -> float:
    t_brake_start = t_ramp + t_cruise
    t_end         = t_brake_start + t_ramp
    if t < 0.0: return 0.0
    elif t <= t_ramp: return v_peak * cosine_ramp(t, 0.0, t_ramp)
    elif t <= t_brake_start: return v_peak
    elif t <= t_end: return v_peak * (1.0 - cosine_ramp(t, t_brake_start, t_end))
    else: return 0.0

def segment_duration(t_ramp: float, t_cruise: float) -> float:
    return 2.0 * t_ramp + t_cruise

SEGMENTS = [
    ('+X forward',   V_XY,   0),
    ('-X backward', -V_XY,   0),
    ('+Y right',     V_XY,   1),
    ('-Y left',     -V_XY,   1),
    ('+Z up',        V_Z,    2),
    ('-Z down',     -V_Z,    2),
    ('+Yaw CW',      V_YAW,  3),
    ('-Yaw CCW',    -V_YAW,  3),
]

class GtVelocityTrajectory(Node):
    def __init__(self):
        super().__init__('gt_bebop_velocity_trajectory')
        self.is_flying = False
        
        self.vx_b = 0.0
        self.vy_b = 0.0
        self.vz_b = 0.0
        self.dyaw_b = 0.0
        
        self.t_history = []
        self.ref_vx_history = []
        self.ref_vy_history = []
        self.ref_vz_history = []
        self.ref_dyaw_history = []
        self.act_vx_history = []
        self.act_vy_history = []
        self.act_vz_history = []
        self.act_dyaw_history = []

        self.sub_flying = self.create_subscription(Bool, '/bebop/is_flying', self._flying_cb, 10)
        self.sub_odom   = self.create_subscription(Odometry, '/bebop/gt_fullodom', self._odom_cb, 10)
        self.pub_ref    = self.create_publisher(Float64MultiArray, '/bebop/ref_vec', 10)

    def _flying_cb(self, msg: Bool):
        self.is_flying = msg.data

    def _odom_cb(self, msg: Odometry):
        self.vx_b   = msg.twist.twist.linear.x
        self.vy_b   = msg.twist.twist.linear.y
        self.vz_b   = msg.twist.twist.linear.z
        self.dyaw_b = msg.twist.twist.angular.z

    def _publish(self, vx=0.0, vy=0.0, vz=0.0, dyaw=0.0):
        msg = Float64MultiArray()
        msg.data = [
            0.0, 0.0, 0.0, 0.0,
            float(vx), float(vy), float(vz), float(dyaw),
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        ]
        self.pub_ref.publish(msg)

        self.t_history.append(time.time())
        self.ref_vx_history.append(vx)
        self.ref_vy_history.append(vy)
        self.ref_vz_history.append(vz)
        self.ref_dyaw_history.append(dyaw)
        
        self.act_vx_history.append(self.vx_b)
        self.act_vy_history.append(self.vy_b)
        self.act_vz_history.append(self.vz_b)
        self.act_dyaw_history.append(self.dyaw_b)

    def _hover(self, duration: float):
        dt = 1.0 / PUBLISH_HZ
        steps = int(duration * PUBLISH_HZ)
        for _ in range(steps):
            if not rclpy.ok(): return
            rclpy.spin_once(self, timeout_sec=0.0)
            self._publish()
            time.sleep(dt)

    def _run_segment(self, name: str, v_peak: float, axis: int):
        dt = 1.0 / PUBLISH_HZ
        dur = segment_duration(T_RAMP, T_CRUISE)
        steps = int(dur * PUBLISH_HZ)
        vel_cmd = [0.0, 0.0, 0.0, 0.0]
        for i in range(steps + 1):
            if not rclpy.ok(): return
            t = i * dt
            v = velocity_profile(t, v_peak, T_RAMP, T_CRUISE)
            vel_cmd[axis] = v
            rclpy.spin_once(self, timeout_sec=0.0)
            self._publish(*vel_cmd)
            time.sleep(dt)
        vel_cmd[axis] = 0.0
        self._publish(*vel_cmd)

    def run(self):
        t0 = time.time()
        while not self.is_flying:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._publish()
            if time.time() - t0 > T_WAIT:
                self.get_logger().error("Timeout: Drone is not flying.")
                return
        self._hover(2.0)
        for idx, (name, v_peak, axis) in enumerate(SEGMENTS):
            if not rclpy.ok(): break
            self._run_segment(name, v_peak, axis)
            self._hover(T_HOVER)
        self._hover(3.0)
        self.plot_results()

    def plot_results(self):
        if not self.t_history:
            return
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        t_data = np.array(self.t_history) - self.t_history[0]
        fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
        
        axs[0].plot(t_data, self.ref_vx_history, 'r--', label='Reference')
        axs[0].plot(t_data, self.act_vx_history, 'b-', label='GT Actual')
        axs[0].set_ylabel('vx (m/s)')
        axs[0].legend()
        axs[0].grid(True)
        
        axs[1].plot(t_data, self.ref_vy_history, 'r--', label='Reference')
        axs[1].plot(t_data, self.act_vy_history, 'b-', label='GT Actual')
        axs[1].set_ylabel('vy (m/s)')
        axs[1].legend()
        axs[1].grid(True)

        axs[2].plot(t_data, self.ref_vz_history, 'r--', label='Reference')
        axs[2].plot(t_data, self.act_vz_history, 'b-', label='GT Actual')
        axs[2].set_ylabel('vz (m/s)')
        axs[2].legend()
        axs[2].grid(True)

        axs[3].plot(t_data, self.ref_dyaw_history, 'r--', label='Reference')
        axs[3].plot(t_data, self.act_dyaw_history, 'b-', label='GT Actual')
        axs[3].set_ylabel('vyaw (rad/s)')
        axs[3].set_xlabel('Time (s)')
        axs[3].legend()
        axs[3].grid(True)
        
        plt.tight_layout()
        os.makedirs('/home/brayan/ros2_ws/src/neroControl/data', exist_ok=True)
        plt.savefig('/home/brayan/ros2_ws/src/neroControl/data/gt_velocity_tracking_test.png')


def main(args=None):
    rclpy.init(args=args)
    node = GtVelocityTrajectory()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
