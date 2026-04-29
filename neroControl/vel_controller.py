#!/usr/bin/env python3
# Velocity controller for Bebop drone using PID and feedforward
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

F1_DIAG = np.array([0.8417,  0.8354,  3.966,  9.8524])
F2_DIAG = np.array([0.18227, 0.17095, 4.001,  4.7295])

VX_MAX   = 2.5  
VY_MAX   = 2.5
VZ_MAX   = 1.0
DYAW_MAX = np.deg2rad(100.0)

REF_TIMEOUT = 0.5 
CONTROL_HZ  = 30.0

PID_PARAMS = {
    'x':   (1.8, 0.01, 0.55, 0.2),
    'y':   (1.8, 0.01, 0.55, 0.2),
    'z':   (1.0, 0.05, 0.3,  0.3),
    'yaw': (0.8, 0.02, 0.2,  0.3),
}

class PIDChannel:
    def __init__(self, kp, ki, kd, e_int_max, tau_d=0.005):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.e_int_max = e_int_max
        self.tau_d = tau_d
        self._e_int = 0.0
        self._e_prev = 0.0
        self._d_filt = 0.0

    def reset(self):
        self._e_int = self._e_prev = self._d_filt = 0.0

    def reset_integral(self):
        self._e_int = 0.0

    def update(self, e: float, dt: float) -> float:
        if dt <= 0.0: return 0.0
        self._e_int = np.clip(self._e_int + e * dt, -self.e_int_max, self.e_int_max)
        e_dot_raw = (e - self._e_prev) / dt
        alpha = dt / (self.tau_d + dt)
        self._d_filt = (1.0 - alpha) * self._d_filt + alpha * e_dot_raw
        self._e_prev = e
        return self.kp * e + self.ki * self._e_int + self.kd * self._d_filt

class BebopVelocityController(Node):
    def __init__(self):
        super().__init__('bebop_velocity_controller')
        self.vx_b = self.vy_b = self.vz_b = self.dyaw_b = self.psi = 0.0
        self.ref_vx_i = self.ref_vy_i = self.ref_vz_i = self.ref_dyaw_i = 0.0
        self.is_flying = self.odom_received = False
        self.last_ref_time = None
        self.pid = {k: PIDChannel(*v) for k, v in PID_PARAMS.items()}
        self.vx_ss, self.vy_ss, self.vz_ss, self.dyaw_ss = F1_DIAG / F2_DIAG
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_ref = self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_is_flying = self.create_subscription(Bool, '/bebop/is_flying', self.is_flying_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/bebop/cmd_vel', 10)
        self.dt = 1.0 / CONTROL_HZ
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg: Odometry):
        self.vx_b = msg.twist.twist.linear.x
        self.vy_b = msg.twist.twist.linear.y
        self.vz_b = msg.twist.twist.linear.z
        self.dyaw_b = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psi = yaw
        self.odom_received = True

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 8: return
        self.ref_vx_i   = float(np.clip(msg.data[4], -VX_MAX,   VX_MAX))
        self.ref_vy_i   = float(np.clip(msg.data[5], -VY_MAX,   VY_MAX))
        self.ref_vz_i   = float(np.clip(msg.data[6], -VZ_MAX,   VZ_MAX))
        self.ref_dyaw_i = float(np.clip(msg.data[7], -DYAW_MAX, DYAW_MAX))
        self.last_ref_time = self.get_clock().now()

    def is_flying_callback(self, msg: Bool):
        if not msg.data and self.is_flying: [p.reset() for p in self.pid.values()]
        self.is_flying = msg.data

    def control_loop(self):
        if not self.odom_received or not self.is_flying:
            self.pub_cmd.publish(Twist())
            return
        if self.last_ref_time:
            if (self.get_clock().now() - self.last_ref_time).nanoseconds * 1e-9 > REF_TIMEOUT:
                self.ref_vx_i = self.ref_vy_i = self.ref_vz_i = self.ref_dyaw_i = 0.0
        cp, sp = np.cos(self.psi), np.sin(self.psi)
        ref_vx_b =  cp * self.ref_vx_i + sp * self.ref_vy_i
        ref_vy_b = -sp * self.ref_vx_i + cp * self.ref_vy_i
        ex_b = ref_vx_b - self.vx_b
        ey_b = ref_vy_b - self.vy_b
        ez_b = self.ref_vz_i - self.vz_b
        eyaw_b = self.ref_dyaw_i - self.dyaw_b
        if abs(self.ref_vx_i) < 0.05: self.pid['x'].reset_integral()
        if abs(self.ref_vy_i) < 0.05: self.pid['y'].reset_integral()
        ff_w = 0.85
        ux   = (ff_w * ref_vx_b / self.vx_ss) + self.pid['x'].update(ex_b, self.dt)
        uy   = (ff_w * ref_vy_b / self.vy_ss) + self.pid['y'].update(ey_b, self.dt)
        uz   = (ff_w * self.ref_vz_i / self.vz_ss) + self.pid['z'].update(ez_b, self.dt)
        uyaw = (ff_w * self.ref_dyaw_i / self.dyaw_ss) + self.pid['yaw'].update(eyaw_b, self.dt)
        cmd = Twist()
        cmd.linear.x  = float(np.clip(ux, -1.0, 1.0))
        cmd.linear.y  = float(np.clip(uy, -1.0, 1.0))
        cmd.linear.z  = float(np.clip(uz, -1.0, 1.0))
        cmd.angular.z = float(np.clip(uyaw, -1.0, 1.0))
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = BebopVelocityController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()