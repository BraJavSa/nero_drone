#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist

F1_DIAG = np.array([0.8417,  0.8354,  3.966,  9.8524])
F2_DIAG = np.array([0.18227, 0.17095, 4.001,  4.7295])

VX_MAX   = 4.5
VY_MAX   = 4.5
VZ_MAX   = 1.0
DYAW_MAX = np.deg2rad(100.0)

REF_TIMEOUT = 0.5
CONTROL_HZ  = 30.0

ACCEL_FF = 0.05
FF_W_XY  = 1.0
FF_W_Z   = 1.1
FF_W_YAW = 1.0
REF_TAU  = 0.18

TAU_DYAW_FILT = 0.30   # antes 0.10 — más suavizado para reducir vibración

REF_TAU_YAW   = 0.25  # NUEVO — filtro de referencia para yaw (igual que vx/vy)

# Modificación sugerida en PID_PARAMS
PID_PARAMS = {
    #         kp    ki     kd    imax  tau_d
    'x':   (5.0,  0.04, 0.35, 0.50, 0.08),
    'y':   (5.0,  0.04, 0.35, 0.50, 0.08),
    'z':   (1.8,  0.06, 0.20, 0.40, 0.12),
    'yaw': (0.35, 0.0001, 0.15, 0.15, 0.05), # Kd reducido y tau_d aumentado
}

class PIDChannel:
    def __init__(self, kp, ki, kd, e_int_max, tau_d=0.08):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.e_int_max = e_int_max
        self.tau_d     = tau_d
        self._e_int     = 0.0
        self._meas_prev = 0.0
        self._d_filt    = 0.0

    def reset(self):
        self._e_int = self._meas_prev = self._d_filt = 0.0

    def update(self, e, measurement, dt):
        if dt <= 0.0:
            return 0.0
        self._e_int = np.clip(self._e_int + e * dt,
                              -self.e_int_max, self.e_int_max)
        d_raw = -(measurement - self._meas_prev) / dt
        alpha = dt / (self.tau_d + dt)
        self._d_filt = (1.0 - alpha) * self._d_filt + alpha * d_raw
        self._meas_prev = measurement
        return self.kp * e + self.ki * self._e_int + self.kd * self._d_filt


class BebopVelocityController(Node):
    def __init__(self):
        super().__init__('bebop_velocity_controller')

        self.vx_b = self.vy_b = self.vz_b = self.dyaw_b = 0.0
        self.dyaw_b_filt = 0.0

        self.ref_vx_b = self.ref_vy_b = self.ref_vz_b = self.ref_dyaw_b = 0.0

        # Filtros de referencia vx/vy
        self.ref_vx_b_filt = self.ref_vy_b_filt = 0.0
        self.ref_vx_b_prev = self.ref_vy_b_prev = 0.0

        # Filtro de referencia yaw — NUEVO
        self.ref_dyaw_b_filt = 0.0
        self.ref_dyaw_b_prev = 0.0

        self.is_flying     = False
        self.odom_received = False
        self.last_ref_time = None

        self.vx_ss, self.vy_ss, self.vz_ss, self.dyaw_ss = F1_DIAG / F2_DIAG

        self.pid = {k: PIDChannel(*v) for k, v in PID_PARAMS.items()}

        self.sub_odom   = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_ref    = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_flying = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)
        self.pub_cmd    = self.create_publisher(Twist, '/bebop/cmd_vel', 10)

        self.dt    = 1.0 / CONTROL_HZ
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info('BebopVelocityController iniciado')

    def odom_callback(self, msg):
        self.vx_b   = msg.twist.twist.linear.x
        self.vy_b   = msg.twist.twist.linear.y
        self.vz_b   = msg.twist.twist.linear.z
        self.dyaw_b = msg.twist.twist.angular.z

        # Filtro de medición yaw — tau aumentado de 0.10 a 0.18
        alpha_yaw = self.dt / (TAU_DYAW_FILT + self.dt)
        self.dyaw_b_filt += alpha_yaw * (self.dyaw_b - self.dyaw_b_filt)

        self.odom_received = True

    def ref_callback(self, msg):
        if len(msg.data) < 8:
            return
        self.ref_vx_b   = float(np.clip(msg.data[4], -VX_MAX,   VX_MAX))
        self.ref_vy_b   = float(np.clip(msg.data[5], -VY_MAX,   VY_MAX))
        self.ref_vz_b   = float(np.clip(msg.data[6], -VZ_MAX,   VZ_MAX))
        self.ref_dyaw_b = float(np.clip(msg.data[7], -DYAW_MAX, DYAW_MAX))
        self.last_ref_time = self.get_clock().now()

    def is_flying_callback(self, msg):
        if not msg.data and self.is_flying:
            for p in self.pid.values():
                p.reset()
            self.ref_vx_b_filt   = self.ref_vy_b_filt   = 0.0
            self.ref_vx_b_prev   = self.ref_vy_b_prev   = 0.0
            self.ref_dyaw_b_filt = 0.0   # NUEVO — reset filtro yaw
            self.ref_dyaw_b_prev = 0.0
            self.dyaw_b_filt     = 0.0
        self.is_flying = msg.data

    def control_loop(self):
        if not self.odom_received or not self.is_flying:
            self.pub_cmd.publish(Twist())
            return

        if self.last_ref_time is not None:
            elapsed = (self.get_clock().now() - self.last_ref_time).nanoseconds * 1e-9
            if elapsed > REF_TIMEOUT:
                self.ref_vx_b = self.ref_vy_b = self.ref_vz_b = self.ref_dyaw_b = 0.0

        # --- Filtros de referencia vx / vy ---
        alpha_ref = self.dt / (REF_TAU + self.dt)
        self.ref_vx_b_filt += alpha_ref * (self.ref_vx_b - self.ref_vx_b_filt)
        self.ref_vy_b_filt += alpha_ref * (self.ref_vy_b - self.ref_vy_b_filt)

        d_ref_vx_b = (self.ref_vx_b_filt - self.ref_vx_b_prev) / self.dt
        d_ref_vy_b = (self.ref_vy_b_filt - self.ref_vy_b_prev) / self.dt
        self.ref_vx_b_prev = self.ref_vx_b_filt
        self.ref_vy_b_prev = self.ref_vy_b_filt

        # --- Filtro de referencia yaw — NUEVO ---
        alpha_yaw_ref = self.dt / (REF_TAU_YAW + self.dt)
        self.ref_dyaw_b_filt += alpha_yaw_ref * (self.ref_dyaw_b - self.ref_dyaw_b_filt)

        d_ref_dyaw_b = (self.ref_dyaw_b_filt - self.ref_dyaw_b_prev) / self.dt
        self.ref_dyaw_b_prev = self.ref_dyaw_b_filt

        # --- Errores ---
        ex_b   = self.ref_vx_b   - self.vx_b
        ey_b   = self.ref_vy_b   - self.vy_b
        ez_b   = self.ref_vz_b   - self.vz_b
        eyaw_b = self.ref_dyaw_b_filt - self.dyaw_b_filt   # error con ref filtrada

        # --- Decay integrador en reposo ---
        decay = 0.95
        if abs(self.ref_vx_b)   < 0.05:            self.pid['x']._e_int   *= decay
        if abs(self.ref_vy_b)   < 0.05:            self.pid['y']._e_int   *= decay
        if abs(self.ref_dyaw_b) < np.deg2rad(2.0): self.pid['yaw']._e_int *= decay

        # --- Salidas de control ---
        ux = (FF_W_XY * self.ref_vx_b_filt / self.vx_ss
              + ACCEL_FF * d_ref_vx_b
              + self.pid['x'].update(ex_b, self.vx_b, self.dt))

        uy = (FF_W_XY * self.ref_vy_b_filt / self.vy_ss
              + ACCEL_FF * d_ref_vy_b
              + self.pid['y'].update(ey_b, self.vy_b, self.dt))

        uz = (FF_W_Z * self.ref_vz_b / self.vz_ss
              + self.pid['z'].update(ez_b, self.vz_b, self.dt))

        # Feedforward con referencia suavizada + término de aceleración — NUEVO
        uyaw = (FF_W_YAW * self.ref_dyaw_b_filt / self.dyaw_ss
                + ACCEL_FF * d_ref_dyaw_b
                + self.pid['yaw'].update(eyaw_b, self.dyaw_b_filt, self.dt))

        cmd = Twist()
        cmd.linear.x  = float(np.clip(ux,   -1.0, 1.0))
        cmd.linear.y  = float(np.clip(uy,   -1.0, 1.0))
        cmd.linear.z  = float(np.clip(uz,   -1.0, 1.0))
        cmd.angular.z = float(np.clip(uyaw, -1.0, 1.0))
        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BebopVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()