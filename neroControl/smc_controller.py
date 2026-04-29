#!/usr/bin/env python3
# Velocity controller for Bebop drone using Sliding Mode Control (SMC) + feedforward
# Drop-in replacement for the PID-based vel_controller.py
import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

# --------------------------------------------------------------------------- #
#  Steady-state gain matrices  (same as original, from system identification)  #
# --------------------------------------------------------------------------- #
F1_DIAG = np.array([0.8417,  0.8354,  3.966,  9.8524])
F2_DIAG = np.array([0.18227, 0.17095, 4.001,  4.7295])

# --------------------------------------------------------------------------- #
#  Velocity limits                                                              #
# --------------------------------------------------------------------------- #
VX_MAX   = 2.5
VY_MAX   = 2.5
VZ_MAX   = 1.0
DYAW_MAX = np.deg2rad(100.0)

# --------------------------------------------------------------------------- #
#  Timing                                                                       #
# --------------------------------------------------------------------------- #
REF_TIMEOUT = 0.5
CONTROL_HZ  = 30.0

# --------------------------------------------------------------------------- #
#  SMC parameters  (per channel: lambda, eta, phi, ki, e_int_max)              #
#                                                                               #
#  lambda   : sliding-surface slope  (how fast error is driven to zero)        #
#  eta      : switching gain          (robustness vs. chattering trade-off)     #
#  phi      : boundary-layer width    (replaces sign() with sat() to reduce    #
#              chattering; set to 0 to get pure SMC)                           #
#  ki       : integral term inside the surface (removes steady-state error)    #
#  e_int_max: anti-windup for the integral                                      #
# --------------------------------------------------------------------------- #
SMC_PARAMS = {
    #         lambda  eta   phi   ki    e_int_max
    'x':   (  2.2,   0.30, 0.12, 0.08,  0.40 ),
    'y':   (  2.2,   0.30, 0.12, 0.08,  0.40 ),
    'z':   (  1.5,   0.20, 0.10, 0.10,  0.30 ),
    'yaw': (  1.2,   0.15, 0.08, 0.06,  0.25 ),
}

# --------------------------------------------------------------------------- #
#  Feedforward weight  (same role as in the original controller)               #
# --------------------------------------------------------------------------- #
FF_WEIGHT = 0.85


# =========================================================================== #
class SMCChannel:
    """
    Single-axis Sliding Mode Controller.

    Sliding surface:
        s = e_dot + lambda * e + ki * integral(e)

    where  e = ref - measured  (velocity error in this controller).

    Because we work at the *velocity* level (not position), e_dot is the
    numerical derivative of e, filtered with a low-pass to reduce noise.

    Control law:
        u_smc = -eta * sat(s / phi)

    The saturation function sat() approximates sign() inside a boundary layer
    of width phi, which smooths the control signal and greatly reduces
    chattering while preserving robustness against bounded disturbances.
    """

    def __init__(self, lam: float, eta: float, phi: float,
                 ki: float, e_int_max: float, tau_d: float = 0.008):
        self.lam       = lam
        self.eta       = eta
        self.phi       = phi if phi > 0.0 else 1e-6   # avoid division by zero
        self.ki        = ki
        self.e_int_max = e_int_max
        self.tau_d     = tau_d          # derivative low-pass time constant [s]

        self._e_int   = 0.0
        self._e_prev  = 0.0
        self._d_filt  = 0.0             # filtered derivative of e

    # ----------------------------------------------------------------------- #
    def reset(self):
        self._e_int = self._e_prev = self._d_filt = 0.0

    def reset_integral(self):
        self._e_int = 0.0

    # ----------------------------------------------------------------------- #
    @staticmethod
    def _sat(x: float, phi: float) -> float:
        """Saturation function for boundary-layer SMC."""
        return float(np.clip(x / phi, -1.0, 1.0))

    # ----------------------------------------------------------------------- #
    def update(self, e: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        # --- integrate error (with anti-windup) ---
        self._e_int = float(np.clip(
            self._e_int + e * dt,
            -self.e_int_max, self.e_int_max
        ))

        # --- filtered derivative of error ---
        e_dot_raw    = (e - self._e_prev) / dt
        alpha        = dt / (self.tau_d + dt)
        self._d_filt = (1.0 - alpha) * self._d_filt + alpha * e_dot_raw
        self._e_prev = e

        # --- sliding surface ---
        s = self._d_filt + self.lam * e + self.ki * self._e_int

        # --- switching control ---
        u_smc = -self.eta * self._sat(s, self.phi)

        return u_smc


# =========================================================================== #
class BebopSMCVelocityController(Node):
    """
    Velocity controller for the Parrot Bebop using Sliding Mode Control.

    Identical ROS2 interface to the original PID controller:
      Subscribes : /odometry/filtered   (nav_msgs/Odometry)
                   /bebop/ref_vec       (std_msgs/Float64MultiArray)
                   /bebop/is_flying     (std_msgs/Bool)
      Publishes  : /bebop/cmd_vel       (geometry_msgs/Twist)

    Architecture
    ------------
    For each axis the total command is:

        u = FF_WEIGHT * (ref_body / v_ss)          <- feedforward
          + (1 - FF_WEIGHT) * SMC_output            <- feedback (SMC)

    The feedforward term provides the bulk of the necessary command so that
    the SMC only has to compensate for model mismatch and disturbances,
    which keeps eta (the switching gain) small and chattering minimal.
    """

    def __init__(self):
        super().__init__('bebop_smc_velocity_controller')

        # State
        self.vx_b = self.vy_b = self.vz_b = self.dyaw_b = self.psi = 0.0
        self.ref_vx_i = self.ref_vy_i = self.ref_vz_i = self.ref_dyaw_i = 0.0
        self.is_flying    = False
        self.odom_received = False
        self.last_ref_time = None

        # SMC channels
        self.smc = {k: SMCChannel(*v) for k, v in SMC_PARAMS.items()}

        # Steady-state velocity-to-command ratios (from system ID)
        self.vx_ss, self.vy_ss, self.vz_ss, self.dyaw_ss = F1_DIAG / F2_DIAG

        # ROS2 interface
        self.sub_odom = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_ref = self.create_subscription(
            Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_is_flying = self.create_subscription(
            Bool, '/bebop/is_flying', self.is_flying_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/bebop/cmd_vel', 10)

        self.dt    = 1.0 / CONTROL_HZ
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info('Bebop SMC velocity controller started.')

    # ----------------------------------------------------------------------- #
    def odom_callback(self, msg: Odometry):
        self.vx_b  = msg.twist.twist.linear.x
        self.vy_b  = msg.twist.twist.linear.y
        self.vz_b  = msg.twist.twist.linear.z
        self.dyaw_b = msg.twist.twist.angular.z
        q = msg.pose.pose.orientation
        _, _, yaw  = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.psi   = yaw
        self.odom_received = True

    # ----------------------------------------------------------------------- #
    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 8:
            return
        self.ref_vx_i   = float(np.clip(msg.data[4], -VX_MAX,   VX_MAX))
        self.ref_vy_i   = float(np.clip(msg.data[5], -VY_MAX,   VY_MAX))
        self.ref_vz_i   = float(np.clip(msg.data[6], -VZ_MAX,   VZ_MAX))
        self.ref_dyaw_i = float(np.clip(msg.data[7], -DYAW_MAX, DYAW_MAX))
        self.last_ref_time = self.get_clock().now()

    # ----------------------------------------------------------------------- #
    def is_flying_callback(self, msg: Bool):
        if not msg.data and self.is_flying:
            for ch in self.smc.values():
                ch.reset()
        self.is_flying = msg.data

    # ----------------------------------------------------------------------- #
    def control_loop(self):
        if not self.odom_received or not self.is_flying:
            self.pub_cmd.publish(Twist())
            return

        # Zero reference if it has gone stale
        if self.last_ref_time is not None:
            age = (self.get_clock().now() - self.last_ref_time).nanoseconds * 1e-9
            if age > REF_TIMEOUT:
                self.ref_vx_i = self.ref_vy_i = self.ref_vz_i = self.ref_dyaw_i = 0.0

        # --- Rotate inertial reference velocities into body frame ---
        cp = np.cos(self.psi)
        sp = np.sin(self.psi)
        ref_vx_b =  cp * self.ref_vx_i + sp * self.ref_vy_i
        ref_vy_b = -sp * self.ref_vx_i + cp * self.ref_vy_i

        # --- Velocity errors in body frame ---
        ex_b   = ref_vx_b      - self.vx_b
        ey_b   = ref_vy_b      - self.vy_b
        ez_b   = self.ref_vz_i - self.vz_b
        eyaw_b = self.ref_dyaw_i - self.dyaw_b

        # --- Detectar canales en modo frenado (ref ≈ 0 pero aún hay velocidad) ---
        braking_x   = abs(self.ref_vx_i)   < 0.05 and abs(self.vx_b)   > 0.05
        braking_y   = abs(self.ref_vy_i)   < 0.05 and abs(self.vy_b)   > 0.05
        braking_z   = abs(self.ref_vz_i)   < 0.05 and abs(self.vz_b)   > 0.05
        braking_yaw = abs(self.ref_dyaw_i) < 0.05 and abs(self.dyaw_b) > 0.02

        # Reset integral solo cuando ya está frenado y quieto
        if abs(self.ref_vx_i) < 0.05 and not braking_x:
            self.smc['x'].reset_integral()
        if abs(self.ref_vy_i) < 0.05 and not braking_y:
            self.smc['y'].reset_integral()

        # --- SMC feedback ---
        smc_x   = self.smc['x'].update(ex_b,    self.dt)
        smc_y   = self.smc['y'].update(ey_b,    self.dt)
        smc_z   = self.smc['z'].update(ez_b,    self.dt)
        smc_yaw = self.smc['yaw'].update(eyaw_b, self.dt)

        # --- Total command = FF + SMC ---
        # En modo frenado: el FF se invierte para oponerse a la velocidad actual
        # (frenado activo), y se ignora el FF de referencia (que sería cero de
        # todas formas). Esto garantiza que el drone desacelere agresivamente.
        fb_w = 1.0 - FF_WEIGHT

        if braking_x:
            # FF apunta contra la velocidad actual; SMC refina la magnitud
            ux = -FF_WEIGHT * (self.vx_b / self.vx_ss) + fb_w * smc_x
        else:
            ux =  FF_WEIGHT * (ref_vx_b  / self.vx_ss) + fb_w * smc_x

        if braking_y:
            uy = -FF_WEIGHT * (self.vy_b / self.vy_ss) + fb_w * smc_y
        else:
            uy =  FF_WEIGHT * (ref_vy_b  / self.vy_ss) + fb_w * smc_y

        if braking_z:
            uz = -FF_WEIGHT * (self.vz_b / self.vz_ss) + fb_w * smc_z
        else:
            uz =  FF_WEIGHT * (self.ref_vz_i / self.vz_ss) + fb_w * smc_z

        if braking_yaw:
            uyaw = -FF_WEIGHT * (self.dyaw_b / self.dyaw_ss) + fb_w * smc_yaw
        else:
            uyaw =  FF_WEIGHT * (self.ref_dyaw_i / self.dyaw_ss) + fb_w * smc_yaw

        # --- Publish ---
        cmd = Twist()
        cmd.linear.x  = float(np.clip(ux,   -1.0, 1.0))
        cmd.linear.y  = float(np.clip(uy,   -1.0, 1.0))
        cmd.linear.z  = float(np.clip(uz,   -1.0, 1.0))
        cmd.angular.z = float(np.clip(uyaw, -1.0, 1.0))
        self.pub_cmd.publish(cmd)


# =========================================================================== #
def main(args=None):
    rclpy.init(args=args)
    node = BebopSMCVelocityController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()