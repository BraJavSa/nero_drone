#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
import numpy as np
from math import sin, cos, atan2

class model_based_controller(Node):
    def __init__(self):
        super().__init__('modelo_basado_node')
        
        # --- Parámetros Identificados ---
        # kx, ax, ky, ay, kz, az, kpsi, apsi
        params = np.array([0.8417, 0.18227, 0.8354, 0.17095, 3.966, 4.001, 9.8524, 4.7295])
        self.Ku = np.diag([params[0], params[2], params[4], params[6]])
        self.Kv = np.diag([params[1], params[3], params[5], params[7]])
        
        # --- Ganancias (Declaradas como Parámetros ROS 2 para el Optimizador) ---
        self.declare_parameter('ksp_x', 2.1834)
        self.declare_parameter('ksp_y', 2.3073)
        self.declare_parameter('ksp_z', 1.0)
        self.declare_parameter('ksp_psi', 2.0)
        
        self.declare_parameter('ksd_x', 1.4530)
        self.declare_parameter('ksd_y', 3.9993)
        self.declare_parameter('ksd_z', 2.0)
        self.declare_parameter('ksd_psi', 0.9)
        
        self.declare_parameter('kp_x', 2.89)
        self.declare_parameter('kp_y', 2.6743)
        self.declare_parameter('kp_z', 2.0)
        self.declare_parameter('kp_psi', 3.5)

        self.update_gains()
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # --- Estado ---
        self.X = np.zeros(4)
        self.v_world = np.zeros(4)
        self.Xd = np.zeros(4)
        self.dXd = np.zeros(4)
        
        self.Ur_ant = np.zeros(4)
        self.dt = 1.0/30.0
        
        self.is_flying = False
        self.ref_received = False
        
        # --- Comms ---
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.sub_ref = self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        self.sub_flying = self.create_subscription(Bool, '/bebop/is_flying', self.flying_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/safe_bebop/cmd_vel', 10)
        
        self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Controlador Dinámico REESTABLECIDO (Editable vía Parámetros)")

    def update_gains(self):
        """Actualiza las matrices de ganancias desde los parámetros de ROS."""
        self.Kp = np.diag([
            self.get_parameter('kp_x').value,
            self.get_parameter('kp_y').value,
            self.get_parameter('kp_z').value,
            self.get_parameter('kp_psi').value
        ])
        self.Ksp = np.diag([
            self.get_parameter('ksp_x').value,
            self.get_parameter('ksp_y').value,
            self.get_parameter('ksp_z').value,
            self.get_parameter('ksp_psi').value
        ])
        self.Ksd = np.diag([
            self.get_parameter('ksd_x').value,
            self.get_parameter('ksd_y').value,
            self.get_parameter('ksd_z').value,
            self.get_parameter('ksd_psi').value
        ])

    def parameters_callback(self, params):
        """Callback ejecutado cuando el optimizador cambia un parámetro."""
        success = True
        for param in params:
            self.get_logger().info(f"Parámetro {param.name} actualizado a {param.value}")
        
        # Programar actualización de matrices (se hará en el siguiente ciclo o aquí mismo)
        self.update_gains()
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v_b = msg.twist.twist.linear
        w_b = msg.twist.twist.angular
        
        yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.X = np.array([p.x, p.y, p.z, yaw])
        
        # Guardar velocidad inercial (Mundo)
        F = np.array([[cos(yaw), -sin(yaw), 0, 0],
                      [sin(yaw),  cos(yaw), 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])
        self.v_world = F @ np.array([v_b.x, v_b.y, v_b.z, w_b.z])

    def ref_callback(self, msg):
        if len(msg.data) >= 8:
            self.Xd = np.array(msg.data[0:4])
            self.dXd = np.array(msg.data[4:8])
            self.ref_received = True

    def flying_callback(self, msg):
        self.is_flying = msg.data

    def control_loop(self):
        if not self.ref_received or not self.is_flying: return

        # 1. Error y Referencia Cinematica
        error = self.Xd - self.X
        error[3] = atan2(sin(error[3]), cos(error[3]))
        Ur = self.dXd + self.Ksp @ np.tanh(self.Kp @ error)
        
        # 2. Aceleración Deseada
        dUr = (Ur - self.Ur_ant) / self.dt
        if np.linalg.norm(dUr) > 10.0: dUr = np.zeros(4)
        self.Ur_ant = Ur.copy()
        
        # 3. Ley de Control Dinámica (Todo en marco Inercial)
        # accel_ref_global = dUr + Ksd*(Ur - V_world) + Kv*V_world
        accel_total = dUr + self.Ksd @ (Ur - self.v_world) + self.Kv @ self.v_world
        
        # 4. Transformación a Comando Body (u)
        yaw = self.X[3]
        cos_y, sin_y = cos(yaw), sin(yaw)
        F_body_to_world = np.array([[cos_y, -sin_y, 0, 0],
                                     [sin_y,  cos_y, 0, 0],
                                     [0, 0, 1, 0],
                                     [0, 0, 0, 1]])
        
        # u = Ku^-1 * F_inv * accel_total
        u = np.linalg.inv(self.Ku) @ (F_body_to_world.T @ accel_total)
        
        # Debug
        if not hasattr(self, '_c'): self._c = 0
        self._c += 1
        if self._c % 30 == 0:
            self.get_logger().info(f"Distancia: {np.linalg.norm(error[:2]):.2f}m | Cmd: {np.around(u, 2)}")

        # 5. Envío
        cmd = Twist()
        cmd.linear.x = float(np.clip(u[0], -1.0, 1.0))
        cmd.linear.y = float(np.clip(u[1], -1.0, 1.0))
        cmd.linear.z = float(np.clip(u[2], -1.0, 1.0))
        cmd.angular.z = float(np.clip(u[3], -1.0, 1.0))
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(model_based_controller())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
