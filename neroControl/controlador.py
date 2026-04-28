#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from math import sin, cos, atan2

class Position:
    def __init__(self):

        self.w_X = np.zeros(8)
        self.w_dX = np.zeros(4)      
        self.w_Xd = np.zeros(4)      
        self.w_dXd = np.zeros(4)     
        self.w_ddXd = np.zeros(4)    
        self.w_Xtil = np.zeros(4)    
        self.w_Xa = np.zeros(8)      

class Parameters:
    def __init__(self):
        self.Model_simp = np.array([0.8417, 0.18227, 0.8354, 0.17095, 3.966, 4.001, 9.8524, 4.7295])
        self.b_uSat = np.array([1.0, 1.0, 1.0, 1.0]) 

class SC:
    def __init__(self):
        self.b_Ud = np.zeros(4)      
        self.w_Ur = np.zeros(4)      
        self.b_Ud_ant = np.zeros(4)  # Estado anterior para el filtro pasa bajos

class Bebop:
    def __init__(self, node: Node):
        self.node = node
        frecuency = 30
        self.dt = 1/frecuency               
        self.pPos = Position()
        self.pPar = Parameters()
        self.pSC = SC()

        self.ref_received = False
        self.is_flying = False
        self.first_ref = True
        self.w_last_ref = np.zeros(8)

        self.opt = 2
        self.base_opt = self.opt
        
        self.opt_params = {
            1: {
                'ksp_x': 0.6,  'ksp_y': 1.2,  'ksp_z': 1.2527,  'ksp_psi': 5.0911,
                'ksd_x': 1.0,  'ksd_y': 1.0,  'ksd_z': 4.5484,  'ksd_psi': 1.2455,
                'kp_x': 2.0,  'kp_y': 2.0,  'kp_z': 1.8747,  'kp_psi': 2.7967,
                'kd_x': 0.001,  'kd_y': 0.001,  'kd_z': 0.0000,  'kd_psi': 0.0000,
            },
            2: {
                'ksp_x': 0.1321,  'ksp_y': 0.9917,  'ksp_z': 1.2527,  'ksp_psi': 5.0911,
                'ksd_x': 0.3908,  'ksd_y': 0.6230,  'ksd_z': 4.5484,  'ksd_psi': 1.2455,
                'kp_x': 1.6856,  'kp_y': 1.2564,  'kp_z': 1.8747,  'kp_psi': 2.7967,
                'kd_x': 1.0,  'kd_y': 1.0,  'kd_z': 1.0,  'kd_psi': 1.0,
            },
            3: {
                'ksp_x': 0.6,  'ksp_y': 1.2,  'ksp_z': 1.2527,  'ksp_psi': 5.0911,
                'ksd_x': 1.0,  'ksd_y': 1.0,  'ksd_z': 4.5484,  'ksd_psi': 1.2455,
                'kp_x': 2.0,  'kp_y': 2.0,  'kp_z': 1.8747,  'kp_psi': 2.7967,
                'kd_x': 1.0,  'kd_y': 1.0,  'kd_z': 1.0,  'kd_psi': 1.0,
            },
            4: {
                'ksp_x': 0.15, 'ksp_y': 0.15, 'ksp_z': 1.5, 'ksp_psi': 1.8,
                'ksd_x': 0.5,  'ksd_y': 0.5,  'ksd_z': 2.0, 'ksd_psi': 2.5,
                'kp_x': 2.0,   'kp_y': 2.0,   'kp_z': 1.5,  'kp_psi': 1.5,
                'kd_x': 0.05,  'kd_y': 0.05,  'kd_z': 0.0,  'kd_psi': 0.0
            }
        }
        
        initial_values = self.opt_params.get(self.opt, self.opt_params[4])

        for name, value in initial_values.items():
            self.node.declare_parameter(name, value)
        
        self.node.add_on_set_parameters_callback(self.parameters_callback)

        self.sub_odom = node.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.sub_ref = node.create_subscription(Float64MultiArray, "/bebop/ref_vec", self.ref_callback, 10)
        self.sub_is_flying = node.create_subscription(Bool, "/bebop/is_flying", self.is_flying_callback, 10)
        self.pub_cmd = node.create_publisher(Twist, "/safe_bebop/cmd_vel", 10)
        self.pOdom = None

    def parameters_callback(self, params):
        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry):
        self.pOdom = msg

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 8: return
        self.ref_received = True
        w_data = np.array(msg.data)
        self.pPos.w_Xd, self.pPos.w_dXd = w_data[0:4], w_data[4:8]
        if not self.first_ref:
            self.pPos.w_ddXd = (w_data[4:8] - self.w_last_ref[4:8]) / self.dt
        else: self.first_ref = False
        self.w_last_ref = w_data

    def is_flying_callback(self, msg: Bool):
        self.is_flying = msg.data

    def rGetSensorData(self):
        if self.pOdom is None: return
        pose, twist = self.pOdom.pose.pose, self.pOdom.twist.twist
        qw, qx, qy, qz = pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
        w_yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        self.pPos.w_X[0:4] = [pose.position.x, pose.position.y, pose.position.z, w_yaw]
        b_twist = np.array([twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.z])
        w_F_b = np.array([[cos(w_yaw), -sin(w_yaw), 0, 0], [sin(w_yaw), cos(w_yaw), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.pPos.w_X[4:8] = w_F_b @ b_twist

    def cController(self):
        if not self.ref_received: return

        Ksp = np.diag([self.node.get_parameter(f'ksp_{a}').value for a in ['x','y','z','psi']])
        Ksd = np.diag([self.node.get_parameter(f'ksd_{a}').value for a in ['x','y','z','psi']])
        Kp  = np.diag([self.node.get_parameter(f'kp_{a}').value for a in ['x','y','z','psi']])
        Kd  = np.diag([self.node.get_parameter(f'kd_{a}').value for a in ['x','y','z','psi']])
        Ku = np.diag([self.pPar.Model_simp[0], self.pPar.Model_simp[2], self.pPar.Model_simp[4], self.pPar.Model_simp[6]])
        Kv = np.diag([self.pPar.Model_simp[1], self.pPar.Model_simp[3], self.pPar.Model_simp[5], self.pPar.Model_simp[7]])
        
        w_X, w_dX = self.pPos.w_X[0:4], self.pPos.w_X[4:8]
        w_Xd, w_dXd = self.pPos.w_Xd, self.pPos.w_dXd

        w_Xtil_raw = w_Xd - w_X
        w_Xtil_raw[3] = atan2(sin(w_Xtil_raw[3]), cos(w_Xtil_raw[3]))
        self.pPos.w_Xtil[0:3] = w_Xtil_raw[0:3]
        self.pPos.w_Xtil[3] = w_Xtil_raw[3]

        # Lógica de cambio dinámico de controlador basado en el error
        error_posicion = float(np.linalg.norm(self.pPos.w_Xtil[0:3]))
        
        if self.base_opt == 2:
            if self.opt == 2 and error_posicion > 0.40:
                self.opt = 3
                # Guardar los valores actuales del optimizador antes de cambiarlos
                self.saved_opt2_params = {
                    name: self.node.get_parameter(name).value 
                    for name in self.opt_params[2].keys()
                }
                new_params = [Parameter(k, Parameter.Type.DOUBLE, v) for k, v in self.opt_params[3].items()]
                self.node.set_parameters(new_params)
                self.node.get_logger().info(f"Error alto ({error_posicion:.2f}m > 0.40m): Cambiando a opt=3")
            
            elif self.opt == 3 and error_posicion < 0.05:
                self.opt = 2
                # Restaurar los valores exactos que tenía opt=2
                if hasattr(self, 'saved_opt2_params'):
                    new_params = [Parameter(k, Parameter.Type.DOUBLE, v) for k, v in self.saved_opt2_params.items()]
                    self.node.set_parameters(new_params)
                    self.node.get_logger().info(f"Error bajo ({error_posicion:.2f}m < 0.05m): Regresando a opt=2")

        w_Ur_ant = np.copy(self.pSC.w_Ur)
        w_Ur = (Kd @ w_dXd) + Ksp @ np.tanh(Kp @ self.pPos.w_Xtil)

        w_dUr = (w_Ur - w_Ur_ant) / self.dt
        self.pSC.w_Ur = np.copy(w_Ur)

        w_yaw = w_X[3]
        w_F_b = np.array([[cos(w_yaw), -sin(w_yaw), 0, 0], [sin(w_yaw), cos(w_yaw), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        b_Ud_raw = np.linalg.inv(w_F_b @ Ku) @ (w_dUr + Ksd @ (w_Ur - w_dX) + Kv @ w_dX)
        
        # Filtro pasa bajos para la acción de control
        if self.opt != 1:
            alpha = 0.1  # Peso del valor actual reducido de 0.3 a 0.1 para un filtrado mucho más fuerte
            self.pSC.b_Ud = alpha * b_Ud_raw + (1.0 - alpha) * self.pSC.b_Ud_ant
            self.pSC.b_Ud_ant = np.copy(self.pSC.b_Ud)
        else:
            self.pSC.b_Ud = np.copy(b_Ud_raw)

    def rSendControlSignals(self):
        if not self.ref_received or not self.is_flying: return
        b_cmd = Twist()
        b_cmd.linear.x = float(np.clip(self.pSC.b_Ud[0], -self.pPar.b_uSat[0], self.pPar.b_uSat[0]))
        b_cmd.linear.y = float(np.clip(self.pSC.b_Ud[1], -self.pPar.b_uSat[1], self.pPar.b_uSat[1]))
        b_cmd.linear.z = float(np.clip(self.pSC.b_Ud[2], -self.pPar.b_uSat[2], self.pPar.b_uSat[2]))
        b_cmd.angular.z = float(np.clip(self.pSC.b_Ud[3], -self.pPar.b_uSat[3], self.pPar.b_uSat[3]))
        self.pub_cmd.publish(b_cmd)

class NeroDroneNode(Node):
    def __init__(self):
        super().__init__("neroControl_node")
        self.drone = Bebop(self)
        time = 1/30
        self.create_timer(time, self.control_loop)

    def control_loop(self):
        self.drone.rGetSensorData()
        self.drone.cController()
        self.drone.rSendControlSignals()

def main(args=None):
    rclpy.init(args=args)
    node = NeroDroneNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()