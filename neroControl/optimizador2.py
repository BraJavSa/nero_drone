#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

class PolicyNetwork(nn.Module):
    def __init__(self, n_inputs, n_outputs):
        super(PolicyNetwork, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(n_inputs, 128),
            nn.LayerNorm(128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, n_outputs),
            nn.Tanh()
        )
    def forward(self, x): return self.net(x)

class NeroConstrainedOptimizer(Node):
    def __init__(self):
        super().__init__('nero_constrained_optimizer')
        self.device = torch.device("cpu")
        
        self.model = PolicyNetwork(18, 6).to(self.device)
        self.optimizer = optim.Adam(self.model.parameters(), lr=0.003)
        
        # Referencia de parametros base segun configuracion optima inicial
        self.base_params = {
            'ksp_x': 0.1321, 'ksd_x': 0.3908, 'kp_x': 1.6856,
            'ksp_y': 0.9917, 'ksd_y': 0.6230, 'kp_y': 1.2564
        }

        
        self.trial_params = np.array([0.1321, 0.3908, 1.6856, 0.9917, 0.6230, 1.2564])
        self.history = {'err_x':[], 'err_y':[], 'cmd_x':[], 'cmd_y':[], 'v_ref':[], 'v_real':[]}
        self.current_ref = np.zeros(8)
        self.current_v_real = np.zeros(4)
        
        self.base_error_xy = None 
        self.error_threshold = None
        self.is_calibrating = True
        self.is_recovering = False
        self.last_state = None
        self.best_cost = float('inf')
        self.evaluation_count = 0
        
        self.client = self.create_client(SetParameters, '/neroControl_node/set_parameters')
        self.sub_odom = self.create_subscription(Odometry, "/odometry/filtered", self.odom_cb, 10)
        self.sub_ref = self.create_subscription(Float64MultiArray, "/bebop/ref_vec", self.ref_cb, 10)
        self.sub_cmd = self.create_subscription(Twist, "/safe_bebop/cmd_vel", self.cmd_cb, 10)

        self.state_counter = 0
        self.create_timer(1.0, self.master_timer_callback)
        self.apply_params(self.base_params)

    def odom_cb(self, msg):
        px, py = msg.pose.pose.position.x, msg.pose.pose.position.y
        if not self.is_recovering:
            self.history['err_x'].append(abs(self.current_ref[0] - px))
            self.history['err_y'].append(abs(self.current_ref[1] - py))
        self.current_v_real = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, 
                                        msg.twist.twist.linear.z, msg.twist.twist.angular.z])

    def ref_cb(self, msg):
        if len(msg.data) == 8: self.current_ref = np.array(msg.data)

    def cmd_cb(self, msg):
        if not self.is_recovering:
            self.history['cmd_x'].append(msg.linear.x)
            self.history['cmd_y'].append(msg.linear.y)
            self.history['v_ref'].append(self.current_ref[4:8])
            self.history['v_real'].append(self.current_v_real)

    def master_timer_callback(self):
        self.state_counter += 1
        
        if self.is_calibrating:
            if self.state_counter >= 20: 
                self.base_error_xy = np.mean(self.history['err_x']) + np.mean(self.history['err_y'])
                self.error_threshold = self.base_error_xy * 1.5
                self.is_calibrating = False
                self.state_counter = 0
                self.get_logger().info(f"CALIBRACION FINALIZADA. Umbral: {self.error_threshold:.4f}")
                self.reset_history()
            return

        # Fin de evaluacion (20 segundos)
        if self.state_counter == 20:
            self.get_logger().info("PROCESANDO RESULTADOS DE EVALUACION")
            self.perform_optimization_step()
            self.is_recovering = True
            self.apply_params(self.base_params)
            
        # Fin de recuperacion (8 segundos)
        elif self.state_counter >= 28:
            self.evaluation_count += 1
            self.is_recovering = False
            self.state_counter = 0
            self.get_logger().info(f"INICIANDO ITERACION NUMERO {self.evaluation_count}")
            trial_dict = {
                'ksp_x': self.trial_params[0], 'ksd_x': self.trial_params[1], 'kp_x': self.trial_params[2],
                'ksp_y': self.trial_params[3], 'ksd_y': self.trial_params[4], 'kp_y': self.trial_params[5]
            }
            self.apply_params(trial_dict)
            self.reset_history()

    def perform_optimization_step(self):
        state, cost, current_err = self.get_constrained_metrics()
        if state is None: return

        if current_err > self.error_threshold:
            self.get_logger().info(f"DESCARTADO: ERROR FUERA DE RANGO ({current_err:.4f})")
        elif cost >= self.best_cost:
            self.get_logger().info(f"DESCARTADO: COSTO NO OPTIMO ({cost:.4f})")
        else:
            self.best_cost = cost
            self.print_best_set(cost, current_err)

        if self.last_state is not None:
            reward = -cost
            loss = -(self.model(self.last_state).mean() * reward)
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()

        with torch.no_grad():
            action = self.model(state).numpy()
        
        self.trial_params = np.clip(self.trial_params + (action * 0.15), 0.1, 4.5)
        self.last_state = state

    def get_constrained_metrics(self):
        if len(self.history['cmd_x']) < 100: return None, None, None
        
        current_err = np.mean(self.history['err_x']) + np.mean(self.history['err_y'])
        jit = [np.var(np.diff(self.history['cmd_x'])), np.var(np.diff(self.history['cmd_y']))]
        v_ref = np.mean(self.history['v_ref'], axis=0)
        v_real = np.mean(self.history['v_real'], axis=0)

        if current_err <= self.error_threshold:
            cost = (jit[0] + jit[1]) * 5000.0 
        else:
            diff = (current_err - self.error_threshold)
            cost = (jit[0] + jit[1]) * 5000.0 + (diff * 50000.0) + (diff**2 * 100000.0)

        return torch.tensor(np.concatenate([[current_err, 0, jit[0]*100, jit[1]*100], v_ref, v_real, self.trial_params]).astype(np.float32), device=self.device), cost, current_err

    def print_best_set(self, cost, error):
        print("\n" + "="*60)
        print("NUEVA CONFIGURACION OPTIMA DETECTADA")
        print(f"COSTO: {cost:.6f} | ERROR: {error:.4f}")
        print("-" * 60)
        print(f"X: ksp_x: {self.trial_params[0]:.4f}, ksd_x: {self.trial_params[1]:.4f}, kp_x: {self.trial_params[2]:.4f}")
        print(f"Y: ksp_y: {self.trial_params[3]:.4f}, ksd_y: {self.trial_params[4]:.4f}, kp_y: {self.trial_params[5]:.4f}")
        print("="*60 + "\n")

    def apply_params(self, p_dict):
        if not self.client.wait_for_service(timeout_sec=0.5): return
        req = SetParameters.Request()
        for name, val in p_dict.items():
            req.parameters.append(Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(val))))
        self.client.call_async(req)

    def reset_history(self):
        for k in self.history: self.history[k] = []

def main():
    rclpy.init()
    node = NeroConstrainedOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()