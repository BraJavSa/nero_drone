# Quaternary optimization logic script.
# Quaternary optimization logic script.
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from tf_transformations import euler_from_quaternion
import numpy as np
from collections import deque
import csv
import os
from datetime import datetime

class SelectiveXYOptimizer(Node):
    def __init__(self):
        super().__init__('selective_xy_optimizer')

        self.freq_odom = 30
        self.window_seconds = 10.0 
        self.buffer_size = int(self.freq_odom * self.window_seconds)

        self.params = {
            'ksp_x': 1.01, 'ksd_x': 8.65, 'kp_x': 0.32,
            'ksp_y': 0.94, 'ksd_y': 8.14, 'kp_y': 0.598,
            
            'ksp_z': 1.8, 'ksd_z': 4.5, 'kp_z': 0.6,
            'ksp_psi': 3.0, 'ksd_psi': 5.5, 'kp_psi': 0.7,
            
            'kd_x': 0.0, 'kd_y': 0.0, 'kd_z': 0.0, 'kd_psi': 0.0
        }

        self.axes_to_optimize = ['x', 'y']
        self.LR = 0.1
        self.DEADZONE = 0.05 

        self.history = {axis: deque(maxlen=self.buffer_size) for axis in self.axes_to_optimize}
        self.current_ref = {axis: 0.0 for axis in self.axes_to_optimize}
        
        self.setup_logger()
        self.param_client = self.create_client(SetParameters, '/neroControl_node/set_parameters')
        
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        
        self.timer = self.create_timer(self.window_seconds, self.optimize_logic)
        self.get_logger().info("Optimizador Selectivo XY started con valores base de imagen.")

    def setup_logger(self):
        base_path = os.path.expanduser("~/ros2_ws/src/neroControl/data")
        os.makedirs(base_path, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(base_path, f"selective_xy_log_{timestamp}.csv")
        header = ['time'] + list(self.params.keys()) + ['error_x', 'error_y']
        with open(self.log_filename, 'w', newline='') as f:
            csv.writer(f).writerow(header)

    def odom_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        data = {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y}
        vel = {'x': msg.twist.twist.linear.x, 'y': msg.twist.twist.linear.y}
        for axis in self.axes_to_optimize:
            self.history[axis].append({'t': t, 'pos': data[axis], 'vel': vel[axis], 'ref': self.current_ref[axis]})

    def ref_callback(self, msg):
        if len(msg.data) >= 2:
            self.current_ref['x'] = msg.data[0]
            self.current_ref['y'] = msg.data[1]

    def optimize_logic(self):
        if not self.param_client.service_is_ready(): return

        log_row = [self.get_clock().now().nanoseconds / 1e9]
        errors_summary = {'x': 0.0, 'y': 0.0}
        
        for axis in self.axes_to_optimize:
            hist = list(self.history[axis])
            if len(hist) < self.buffer_size: continue

            errors = [abs(h['pos'] - h['ref']) for h in hist]
            avg_error = np.mean(errors)
            errors_summary[axis] = avg_error
            vel_std = np.std([h['vel'] for h in hist])
            
            max_pos = max([h['pos'] for h in hist]); min_pos = min([h['pos'] for h in hist])
            ref = self.current_ref[axis]
            overshoot = max(0, max_pos - ref) if ref >= 0 else max(0, abs(min_pos) - abs(ref))

            if vel_std > 0.08 and avg_error < 0.15:
                self.params[f'ksd_{axis}'] *= (1.0 + self.LR * 1.8)
                self.params[f'kp_{axis}'] *= 0.95
                self.get_logger().info(f"[{axis.upper()}] Reduciendo oscilación.")
            elif overshoot > 0.08:
                self.params[f'ksd_{axis}'] *= (1.0 + self.LR * 1.2)
                self.params[f'ksp_{axis}'] *= 0.96
                self.get_logger().info(f"[{axis.upper()}] Corrigiendo overshoot.")
            elif avg_error > self.DEADZONE and vel_std < 0.04:
                self.params[f'kp_{axis}'] *= (1.0 + self.LR * 1.0)
                self.get_logger().info(f"[{axis.upper()}] Ajustando error de posición.")

            self.params[f'kp_{axis}'] = np.clip(self.params[f'kp_{axis}'], 0.1, 1.5)
            self.params[f'ksd_{axis}'] = np.clip(self.params[f'ksd_{axis}'], 0.5, 8.0)
            self.params[f'ksp_{axis}'] = np.clip(self.params[f'ksp_{axis}'], 0.2, 3.0)

        new_params_list = []
        for p_name, p_value in self.params.items():
            new_params_list.append(Parameter(
                name=p_name,
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(p_value))
            ))
            log_row.append(p_value)

        log_row += [errors_summary['x'], errors_summary['y']]
        with open(self.log_filename, 'a', newline='') as f:
            csv.writer(f).writerow(log_row)

        self.send_params(new_params_list)

    def send_params(self, param_list):
        req = SetParameters.Request()
        req.parameters = param_list
        self.param_client.call_async(req)

    def response_callback(self, future):
        try:
            future.result()
            self.get_logger().info("Parámetros sincronizados exitosamente.")
        except Exception as e:
            self.get_logger().error(f"Error en comunicación: {e}")

def main():
    rclpy.init()
    node = SelectiveXYOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()