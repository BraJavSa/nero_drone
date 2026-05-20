# Auxiliary script for handling small-scale variable optimizations.
# Auxiliary script for handling small-scale variable optimizations.
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

class DroneIntelligentOptimizer(Node):
    def __init__(self):
        super().__init__('drone_intelligent_optimizer')

        self.freq_odom = 30
        self.window_seconds = 10.0 
        self.buffer_size = int(self.freq_odom * self.window_seconds)

        self.axes = ['x', 'y', 'z', 'psi']
        self.params = {
            'ksp_x': 0.6, 'ksd_x': 1.5, 'kp_x': 0.3,
            'ksp_y': 0.6, 'ksd_y': 1.5, 'kp_y': 0.3,
            'ksp_z': 2.5, 'ksd_z': 5.5, 'kp_z': 0.8,
            'ksp_psi': 3.0, 'ksd_psi': 7.5, 'kp_psi': 0.6
        }

        self.LR = 0.03
        self.history = {axis: deque(maxlen=self.buffer_size) for axis in self.axes}
        self.current_ref = {axis: 0.0 for axis in self.axes}
        
        self.setup_logger()

        self.param_client = self.create_client(SetParameters, '/neroControl_node/set_parameters')
        
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_callback, 10)
        
        self.timer = self.create_timer(self.window_seconds, self.optimize_logic)
        self.get_logger().info("Nodo de Optimización started. Monitoreando /neroControl_node cada 10s.")

    def setup_logger(self):
        base_path = os.path.expanduser("~/ros2_ws/src/neroControl/data")
        os.makedirs(base_path, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(base_path, f"learning_log_10s_{timestamp}.csv")
        header = ['time']
        for axis in self.axes:
            header += [f'ksp_{axis}', f'ksd_{axis}', f'kp_{axis}', f'error_{axis}', f'osc_{axis}']
        with open(self.log_filename, 'w', newline='') as f:
            csv.writer(f).writerow(header)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        t = self.get_clock().now().nanoseconds / 1e9
        data = {
            'x': (msg.pose.pose.position.x, msg.twist.twist.linear.x),
            'y': (msg.pose.pose.position.y, msg.twist.twist.linear.y),
            'z': (msg.pose.pose.position.z, msg.twist.twist.linear.z),
            'psi': (yaw, msg.twist.twist.angular.z)
        }
        for axis in self.axes:
            pos, vel = data[axis]
            self.history[axis].append({'t': t, 'pos': pos, 'vel': vel, 'ref': self.current_ref[axis]})

    def ref_callback(self, msg):
        if len(msg.data) >= 4:
            for i, axis in enumerate(self.axes):
                self.current_ref[axis] = msg.data[i]

    def optimize_logic(self):
        if not self.param_client.service_is_ready():
            self.get_logger().warn("Esperando que el servicio /set_parameters esté disponible...")
            return

        new_params = []
        log_row = [self.get_clock().now().nanoseconds / 1e9]
        
        for axis in self.axes:
            hist = list(self.history[axis])
            if len(hist) < self.buffer_size:
                log_row += [self.params[f'ksp_{axis}'], self.params[f'ksd_{axis}'], self.params[f'kp_{axis}'], 0.0, 0.0]
                continue

            errors = [abs(h['pos'] - h['ref']) for h in hist]
            avg_error = np.mean(errors)
            vels = [h['vel'] for h in hist]
            zero_crossings = np.count_nonzero(np.diff(np.sign(vels)))
            max_pos = max([h['pos'] for h in hist]); min_pos = min([h['pos'] for h in hist])
            
            overshoot = max(0, max_pos - self.current_ref[axis]) if self.current_ref[axis] >= 0 else max(0, abs(min_pos) - abs(self.current_ref[axis]))

            if zero_crossings > 8:
                self.params[f'ksd_{axis}'] *= (1.0 + self.LR * 1.5); self.params[f'kp_{axis}'] *= 0.92
            elif overshoot > 0.05:
                self.params[f'ksd_{axis}'] *= (1.0 + self.LR * 0.8); self.params[f'kp_{axis}'] *= 0.97
            elif avg_error > 0.10:
                self.params[f'kp_{axis}'] *= (1.0 + self.LR * 1.0); self.params[f'ksp_{axis}'] *= (1.0 + self.LR * 0.7)

            self.params[f'kp_{axis}'] = np.clip(self.params[f'kp_{axis}'], 0.1, 7.0)
            self.params[f'ksd_{axis}'] = np.clip(self.params[f'ksd_{axis}'], 0.5, 12.0)
            self.params[f'ksp_{axis}'] = np.clip(self.params[f'ksp_{axis}'], 0.2, 4.0)

            log_row += [self.params[f'ksp_{axis}'], self.params[f'ksd_{axis}'], self.params[f'kp_{axis}'], avg_error, zero_crossings]

            for suffix in ['ksp', 'ksd', 'kp']:
                p_name = f'{suffix}_{axis}'
                new_params.append(Parameter(name=p_name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(self.params[p_name]))))

        with open(self.log_filename, 'a', newline='') as f:
            csv.writer(f).writerow(log_row)

        req = SetParameters.Request()
        req.parameters = new_params
        future = self.param_client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Éxito: {len(res.results)} parámetros actualizados en /neroControl_node.")
        except Exception as e:
            self.get_logger().error(f"Error al actualizar parámetros: {e}")

def main():
    rclpy.init()
    node = DroneIntelligentOptimizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()