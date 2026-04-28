#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import numpy as np
import optuna
import threading
import time
from math import atan2, sin, cos

class FastOptimizer(Node):
    def __init__(self):
        super().__init__('fast_optimizer_node')
        
        # --- Configuración ---
        self.target_node = 'modelo_basado_node'
        self.n_waypoints_to_test = 4 # Ahora evaluamos los 4 puntos del ciclo
        self.start_evaluating = False 
        self.target_first_wp = np.array([0.8, 0.8, 1.5]) # L/2=0.8
        
        # --- Estado ---
        self.current_pos = np.zeros(4)
        self.current_ref = np.zeros(4)
        self.is_flying = False
        
        # --- Métricas ---
        self.trial_errors = []
        self.trial_times = []
        self.wp_count = 0
        self.last_ref = np.zeros(4)
        self.event_cycle_done = threading.Event()
        
        # --- ROS Comms ---
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_ref = self.create_subscription(Float64MultiArray, '/bebop/ref_vec', self.ref_cb, 10)
        self.sub_flying = self.create_subscription(Bool, '/bebop/is_flying', self.flying_cb, 10)
        self.client_params = self.create_client(SetParameters, f'/{self.target_node}/set_parameters')
        
        self.get_logger().info("Optimizador Rápido listo. Esperando despegue...")

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.current_pos = np.array([p.x, p.y, p.z, yaw])
        
        # Si estamos evaluando, acumular error
        if self.wp_count > 0:
            err = np.linalg.norm(self.current_ref[:3] - self.current_pos[:3])
            self.trial_errors.append(err)
            self.trial_times.append(time.time())

    def ref_cb(self, msg):
        if len(msg.data) < 4: return
        new_ref = np.array(msg.data[:4])
        
        # Detectar cambio brusco de referencia (nuevo waypoint)
        if np.linalg.norm(new_ref[:3] - self.last_ref[:3]) > 0.1:
            # Sincronización: Si no estamos evaluando, detectamos si es el inicio del ciclo
            if not self.start_evaluating:
                if np.linalg.norm(new_ref[:3] - self.target_first_wp) < 0.1:
                    self.start_evaluating = True
                    self.wp_count = 1
                    self.trial_errors = []
                    self.trial_times = []
                    self.get_logger().info("Sincronizado con el inicio del ciclo (Punto 1)")
            else:
                self.wp_count += 1
                self.get_logger().info(f"Waypoint {self.wp_count}/{self.n_waypoints_to_test} detectado")
            
            self.current_ref = new_ref.copy()
            if self.wp_count > self.n_waypoints_to_test:
                self.event_cycle_done.set()
        
        self.last_ref = new_ref.copy()

    def flying_cb(self, msg):
        self.is_flying = msg.data

    def set_params(self, params_dict):
        if not self.client_params.wait_for_service(timeout_sec=1.0):
            return False
        
        req = SetParameters.Request()
        for name, val in params_dict.items():
            p = Parameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(val)
            req.parameters.append(p)
        
        self.client_params.call_async(req)
        return True

def objective(trial, node):
    # --- Espacio de Búsqueda (Solo X e Y) ---
    ksp_x  = trial.suggest_float('ksp_x', 0.5, 3.0)
    ksp_y  = trial.suggest_float('ksp_y', 0.5, 3.0)
    ksd_x  = trial.suggest_float('ksd_x', 0.5, 5.0)
    ksd_y  = trial.suggest_float('ksd_y', 0.5, 5.0)
    kp_x   = trial.suggest_float('kp_x', 1.0, 4.0)
    kp_y   = trial.suggest_float('kp_y', 1.0, 4.0)
    
    params = {
        'ksp_x': ksp_x, 'ksp_y': ksp_y,
        'ksd_x': ksd_x, 'ksd_y': ksd_y,
        'kp_x': kp_x,   'kp_y': kp_y,
    }
    
    # Aplicar al dron
    node.set_params(params)
    
    # Reiniciar métricas y esperar sincronización con el inicio del ciclo
    node.wp_count = 0
    node.start_evaluating = False 
    node.trial_errors = []
    node.trial_times = []
    node.event_cycle_done.clear()
    
    node.get_logger().info(f"Evaluando Trial {trial.number}: X({ksp_x:.2f},{ksd_x:.2f},{kp_x:.2f}) Y({ksp_y:.2f},{ksd_y:.2f},{kp_y:.2f})")
    node.get_logger().info("Esperando a que ref_pos.py reinicie el ciclo (Punto 1)...")
    
    # Esperar a que el script de referencia haga su trabajo (aumentado el timeout porque tiene que esperar al reset)
    if not node.event_cycle_done.wait(timeout=60.0):
        return 9999.0
    
    errors = np.array(node.trial_errors)
    times = np.array(node.trial_times)
    if len(errors) < 50: return 9999.0
    
    dt = np.diff(times, prepend=times[0])
    
    # 1. ITAE (Velocidad de respuesta)
    itae = np.sum((times - times[0]) * errors * dt)
    
    # 2. Overshoot y Oscilaciones Estacionarias
    # Dividimos los errores por waypoints (aproximadamente)
    # Buscamos el overshoot después de llegar a un umbral
    overshoot_total = 0
    steady_state_oscillation = 0
    
    # Umbral de "llegada"
    arrival_thresh = 0.25
    
    # Analizamos los últimos ticks de cada ciclo (estado estacionario)
    # Asumiendo que el drone se queda en el punto un tiempo
    # Tomamos muestras de los últimos 2 segundos de cada waypoint
    samples_per_wp = len(errors) // node.n_waypoints_to_test
    for i in range(node.n_waypoints_to_test):
        wp_slice = errors[i*samples_per_wp : (i+1)*samples_per_wp]
        if len(wp_slice) < 10: continue
        
        # Detectar primera llegada
        arrivals = np.where(wp_slice < arrival_thresh)[0]
        if len(arrivals) > 0:
            idx_first = arrivals[0]
            # Overshoot: máximo error después de la primera entrada al umbral
            post_arrival = wp_slice[idx_first:]
            if len(post_arrival) > 0:
                overshoot_total += np.max(post_arrival)
            
            # Oscilaciones en el punto: Desviación estándar de los últimos 30 ticks
            if len(wp_slice) > 30:
                steady_state_oscillation += np.std(wp_slice[-30:])

    cost = itae + (10.0 * overshoot_total) + (50.0 * steady_state_oscillation)
    
    node.get_logger().info(f"Trial {trial.number} -> Cost: {cost:.2f} (ITAE: {itae:.1f}, OS: {overshoot_total:.2f}, OSC: {steady_state_oscillation:.2f})")
    return cost

def main():
    rclpy.init()
    node = FastOptimizer()
    
    # Spinear el nodo en un hilo separado
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    # Esperar a que esté volando
    while not node.is_flying:
        print("Esperando a que el dron despegue...")
        time.sleep(1.0)
    
    # Optuna Study
    study = optuna.create_study(direction='minimize')

    # --- Punto de partida: Cargar valores actuales del nodo ---
    from rcl_interfaces.srv import GetParameters
    client_get = node.create_client(GetParameters, f'/{node.target_node}/get_parameters')
    if client_get.wait_for_service(timeout_sec=2.0):
        req = GetParameters.Request()
        req.names = ['ksp_x', 'ksp_y', 'ksd_x', 'ksd_y', 'kp_x', 'kp_y']
        future = client_get.call_async(req)
        
        # Esperar resultado
        start_t = time.time()
        while not future.done() and (time.time() - start_t) < 2.0:
            time.sleep(0.1)
        
        if future.done():
            res = future.result()
            initial_params = {}
            for i, p_val in enumerate(res.values):
                initial_params[req.names[i]] = p_val.double_value
            
            node.get_logger().info(f"Punto de partida detectado: {initial_params}")
            study.enqueue_trial(initial_params)

    try:
        study.optimize(lambda t: objective(t, node), n_trials=40)
    except KeyboardInterrupt:
        pass
    
    print("\n--- OPTIMIZACIÓN FINALIZADA ---")
    print("Mejores parámetros encontrados:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value:.4f}")
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
