#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
╔══════════════════════════════════════════════════════════════╗
║          DRONE CONTROLLER — PARAMETER OPTIMIZER             ║
║          Bayesian Optimization (Optuna TPE)                 ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  CÓMO FUNCIONA:                                              ║
║  ┌─────────────┐   /bebop/ref_vec   ┌──────────────────┐    ║
║  │  ref_pos.py │──────────────────▶│  neroControl_node│    ║
║  │  (corre     │                   │  (controlador)   │    ║
║  │   siempre)  │◀── set_params ────│                  │    ║
║  └─────────────┘        ▲          └────────┬─────────┘    ║
║                          │                   │ /cmd_vel     ║
║  ┌──────────────────┐    │          ┌────────▼─────────┐    ║
║  │  drone_optimizer │────┘          │    Drone/Gazebo  │    ║
║  │  (este archivo)  │◀─ odom ──────│                  │    ║
║  │  observa y mide  │◀─ ref  ──────│ /odometry/filter │    ║
║  └──────────────────┘              └──────────────────┘    ║
║                                                              ║
║  El optimizador NO controla los waypoints.                  ║
║  Solo observa cuánto error hay y ajusta los parámetros      ║
║  del controlador entre ciclos de 5 waypoints.               ║
╚══════════════════════════════════════════════════════════════╝

USO:
  Terminal 1:  ros2 run <pkg> neroControl_node
  Terminal 2:  python3 ref_pos.py
  Terminal 3:  python3 drone_optimizer.py

  Flags opcionales:
    --trials 60        Número de evaluaciones Optuna (default: 50)
    --cycles-per-trial 2   Ciclos completos por evaluación (default: 1)
    --resume           Continuar estudio guardado
    --dry-run          Solo imprime métricas, no cambia params

DEPENDENCIAS:
  pip install optuna
"""

import argparse
import json
import math
import os
import threading
import time
from collections import deque
from math import atan2, cos, sin, sqrt

import numpy as np
import optuna
from optuna.samplers import TPESampler

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Bool
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters

optuna.logging.set_verbosity(optuna.logging.WARNING)


# ═══════════════════════════════════════════════════════════════
#  CONFIGURACIÓN
# ═══════════════════════════════════════════════════════════════
class Config:
    # Nodo controlador al que se le cambian los parámetros
    CONTROLLER_NODE = "neroControl_node"

    # Número de waypoints que publica ref_pos.py
    # El optimizador espera este número de cambios para completar un ciclo
    N_WAYPOINTS = 5

    # Tiempo máximo esperado por waypoint (seg) — mismo que hold_time en ref_pos.py
    WAYPOINT_HOLD_TIME = 10.0

    # Umbral para detectar que la referencia cambió de waypoint
    REF_CHANGE_THRESHOLD = 0.08   # metros

    # Ventana de estado estacionario al final de cada waypoint (seg)
    SS_WINDOW = 3.0

    # Umbral de "llegada" al waypoint (metros)
    ARRIVAL_THRESHOLD = 0.12

    # ── Pesos de la función de costo ────────────────────────
    # Ajusta estos pesos para priorizar un comportamiento u otro:
    #   W_ITAE      → velocidad de llegada (más peso = llega más rápido)
    #   W_OVERSHOOT → estabilidad (más peso = menos oscilaciones)
    #   W_SS_ERROR  → mantenimiento (más peso = menos error en estado estable)
    #   W_SETTLE    → tiempo de establecimiento
    W_ITAE      = 1.0
    W_OVERSHOOT = 2.0
    W_SS_ERROR  = 3.0
    W_SETTLE    = 0.5

    # ── Espacio de búsqueda de parámetros ───────────────────
    BOUNDS = {
        'ksp_x':   (0.1, 2.5),
        'ksp_y':   (0.1, 2.5),
        'ksp_z':   (0.3, 4.0),
        'ksp_psi': (0.5, 6.0),
        'ksd_x':   (0.1, 3.0),
        'ksd_y':   (0.1, 3.0),
        'ksd_z':   (0.5, 6.0),
        'ksd_psi': (0.5, 8.0),
        'kp_x':    (0.2, 3.5),
        'kp_y':    (0.2, 3.5),
        'kp_z':    (0.2, 3.0),
        'kp_psi':  (0.2, 4.0),
        'kg1':     (0.5, 5.0),
        'kg2':     (0.5, 5.0),
    }

    # Archivos de salida
    BEST_PARAMS_FILE = "best_params.json"
    STUDY_DB         = "sqlite:///optuna_drone.db"


# ═══════════════════════════════════════════════════════════════
#  MÉTRICAS POR WAYPOINT
# ═══════════════════════════════════════════════════════════════
class WaypointMetrics:
    """
    Acumula datos de posición durante el tiempo que el drone
    intenta llegar a un waypoint y mantenerse en él.
    """
    def __init__(self, target: np.ndarray):
        self.target   = target      # [x, y, z, yaw_rad]
        self.errors   = []          # error escalar en cada tick
        self.times    = []          # tiempo relativo de cada tick
        self.t_start  = None
        self.arrived  = False
        self.t_arrive = None
        self.max_error_post_arrival = 0.0
        self._ss_buffer = deque()   # buffer de últimos SS_WINDOW seg

    def tick(self, pos: np.ndarray, t: float):
        """Llamado en cada muestra de odometría mientras este waypoint es activo."""
        if self.t_start is None:
            self.t_start = t

        rel_t = t - self.t_start
        err   = self._error(pos)

        self.errors.append(err)
        self.times.append(rel_t)

        # Detectar primera llegada al umbral
        if not self.arrived and err < Config.ARRIVAL_THRESHOLD:
            self.arrived  = True
            self.t_arrive = rel_t

        # Post-llegada: medir overshoot y estado estacionario
        if self.arrived:
            self.max_error_post_arrival = max(self.max_error_post_arrival, err)
            self._ss_buffer.append((rel_t, err))
            # Mantener solo la ventana SS_WINDOW
            while self._ss_buffer and (rel_t - self._ss_buffer[0][0]) > Config.SS_WINDOW:
                self._ss_buffer.popleft()

    def _error(self, pos: np.ndarray) -> float:
        """Error combinado posición + yaw."""
        dp   = pos[:3] - self.target[:3]
        dyaw = atan2(sin(pos[3] - self.target[3]), cos(pos[3] - self.target[3]))
        return sqrt(float(np.dot(dp, dp)) + 0.5 * dyaw**2)

    def cost(self) -> dict:
        """Calcula las 4 métricas de costo para este waypoint."""
        if not self.errors:
            return {'itae': 999.0, 'overshoot': 999.0, 'ss_error': 999.0,
                    'settle': Config.WAYPOINT_HOLD_TIME, 'total': 9999.0}

        t_arr  = np.array(self.times)
        e_arr  = np.array(self.errors)

        itae      = float(np.trapezoid(t_arr * e_arr, t_arr))
        overshoot = self.max_error_post_arrival
        ss_error  = (float(np.mean([e for _, e in self._ss_buffer]))
                     if self._ss_buffer else e_arr[-1])
        settle    = self.t_arrive if self.arrived else Config.WAYPOINT_HOLD_TIME

        total = (Config.W_ITAE      * itae
               + Config.W_OVERSHOOT * overshoot
               + Config.W_SS_ERROR  * ss_error
               + Config.W_SETTLE    * settle)

        return {'itae': itae, 'overshoot': overshoot,
                'ss_error': ss_error, 'settle': settle, 'total': total}


# ═══════════════════════════════════════════════════════════════
#  NODO ROS2 DEL OPTIMIZADOR
# ═══════════════════════════════════════════════════════════════
class OptimizerNode(Node):
    """
    Suscribe a /bebop/ref_vec y /odometry/filtered.
    Detecta cambios de waypoint y acumula métricas.
    Cuando completa un ciclo (N_WAYPOINTS waypoints), señaliza al
    hilo de Optuna que hay un costo listo para reportar.
    """

    def __init__(self, dry_run: bool = False):
        super().__init__("drone_optimizer_node")
        self.dry_run = dry_run

        # ── Estado del drone ────────────────────────────────
        self._pos        = np.zeros(4)    # [x, y, z, yaw]
        self._ref        = None           # referencia actual [x, y, z, yaw]
        self._is_flying  = False
        self._lock       = threading.Lock()

        # ── Estado de la medición ───────────────────────────
        self._wp_metrics_list  = []       # métricas acumuladas del ciclo actual
        self._current_wp_met   = None     # métricas del waypoint en curso
        self._wp_count         = 0        # waypoints vistos en el ciclo actual
        self._cycle_ready      = threading.Event()   # señal: ciclo completado
        self._cycle_cost_value = 0.0      # costo del último ciclo completado

        # ── Suscripciones ───────────────────────────────────
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self._odom_cb, 10)
        self.create_subscription(Float64MultiArray, "/bebop/ref_vec",
                                 self._ref_cb, 10)
        self.create_subscription(Bool, "/bebop/is_flying",
                                 self._flying_cb, 10)

        # ── Cliente de parámetros del controlador ───────────
        self._param_client = self.create_client(
            SetParameters, f"/{Config.CONTROLLER_NODE}/set_parameters")
        self.get_logger().info("Optimizer node iniciado — esperando controlador...")
        if not self._param_client.wait_for_service(timeout_sec=15.0):
            self.get_logger().error(
                f"Servicio de parámetros de '{Config.CONTROLLER_NODE}' no disponible.")

    # ── Callbacks ──────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        pose = msg.pose.pose
        qw, qx, qy, qz = (pose.orientation.w, pose.orientation.x,
                           pose.orientation.y, pose.orientation.z)
        yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))

        with self._lock:
            self._pos[:] = [pose.position.x, pose.position.y,
                            pose.position.z, yaw]
            # Actualizar métricas del waypoint actual
            if self._current_wp_met is not None:
                self._current_wp_met.tick(self._pos.copy(), time.time())

    def _ref_cb(self, msg: Float64MultiArray):
        if len(msg.data) != 8:
            return

        new_ref = np.array(msg.data[:4])   # [x, y, z, yaw]

        with self._lock:
            if self._ref is None:
                # Primera referencia recibida: arrancar medición
                self._ref = new_ref.copy()
                self._start_new_waypoint(new_ref)
                return

            # ¿Cambió el waypoint? (salto en la referencia)
            delta = np.linalg.norm(new_ref[:3] - self._ref[:3])
            if delta > Config.REF_CHANGE_THRESHOLD:
                # Cerrar el waypoint anterior
                self._close_current_waypoint()
                # Actualizar referencia y abrir nueva medición
                self._ref = new_ref.copy()
                self._start_new_waypoint(new_ref)

    def _flying_cb(self, msg: Bool):
        self._is_flying = msg.data

    # ── Gestión de waypoints ───────────────────────────────────
    def _start_new_waypoint(self, ref: np.ndarray):
        """Abre una nueva ventana de medición para el waypoint actual."""
        self._current_wp_met = WaypointMetrics(target=ref.copy())

    def _close_current_waypoint(self):
        """
        Cierra el waypoint en curso, guarda sus métricas y verifica
        si el ciclo está completo (N_WAYPOINTS waypoints vistos).
        """
        if self._current_wp_met is not None:
            self._wp_metrics_list.append(self._current_wp_met)
            self._current_wp_met = None

        self._wp_count += 1

        if self._wp_count >= Config.N_WAYPOINTS:
            # Ciclo completo: calcular costo total
            costs = [m.cost() for m in self._wp_metrics_list]
            total_cost = sum(c['total'] for c in costs) / len(costs)

            self._cycle_cost_value = total_cost
            self._print_cycle_summary(costs, total_cost)

            # Resetear para el siguiente ciclo
            self._wp_metrics_list = []
            self._wp_count        = 0

            # Señalizar al hilo de Optuna
            self._cycle_ready.set()

    def _print_cycle_summary(self, costs: list, total: float):
        lines = [f"\n  ── Ciclo completado | Costo medio: {total:.4f} ──"]
        headers = ('WP', 'ITAE', 'Overshoot', 'SS error', 'Settle(s)', 'Llegó?')
        lines.append(f"  {'WP':>3} {'ITAE':>7} {'Overshoot':>10} "
                     f"{'SS err':>8} {'Settle':>8}  Llegó")
        lines.append("  " + "─"*50)
        for i, c in enumerate(costs):
            ok = "✓" if c['settle'] < Config.WAYPOINT_HOLD_TIME else "✗"
            lines.append(f"  {i+1:>3} {c['itae']:>7.3f} {c['overshoot']:>10.4f} "
                         f"{c['ss_error']:>8.4f} {c['settle']:>8.2f}  {ok}")
        self.get_logger().info("\n".join(lines))

    # ── API pública para el hilo de Optuna ─────────────────────
    def wait_for_cycle(self, timeout: float) -> bool:
        """Bloquea hasta que un ciclo completo termine o timeout."""
        completed = self._cycle_ready.wait(timeout=timeout)
        if completed:
            self._cycle_ready.clear()
        return completed

    def get_cycle_cost(self) -> float:
        return self._cycle_cost_value

    def get_pos(self) -> np.ndarray:
        with self._lock:
            return self._pos.copy()

    def set_controller_params(self, params: dict) -> bool:
        """Envía parámetros al controlador vía servicio ROS2."""
        if self.dry_run:
            self.get_logger().info("  [dry-run] Params NO aplicados")
            return True

        req = SetParameters.Request()
        for name, value in params.items():
            p         = Parameter()
            p.name    = name
            p.value   = ParameterValue()
            p.value.type         = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(value)
            req.parameters.append(p)

        future   = self._param_client.call_async(req)
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.01)

        if not future.done():
            self.get_logger().warn("Timeout seteando parámetros")
            return False

        ok = all(r.successful for r in future.result().results)
        if not ok:
            self.get_logger().warn("Algunos parámetros no se pudieron setear")
        return ok


# ═══════════════════════════════════════════════════════════════
#  HILO DE OPTIMIZACIÓN (Optuna ask-and-tell)
# ═══════════════════════════════════════════════════════════════
def optimization_loop(node: OptimizerNode, study: optuna.Study,
                      n_trials: int, cycles_per_trial: int):
    """
    Corre en un hilo separado.
    Usa la interfaz ask-and-tell de Optuna para desacoplar la sugerencia
    de parámetros de la espera de resultados.

    Flujo por trial:
      1. Optuna sugiere parámetros  (ask)
      2. Aplicar params al controlador
      3. Esperar cycles_per_trial ciclos completos
      4. Reportar costo a Optuna     (tell)
    """
    node.get_logger().info(
        f"\n{'═'*58}\n"
        f"  INICIANDO OPTIMIZACIÓN\n"
        f"  Trials: {n_trials}  |  Ciclos por trial: {cycles_per_trial}\n"
        f"  Tiempo estimado: "
        f"~{n_trials * cycles_per_trial * Config.N_WAYPOINTS * Config.WAYPOINT_HOLD_TIME / 60:.0f} min\n"
        f"{'═'*58}"
    )

    # Warm-start: cargar params previos si existen
    if os.path.exists(Config.BEST_PARAMS_FILE):
        try:
            with open(Config.BEST_PARAMS_FILE) as f:
                saved = json.load(f)
            enqueue = {k: v for k, v in saved.items()
                       if not k.startswith('_') and k in Config.BOUNDS}
            study.enqueue_trial(enqueue)
            node.get_logger().info(
                f"  [Warm-start] Cargado {Config.BEST_PARAMS_FILE}")
        except Exception as e:
            node.get_logger().warn(f"  [Warm-start] Error: {e}")

    # Esperar a que el drone esté volando antes de empezar
    node.get_logger().info("  Esperando que el drone esté volando...")
    while not node._is_flying:
        time.sleep(0.5)
    node.get_logger().info("  Drone volando — iniciando trials")

    for trial_num in range(n_trials):
        # ── 1. Optuna sugiere parámetros ──
        trial  = study.ask()
        params = {name: trial.suggest_float(name, lo, hi)
                  for name, (lo, hi) in Config.BOUNDS.items()}

        node.get_logger().info(
            f"\n{'─'*58}\n"
            f"  Trial {trial_num+1}/{n_trials}\n"
            f"  ksp_x={params['ksp_x']:.3f}  ksp_z={params['ksp_z']:.3f}  "
            f"ksd_x={params['ksd_x']:.3f}  ksd_z={params['ksd_z']:.3f}\n"
            f"  kp_x={params['kp_x']:.3f}   kg1={params['kg1']:.3f}   "
            f"kg2={params['kg2']:.3f}\n"
            f"{'─'*58}"
        )

        # ── 2. Aplicar parámetros ──
        node.set_controller_params(params)

        # ── 3. Esperar ciclos de medición ──
        accumulated_cost = 0.0
        valid_cycles     = 0
        timeout_per_cycle = (Config.N_WAYPOINTS * Config.WAYPOINT_HOLD_TIME * 1.5)

        for cycle_idx in range(cycles_per_trial):
            completed = node.wait_for_cycle(timeout=timeout_per_cycle)
            if completed:
                accumulated_cost += node.get_cycle_cost()
                valid_cycles     += 1
            else:
                node.get_logger().warn(
                    f"  Trial {trial_num+1} ciclo {cycle_idx+1}: timeout")
                accumulated_cost += 9999.0
                valid_cycles     += 1

        cost = accumulated_cost / max(valid_cycles, 1)

        # ── 4. Reportar a Optuna ──
        study.tell(trial, cost)

        # Mostrar progreso
        best = study.best_value if study.best_trials else cost
        node.get_logger().info(
            f"  Trial {trial_num+1} → Costo: {cost:.4f}  "
            f"| Mejor hasta ahora: {best:.4f}")

    # Fin de la optimización
    _save_and_print_results(study, node)


# ═══════════════════════════════════════════════════════════════
#  GUARDAR Y MOSTRAR RESULTADOS
# ═══════════════════════════════════════════════════════════════
def _save_and_print_results(study: optuna.Study, node: OptimizerNode):
    if not study.best_trials:
        node.get_logger().warn("No hay trials completados.")
        return

    best = study.best_params.copy()
    best['_best_value'] = study.best_value
    best['_best_trial'] = study.best_trial.number

    with open(Config.BEST_PARAMS_FILE, 'w') as f:
        json.dump(best, f, indent=2)

    # Imprimir resultado
    sep = "═" * 58
    print(f"\n{sep}")
    print("  OPTIMIZACIÓN FINALIZADA")
    print(sep)
    print(f"  Mejor costo : {study.best_value:.4f}")
    print(f"  Trial #     : {study.best_trial.number + 1}")
    print(f"\n  Parámetros óptimos:")
    print("  " + "─"*54)
    for k, v in best.items():
        if not k.startswith('_'):
            print(f"    {k:<14s} = {v:.6f}")
    print(sep)
    print(f"\n  Guardado en: {Config.BEST_PARAMS_FILE}")

    # Bloque listo para pegar en el controlador
    print("\n  ── Pegar en initial_values del controlador (opt == 1): ──\n")
    print("  initial_values = {")
    groups = [
        ['ksp_x', 'ksp_y', 'ksp_z', 'ksp_psi'],
        ['ksd_x', 'ksd_y', 'ksd_z', 'ksd_psi'],
        ['kp_x',  'kp_y',  'kp_z',  'kp_psi'],
        ['kd_x',  'kd_y',  'kd_z',  'kd_psi'],
        ['kg1',   'kg2'],
    ]
    kd_defaults = {'kd_x': 0.0, 'kd_y': 0.0, 'kd_z': 0.0, 'kd_psi': 0.0}
    for group in groups:
        parts = []
        for k in group:
            v = best.get(k, kd_defaults.get(k, 0.0))
            parts.append(f"'{k}': {v:.4f}")
        print("      " + ",  ".join(parts) + ",")
    print("  }")

    # Importancia de parámetros
    if len(study.trials) >= 15:
        try:
            imp = optuna.importance.get_param_importances(study)
            print("\n  ── Importancia de parámetros: ──")
            for param, val in list(imp.items())[:10]:
                bar = "█" * int(val * 30)
                print(f"    {param:<14s} {bar:<30s} {val:.3f}")
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description="Drone Controller Optimizer — observa ref_pos.py y optimiza el controlador"
    )
    parser.add_argument('--trials',           type=int,   default=50,
                        help='Número de trials Optuna (default: 50)')
    parser.add_argument('--cycles-per-trial', type=int,   default=1,
                        help='Ciclos completos de 5 waypoints por trial (default: 1)')
    parser.add_argument('--resume',           action='store_true',
                        help='Continuar estudio guardado en SQLite')
    parser.add_argument('--dry-run',          action='store_true',
                        help='Solo mide métricas, no cambia parámetros del controlador')
    args = parser.parse_args()

    # ── Inicializar ROS2 ──────────────────────────────────────
    rclpy.init()
    node = OptimizerNode(dry_run=args.dry_run)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(2.0)  # Dar tiempo a que se establezcan las suscripciones

    # ── Crear estudio Optuna ──────────────────────────────────
    sampler = TPESampler(
        n_startup_trials=10,    # 10 trials aleatorios para explorar antes de explotar
        multivariate=True,      # modela correlaciones entre parámetros
        seed=42
    )

    study = optuna.create_study(
        study_name     = "drone_ctrl_opt",
        direction      = "minimize",
        sampler        = sampler,
        storage        = Config.STUDY_DB if args.resume else None,
        load_if_exists = args.resume,
    )

    # ── Lanzar el hilo de optimización ───────────────────────
    opt_thread = threading.Thread(
        target=optimization_loop,
        args=(node, study, args.trials, args.cycles_per_trial),
        daemon=True
    )
    opt_thread.start()

    try:
        opt_thread.join()
    except KeyboardInterrupt:
        print("\n\n  [!] Interrumpido — guardando resultados parciales...")
        if study.best_trials:
            _save_and_print_results(study, node)
    finally:
        executor.shutdown(wait=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()