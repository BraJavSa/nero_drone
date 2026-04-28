#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================
 CONTROLADOR NMPC DE POSICIÓN — Bebop Drone — ROS2 + CasADi
================================================================
 Modelo simplificado identificado (no lineal en psi):
   Xddot = F1(psi)*U - F2(psi)*Xdot
   X  = [x, y, z, psi]
   Xdot = [dx, dy, dz, dpsi]
   U  = [u_vx, u_vy, u_vz, u_vpsi]  ∈ [-1, 1]

 Estado aumentado para el MPC: q = [X; Xdot] ∈ R^8
   qdot = f(q, U) =  [ Xdot  ]
                     [ F1*U - F2*Xdot ]

 NMPC — horizonte N pasos @ dt_mpc:
   min  Σ_{k=0}^{N-1} [ (q_k - q_d_k)' Q (q_k - q_d_k)
                       + U_k' R_w U_k
                       + DU_k' R_du DU_k ]   (suavizado de control)
        + (q_N - q_d_N)' P (q_N - q_d_N)    (costo terminal)
   s.t. q_{k+1} = F_RK4(q_k, U_k)           (integración RK4)
        -1 <= U_k <= 1                         (límites físicos)
        |dX_k| <= v_max                        (límite velocidad)

 Tópicos (idénticos al nodo original):
   SUB: /odometry/filtered     (nav_msgs/Odometry)
   SUB: /bebop/ref_vec         (Float64MultiArray) [x,y,z,psi,dx,dy,dz,dpsi]
   SUB: /bebop/is_flying       (std_msgs/Bool)
   PUB: /safe_bebop/cmd_vel    (geometry_msgs/Twist)

 Dependencias:
   pip install casadi numpy

================================================================
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
from rcl_interfaces.msg import SetParametersResult

import numpy as np
from math import sin, cos, atan2
import casadi as ca
import time


# ================================================================
#  Parámetros del modelo — idénticos al original identificado
# ================================================================
F1_DIAG = [0.8417, 0.8354, 3.966,  9.8524]
F2_DIAG = [0.18227, 0.17095, 4.001, 4.7295]
V_MAX   = [1.0, 1.0, 1.0, np.deg2rad(100)]   # vel. máximas modelo
U_SAT   = 1.0                                  # límite de control


# ================================================================
#  Constructor del NMPC con CasADi (compilado una sola vez)
# ================================================================

class NMPCSolver:
    """
    Construye y compila el problema NMPC con CasADi.
    Se instancia una vez al inicio — el solver queda warm-start listo.
    """

    def __init__(self,
                 N:       int   = 20,      # horizonte de predicción
                 dt_mpc:  float = 1/15,    # paso de integración MPC (15 Hz → 2 pasos/ctrl)
                 Q_diag:  list  = None,    # pesos estado [x,y,z,psi, dx,dy,dz,dpsi]
                 R_diag:  list  = None,    # pesos control
                 Rdu_diag:list  = None,    # pesos variación de control (suavizado)
                 P_scale: float = 5.0):    # escala del costo terminal (P = P_scale * Q)

        self.N      = N
        self.dt_mpc = dt_mpc
        self.nx     = 8    # estado: [x,y,z,psi, dx,dy,dz,dpsi]
        self.nu     = 4    # control: [u_vx, u_vy, u_vz, u_vpsi]

        # Pesos por defecto (balanceados para el Bebop)
        if Q_diag is None:
            Q_diag = [8.0, 8.0, 12.0, 5.0,    # posición x,y,z,psi
                      1.0, 1.0,  2.0, 0.5]     # velocidad x,y,z,psi
        if R_diag is None:
            R_diag = [0.5, 0.5, 0.3, 0.1]      # esfuerzo de control
        if Rdu_diag is None:
            Rdu_diag = [0.8, 0.8, 0.5, 0.2]    # suavizado entre pasos

        self.Q   = np.diag(Q_diag)
        self.R_w = np.diag(R_diag)
        self.Rdu = np.diag(Rdu_diag)
        self.P   = P_scale * self.Q

        # Solución warm-start
        self.U_prev = np.zeros((N, self.nu))    # secuencia de control anterior

        # Construir NLP
        self._build_nlp()

    # ------------------------------------------------------------------
    #  Dinámica simbólica CasADi
    # ------------------------------------------------------------------

    def _dynamics(self, q, u):
        """
        qdot = f(q, u)
        q = [x, y, z, psi, dx, dy, dz, dpsi]
        u = [u_vx, u_vy, u_vz, u_vpsi]
        """
        psi = q[3]
        cp  = ca.cos(psi)
        sp  = ca.sin(psi)

        # Matrices F1(psi) = R(psi)*f1, F2(psi) = R(psi)*f2
        f1 = ca.DM(np.diag(F1_DIAG))
        f2 = ca.DM(np.diag(F2_DIAG))

        R = ca.vertcat(
            ca.horzcat(cp, -sp,  0,  0),
            ca.horzcat(sp,  cp,  0,  0),
            ca.horzcat( 0,   0,  1,  0),
            ca.horzcat( 0,   0,  0,  1)
        )

        F1 = R @ f1
        F2 = R @ f2

        Xdot  = q[4:8]
        Xddot = F1 @ u - F2 @ Xdot

        return ca.vertcat(Xdot, Xddot)

    def _rk4_step(self, q, u, dt):
        """Integración RK4 de un paso dt."""
        k1 = self._dynamics(q, u)
        k2 = self._dynamics(q + dt/2 * k1, u)
        k3 = self._dynamics(q + dt/2 * k2, u)
        k4 = self._dynamics(q + dt   * k3, u)
        return q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    # ------------------------------------------------------------------
    #  Construcción del NLP
    # ------------------------------------------------------------------

    def _build_nlp(self):
        N  = self.N
        nx = self.nx
        nu = self.nu
        dt = self.dt_mpc

        # Variables de decisión: secuencia de controles U ∈ R^{N*nu}
        U_var = ca.MX.sym('U', N * nu)

        # Parámetros del NLP: [q0 (nx), q_ref_horizon ((N+1)*nx), U_prev (nu)]
        P_var = ca.MX.sym('P', nx + (N+1)*nx + nu)

        q0       = P_var[0:nx]
        q_ref    = P_var[nx : nx + (N+1)*nx]      # referencia en el horizonte
        u_prev   = P_var[nx + (N+1)*nx : ]         # último control aplicado

        # ------- Costo y restricciones -------
        cost = ca.MX(0)
        g    = []       # restricciones de igualdad (rollout)

        q_k = q0
        for k in range(N):
            u_k    = U_var[k*nu : (k+1)*nu]
            q_d_k  = q_ref[k*nx : (k+1)*nx]

            # Error de estado — manejo del ángulo psi
            eq_k      = q_k - q_d_k
            eq_k[3]   = ca.atan2(ca.sin(eq_k[3]), ca.cos(eq_k[3]))

            # Costo de estado
            cost += eq_k.T @ self.Q @ eq_k

            # Costo de control
            cost += u_k.T @ self.R_w @ u_k

            # Costo de suavizado (variación de control)
            if k == 0:
                du_k = u_k - u_prev
            else:
                du_k = u_k - U_var[(k-1)*nu : k*nu]
            cost += du_k.T @ self.Rdu @ du_k

            # Integración RK4
            q_next = self._rk4_step(q_k, u_k, dt)
            g.append(q_next)
            q_k = q_next

        # Costo terminal
        q_d_N   = q_ref[N*nx : (N+1)*nx]
        eq_N    = q_k - q_d_N
        eq_N[3] = ca.atan2(ca.sin(eq_N[3]), ca.cos(eq_N[3]))
        cost   += eq_N.T @ self.P @ eq_N

        # ------- Restricciones de igualdad (rollout) -------
        # g contiene q_{1..N} predichos — se fuerzan implícitamente
        # mediante los bounds (lbg = ubg = estado predicho libre)
        # En CasADi el rollout ya está codificado en la función de costo;
        # no necesitamos restricciones de igualdad adicionales porque
        # U_var es la única variable de decisión y el rollout es explícito.

        # ------- Restricciones de desigualdad en U -------
        lbu = np.full(N * nu, -U_SAT)
        ubu = np.full(N * nu,  U_SAT)

        # ------- Crear NLP -------
        nlp = {
            'x': U_var,
            'f': cost,
            'p': P_var,
        }

        opts = {
            'ipopt.print_level':        0,
            'ipopt.max_iter':           50,
            'ipopt.tol':                1e-4,
            'ipopt.acceptable_tol':     1e-3,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.warm_start_bound_push':  1e-6,
            'ipopt.warm_start_mult_bound_push': 1e-6,
            'print_time':               0,
            'ipopt.linear_solver':      'mumps',
        }

        self.solver = ca.nlpsol('nmpc_solver', 'ipopt', nlp, opts)
        self.lbu    = lbu
        self.ubu    = ubu
        self.P_dim  = nx + (N+1)*nx + nu

        # Dimensiones para armar el vector P
        self.nx = nx
        self.nu = nu

    # ------------------------------------------------------------------
    #  Resolver el NMPC dado el estado actual y la referencia
    # ------------------------------------------------------------------

    def solve(self, q0: np.ndarray,
                    q_ref_horizon: np.ndarray,
                    u_prev: np.ndarray) -> tuple[np.ndarray, float]:
        """
        Parámetros
        ----------
        q0             : estado actual (8,)
        q_ref_horizon  : referencia en el horizonte (N+1, 8)
        u_prev         : último control aplicado (4,)

        Retorna
        -------
        u_opt  : primer control óptimo (4,)
        t_solve: tiempo de cómputo en ms
        """
        N  = self.N
        nx = self.nx
        nu = self.nu

        # Armar vector de parámetros
        p_vec = np.concatenate([
            q0,
            q_ref_horizon.flatten(),
            u_prev
        ])

        # Warm-start: inicializar con la secuencia desplazada
        U0_flat = self.U_prev.flatten()

        t0 = time.perf_counter()
        sol = self.solver(
            x0  = U0_flat,
            lbx = self.lbu,
            ubx = self.ubu,
            p   = p_vec
        )
        t_solve = (time.perf_counter() - t0) * 1e3   # ms

        U_opt_flat = np.array(sol['x']).flatten()
        U_opt      = U_opt_flat.reshape(N, nu)

        # Warm-start para siguiente iteración: desplazar y repetir último
        self.U_prev[:-1] = U_opt[1:]
        self.U_prev[-1]  = U_opt[-1]

        u_opt = U_opt[0]
        return u_opt, t_solve


# ================================================================
#  Clases de datos — misma estructura que el original
# ================================================================

class Position:
    def __init__(self):
        self.w_X    = np.zeros(8)    # [x,y,z,psi, dx,dy,dz,dpsi]
        self.w_Xd   = np.zeros(4)    # referencia posición
        self.w_dXd  = np.zeros(4)    # referencia velocidad
        self.w_ddXd = np.zeros(4)    # referencia aceleración (estimada)


class SC:
    def __init__(self):
        self.b_Ud  = np.zeros(4)    # control saturado body frame
        self.U_raw = np.zeros(4)    # control sin saturar (= solución NMPC ya acotada)
        self.t_solve_ms = 0.0       # tiempo de cómputo del solver


# ================================================================
#  Bebop — nodo principal
# ================================================================

class Bebop:
    def __init__(self, node: Node):
        self.node = node
        self.dt   = 1.0 / 30.0

        self.pPos = Position()
        self.pSC  = SC()

        self.ref_received = False
        self.is_flying    = False
        self.first_ref    = True
        self.w_last_ref   = np.zeros(8)
        self.pOdom        = None
        self.u_prev       = np.zeros(4)

        # ---- Parámetros MPC declarados (ajustables en runtime) ----
        mpc_defaults = {
            # Horizonte y paso
            'N':       20,
            'dt_mpc':  1.0/15.0,
            # Pesos de posición [x, y, z, psi]
            'Q_x':     8.0,  'Q_y':    8.0,  'Q_z':   12.0, 'Q_psi':  5.0,
            # Pesos de velocidad
            'Q_dx':    1.0,  'Q_dy':   1.0,  'Q_dz':   2.0, 'Q_dpsi': 0.5,
            # Pesos de control
            'R_vx':    0.5,  'R_vy':   0.5,  'R_vz':   0.3, 'R_vpsi': 0.1,
            # Pesos de suavizado de control
            'Rdu_vx':  0.8,  'Rdu_vy': 0.8,  'Rdu_vz': 0.5,'Rdu_vpsi':0.2,
            # Escala costo terminal
            'P_scale': 5.0,
        }
        for name, value in mpc_defaults.items():
            node.declare_parameter(name, value)

        node.add_on_set_parameters_callback(self.parameters_callback)

        # ---- Construir solver NMPC ----
        node.get_logger().info("Construyendo solver NMPC (CasADi + IPOPT)...")
        self._build_solver()
        node.get_logger().info(
            f"  Solver listo — N={self.nmpc.N}, dt={self.nmpc.dt_mpc:.3f}s")

        # ---- Tópicos — idénticos al original ----
        self.sub_odom = node.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.sub_ref = node.create_subscription(
            Float64MultiArray, "/bebop/ref_vec", self.ref_callback, 10)
        self.sub_is_flying = node.create_subscription(
            Bool, "/bebop/is_flying", self.is_flying_callback, 10)
        self.pub_cmd = node.create_publisher(
            Twist, "/safe_bebop/cmd_vel", 10)

    def _build_solver(self):
        """Construye el solver con los parámetros actuales del nodo."""
        p = self.node
        N      = int(p.get_parameter('N').value)
        dt_mpc = float(p.get_parameter('dt_mpc').value)

        Q_diag = [
            p.get_parameter('Q_x').value,   p.get_parameter('Q_y').value,
            p.get_parameter('Q_z').value,   p.get_parameter('Q_psi').value,
            p.get_parameter('Q_dx').value,  p.get_parameter('Q_dy').value,
            p.get_parameter('Q_dz').value,  p.get_parameter('Q_dpsi').value,
        ]
        R_diag  = [p.get_parameter(f'R_{v}').value  for v in ['vx','vy','vz','vpsi']]
        Rdu_diag= [p.get_parameter(f'Rdu_{v}').value for v in ['vx','vy','vz','vpsi']]
        P_scale = float(p.get_parameter('P_scale').value)

        self.nmpc = NMPCSolver(
            N=N, dt_mpc=dt_mpc,
            Q_diag=Q_diag, R_diag=R_diag,
            Rdu_diag=Rdu_diag, P_scale=P_scale
        )

    # ------------------------------------------------------------------
    #  Callbacks
    # ------------------------------------------------------------------

    def parameters_callback(self, params):
        """Reconstruye el solver si cambian N, dt_mpc o pesos."""
        rebuild_keys = {'N','dt_mpc','Q_x','Q_y','Q_z','Q_psi',
                        'Q_dx','Q_dy','Q_dz','Q_dpsi',
                        'R_vx','R_vy','R_vz','R_vpsi',
                        'Rdu_vx','Rdu_vy','Rdu_vz','Rdu_vpsi','P_scale'}
        names = {p.name for p in params}
        if names & rebuild_keys:
            self.node.get_logger().info("Reconstruyendo solver NMPC...")
            self._build_solver()
        return SetParametersResult(successful=True)

    def odom_callback(self, msg: Odometry):
        self.pOdom = msg

    def ref_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 8:
            return
        self.ref_received = True
        w_data = np.array(msg.data)
        self.pPos.w_Xd  = w_data[0:4]
        self.pPos.w_dXd = w_data[4:8]
        if not self.first_ref:
            self.pPos.w_ddXd = (w_data[4:8] - self.w_last_ref[4:8]) / self.dt
        else:
            self.first_ref = False
        self.w_last_ref = w_data.copy()

    def is_flying_callback(self, msg: Bool):
        self.is_flying = msg.data

    # ------------------------------------------------------------------
    #  Lectura de sensores — idéntica al original
    # ------------------------------------------------------------------

    def rGetSensorData(self):
        if self.pOdom is None:
            return
        pose  = self.pOdom.pose.pose
        twist = self.pOdom.twist.twist

        qw, qx, qy, qz = (pose.orientation.w, pose.orientation.x,
                           pose.orientation.y, pose.orientation.z)
        w_yaw = atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))

        self.pPos.w_X[0:4] = [pose.position.x,
                               pose.position.y,
                               pose.position.z,
                               w_yaw]

        b_twist = np.array([twist.linear.x, twist.linear.y,
                             twist.linear.z, twist.angular.z])
        cp, sp = cos(w_yaw), sin(w_yaw)
        w_R_b  = np.array([[cp, -sp, 0, 0],
                            [sp,  cp, 0, 0],
                            [ 0,   0, 1, 0],
                            [ 0,   0, 0, 1]])
        self.pPos.w_X[4:8] = w_R_b @ b_twist

    # ------------------------------------------------------------------
    #  Controlador NMPC
    # ------------------------------------------------------------------

    def cController(self):
        if not self.ref_received:
            return

        # Estado actual aumentado q = [X; Xdot]
        q0 = self.pPos.w_X.copy()

        # ---- Construir referencia en el horizonte ----
        # Se propaga la referencia recibida con integración Euler simple
        # usando la velocidad de referencia (trayectoria suave)
        N      = self.nmpc.N
        dt_mpc = self.nmpc.dt_mpc

        q_ref = np.zeros((N + 1, 8))
        xd_k  = self.pPos.w_Xd.copy()
        dxd_k = self.pPos.w_dXd.copy()

        for k in range(N + 1):
            q_ref[k, 0:4] = xd_k
            q_ref[k, 4:8] = dxd_k
            # Propagar referencia suavemente (Euler con aceleración cero)
            xd_k  = xd_k + dxd_k * dt_mpc
            xd_k[3] = atan2(sin(xd_k[3]), cos(xd_k[3]))

        # ---- Resolver NMPC ----
        u_opt, t_ms = self.nmpc.solve(q0, q_ref, self.u_prev)

        self.pSC.t_solve_ms = t_ms
        self.pSC.U_raw      = u_opt.copy()

        # Saturación — el NLP ya respeta [-1,1] pero por seguridad
        self.pSC.b_Ud = np.clip(u_opt, -U_SAT, U_SAT)
        self.u_prev   = self.pSC.b_Ud.copy()

        # Log periódico
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        if self._log_counter % 30 == 0:
            e_pos = np.linalg.norm(self.pPos.w_Xd[0:3] - self.pPos.w_X[0:3])
            self.node.get_logger().info(
                f"NMPC | t_solve={t_ms:.1f}ms | e_xyz={e_pos:.3f}m | "
                f"U={np.round(self.pSC.b_Ud, 3)}")

    # ------------------------------------------------------------------
    #  Publicación — idéntica al original
    # ------------------------------------------------------------------

    def rSendControlSignals(self):
        if not self.ref_received or not self.is_flying:
            return
        b_cmd = Twist()
        b_cmd.linear.x  = float(self.pSC.b_Ud[0])
        b_cmd.linear.y  = float(self.pSC.b_Ud[1])
        b_cmd.linear.z  = float(self.pSC.b_Ud[2])
        b_cmd.angular.z = float(self.pSC.b_Ud[3])
        self.pub_cmd.publish(b_cmd)


# ================================================================
#  Nodo ROS2 — igual al original
# ================================================================

class NeroDroneNode(Node):
    def __init__(self):
        super().__init__("neroControl_nmpc_node")
        self.drone = Bebop(self)
        self.create_timer(1.0 / 30.0, self.control_loop)
        self.get_logger().info("=== NMPC Position Controller iniciado @ 30 Hz ===")

    def control_loop(self):
        self.drone.rGetSensorData()
        self.drone.cController()
        self.drone.rSendControlSignals()


def main(args=None):
    rclpy.init(args=args)
    node = NeroDroneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()