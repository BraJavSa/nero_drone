#!/usr/bin/env python3
r"""
Interactive Gain Tuning Dashboard and Optimizer for Bebop Drone Controllers.

System Model (Continuous State-Space):
    \dot{x}(t) = f(x(t), U(t))
    f(x, U)    = [ J(psi) * nu ]
                 [ f1 * U - f2 * nu ]

Discrete Integration (4th-Order Runge-Kutta Integrator - Ideal System):
    x_{k+1} = RK4(x_k, U_k, dt)

Controllers Supported for Optimization & Simulation:
    - OPT 1: GT Extended Controller (Inverse Dynamics)
    - OPT 2: First Order Controller (Kinematic + Dynamic Compensator)
    - OPT 3: Second Order Controller (Inverse Dynamics No Jdot)

Target Optimization Mode:
    - Target: Position Gains (KP, KSP, KD, KSD) while enforcing Max Body Velocity constraints.
    - Saves gains per-controller in optimal_gains.json without overwriting other controllers.
"""

import sys
import os
import json
import threading
import queue
import time
import numpy as np

import tkinter as tk
from tkinter import messagebox
from scipy.optimize import differential_evolution

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ==============================================================================
# Load Identified OptiTrack Plant Parameters
# ==============================================================================
JSON_PATH = os.path.join(
    os.path.dirname(__file__),
    "../optitrak_data/system_identification/system_identification_parameters_optitrack_4dof.json"
)

if os.path.exists(JSON_PATH):
    try:
        with open(JSON_PATH, "r") as f:
            jdata = json.load(f)["identified_parameters"]
            _F1_DIAG = np.array([jdata["x"]["f1"], jdata["y"]["f1"], jdata["z"]["f1"], jdata["yaw"]["f1"]])
            _F2_DIAG = np.array([jdata["x"]["f2"], jdata["y"]["f2"], jdata["z"]["f2"], jdata["yaw"]["f2"]])
    except Exception:
        _F1_DIAG = np.array([0.921527, 1.053286, 3.8086879470003603, 8.772786])
        _F2_DIAG = np.array([0.247044, 0.395160, 3.7414469817348253, 6.101834])
else:
    _F1_DIAG = np.array([0.921527, 1.053286, 3.8086879470003603, 8.772786])
    _F2_DIAG = np.array([0.247044, 0.395160, 3.7414469817348253, 6.101834])

DELAYS_IDEAL = np.zeros(4)  # Ideal system (zero delay)
CTRL_DT = 1.0 / 15.0        # 15 Hz execution
NU_MAX = np.array([0.9, 0.9, 0.8, 0.8])
U_MAX = np.ones(4)

HOLD_TIME    = 15.0
L            = 1.2
SIM_DURATION = 90.0

POINTS = np.array([
    [0.0, 0.0, 1.8],
    [-L/2, -L/2, 1.6],
    [L, L, 1.7],
    [0, L, 1.4],
    [-L, 0, 1.9],
    [0, 0, 1.5],
])
YAWS = np.deg2rad([0.0, 35.0, -35.0, -60.0, 300.0, 190.0])

CONTROLLER_KEYS = {
    1: "gt_extended_controller",
    2: "first_order_controller",
    3: "second_order_controller"
}


# ==============================================================================
# Kinematics & Utility Functions
# ==============================================================================

def wrap_angle(a):
    """Wrap angle into [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def J(psi: float) -> np.ndarray:
    """Kinematic matrix J(psi)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c, -s, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])


def J_inv(psi: float) -> np.ndarray:
    """Inverse kinematic matrix J^-1(psi)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c,  s, 0., 0.],
        [-s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])


def get_ref_at(t: float):
    """Return reference trajectory (eta_d, nu_d, alpha_d)."""
    idx     = int(t / HOLD_TIME) % len(POINTS)
    eta_d   = np.array([POINTS[idx, 0], POINTS[idx, 1], POINTS[idx, 2], YAWS[idx]])
    nu_d    = np.zeros(4)
    alpha_d = np.zeros(4)
    return eta_d, nu_d, alpha_d


# ==============================================================================
# Continuous State-Space & RK4 Integrator
# ==============================================================================

def state_space_f(x: np.ndarray, U: np.ndarray) -> np.ndarray:
    r"""
    State space continuous dynamic function f(x, U):
    x = [eta (4), nu (4)]^T
    \dot{eta} = J(psi) * nu
    \dot{nu}  = f1 * U - f2 * nu
    """
    eta = x[0:4]
    nu = x[4:8]
    psi = eta[3]

    Jmat = J(psi)
    eta_dot = Jmat @ nu
    nu_dot = _F1_DIAG * U - _F2_DIAG * nu
    return np.concatenate([eta_dot, nu_dot])


def rk4_step(x: np.ndarray, U: np.ndarray, dt: float) -> np.ndarray:
    """4th-order Runge-Kutta discrete state integrator step."""
    k1 = state_space_f(x, U)
    k2 = state_space_f(x + 0.5 * dt * k1, U)
    k3 = state_space_f(x + 0.5 * dt * k2, U)
    k4 = state_space_f(x + dt * k3, U)

    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x_next[3] = wrap_angle(x_next[3])
    return x_next


# ==============================================================================
# Controller Implementations for Optimization & Dashboard
# ==============================================================================

class GtExtendedController:
    """OPT = 1: Inverse Dynamics Controller (from gt_extended_controller.py)"""
    def __init__(self, KP, KSP, KD, KSD, dt=CTRL_DT):
        self.Ts = dt
        self.KP = KP
        self.KSP = KSP
        self.KD = KD
        self.KSD = KSD
        self.f1_inv = np.diag(1.0 / _F1_DIAG)
        self.f2 = np.diag(_F2_DIAG)
        self.U_MAX = U_MAX

    def compute_control(self, eta, nu, eta_d, dX_d=None, ddX_d=None):
        if dX_d is None: dX_d = np.zeros(4)
        if ddX_d is None: ddX_d = np.zeros(4)

        psi = eta[3]
        Jmat = J(psi)
        Jinv = J_inv(psi)

        eta_dot = Jmat @ nu
        eta_tilde = eta_d - eta
        eta_tilde[3] = wrap_angle(eta_tilde[3])
        eta_dot_tilde = dX_d - eta_dot

        alpha = (ddX_d
                 + self.KSD @ np.tanh(self.KD @ eta_dot_tilde)
                 + self.KSP @ np.tanh(self.KP @ eta_tilde))

        Ud = self.f1_inv @ (Jinv @ alpha + self.f2 @ nu)
        return np.clip(Ud, -self.U_MAX, self.U_MAX)


class FirstOrderController:
    """OPT = 2: First-Order Cascade Controller (from first_order_controller.py)"""
    def __init__(self, KP, KSP, KD, KSD, dt=CTRL_DT):
        self.Ts = dt
        self.KP = KP
        self.KSP = KSP
        self.KD = KD
        self.KSD = KSD
        self.f1 = np.diag(_F1_DIAG)
        self.f2 = np.diag(_F2_DIAG)
        self.U_MAX = U_MAX
        self.Ucw_prev = np.zeros(4)

    def compute_control(self, eta, nu, eta_d, dX_d=None, ddX_d=None):
        if dX_d is None: dX_d = np.zeros(4)

        psi = eta[3]
        Jmat = J(psi)
        dX = Jmat @ nu

        Xtil = eta_d - eta
        Xtil[3] = wrap_angle(Xtil[3])

        Ucw = dX_d + self.KSP @ np.tanh(self.KP @ Xtil)
        dUcw = (Ucw - self.Ucw_prev) / self.Ts
        self.Ucw_prev = Ucw.copy()

        A = Jmat @ self.f1
        b = dUcw + self.KSD @ (Ucw - dX) + self.f2 @ dX
        Udw = np.linalg.solve(A, b)

        return np.clip(Udw, -self.U_MAX, self.U_MAX)


class SecondOrderController:
    """OPT = 3: Second-Order Inverse Dynamics Controller (from second_order_controller.py)"""
    def __init__(self, KP, KSP, KD, KSD, dt=CTRL_DT):
        self.Ts = dt
        self.KP = KP
        self.KSP = KSP
        self.KD = KD
        self.KSD = KSD
        self.f1_inv = np.diag(1.0 / _F1_DIAG)
        self.f2 = np.diag(_F2_DIAG)
        self.U_MAX = U_MAX

    def compute_control(self, eta, nu, eta_d, dX_d=None, ddX_d=None):
        if dX_d is None: dX_d = np.zeros(4)
        if ddX_d is None: ddX_d = np.zeros(4)

        psi = eta[3]
        Jmat = J(psi)
        dX = Jmat @ nu

        Xtil = eta_d - eta
        Xtil[3] = wrap_angle(Xtil[3])
        dXtil = dX_d - dX

        alpha = (ddX_d
                 + self.KSP @ np.tanh(self.KP @ Xtil)
                 + self.KSD @ np.tanh(self.KD @ dXtil))

        Jinv = J_inv(psi)
        Udw = self.f1_inv @ (Jinv @ alpha + self.f2 @ nu)

        return np.clip(Udw, -self.U_MAX, self.U_MAX)


def get_controller_instance(opt: int, KP, KSP, KD, KSD, dt=CTRL_DT):
    """Factory helper to instantiate controller by option (1, 2, or 3)."""
    if opt == 1:
        return GtExtendedController(KP, KSP, KD, KSD, dt=dt)
    elif opt == 2:
        return FirstOrderController(KP, KSP, KD, KSD, dt=dt)
    elif opt == 3:
        return SecondOrderController(KP, KSP, KD, KSD, dt=dt)
    else:
        return FirstOrderController(KP, KSP, KD, KSD, dt=dt)


# ==============================================================================
# Simulation & Cost Evaluation
# ==============================================================================

def unpack(params):
    """Unpack 16-parameter gain vector into 4 diagonal gain matrices."""
    KP  = np.diag(params[0:4])
    KSP = np.diag(params[4:8])
    KD  = np.diag(params[8:12])
    KSD = np.diag(params[12:16])
    return KP, KSP, KD, KSD


def simulate(params, opt=2, delays=DELAYS_IDEAL, nu_max=NU_MAX, return_history=False):
    """
    Simulate drone closed-loop response using RK4 integrator on ideal system.
    """
    KP, KSP, KD, KSD = unpack(params)

    # Initial state x = [eta, nu] with Z = 1.5 m
    x = np.array([0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0])

    controller = get_controller_instance(opt, KP, KSP, KD, KSD, dt=CTRL_DT)

    N = int(SIM_DURATION / CTRL_DT)
    max_delay = int(np.max(delays)) if len(delays) > 0 else 0
    u_buffer = [np.zeros(4) for _ in range(max_delay + 1)]

    errors = []
    eta_hist = []
    eta_d_hist = []
    nu_hist = []
    u_hist = []
    t_hist = []

    for k in range(N):
        t = k * CTRL_DT
        eta = x[0:4]
        nu = x[4:8]

        eta_d, nu_d, alpha_d = get_ref_at(t)

        # Tracking error
        X_tilde = eta_d - eta
        X_tilde[3] = wrap_angle(X_tilde[3])
        errors.append(X_tilde.copy())

        if return_history:
            eta_hist.append(eta.copy())
            eta_d_hist.append(eta_d.copy())
            nu_hist.append(nu.copy())

        # Compute control action U_k
        U_k = controller.compute_control(eta, nu, eta_d, dX_d=nu_d, ddX_d=alpha_d)
        u_buffer.append(U_k)

        # Apply delay (delays=DELAYS_IDEAL => zero delay)
        U_applied = np.zeros(4)
        for i in range(4):
            d_i = int(delays[i])
            U_applied[i] = u_buffer[-(1 + d_i)][i]

        if return_history:
            u_hist.append(U_applied.copy())
            t_hist.append(t)

        # Plant RK4 Integration step
        x = rk4_step(x, U_applied, CTRL_DT)

    if return_history:
        return (np.array(errors), np.array(eta_hist), np.array(eta_d_hist),
                np.array(nu_hist), np.array(u_hist), np.array(t_hist))
    return np.array(errors)


def cost(params, opt=2, nu_max=NU_MAX, delays=DELAYS_IDEAL):
    """
    Hierarchical Multi-Objective Cost Function:
    Priority 1: Enforce strict Max Body Velocity limits (heavy penalty for X, Y, Z).
    Priority 2: Position Tracking Convergence (Weights: XY > Yaw > Z).
    Priority 3: Zero Body Velocity Maintenance at setpoints (nu -> 0 when reference is held constant).
    """
    KP, KSP, KD, KSD = unpack(params)

    try:
        (errors, eta_hist, eta_d_hist, nu_hist, u_hist, t_hist) = simulate(
            params, opt=opt, delays=delays, nu_max=nu_max, return_history=True
        )
    except Exception:
        return 1e9

    if not np.all(np.isfinite(errors)):
        return 1e9

    # 1. PRIORITY 1: Strict Body Velocity Limit Constraints (Heavy penalty for X, Y, Z)
    # Excess velocity beyond limit: [Vx, Vy, Vz, Vyaw]
    w_vel_excess = np.array([20000.0, 20000.0, 20000.0, 10000.0])
    vel_excess = np.maximum(0.0, np.abs(nu_hist) - nu_max)
    p1_vel_max_penalty = np.sum((vel_excess ** 2) * w_vel_excess)

    # Penalty for KSP gains exceeding nu_max limits
    w_ksp_excess = np.array([10000.0, 10000.0, 10000.0, 5000.0])
    ksp_diag = np.diag(KSP)
    ksp_excess = np.maximum(0.0, ksp_diag - nu_max)
    p1_ksp_penalty = np.sum((ksp_excess ** 2) * w_ksp_excess)

    # 2. PRIORITY 2: Position Tracking Error (Weights: XY > Yaw > Z)
    # Order: [X, Y, Z, Yaw] -> XY (4.0) > Yaw (2.5) > Z (1.0)
    w_pos = np.array([4.0, 4.0, 1.0, 2.5])

    N = len(errors)
    ise_pos = np.mean((errors ** 2) * w_pos)
    n_trans = N // 4
    transient_penalty = np.mean((errors[:n_trans] ** 2) * w_pos) * 0.4
    ss_penalty = np.mean((errors[-n_trans:] ** 2) * w_pos) * 1.5
    p2_pos_error_cost = (ise_pos + transient_penalty + ss_penalty) * 100.0

    # 3. PRIORITY 3: Stationary Body Velocity Regulation (nu -> 0 when reference is constant)
    w_stat_vel = np.array([2.0, 2.0, 0.8, 1.5])
    p3_stationary_vel_cost = np.mean((nu_hist ** 2) * w_stat_vel) * 100.0

    return p1_vel_max_penalty + p1_ksp_penalty + p2_pos_error_cost + p3_stationary_vel_cost


# Bounds for KP, KSP, KD, KSD
BOUNDS = [
    (0.1,  5.0), (0.1,  5.0), (1.0,  8.0), (1.0, 14.0),  # KP: x, y, z, yaw
    (0.2,  0.9), (0.2,  0.9), (0.1,  0.8), (0.1,  0.8),  # KSP: x, y, z, yaw
    (1.0, 10.0), (1.0, 10.0), (0.5,  5.0), (0.3,  3.0),  # KD: x, y, z, yaw
    (0.0,  2.0), (0.0,  2.0), (0.0,  2.0), (0.0,  2.0),  # KSD: x, y, z, yaw
]

# Default preset gains
DEFAULT_POS_GAINS = [
    3.880000, 3.416000, 8.000000, 12.533000,  # KP
    0.203000, 0.203000, 0.200000, 0.301000,   # KSP
    8.835000, 10.000000, 2.376000, 1.962000,   # KD
    2.000000, 2.000000, 0.985000, 0.800000    # KSD
]


# ==============================================================================
# UI Styling Colors
# ==============================================================================
BG_DARK      = '#11111b'
BG_PANEL     = '#181825'
BG_CARD      = '#313244'
TEXT_MAIN    = '#cdd6f4'
TEXT_MUTED   = '#a6adc8'
ACCENT_BLUE  = '#89b4fa'
ACCENT_GREEN = '#a6e3a1'
ACCENT_RED   = '#f38ba8'
ACCENT_PURPLE = '#cba6f7'


# ==============================================================================
# Helper UI Elements
# ==============================================================================

def create_button(parent, text, command, bg_color, fg_color=TEXT_MAIN, font=('Arial', 10, 'bold')):
    btn = tk.Button(
        parent,
        text=text,
        command=command,
        bg=bg_color,
        fg=fg_color,
        activebackground=lighten_color(bg_color),
        activeforeground=fg_color,
        relief=tk.FLAT,
        bd=0,
        padx=12,
        pady=8,
        font=font,
        cursor='hand2'
    )
    def on_enter(e):
        btn.config(bg=lighten_color(bg_color))
    def on_leave(e):
        btn.config(bg=bg_color)
    btn.bind("<Enter>", on_enter)
    btn.bind("<Leave>", on_leave)
    return btn


def lighten_color(hex_color):
    hex_color = hex_color.lstrip('#')
    rgb = tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    new_rgb = tuple(min(255, int(c * 1.15)) for c in rgb)
    return f"#{new_rgb[0]:02x}{new_rgb[1]:02x}{new_rgb[2]:02x}"


class GainSlider:
    def __init__(self, parent, name, min_val, max_val, default_val, command_cb):
        self.frame = tk.Frame(parent, bg=BG_CARD, padx=4, pady=4, bd=1, relief=tk.RIDGE)
        self.name = name
        self.command_cb = command_cb

        self.label = tk.Label(
            self.frame,
            text=f"{name}: {default_val:.2f}",
            bg=BG_CARD,
            fg=TEXT_MAIN,
            font=('Arial', 9, 'bold')
        )
        self.label.pack(anchor='w')

        self.scale = tk.Scale(
            self.frame,
            from_=min_val,
            to=max_val,
            resolution=0.001,
            orient=tk.HORIZONTAL,
            bg=BG_CARD,
            fg=TEXT_MAIN,
            troughcolor=BG_DARK,
            activebackground=ACCENT_BLUE,
            highlightbackground=BG_CARD,
            highlightcolor=ACCENT_BLUE,
            bd=0,
            showvalue=False,
            command=self.on_change
        )
        self.scale.set(default_val)
        self.scale.pack(fill=tk.X, expand=True)

    def on_change(self, val):
        self.label.config(text=f"{self.name}: {float(val):.3f}")
        self.command_cb(val)

    def get(self):
        return self.scale.get()

    def set(self, val):
        self.scale.set(val)
        self.label.config(text=f"{self.name}: {float(val):.3f}")


# ==============================================================================
# Background Optimization Worker Thread
# ==============================================================================

class OptimizerWorker(threading.Thread):
    def __init__(self, q, current_params, bounds, opt=2, nu_max=NU_MAX):
        super().__init__()
        self.q = q
        self.current_params = current_params
        self.bounds = bounds
        self.opt = opt
        self.nu_max = nu_max
        self.stop_requested = False

    def run(self):
        def callback(xk, *args, **kwargs):
            if self.stop_requested:
                return True
            try:
                c = cost(xk, opt=self.opt, nu_max=self.nu_max, delays=DELAYS_IDEAL)
            except Exception:
                c = 9e9
            self.q.put(('update', xk.copy(), c))

        try:
            result = differential_evolution(
                lambda x: cost(x, opt=self.opt, nu_max=self.nu_max, delays=DELAYS_IDEAL),
                self.bounds,
                strategy='best1bin',
                maxiter=12,
                popsize=18,
                tol=1e-6,
                mutation=(0.5, 1.2),
                recombination=0.85,
                seed=42,
                polish=True,
                disp=False,
                workers=1,
                updating='deferred',
                callback=callback
            )
            if not self.stop_requested:
                self.q.put(('done', result.x, result.fun))
        except Exception as e:
            self.q.put(('error', str(e)))


# ==============================================================================
# Main GUI Class
# ==============================================================================

class GainsTuningGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("🚁 Bebop Controller Gain Tuning & Ideal RK4 Optimization Dashboard")
        self.root.geometry("1400x850")
        self.root.configure(bg=BG_DARK)

        self.updating_gui = False
        self.update_timer_id = None
        self.opt_thread = None
        self.opt_queue = None

        self.controller_opt = 2  # Default to OPT 2: First Order
        self.target_opt = "position"

        # Build layout
        self.setup_ui()

        # Initial load from JSON or default
        self.load_gains_for_active_controller()

    def setup_ui(self):
        # 1. Header Frame
        header_frame = tk.Frame(self.root, bg=BG_PANEL, height=50)
        header_frame.pack(side=tk.TOP, fill=tk.X)

        header_title = tk.Label(
            header_frame,
            text="🚁 BEBOP CONTROLLER TUNING & RK4 AUTO-OPTIMIZER (IDEAL SYSTEM)",
            bg=BG_PANEL,
            fg=ACCENT_PURPLE,
            font=('Arial', 14, 'bold'),
            pady=10
        )
        header_title.pack()

        # 2. Main content container split in Left (Plots) and Right (Controls)
        main_container = tk.Frame(self.root, bg=BG_DARK)
        main_container.pack(fill=tk.BOTH, expand=True)

        main_container.columnconfigure(0, weight=6)
        main_container.columnconfigure(1, weight=5)
        main_container.rowconfigure(0, weight=1)

        # Left Panel (Plots)
        self.left_frame = tk.Frame(main_container, bg=BG_DARK, padx=10, pady=10)
        self.left_frame.grid(row=0, column=0, sticky='nsew')

        # Right Panel (Controls & Stats)
        self.right_frame = tk.Frame(main_container, bg=BG_PANEL, padx=15, pady=15)
        self.right_frame.grid(row=0, column=1, sticky='nsew')

        # 3. Setup Matplotlib Figures inside Left Frame
        self.setup_plots()

        # 4. Setup Control Panel widgets inside Right Frame
        self.setup_controls()

    def get_nu_max(self):
        nu_max = np.array([0.9, 0.9, 0.8, 0.8])
        if hasattr(self, 'vel_entries') and len(self.vel_entries) == 4:
            for i in range(4):
                try:
                    v = float(self.vel_entries[i].get())
                    if v > 0:
                        nu_max[i] = v
                except ValueError:
                    pass
        return nu_max

    def setup_plots(self):
        plt.style.use('dark_background')
        self.fig, self.axs = plt.subplots(4, 3, figsize=(10, 7.5), sharex=True)
        self.fig.patch.set_facecolor(BG_DARK)

        nu_max = self.get_nu_max()
        initial_params = DEFAULT_POS_GAINS
        errors, eta_hist, eta_d_hist, nu_hist, u_hist, t_hist = simulate(
            initial_params, opt=self.controller_opt, delays=DELAYS_IDEAL, nu_max=nu_max, return_history=True
        )

        self.t_hist = t_hist
        self.ref_lines = []
        self.sim_lines = []
        self.vel_lines = []
        self.vel_limit_pos_lines = []
        self.vel_limit_neg_lines = []
        self.u_lines = []
        self.u_limit_pos_lines = []
        self.u_limit_neg_lines = []

        pos_labels = ["X Position", "Y Position", "Z Altitude", "Yaw Angle"]
        pos_units = ["m", "m", "m", "deg"]
        vel_labels = ["Vx Body", "Vy Body", "Vz Body", "Yaw Rate r"]
        vel_units = ["m/s", "m/s", "m/s", "deg/s"]
        u_labels = ["Cmd Ux (Roll)", "Cmd Uy (Pitch)", "Cmd Uz (Gaz)", "Cmd Uyaw"]
        u_units = ["norm", "norm", "norm", "norm"]

        for i in range(4):
            # --- Column 0: Position Tracking ---
            ax_pos = self.axs[i, 0]
            ax_pos.set_facecolor(BG_PANEL)
            ax_pos.set_title(f"{pos_labels[i]}", color=TEXT_MAIN, fontsize=8, fontweight='bold', pad=3)
            ax_pos.set_ylabel(f"[{pos_units[i]}]", color=TEXT_MUTED, fontsize=8)
            ax_pos.grid(True, color='#313244', linestyle=':', alpha=0.6)

            for spine in ['top', 'right']:
                ax_pos.spines[spine].set_visible(False)
            for spine in ['bottom', 'left']:
                ax_pos.spines[spine].set_color('#45475a')

            ref_data = eta_d_hist[:, i]
            if i == 3:
                ref_data = np.degrees(ref_data)
            ref_line, = ax_pos.plot(t_hist, ref_data, color=ACCENT_RED, linestyle='--', linewidth=1.2, label="Reference")
            self.ref_lines.append(ref_line)

            sim_data = eta_hist[:, i]
            if i == 3:
                sim_data = np.degrees(sim_data)
            sim_line, = ax_pos.plot(t_hist, sim_data, color=ACCENT_BLUE, linewidth=1.4, label="Simulated")
            self.sim_lines.append(sim_line)

            if i == 0:
                ax_pos.legend(facecolor=BG_PANEL, edgecolor='#313244', fontsize=7, loc='upper right')

            margin_min = min(np.min(ref_data), np.min(sim_data))
            margin_max = max(np.max(ref_data), np.max(sim_data))
            span = margin_max - margin_min
            if span < 1e-3:
                span = 1.0
            ax_pos.set_ylim(margin_min - 0.15 * span, margin_max + 0.15 * span)
            ax_pos.set_xlim(0, SIM_DURATION)
            ax_pos.tick_params(colors=TEXT_MUTED, labelsize=7)

            # --- Column 1: Body Velocity ---
            ax_vel = self.axs[i, 1]
            ax_vel.set_facecolor(BG_PANEL)
            ax_vel.set_title(f"{vel_labels[i]}", color=TEXT_MAIN, fontsize=8, fontweight='bold', pad=3)
            ax_vel.set_ylabel(f"[{vel_units[i]}]", color=TEXT_MUTED, fontsize=8)
            ax_vel.grid(True, color='#313244', linestyle=':', alpha=0.6)

            for spine in ['top', 'right']:
                ax_vel.spines[spine].set_visible(False)
            for spine in ['bottom', 'left']:
                ax_vel.spines[spine].set_color('#45475a')

            vel_data = nu_hist[:, i]
            if i == 3:
                vel_data = np.degrees(vel_data)
            vel_line, = ax_vel.plot(t_hist, vel_data, color=ACCENT_GREEN, linewidth=1.4, label="Velocity")
            self.vel_lines.append(vel_line)

            v_limit = nu_max[i]
            if i == 3:
                v_limit = np.degrees(v_limit)
            lim_pos = ax_vel.axhline(v_limit, color=ACCENT_RED, linestyle=':', linewidth=1.0, alpha=0.8)
            lim_neg = ax_vel.axhline(-v_limit, color=ACCENT_RED, linestyle=':', linewidth=1.0, alpha=0.8)
            self.vel_limit_pos_lines.append(lim_pos)
            self.vel_limit_neg_lines.append(lim_neg)

            if i == 0:
                ax_vel.legend(facecolor=BG_PANEL, edgecolor='#313244', fontsize=7, loc='upper right')
            ax_vel.tick_params(colors=TEXT_MUTED, labelsize=7)
            ax_vel.set_ylim(-v_limit * 1.25, v_limit * 1.25)

            # --- Column 2: Control Action ---
            ax_u = self.axs[i, 2]
            ax_u.set_facecolor(BG_PANEL)
            ax_u.set_title(f"{u_labels[i]}", color=TEXT_MAIN, fontsize=8, fontweight='bold', pad=3)
            ax_u.set_ylabel(f"[{u_units[i]}]", color=TEXT_MUTED, fontsize=8)
            ax_u.grid(True, color='#313244', linestyle=':', alpha=0.6)

            for spine in ['top', 'right']:
                ax_u.spines[spine].set_visible(False)
            for spine in ['bottom', 'left']:
                ax_u.spines[spine].set_color('#45475a')

            u_data = u_hist[:, i]
            u_line, = ax_u.plot(t_hist, u_data, color=ACCENT_PURPLE, linewidth=1.4, label="Control Cmd")
            self.u_lines.append(u_line)

            u_max_val = U_MAX[i]
            u_lim_pos = ax_u.axhline(u_max_val, color=ACCENT_RED, linestyle=':', linewidth=1.0, alpha=0.8)
            u_lim_neg = ax_u.axhline(-u_max_val, color=ACCENT_RED, linestyle=':', linewidth=1.0, alpha=0.8)
            self.u_limit_pos_lines.append(u_lim_pos)
            self.u_limit_neg_lines.append(u_lim_neg)

            if i == 0:
                ax_u.legend(facecolor=BG_PANEL, edgecolor='#313244', fontsize=7, loc='upper right')
            ax_u.tick_params(colors=TEXT_MUTED, labelsize=7)
            ax_u.set_ylim(-1.15, 1.15)

        self.axs[3, 0].set_xlabel("Time [s]", color=TEXT_MUTED, fontsize=8)
        self.axs[3, 1].set_xlabel("Time [s]", color=TEXT_MUTED, fontsize=8)
        self.axs[3, 2].set_xlabel("Time [s]", color=TEXT_MUTED, fontsize=8)

        self.fig.tight_layout()

        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.left_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_controls(self):
        # 1. Controller Selection Frame
        ctrl_frame = tk.LabelFrame(
            self.right_frame,
            text="Controller Strategy Selection",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=8,
            pady=6
        )
        ctrl_frame.pack(fill=tk.X, pady=(0, 8))

        # Controller Selector Buttons (OPT 1, 2, 3)
        tk.Label(ctrl_frame, text="Active Controller Strategy:", bg=BG_PANEL, fg=TEXT_MUTED, font=('Arial', 9, 'bold')).pack(anchor='w')
        btn_box = tk.Frame(ctrl_frame, bg=BG_PANEL)
        btn_box.pack(fill=tk.X, pady=(2, 2))

        self.opt_btn1 = create_button(
            btn_box, text="OPT 1: GT Extended", command=lambda: self.set_controller_opt(1), bg_color=BG_CARD, font=('Arial', 8, 'bold')
        )
        self.opt_btn1.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.opt_btn2 = create_button(
            btn_box, text="OPT 2: First Order", command=lambda: self.set_controller_opt(2), bg_color=ACCENT_BLUE, fg_color=BG_DARK, font=('Arial', 8, 'bold')
        )
        self.opt_btn2.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        self.opt_btn3 = create_button(
            btn_box, text="OPT 3: Second Order", command=lambda: self.set_controller_opt(3), bg_color=BG_CARD, font=('Arial', 8, 'bold')
        )
        self.opt_btn3.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)

        # 2. Sliders Grid Header & Frame
        sliders_title = tk.Label(
            self.right_frame,
            text="Gain Parameters Grid",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 11, 'bold')
        )
        sliders_title.pack(anchor='w', pady=(4, 6))

        grid_frame = tk.Frame(self.right_frame, bg=BG_PANEL)
        grid_frame.pack(fill=tk.BOTH, expand=True)

        for c in range(5):
            grid_frame.columnconfigure(c, weight=1)
        for r in range(5):
            grid_frame.rowconfigure(r, weight=1)

        cols = ["KP", "KSP", "KD", "KSD"]
        rows = ["X", "Y", "Z", "Yaw"]

        for c, col_name in enumerate(cols):
            lbl = tk.Label(
                grid_frame,
                text=col_name,
                bg=BG_PANEL,
                fg=ACCENT_PURPLE,
                font=('Arial', 10, 'bold')
            )
            lbl.grid(row=0, column=c+1, padx=4, pady=2, sticky='ew')

        for r, row_name in enumerate(rows):
            lbl = tk.Label(
                grid_frame,
                text=row_name,
                bg=BG_PANEL,
                fg=ACCENT_BLUE,
                font=('Arial', 10, 'bold')
            )
            lbl.grid(row=r+1, column=0, padx=4, pady=2, sticky='ns')

        self.sliders = [None] * 16
        for r in range(4):
            for c in range(4):
                idx = c * 4 + r
                min_v, max_v = BOUNDS[idx]
                def_v = DEFAULT_POS_GAINS[idx]
                name = f"{cols[c]}_{rows[r].lower()}"

                slider = GainSlider(grid_frame, name, min_v, max_v, def_v, self.on_slider_change)
                slider.frame.grid(row=r+1, column=c+1, padx=3, pady=3, sticky='nsew')
                self.sliders[idx] = slider

        # 3. Presets Frame
        presets_frame = tk.LabelFrame(
            self.right_frame,
            text="Presets & Quick Actions",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=6
        )
        presets_frame.pack(fill=tk.X, pady=(10, 5))

        btn_pos = create_button(
            presets_frame,
            text="📍 Reset Default Position Gains",
            command=self.load_default_gains,
            bg_color=BG_CARD
        )
        btn_pos.pack(fill=tk.X, expand=True, padx=5)

        # 3b. Max Body Velocities Frame
        max_vel_frame = tk.LabelFrame(
            self.right_frame,
            text="Max Body Velocity Constraints (m/s & rad/s)",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=6
        )
        max_vel_frame.pack(fill=tk.X, pady=(0, 5))

        vel_labels = ["Vx max (m/s)", "Vy max (m/s)", "Vz max (m/s)", "Vyaw max (rad/s)"]
        default_vels = [0.9, 0.9, 0.8, 0.8]
        self.vel_entries = []

        for i in range(4):
            max_vel_frame.columnconfigure(i, weight=1)
            box = tk.Frame(max_vel_frame, bg=BG_CARD, padx=4, pady=4)
            box.grid(row=0, column=i, padx=3, pady=2, sticky='nsew')

            tk.Label(box, text=vel_labels[i], bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
            entry = tk.Entry(box, bg=BG_DARK, fg=ACCENT_BLUE, font=('Arial', 10, 'bold'), justify='center', width=8)
            entry.insert(0, str(default_vels[i]))
            entry.pack(pady=2)
            entry.bind("<KeyRelease>", lambda e: self.on_slider_change(None))
            self.vel_entries.append(entry)

        # 4. Live Statistics Cards Frame
        stats_frame = tk.LabelFrame(
            self.right_frame,
            text="Live Simulation Metrics",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=6
        )
        stats_frame.pack(fill=tk.X, pady=(0, 5))

        for i in range(3):
            stats_frame.columnconfigure(i, weight=1)

        cost_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        cost_card.grid(row=0, column=0, padx=4, pady=4, sticky='nsew')
        tk.Label(cost_card, text="ISE Cost", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.cost_val_label = tk.Label(cost_card, text="0.00000", bg=BG_CARD, fg=ACCENT_PURPLE, font=('Arial', 11, 'bold'))
        self.cost_val_label.pack()

        rms_x_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_x_card.grid(row=0, column=1, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_x_card, text="RMS Error X", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_x_label = tk.Label(rms_x_card, text="0.00 cm", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_x_label.pack()

        rms_y_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_y_card.grid(row=0, column=2, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_y_card, text="RMS Error Y", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_y_label = tk.Label(rms_y_card, text="0.00 cm", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_y_label.pack()

        rms_z_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_z_card.grid(row=1, column=0, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_z_card, text="RMS Error Z", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_z_label = tk.Label(rms_z_card, text="0.00 cm", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_z_label.pack()

        rms_yaw_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_yaw_card.grid(row=1, column=1, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_yaw_card, text="RMS Error Yaw", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_yaw_label = tk.Label(rms_yaw_card, text="0.000°", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_yaw_label.pack()

        save_btn_card = tk.Frame(stats_frame, bg=BG_PANEL)
        save_btn_card.grid(row=1, column=2, padx=4, pady=4, sticky='nsew')
        self.btn_save = create_button(
            save_btn_card,
            text="💾 Save Gains to JSON",
            command=self.save_gains_to_file,
            bg_color=ACCENT_BLUE,
            fg_color=BG_DARK
        )
        self.btn_save.pack(fill=tk.BOTH, expand=True)

        # 5. Optimization Controls Frame
        opt_frame = tk.LabelFrame(
            self.right_frame,
            text="Evolutionary Auto-Optimization",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=6
        )
        opt_frame.pack(fill=tk.X, pady=(0, 5))

        self.status_label = tk.Label(
            opt_frame,
            text="Status: Idle",
            bg=BG_PANEL,
            fg=TEXT_MUTED,
            font=('Arial', 9, 'italic'),
            anchor='w'
        )
        self.status_label.pack(fill=tk.X, pady=(0, 6))

        btn_box_opt = tk.Frame(opt_frame, bg=BG_PANEL)
        btn_box_opt.pack(fill=tk.X)

        self.opt_button = create_button(
            btn_box_opt,
            text="🚀 Run Auto-Optimization",
            command=self.start_optimization,
            bg_color=ACCENT_GREEN,
            fg_color=BG_DARK
        )
        self.opt_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))

        self.stop_button = create_button(
            btn_box_opt,
            text="🛑 Stop",
            command=self.stop_optimization,
            bg_color=BG_CARD
        )
        self.stop_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))
        self.stop_button.config(state=tk.DISABLED)

    def set_controller_opt(self, opt_num: int):
        self.controller_opt = opt_num
        self.opt_btn1.config(bg=ACCENT_BLUE if opt_num == 1 else BG_CARD, fg=BG_DARK if opt_num == 1 else TEXT_MAIN)
        self.opt_btn2.config(bg=ACCENT_BLUE if opt_num == 2 else BG_CARD, fg=BG_DARK if opt_num == 2 else TEXT_MAIN)
        self.opt_btn3.config(bg=ACCENT_BLUE if opt_num == 3 else BG_CARD, fg=BG_DARK if opt_num == 3 else TEXT_MAIN)

        self.load_gains_for_active_controller()

    def set_target_opt(self, target_str: str):
        self.target_opt = target_str
        self.status_label.config(text=f"Optimization Target: {target_str.capitalize()} Gains", fg=ACCENT_PURPLE)

    def load_gains_for_active_controller(self):
        """Load gains for active controller from optimal_gains.json if present, else default."""
        ctrl_key = CONTROLLER_KEYS.get(self.controller_opt, f"controller_opt_{self.controller_opt}")
        out_path = os.path.abspath("optimal_gains.json")

        loaded = False
        if os.path.exists(out_path):
            try:
                with open(out_path, "r") as f:
                    data = json.load(f)
                    if ctrl_key in data and "gains" in data[ctrl_key]:
                        g = data[ctrl_key]["gains"]
                        vec = (g["KP"] + g["KSP"] + g["KD"] + g["KSD"])
                        if len(vec) == 16:
                            self.updating_gui = True
                            for i, val in enumerate(vec):
                                self.sliders[i].set(val)
                            self.updating_gui = False
                            loaded = True
                            self.status_label.config(text=f"Status: Loaded saved JSON gains for {ctrl_key}", fg=ACCENT_BLUE)
            except Exception as e:
                print(f"Notice: Could not parse {out_path}: {e}")

        if not loaded:
            self.load_default_gains()

    # ==============================================================================
    # Event Handlers & Dynamic Drawing
    # ==============================================================================
    def on_slider_change(self, val):
        if self.updating_gui:
            return

        if self.update_timer_id is not None:
            self.root.after_cancel(self.update_timer_id)
        self.update_timer_id = self.root.after(40, self.update_plot_immediate)

    def update_plot_immediate(self):
        self.update_timer_id = None

        params = [slider.get() for slider in self.sliders]
        nu_max = self.get_nu_max()

        try:
            errors, eta_hist, eta_d_hist, nu_hist, u_hist, t_hist = simulate(
                params, opt=self.controller_opt, delays=DELAYS_IDEAL, nu_max=nu_max, return_history=True
            )
        except Exception as e:
            print(f"Simulation error: {e}")
            return

        rms = np.sqrt(np.mean(errors ** 2, axis=0))
        current_cost = cost(params, opt=self.controller_opt, nu_max=nu_max, delays=DELAYS_IDEAL)

        self.cost_val_label.config(text=f"{current_cost:.5f}")
        self.rms_x_label.config(text=f"{rms[0]*100:.2f} cm")
        self.rms_y_label.config(text=f"{rms[1]*100:.2f} cm")
        self.rms_z_label.config(text=f"{rms[2]*100:.2f} cm")
        self.rms_yaw_label.config(text=f"{np.degrees(rms[3]):.3f}°")

        for i in range(4):
            # Update Position
            sim_data = eta_hist[:, i]
            if i == 3:
                sim_data = np.degrees(sim_data)
            self.sim_lines[i].set_ydata(sim_data)

            ax_pos = self.axs[i, 0]
            ymin, ymax = ax_pos.get_ylim()
            min_d, max_d = np.min(sim_data), np.max(sim_data)
            ref_data = eta_d_hist[:, i]
            if i == 3:
                ref_data = np.degrees(ref_data)
            min_ref, max_ref = np.min(ref_data), np.max(ref_data)

            if min_d < ymin or max_d > ymax or (ymax - ymin) > 4.0 * (max_d - min_d + 1e-2):
                margin_min = min(min_d, min_ref)
                margin_max = max(max_d, max_ref)
                span = margin_max - margin_min
                if span < 1e-3:
                    span = 1.0
                ax_pos.set_ylim(margin_min - 0.15 * span, margin_max + 0.15 * span)

            # Update Body Velocity
            vel_data = nu_hist[:, i]
            if i == 3:
                vel_data = np.degrees(vel_data)
            self.vel_lines[i].set_ydata(vel_data)

            v_lim = nu_max[i]
            if i == 3:
                v_lim = np.degrees(v_lim)
            self.vel_limit_pos_lines[i].set_ydata([v_lim, v_lim])
            self.vel_limit_neg_lines[i].set_ydata([-v_lim, -v_lim])

            ax_vel = self.axs[i, 1]
            limit_margin = v_lim * 1.25
            ax_vel.set_ylim(-limit_margin, limit_margin)

            # Update Control Action
            u_data = u_hist[:, i]
            self.u_lines[i].set_ydata(u_data)

        self.canvas.draw_idle()

    # ==============================================================================
    # Presets & Save logic
    # ==============================================================================
    def load_default_gains(self):
        self.updating_gui = True
        for i, val in enumerate(DEFAULT_POS_GAINS):
            self.sliders[i].set(val)
        self.updating_gui = False
        self.update_plot_immediate()
        self.status_label.config(text="Status: Reset to default position gains", fg=ACCENT_BLUE)

    def save_gains_to_file(self):
        params = [slider.get() for slider in self.sliders]
        KP, KSP, KD, KSD = unpack(params)
        nu_max = self.get_nu_max()
        current_cost = cost(params, opt=self.controller_opt, nu_max=nu_max, delays=DELAYS_IDEAL)

        ctrl_key = CONTROLLER_KEYS.get(self.controller_opt, f"controller_opt_{self.controller_opt}")
        out_path = os.path.abspath("optimal_gains.json")

        # Load existing json to preserve gains of other controllers
        data = {}
        if os.path.exists(out_path):
            try:
                with open(out_path, "r") as f:
                    data = json.load(f)
            except Exception:
                data = {}

        data[ctrl_key] = {
            "opt": self.controller_opt,
            "target": self.target_opt,
            "system_type": "ideal",
            "nu_max": nu_max.tolist(),
            "total_cost": float(current_cost),
            "gains": {
                "KP": np.diag(KP).tolist(),
                "KSP": np.diag(KSP).tolist(),
                "KD": np.diag(KD).tolist(),
                "KSD": np.diag(KSD).tolist(),
            }
        }

        try:
            with open(out_path, "w") as f:
                json.dump(data, f, indent=4)
            formatted_json = json.dumps(data[ctrl_key], indent=2)
            messagebox.showinfo(
                "Gains Saved to JSON",
                f"Gains for '{ctrl_key}' successfully saved to:\n{out_path}\n\nSaved JSON Object:\n{formatted_json}"
            )
            self.status_label.config(text=f"Status: Saved {ctrl_key} gains to JSON", fg=ACCENT_GREEN)
        except Exception as e:
            messagebox.showerror("Error", f"Could not save gains to JSON file:\n{e}")

    # ==============================================================================
    # Optimization Thread management
    # ==============================================================================
    def start_optimization(self):
        if self.opt_thread and self.opt_thread.is_alive():
            return

        ctrl_key = CONTROLLER_KEYS.get(self.controller_opt, f"controller_opt_{self.controller_opt}")
        self.opt_button.config(state=tk.DISABLED, bg=BG_CARD)
        self.stop_button.config(state=tk.NORMAL, bg=ACCENT_RED, fg=BG_DARK)
        self.status_label.config(text=f"Status: Optimizing {ctrl_key} (DE population)...", fg=ACCENT_GREEN)

        current_params = np.array([slider.get() for slider in self.sliders])
        nu_max = self.get_nu_max()
        self.opt_queue = queue.Queue()
        self.opt_thread = OptimizerWorker(
            self.opt_queue, current_params, BOUNDS, opt=self.controller_opt, nu_max=nu_max
        )
        self.opt_thread.start()

        self.root.after(100, self.poll_opt_queue)

    def stop_optimization(self):
        if self.opt_thread and self.opt_thread.is_alive():
            self.opt_thread.stop_requested = True
            self.status_label.config(text="Status: Stopping optimization...", fg=ACCENT_RED)

    def poll_opt_queue(self):
        if not self.opt_queue:
            return

        try:
            while True:
                msg_type, *data = self.opt_queue.get_nowait()

                if msg_type == 'update':
                    xk, current_cost = data
                    self.updating_gui = True
                    for i, val in enumerate(xk):
                        self.sliders[i].set(val)
                    self.updating_gui = False

                    self.update_plot_immediate()
                    ctrl_key = CONTROLLER_KEYS.get(self.controller_opt, f"controller_opt_{self.controller_opt}")
                    self.status_label.config(text=f"Status: Optimizing {ctrl_key}... Best Cost: {current_cost:.5f}", fg=ACCENT_GREEN)

                elif msg_type == 'done':
                    xk, final_cost = data
                    self.updating_gui = True
                    for i, val in enumerate(xk):
                        self.sliders[i].set(val)
                    self.updating_gui = False

                    self.update_plot_immediate()
                    ctrl_key = CONTROLLER_KEYS.get(self.controller_opt, f"controller_opt_{self.controller_opt}")
                    self.status_label.config(text=f"Status: Optimization Complete for {ctrl_key}! (Final Cost: {final_cost:.5f})", fg=ACCENT_GREEN)
                    self.cleanup_opt_thread()
                    messagebox.showinfo("Optimization Complete", f"Optimization finished successfully for '{ctrl_key}'!\nFinal Cost: {final_cost:.6f}")
                    return

                elif msg_type == 'error':
                    err_msg = data[0]
                    self.status_label.config(text=f"Status: Error - {err_msg}", fg=ACCENT_RED)
                    self.cleanup_opt_thread()
                    messagebox.showerror("Optimization Error", f"An error occurred during optimization:\n{err_msg}")
                    return

        except queue.Empty:
            pass

        if self.opt_thread and self.opt_thread.is_alive():
            self.root.after(100, self.poll_opt_queue)
        else:
            self.cleanup_opt_thread()

    def cleanup_opt_thread(self):
        self.opt_button.config(state=tk.NORMAL, bg=ACCENT_GREEN, fg=BG_DARK)
        self.stop_button.config(state=tk.DISABLED, bg=BG_CARD, fg=TEXT_MUTED)
        self.opt_thread = None
        self.opt_queue = None


# ==============================================================================
# Entry Point
# ==============================================================================
if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(1200, 750)

    app = GainsTuningGUI(root)

    def on_closing():
        plt.close('all')
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
