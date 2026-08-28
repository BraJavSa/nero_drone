#!/usr/bin/env python3
r"""
System Comparison Simulator (No Delays): Direct Acceleration vs Full Dynamics.

Compares two continuous state-space drone plant dynamics using RK4 (15 Hz) without delays:
  1. Direct Acceleration System:
     \dot{\boldsymbol{\nu}} = f_1\,\boldsymbol{U} - f_2\,\boldsymbol{\nu}
     \dot{\boldsymbol{\eta}} = J(\psi) \boldsymbol{\nu}

  2. Full Dynamics System:
     Includes kinematic coupling term \dot{\psi}\,J_p(\psi)\,\boldsymbol{\nu}:
     \ddot{\boldsymbol{\eta}} = \dot{\psi}\,J_p(\psi)\,\boldsymbol{\nu} + J(\psi)\,\dot{\boldsymbol{\nu}}

Interactive Controller Selection (Buttons in GUI):
  - OPT 1: gt_extended_controller.py (GT Extended / Inverse Dynamics)
  - OPT 2: first_order_controller.py (First-Order Cascade Controller)
  - OPT 3: second_order_controller.py (Second-Order Inverse Dynamics No Jdot)
"""

import json
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# ─────────────────────── Controller Option Selector ───────────────────────
OPT = 2  # Default controller on startup


# ─────────────────────── Load Identified Plant Parameters ───────────────
JSON_PATH = os.path.join(
    os.path.dirname(__file__),
    "system_identification_parameters_optitrack_4dof.json"
)

if os.path.exists(JSON_PATH):
    with open(JSON_PATH, "r") as f:
        json_data = json.load(f)["identified_parameters"]
        F1_DIAG = np.array([
            json_data["x"]["f1"],
            json_data["y"]["f1"],
            json_data["z"]["f1"],
            json_data["yaw"]["f1"]
        ])
        F2_DIAG = np.array([
            json_data["x"]["f2"],
            json_data["y"]["f2"],
            json_data["z"]["f2"],
            json_data["yaw"]["f2"]
        ])
else:
    # Fallback default values
    F1_DIAG = np.array([0.921527, 1.053286, 3.8086879470003603, 8.772786])
    F2_DIAG = np.array([0.247044, 0.395160, 3.7414469817348253, 6.101834])

# ─────────────────────── Reference Sequence ───────────────────────────
HOLD_TIME = 10.0  # seconds per setpoint
L = 1.0
WAYPOINTS = np.array([
    [0.0, 0.0, 1.5, 0.0],
    [-L,  -L,  1.5, 0.0],
    [ L,  -L,  1.5, 0.0],
    [-L,   L,  1.5, 0.0],
    [ L,   L,  1.5, 0.0],
])

# ─────────────────────── Simulation Parameters ─────────────────────────
RATE_HZ = 15.0
DT = 1.0 / RATE_HZ
T_TOTAL = len(WAYPOINTS) * HOLD_TIME  # 50.0 seconds
DELAYS_ZERO = np.zeros(4)            # Zero delay mode for direct comparison


# ─────────────────────── Kinematics & Partial Derivatives ─────────────

def wrap_angle(angle: float) -> float:
    """Wrap angle into [-pi, pi]."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def jacobian(psi: float) -> np.ndarray:
    """Kinematic transformation matrix J(psi)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c, -s, 0.0, 0.0],
        [ s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_inv(psi: float) -> np.ndarray:
    """Inverse of kinematic transformation matrix J(psi)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c,  s, 0.0, 0.0],
        [-s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_p(psi: float) -> np.ndarray:
    """Partial derivative dJ(psi)/dpsi."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [-s, -c, 0.0, 0.0],
        [ c, -s, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ])


def get_reference(t: float):
    """Return (eta_d, nu_d) for a given time t."""
    cycle_t = t % (HOLD_TIME * len(WAYPOINTS))
    idx = int(cycle_t // HOLD_TIME)
    idx = min(idx, len(WAYPOINTS) - 1)
    eta_d = WAYPOINTS[idx].copy()
    nu_d = np.zeros(4)
    return eta_d, nu_d


# ─────────────────────── Continuous State Space Models ─────────────────

def state_space_direct_acc(x: np.ndarray, U: np.ndarray) -> np.ndarray:
    """System 1: Direct Acceleration Model."""
    eta = x[0:4]
    nu = x[4:8]
    psi = eta[3]

    J_mat = jacobian(psi)
    eta_dot = J_mat @ nu
    nu_dot = F1_DIAG * U - F2_DIAG * nu

    return np.concatenate([eta_dot, nu_dot])


def state_space_full_dynamics(x: np.ndarray, U: np.ndarray) -> np.ndarray:
    """System 2: Full Dynamics Model with Kinematic Coupling Coupling."""
    eta = x[0:4]
    nu = x[4:8]
    psi = eta[3]

    J_mat = jacobian(psi)
    eta_dot = J_mat @ nu
    nu_dot = F1_DIAG * U - F2_DIAG * nu

    return np.concatenate([eta_dot, nu_dot])


# ─────────────────────── RK4 Integrator ────────────────────────────────

def rk4_step(x: np.ndarray, U: np.ndarray, dt: float, model_type: str = "full") -> np.ndarray:
    f_dyn = state_space_full_dynamics if model_type == "full" else state_space_direct_acc

    k1 = f_dyn(x, U)
    k2 = f_dyn(x + 0.5 * dt * k1, U)
    k3 = f_dyn(x + 0.5 * dt * k2, U)
    k4 = f_dyn(x + dt * k3, U)

    x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x_next[3] = wrap_angle(x_next[3])
    return x_next


# ─────────────────────── Controller Implementations ─────────────────────

class GtExtendedController:
    """OPT = 1: Inverse Dynamics Controller (from gt_extended_controller.py)"""
    def __init__(self, dt: float = DT):
        self.Ts = dt
        self.KP  = np.diag([1.000000, 1.000000, 1.000000, 1.000000])
        self.KSP = np.diag([1.000000, 1.000000, 1.000000, 1.000000])
        self.KD  = np.diag([1.000000, 1.000000, 1.000000, 1.000000])
        self.KSD = np.diag([1.200000, 1.200000, 1.000000, 1.000000])
        self.f1_inv = np.diag(1.0 / F1_DIAG)
        self.f2 = np.diag(F2_DIAG)
        self.U_MAX = np.ones(4)

    def reset(self):
        pass

    def compute_control(self, eta: np.ndarray, nu: np.ndarray, eta_d: np.ndarray, dX_d: np.ndarray = None, ddX_d: np.ndarray = None) -> np.ndarray:
        if dX_d is None: dX_d = np.zeros(4)
        if ddX_d is None: ddX_d = np.zeros(4)

        psi = eta[3]
        J_mat = jacobian(psi)
        J_inv_mat = jacobian_inv(psi)

        eta_dot = J_mat @ nu
        eta_tilde = eta_d - eta
        eta_tilde[3] = wrap_angle(eta_tilde[3])
        eta_dot_tilde = dX_d - eta_dot

        alpha = (ddX_d
                 + self.KSD @ np.tanh(self.KD @ eta_dot_tilde)
                 + self.KSP @ np.tanh(self.KP @ eta_tilde))

        Ud = self.f1_inv @ (J_inv_mat @ alpha + self.f2 @ nu)
        return np.clip(Ud, -self.U_MAX, self.U_MAX)


class FirstOrderController:
    """OPT = 2: First-Order Cascade Controller (from first_order_controller.py)"""
    def __init__(self, dt: float = DT):
        self.Ts = dt
        self.KP  = np.diag([3.880000, 3.416000, 8.000000, 12.533000])
        self.KSP = np.diag([0.203000, 0.203000, 0.200000, 0.301000])
        self.KD  = np.diag([8.835000, 10.000000, 2.376000, 1.962000])
        self.KSD = np.diag([2.000000, 2.000000, 0.985000, 0.800000])
        self.f1 = np.diag(F1_DIAG)
        self.f2 = np.diag(F2_DIAG)
        self.U_MAX = np.ones(4)
        self.Ucw_prev = np.zeros(4)

    def reset(self):
        self.Ucw_prev = np.zeros(4)

    def compute_control(self, eta: np.ndarray, nu: np.ndarray, eta_d: np.ndarray, dX_d: np.ndarray = None, ddX_d: np.ndarray = None) -> np.ndarray:
        if dX_d is None: dX_d = np.zeros(4)

        psi = eta[3]
        J_mat = jacobian(psi)
        dX = J_mat @ nu

        Xtil = eta_d - eta
        Xtil[3] = wrap_angle(Xtil[3])

        Ucw = dX_d + self.KSP @ np.tanh(self.KP @ Xtil)
        dUcw = (Ucw - self.Ucw_prev) / self.Ts
        self.Ucw_prev = Ucw.copy()

        A = J_mat @ self.f1
        b = dUcw + self.KSD @ (Ucw - dX) + self.f2 @ dX
        Udw = np.linalg.solve(A, b)

        return np.clip(Udw, -self.U_MAX, self.U_MAX)


class SecondOrderController:
    """OPT = 3: Second-Order Inverse Dynamics Controller (from second_order_controller.py)"""
    def __init__(self, dt: float = DT):
        self.Ts = dt
        self.KP  = np.diag([1.000000, 1.000000, 1.000000, 1.000000])
        self.KSP = np.diag([1.000000, 1.000000, 1.000000, 1.000000])
        self.KD  = np.diag([1.000000, 1.000000, 1.000000, 1.000000])
        self.KSD = np.diag([1.200000, 1.200000, 1.000000, 1.000000])
        self.f1_inv = np.diag(1.0 / F1_DIAG)
        self.f2 = np.diag(F2_DIAG)
        self.U_MAX = np.ones(4)

    def reset(self):
        pass

    def compute_control(self, eta: np.ndarray, nu: np.ndarray, eta_d: np.ndarray, dX_d: np.ndarray = None, ddX_d: np.ndarray = None) -> np.ndarray:
        if dX_d is None: dX_d = np.zeros(4)
        if ddX_d is None: ddX_d = np.zeros(4)

        psi = eta[3]
        J_mat = jacobian(psi)
        dX = J_mat @ nu

        Xtil = eta_d - eta
        Xtil[3] = wrap_angle(Xtil[3])
        dXtil = dX_d - dX

        alpha = (ddX_d
                 + self.KSP @ np.tanh(self.KP @ Xtil)
                 + self.KSD @ np.tanh(self.KD @ dXtil))

        J_inv_mat = jacobian_inv(psi)
        Udw = self.f1_inv @ (J_inv_mat @ alpha + self.f2 @ nu)

        return np.clip(Udw, -self.U_MAX, self.U_MAX)


def get_controller_by_opt(opt: int, dt: float = DT):
    if opt == 1:
        return GtExtendedController(dt=dt), "GT Extended (gt_extended_controller.py)"
    elif opt == 2:
        return FirstOrderController(dt=dt), "First Order (first_order_controller.py)"
    elif opt == 3:
        return SecondOrderController(dt=dt), "Second Order (second_order_controller.py)"
    else:
        return FirstOrderController(dt=dt), "First Order (first_order_controller.py)"


# ─────────────────────── Simulation Engine ─────────────────────────────

def run_simulation(model_type: str = "direct", opt: int = OPT):
    """
    Run closed-loop RK4 simulation (no delays) for a given system model type ("direct" or "full").
    Returns: (t_hist, eta_hist, nu_hist, u_hist, ref_hist, ctrl_name)
    """
    controller, ctrl_name = get_controller_by_opt(opt, dt=DT)
    controller.reset()

    # Initial state: [x, y, z, yaw, u, v, w, r]^T with z0 = 1.5 m
    x = np.array([0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0])

    N_steps = int(T_TOTAL / DT)
    t_hist = np.zeros(N_steps)
    eta_hist = np.zeros((N_steps, 4))
    nu_hist = np.zeros((N_steps, 4))
    u_hist = np.zeros((N_steps, 4))
    ref_hist = np.zeros((N_steps, 4))

    for k in range(N_steps):
        t = k * DT
        eta = x[0:4]
        nu = x[4:8]

        eta_d, nu_d = get_reference(t)

        # Compute control action U_k
        U_k = controller.compute_control(eta, nu, eta_d, dX_d=nu_d)

        # Log history
        t_hist[k] = t
        eta_hist[k] = eta.copy()
        nu_hist[k] = nu.copy()
        u_hist[k] = U_k.copy()
        ref_hist[k] = eta_d.copy()

        # Integrate state forward with RK4 (No delay)
        x = rk4_step(x, U_k, DT, model_type=model_type)

    return t_hist, eta_hist, nu_hist, u_hist, ref_hist, ctrl_name


# ─────────────────────── Overshoot & Settling Stats ─────────────────────

def compute_overshoot_stats(t_hist, eta_hist, ref_hist, label=""):
    step_indices = []
    for k in range(1, len(ref_hist)):
        if not np.allclose(ref_hist[k], ref_hist[k-1], atol=1e-5):
            step_indices.append(k)
    step_indices.append(len(ref_hist))

    axis_names = ['X', 'Y', 'Z', 'Yaw']
    print(f"\n============================================================")
    print(f"OVERSHOOT & SETTLING ANALYSIS ({label})")
    print(f"============================================================")

    for i in range(4):
        overshoots = []
        settling_times = []

        for s in range(len(step_indices) - 1):
            k_start = step_indices[s]
            k_end = step_indices[s+1]

            val_start = ref_hist[k_start - 1, i]
            val_target = ref_hist[k_start, i]
            step_size = abs(val_target - val_start)

            if step_size < 1e-3:
                continue

            resp = eta_hist[k_start:k_end, i]

            if val_target > val_start:
                peak = np.max(resp)
                ov = (peak - val_target) / step_size * 100.0
            else:
                peak = np.min(resp)
                ov = (val_target - peak) / step_size * 100.0

            overshoots.append(max(0.0, ov))

            tol = 0.05 * step_size
            settled = np.abs(resp - val_target) <= tol

            settling_idx = None
            for j in range(len(settled) - 1, -1, -1):
                if not settled[j]:
                    settling_idx = j + 1
                    break
            if settling_idx is not None and settling_idx < len(settled):
                settling_times.append(settling_idx * DT)
            else:
                settling_times.append(0.0)

        if overshoots:
            print(f"  {axis_names[i]:4s}: Avg Overshoot = {np.mean(overshoots):6.1f}%  "
                  f"Max Overshoot = {np.max(overshoots):6.1f}%  "
                  f"Avg Settling = {np.mean(settling_times):5.2f}s")
        else:
            print(f"  {axis_names[i]:4s}: No step changes detected")

    print("=" * 60)


# ─────────────────────── Interactive Plotting GUI ───────────────────────

class InteractiveSimulatorGUI:
    """Interactive Matplotlib GUI Comparing Direct Acc vs Full Dynamics (No Delays)."""
    def __init__(self, initial_opt: int = OPT):
        self.opt = initial_opt

        # Run simulations for both systems without delays
        self.t0, self.eta0, self.nu0, self.u0, self.ref0, self.ctrl_name = run_simulation(
            model_type="direct", opt=self.opt
        )
        self.t1, self.eta1, self.nu1, self.u1, self.ref1, _ = run_simulation(
            model_type="full", opt=self.opt
        )

        compute_overshoot_stats(self.t0, self.eta0, self.ref0, label=f"{self.ctrl_name} - Direct Acc System")
        compute_overshoot_stats(self.t1, self.eta1, self.ref1, label=f"{self.ctrl_name} - Full Dynamics System")

        # Styling
        plt.rcParams.update({
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "black",
            "axes.linewidth": 0.8,
            "axes.grid": True,
            "grid.color": "#d0d0d0",
            "grid.linewidth": 0.45,
            "grid.linestyle": "--",
            "font.family": "serif",
            "font.size": 9,
            "legend.frameon": True,
            "legend.framealpha": 1.0,
            "legend.edgecolor": "black",
            "legend.facecolor": "white",
            "legend.fontsize": 8,
            "lines.linewidth": 1.2,
        })

        self.fig, self.axes = plt.subplots(4, 3, figsize=(17, 10), sharex=True)
        self.fig.subplots_adjust(top=0.91, bottom=0.08)

        # Interactive Controller Selection Buttons at top
        ax_b1 = self.fig.add_axes([0.15, 0.94, 0.22, 0.04])
        ax_b2 = self.fig.add_axes([0.39, 0.94, 0.22, 0.04])
        ax_b3 = self.fig.add_axes([0.63, 0.94, 0.22, 0.04])

        self.btn1 = Button(ax_b1, 'OPT 1: GT Extended', color='#e0e0e0', hovercolor='#bbdefb')
        self.btn2 = Button(ax_b2, 'OPT 2: First Order', color='#e0e0e0', hovercolor='#bbdefb')
        self.btn3 = Button(ax_b3, 'OPT 3: Second Order', color='#e0e0e0', hovercolor='#bbdefb')

        self.btn1.on_clicked(lambda event: self.update_controller(1))
        self.btn2.on_clicked(lambda event: self.update_controller(2))
        self.btn3.on_clicked(lambda event: self.update_controller(3))

        axis_labels = ['X [m]', 'Y [m]', 'Z [m]', 'Yaw [rad]']
        vel_labels = ['v_x Body [m/s]', 'v_y Body [m/s]', 'v_z Body [m/s]', 'r Body [rad/s]']
        u_labels = ['ux [cmd]', 'uy [cmd]', 'uz [cmd]', 'uψ [cmd]']

        self.lines_pos_ref = []
        self.lines_pos_direct = []
        self.lines_pos_full = []

        self.lines_vel_direct = []
        self.lines_vel_full = []

        self.lines_u_direct = []
        self.lines_u_full = []

        for i in range(4):
            # Col 0: Position Tracking
            ax0 = self.axes[i, 0]
            l_ref, = ax0.plot(self.t0, self.ref0[:, i], 'k--', linewidth=1.5, label='Reference', zorder=10)
            l_direct, = ax0.plot(self.t0, self.eta0[:, i], color='#1a56a0', label='Direct Acc Model', zorder=5)
            l_full, = ax0.plot(self.t1, self.eta1[:, i], color='#d35400', label='Full Dynamics Model', zorder=5)
            ax0.set_ylabel(axis_labels[i])
            if i == 0:
                ax0.set_title('Position Tracking (No Delays)')
                ax0.legend(loc='upper right', fontsize=8)

            self.lines_pos_ref.append(l_ref)
            self.lines_pos_direct.append(l_direct)
            self.lines_pos_full.append(l_full)

            # Col 1: Body Velocity
            ax1 = self.axes[i, 1]
            l_v_direct, = ax1.plot(self.t0, self.nu0[:, i], color='#1a56a0', label='Direct Acc Model')
            l_v_full, = ax1.plot(self.t1, self.nu1[:, i], color='#d35400', label='Full Dynamics Model')
            ax1.set_ylabel(vel_labels[i])
            ax1.set_ylim(-1.2, 1.2)
            if i == 0:
                ax1.set_title('Body Velocity (No Delays)')
                ax1.legend(loc='upper right', fontsize=8)

            self.lines_vel_direct.append(l_v_direct)
            self.lines_vel_full.append(l_v_full)

            # Col 2: Control Action
            ax2 = self.axes[i, 2]
            l_u_direct, = ax2.plot(self.t0, self.u0[:, i], color='#1a56a0', label='Direct Acc Model')
            l_u_full, = ax2.plot(self.t1, self.u1[:, i], color='#d35400', label='Full Dynamics Model')
            ax2.axhline(1.0, color='red', linestyle=':', linewidth=0.8, alpha=0.5)
            ax2.axhline(-1.0, color='red', linestyle=':', linewidth=0.8, alpha=0.5)
            ax2.set_ylabel(u_labels[i])
            ax2.set_ylim(-1.15, 1.15)
            if i == 0:
                ax2.set_title('Control Action Applied (No Delays)')
                ax2.legend(loc='upper right', fontsize=8)

            self.lines_u_direct.append(l_u_direct)
            self.lines_u_full.append(l_u_full)

        for j in range(3):
            self.axes[3, j].set_xlabel('Time [s]')

        self._highlight_button(self.opt)

    def _highlight_button(self, opt):
        buttons = [self.btn1, self.btn2, self.btn3]
        for idx, btn in enumerate(buttons, start=1):
            if idx == opt:
                btn.ax.set_facecolor('#90caf9')  # Highlight blue
            else:
                btn.ax.set_facecolor('#e0e0e0')  # Default gray

    def update_controller(self, new_opt: int):
        if self.opt == new_opt:
            return
        self.opt = new_opt
        self._highlight_button(new_opt)

        print(f"\n" + "#" * 60)
        print(f"SWITCHING TO CONTROLLER OPT = {new_opt}")
        print("#" * 60)

        self.t0, self.eta0, self.nu0, self.u0, self.ref0, self.ctrl_name = run_simulation(
            model_type="direct", opt=self.opt
        )
        self.t1, self.eta1, self.nu1, self.u1, self.ref1, _ = run_simulation(
            model_type="full", opt=self.opt
        )

        compute_overshoot_stats(self.t0, self.eta0, self.ref0, label=f"{self.ctrl_name} - Direct Acc System")
        compute_overshoot_stats(self.t1, self.eta1, self.ref1, label=f"{self.ctrl_name} - Full Dynamics System")

        for i in range(4):
            self.lines_pos_ref[i].set_ydata(self.ref0[:, i])
            self.lines_pos_direct[i].set_ydata(self.eta0[:, i])
            self.lines_pos_full[i].set_ydata(self.eta1[:, i])

            self.lines_vel_direct[i].set_ydata(self.nu0[:, i])
            self.lines_vel_full[i].set_ydata(self.nu1[:, i])

            self.lines_u_direct[i].set_ydata(self.u0[:, i])
            self.lines_u_full[i].set_ydata(self.u1[:, i])

            ax0 = self.axes[i, 0]
            ax0.relim()
            ax0.autoscale_view(scalex=False, scaley=True)

        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


# ─────────────────────── Entry Point ───────────────────────────────────

if __name__ == '__main__':
    gui = InteractiveSimulatorGUI(initial_opt=OPT)
    gui.show()
