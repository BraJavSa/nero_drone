#!/usr/bin/env python3
"""
Interactive Drone Dynamics Simulation and Control Plotter
========================================================
Simulates the closed-loop 4DOF drone dynamics using:
- The identified stable discrete-time state-space model (Ad, Bd) at 10 Hz.
- The sliding-mode controller (SCController) with updated Model_simp parameters.
- 16 interactive gain sliders (Ksp, Ksd, Kp, Kd for all axes) for real-time tuning.
- Displays positions, body velocities, control commands, and tracking FIT % dynamically.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button

# ─── 1. SIMULATION PARAMETERS ────────────────────────────────────────────────
HZ = 10.0
DT = 1.0 / HZ
DURATION = 60.0
N_STEPS = int(DURATION / DT)
T_AXIS = np.arange(N_STEPS) * DT

# Initial State
X0_POS = np.array([0.0, 0.0, 1.5, 0.0])   # [x, y, z, yaw]
V0_BODY = np.zeros(4)                      # [vx_b, vy_b, vz_b, r]

# Reference Trajectory (from ref_pos.py with L = 1.5)
L = 1.5
REF_POINTS = np.array([
    [ L/2,  L/2, 1.8],
    [-L/2,  L/2, 1.5],
    [-L/2, -L/2, 1.3],
    [ 0.0,  0.0, 1.6],
])
REF_YAWS = np.deg2rad([45.0, 135.0, -135.0, 0.0])
REF_HOLD = 10.0  # seconds per pose

# ─── 2. IDENTIFIED CLOSED-LOOP SYSTEM MATRICES (Ad, Bd) ──────────────────────
Ad = np.array([
    [0.764600, -0.030258, 0.000000, 0.017207],
    [0.005115,  0.859005, 0.000000, 0.006239],
    [0.000000,  0.000000, 0.793746, 0.000000],
    [0.000000,  0.000000, 0.000000, 0.750789]
])

Bd = np.array([
    [0.102625, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.072230, 0.000000, 0.000000],
    [0.000000, 0.000000, 0.234511, 0.000000],
    [0.000000, 0.000000, 0.000000, 0.256992]
])

# Simplified continuous parameters Kv and Ku derived via: Ku = Bd/dt, Kv = (I-Ad)/dt
MODEL_SIMP = np.array([1.02625, 2.35400, 0.72230, 1.40995,
                       2.34511, 2.06254, 2.56992, 2.49211])

# ─── 3. SLIDING MODE CONTROLLER (SCController) ───────────────────────────────
class SCController:
    def __init__(self, gains: dict):
        self.gains     = gains
        self.w_Ur_prev = np.zeros(4)
        self.b_Ud      = np.zeros(4)
        self.b_Ud_ant  = np.zeros(4)

    def compute(self, w_X, w_dX, w_Xd, w_dXd=None) -> np.ndarray:
        if w_dXd is None:
            w_dXd = np.zeros(4)
        g = self.gains
        Ksp = np.diag([g['ksp_x'], g['ksp_y'], g['ksp_z'], g['ksp_psi']])
        Ksd = np.diag([g['ksd_x'], g['ksd_y'], g['ksd_z'], g['ksd_psi']])
        Kp  = np.diag([g['kp_x'],  g['kp_y'],  g['kp_z'],  g['kp_psi']])
        Kd  = np.diag([g['kd_x'],  g['kd_y'],  g['kd_z'],  g['kd_psi']])
        
        Ku  = np.diag(MODEL_SIMP[[0, 2, 4, 6]])
        Kv  = np.diag(MODEL_SIMP[[1, 3, 5, 7]])

        # Position tracking error
        w_Xtil    = w_Xd - w_X
        w_Xtil[3] = math.atan2(math.sin(w_Xtil[3]), math.cos(w_Xtil[3]))

        # Reference auxiliary signal
        w_Ur  = Kd @ w_dXd + Ksp @ np.tanh(Kp @ w_Xtil)
        w_dUr = (w_Ur - self.w_Ur_prev) / DT
        self.w_Ur_prev = w_Ur.copy()

        # Rotation matrix Inertial -> Body
        psi   = w_X[3]
        c, s  = math.cos(psi), math.sin(psi)
        w_F_b = np.array([[ c, -s, 0, 0],
                           [ s,  c, 0, 0],
                           [ 0,  0, 1, 0],
                           [ 0,  0, 0, 1]])

        # Sliding Control Law
        b_Ud_raw  = np.linalg.inv(w_F_b @ Ku) @ (w_dUr + Ksd @ (w_Ur - w_dX) + Kv @ w_dX)

        # Output low-pass filter (alpha = 0.6)
        alpha         = 0.6
        self.b_Ud     = alpha * b_Ud_raw + (1.0 - alpha) * self.b_Ud_ant
        self.b_Ud_ant = self.b_Ud.copy()

        return np.clip(self.b_Ud, -1.0, 1.0)


# Default Gains
DEFAULT_GAINS = {
    'ksp_x': 1.2,   'ksp_y': 1.2,   'ksp_z': 1.2527,  'ksp_psi': 3.0,
    'ksd_x': 0.7,   'ksd_y': 0.7,   'ksd_z': 2.5484,  'ksd_psi': 1.2455,
    'kp_x':  1.95,  'kp_y':  1.95,  'kp_z':  1.5747,  'kp_psi':  2.0,
    'kd_x':  0.0,   'kd_y':  0.0,   'kd_z':  0.0,     'kd_psi':  0.0,
}

SLIDER_CFG = [
    # (key,       label,    min,   max)
    ('ksp_x',   'Ksp x',  0.0,   5.0),
    ('ksp_y',   'Ksp y',  0.0,   5.0),
    ('ksp_z',   'Ksp z',  0.0,   8.0),
    ('ksp_psi', 'Ksp ψ',  0.0,   8.0),
    ('ksd_x',   'Ksd x',  0.0,   8.0),
    ('ksd_y',   'Ksd y',  0.0,   8.0),
    ('ksd_z',   'Ksd z',  0.0,  12.0),
    ('ksd_psi', 'Ksd ψ',  0.0,   6.0),
    ('kp_x',    'Kp x',   0.0,   6.0),
    ('kp_y',    'Kp y',   0.0,   6.0),
    ('kp_z',    'Kp z',   0.0,   6.0),
    ('kp_psi',  'Kp ψ',   0.0,   6.0),
    ('kd_x',    'Kd x',   0.0,   3.0),
    ('kd_y',    'Kd y',   0.0,   3.0),
    ('kd_z',    'Kd z',   0.0,   3.0),
    ('kd_psi',  'Kd ψ',   0.0,   3.0),
]

# ─── 4. DYNAMICS SIMULATION & INTEGRATION ────────────────────────────────────
def get_reference(t: float) -> np.ndarray:
    idx = int(t / REF_HOLD) % len(REF_POINTS)
    p = REF_POINTS[idx]
    return np.array([p[0], p[1], p[2], REF_YAWS[idx]])

def kinematics(state, v_body):
    psi = state[3]
    vx, vy, vz, r = v_body
    xdot = np.cos(psi) * vx - np.sin(psi) * vy
    ydot = np.sin(psi) * vx + np.cos(psi) * vy
    zdot = vz
    psidot = r
    return np.array([xdot, ydot, zdot, psidot])

def run_sim(gains: dict):
    ctrl = SCController(gains)
    
    pos_log = np.zeros((N_STEPS, 4))
    ref_log = np.zeros((N_STEPS, 4))
    vel_log = np.zeros((N_STEPS, 4))
    cmd_log = np.zeros((N_STEPS, 4))

    w_X = X0_POS.copy()
    b_V = V0_BODY.copy()

    for k in range(N_STEPS):
        t = k * DT
        w_Xd = get_reference(t)
        
        psi = w_X[3]
        c, s = math.cos(psi), math.sin(psi)
        w_F_b = np.array([[ c, -s, 0, 0],
                           [ s,  c, 0, 0],
                           [ 0,  0, 1, 0],
                           [ 0,  0, 0, 1]])
        w_dX = w_F_b @ b_V

        u_cmd = ctrl.compute(w_X, w_dX, w_Xd)

        pos_log[k] = w_X.copy()
        ref_log[k] = w_Xd.copy()
        vel_log[k] = b_V.copy()
        cmd_log[k] = u_cmd.copy()

        b_V_new = Ad @ b_V + Bd @ u_cmd

        k1 = kinematics(w_X, b_V)
        k2 = kinematics(w_X + 0.5 * DT * k1, 0.5 * (b_V + b_V_new))
        k3 = kinematics(w_X + 0.5 * DT * k2, 0.5 * (b_V + b_V_new))
        k4 = kinematics(w_X + DT * k3, b_V_new)
        w_X = w_X + (DT / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)

        w_X[3] = math.remainder(w_X[3], 2.0 * math.pi)
        b_V = b_V_new

    return pos_log, ref_log, vel_log, cmd_log

def fit(y, yhat):
    var_y = np.linalg.norm(y - np.mean(y))
    if var_y == 0: return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y - yhat) / var_y)

# ─── 5. GUI & LAYOUT DESIGN ──────────────────────────────────────────────────
fig = plt.figure(figsize=(18, 10))
fig.canvas.manager.set_window_title('Tuning de Control SC - 16 Sliders')

fig.text(0.5, 0.97, 'Tuning Interactivo de Ganancias del Controlador SC (10 Hz)', 
         ha='center', fontsize=15, fontweight='bold', color='#1A2530')
fig.text(0.5, 0.945, 'Mueve y suelta cualquier slider para simular y ver el impacto dinámico en tiempo real', 
         ha='center', fontsize=9, color='gray')

# Layout: GridSpec for subplots (upper part) and sliders (lower part)
gs_plots = gridspec.GridSpec(3, 4, figure=fig, left=0.05, right=0.97, top=0.92, bottom=0.40, hspace=0.38, wspace=0.28)
gs_sliders = gridspec.GridSpec(4, 4, figure=fig, left=0.05, right=0.97, top=0.34, bottom=0.04, hspace=0.95, wspace=0.45)

colors_est = ['#1E88E5', '#43A047', '#E53935', '#8E24AA']
colors_ref = '#757575'
colors_cmd = ['#0277BD', '#2E7D32', '#C62828', '#6A1B9A']

labels_pos = ['X (m)', 'Y (m)', 'Z (m)', 'Yaw (°)']
titles_pos = ['Posición X', 'Posición Y', 'Altitud Z', 'Guiñada ψ']

labels_vel = ['Vx (m/s)', 'Vy (m/s)', 'Vz (m/s)', 'Yaw Rate (rad/s)']
titles_vel = ['Velocidad VX', 'Velocidad VY', 'Velocidad VZ', 'Yaw Rate r']

labels_cmd = ['Ux (cmd)', 'Uy (cmd)', 'Uz (cmd)', 'Upsi (cmd)']
titles_cmd = ['Comando Ux', 'Comando Uy', 'Comando Uz', 'Comando Uψ']

# Initial simulation
pos0, ref0, vel0, cmd0 = run_sim(DEFAULT_GAINS.copy())

axes_pos, lines_pos_ref, lines_pos_est = [], [], []
axes_vel, lines_vel = [], []
axes_cmd, lines_cmd = [], []

# ROW 1: POSITIONS (Ref vs Sim)
for i in range(4):
    ax = fig.add_subplot(gs_plots[0, i])
    yref = np.degrees(ref0[:, i]) if i == 3 else ref0[:, i]
    yest = np.degrees(pos0[:, i]) if i == 3 else pos0[:, i]
    
    lr, = ax.plot(T_AXIS, yref, color=colors_ref, lw=1.1, ls='--', label='Ref')
    le, = ax.plot(T_AXIS, yest, color=colors_est[i], lw=1.8, label='Simulado')
    
    lines_pos_ref.append(lr)
    lines_pos_est.append(le)
    axes_pos.append(ax)

    for k in range(1, int(DURATION / REF_HOLD)):
        ax.axvline(x=k * REF_HOLD, color='#CFD8DC', lw=0.7, ls=':')

    ax.set_title(f"{titles_pos[i]} - FIT: {fit(yref, yest):.2f}%", fontsize=10, fontweight='bold', color='#2C3E50')
    ax.set_xlabel('t (s)', fontsize=8)
    ax.set_ylabel(labels_pos[i], fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, linewidth=0.5, alpha=0.4, ls=':')
    if i == 0:
        ax.legend(fontsize=7, loc='upper right', framealpha=0.6)

# ROW 2: VELOCITIES
for i in range(4):
    ax = fig.add_subplot(gs_plots[1, i])
    lv, = ax.plot(T_AXIS, vel0[:, i], color=colors_est[i], lw=1.5)
    
    lines_vel.append(lv)
    axes_vel.append(ax)
    
    for k in range(1, int(DURATION / REF_HOLD)):
        ax.axvline(x=k * REF_HOLD, color='#CFD8DC', lw=0.7, ls=':')

    ax.set_title(titles_vel[i], fontsize=10, fontweight='bold', color='#2C3E50')
    ax.set_xlabel('t (s)', fontsize=8)
    ax.set_ylabel(labels_vel[i], fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, linewidth=0.5, alpha=0.4, ls=':')

# ROW 3: CONTROL INPUTS
for i in range(4):
    ax = fig.add_subplot(gs_plots[2, i])
    lc, = ax.plot(T_AXIS, cmd0[:, i], color=colors_cmd[i], lw=1.3)
    
    lines_cmd.append(lc)
    axes_cmd.append(ax)
    
    for k in range(1, int(DURATION / REF_HOLD)):
        ax.axvline(x=k * REF_HOLD, color='#CFD8DC', lw=0.7, ls=':')

    ax.set_title(titles_cmd[i], fontsize=10, fontweight='bold', color='#2C3E50')
    ax.set_xlabel('t (s)', fontsize=8)
    ax.set_ylabel(labels_cmd[i], fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, linewidth=0.5, alpha=0.4, ls=':')

# Create Sliders
sliders = {}
for idx, (key, lbl, vmin, vmax) in enumerate(SLIDER_CFG):
    row = idx // 4
    col = idx %  4
    ax_s = fig.add_subplot(gs_sliders[row, col])
    sl = Slider(ax=ax_s, label=lbl,
                valmin=vmin, valmax=vmax,
                valinit=DEFAULT_GAINS[key],
                valstep=round((vmax - vmin) / 500, 5))
    sl.label.set_fontsize(8)
    sl.valtext.set_fontsize(7)
    sliders[key] = sl

# Reset Button
ax_btn = fig.add_axes([0.43, 0.004, 0.14, 0.022])
btn_reset = Button(ax_btn, 'Resetear Ganancias')
btn_reset.label.set_fontsize(8)

# Callbacks
def _redraw(pos_new, ref_new, vel_new, cmd_new):
    for i in range(4):
        yref = np.degrees(ref_new[:, i]) if i == 3 else ref_new[:, i]
        yest = np.degrees(pos_new[:, i]) if i == 3 else pos_new[:, i]
        lines_pos_ref[i].set_ydata(yref)
        lines_pos_est[i].set_ydata(yest)
        axes_pos[i].relim()
        axes_pos[i].autoscale_view()
        axes_pos[i].set_title(f"{titles_pos[i]} - FIT: {fit(yref, yest):.2f}%", fontsize=10, fontweight='bold', color='#2C3E50')
    
    for i in range(4):
        lines_vel[i].set_ydata(vel_new[:, i])
        axes_vel[i].relim()
        axes_vel[i].autoscale_view()
        
    for i in range(4):
        lines_cmd[i].set_ydata(cmd_new[:, i])
        axes_cmd[i].relim()
        axes_cmd[i].autoscale_view()
        
    fig.canvas.draw_idle()

def _on_release(event):
    for sl in sliders.values():
        if event.inaxes == sl.ax:
            g = {k: s.val for k, s in sliders.items()}
            pos_new, ref_new, vel_new, cmd_new = run_sim(g)
            _redraw(pos_new, ref_new, vel_new, cmd_new)
            return

def _on_reset(event):
    for key, sl in sliders.items():
        sl.set_val(DEFAULT_GAINS[key])
    pos_new, ref_new, vel_new, cmd_new = run_sim(DEFAULT_GAINS.copy())
    _redraw(pos_new, ref_new, vel_new, cmd_new)

fig.canvas.mpl_connect('button_release_event', _on_release)
btn_reset.on_clicked(_on_reset)

plt.show()
