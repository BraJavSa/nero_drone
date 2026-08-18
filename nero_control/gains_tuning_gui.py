#!/usr/bin/env python3
"""
Interactive Gain Tuning Dashboard and Optimizer for the Bebop drone cascade controller.
"""

import sys
import os
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
# Simulator Dynamics (matching gains_optimization_position.py exactly)
# ==============================================================================
# Identified OptiTrack parameters (system_identification_parameters_optitrack_4dof.json)
_F1 = np.diag([0.921527, 1.053286, 4.173221, 8.772786])
_F2 = np.diag([0.247044, 0.395160, 1.975836, 6.101834])

DYN_DT  = 0.064        # 64 ms
CTRL_DT = 1.0 / 15.0  # 15 Hz

CTRL_F1 = np.array([
    [0.921527, 0.000000, 0.000000, 0.000000],
    [0.000000, 1.053286, 0.000000, 0.000000],
    [0.000000, 0.000000, 4.173221, 0.000000],
    [0.000000, 0.000000, 0.000000, 8.772786],
])
CTRL_F2 = np.array([
    [0.247044, 0.000000, 0.000000, 0.000000],
    [0.000000, 0.395160, 0.000000, 0.000000],
    [0.000000, 0.000000, 1.975836, 0.000000],
    [0.000000, 0.000000, 0.000000, 6.101834],
])

NU_MAX = np.array([0.9, 0.9, 0.8, 0.8])
U_MAX = np.ones(4)

HOLD_TIME    = 10.0
L            = 1.5
SIM_DURATION = 60.0

POINTS = np.array([
    [0.0, 0.0, 1.8],
    [-L/2, -L/2, 1.6],
    [L/2, -L/2, 1.8],
    [-L/2, L/2, 1.5],
])
YAWS = np.deg2rad([45.0, 45.0, -45.0, -45.0])

def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def J(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c, -s, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

def J_dot(psi, r):
    c, s = np.cos(psi), np.sin(psi)
    return r * np.array([
        [-s, -c, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 0., 0.],
        [0., 0., 0., 0.],
    ])

def get_ref_at(t):
    idx     = int(t / HOLD_TIME) % len(POINTS)
    eta_d   = np.array([POINTS[idx, 0], POINTS[idx, 1], POINTS[idx, 2], YAWS[idx]])
    nu_d    = np.zeros(4)
    alpha_d = np.zeros(4)
    return eta_d, nu_d, alpha_d

def simulate(KP, KSP, KD, KSD, return_history=False):
    eta = np.array([0., 0., 1.5, 0.])
    nu  = np.zeros(4)
    X_dot_ref_prev = np.zeros(4)
    errors = []
    eta_hist = []
    eta_d_hist = []
    t_hist = []
    t = 0.0
    sim_steps = max(1, round(CTRL_DT / DYN_DT))

    while t < SIM_DURATION:
        eta_d, nu_d, alpha_d = get_ref_at(t)

        psi  = eta[3]
        r    = nu[3]

        Jmat   = J(psi)
        Jdot   = J_dot(psi, r)
        F1_eff = Jmat @ CTRL_F1
        F2_eff = Jmat @ CTRL_F2 - Jdot
        F1_inv = np.linalg.inv(F1_eff)

        X_dot = Jmat @ nu

        X_tilde    = eta_d - eta
        X_tilde[3] = wrap_angle(X_tilde[3])

        X_dot_ref   = nu_d + KSP @ np.tanh(KP @ X_tilde)

        X_dot_tilde = X_dot_ref - X_dot
        X_ddot_ref  = (X_dot_ref - X_dot_ref_prev) / CTRL_DT
        X_dot_ref_prev = X_dot_ref.copy()

        Ud = F1_inv @ (
            alpha_d
            + X_ddot_ref
            + KSD @ np.tanh(KD @ X_dot_tilde)
            + F2_eff @ nu
        )
        U_body = np.clip(Ud, -U_MAX, U_MAX)

        errors.append(X_tilde.copy())
        if return_history:
            eta_hist.append(eta.copy())
            eta_d_hist.append(eta_d.copy())
            t_hist.append(t)

        for _ in range(sim_steps):
            nu_dot = _F1 @ U_body - _F2 @ nu
            nu     = nu + DYN_DT * nu_dot
            nu     = np.clip(nu, -NU_MAX, NU_MAX)

            eta_dot  = J(eta[3]) @ nu
            eta      = eta + DYN_DT * eta_dot
            eta[3]   = wrap_angle(eta[3])
            if eta[2] < 0.05:
                eta[2] = 0.05
                nu[2]  = 0.0

        t += CTRL_DT

    if return_history:
        return np.array(errors), np.array(eta_hist), np.array(eta_d_hist), np.array(t_hist)
    return np.array(errors)

def unpack(params):
    KP  = np.diag(params[0:4])
    KSP = np.diag(params[4:8])
    KD  = np.diag(params[8:12])
    KSD = np.diag(params[12:16])
    return KP, KSP, KD, KSD

def cost(params):
    KP, KSP, KD, KSD = unpack(params)

    try:
        errors = simulate(KP, KSP, KD, KSD)
    except Exception:
        return 1e9

    if not np.all(np.isfinite(errors)):
        return 1e9

    N = len(errors)
    w = np.array([1.0, 1.0, 2.5, 2.5])

    ise = np.mean((errors ** 2) * w)

    n_trans = N // 4
    transient_penalty = np.mean((errors[:n_trans] ** 2) * w) * 0.4
    ss_penalty = np.mean((errors[-n_trans:] ** 2) * w) * 1.5

    return ise + transient_penalty + ss_penalty

# Bounds from gains_optimization_position.py
BOUNDS = [
    (0.3,  4.0), (0.3,  4.0), (4.0,  8.0), (8.0, 14.0),  # KP: x, y, z, yaw
    (0.2,  0.9), (0.2,  0.9), (0.1,  0.8), (0.1,  0.8),  # KSP: x, y, z, yaw
    (1.0, 10.0), (1.0, 10.0), (0.5,  5.0), (0.3,  3.0),  # KD: x, y, z, yaw
    (0.2,  2.0), (0.2,  2.0), (0.1,  1.0), (0.1,  0.8),  # KSD: x, y, z, yaw
]

# Preset optimal position gains
DEFAULT_POS_GAINS = [
    4.000000, 4.000000, 8.000000, 11.324606,  # KP
    0.900000, 0.900000, 0.800000, 0.800000,   # KSP
    5.029369, 3.317635, 5.000000, 3.000000,   # KD
    1.917397, 1.986195, 1.000000, 0.800000    # KSD
]

# Preset initial/trajectory gains (just for a reference, clipped to bounds where necessary)
TRAJECTORY_GAINS_RAW = [
    2.500000, 2.500000, 3.045698, 10.000000,  # KP
    0.600000, 0.600000, 0.500000, 0.060002,   # KSP
    4.000000, 4.000000, 1.354406, 0.300000,   # KD
    1.000000, 0.753363, 0.066052, 0.301276    # KSD
]

# Clip trajectory gains to bounds to prevent slider overflow/underflow
TRAJECTORY_GAINS = [
    np.clip(val, BOUNDS[i][0], BOUNDS[i][1]) for i, val in enumerate(TRAJECTORY_GAINS_RAW)
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
    def __init__(self, q, current_params, bounds):
        super().__init__()
        self.q = q
        self.current_params = current_params
        self.bounds = bounds
        self.stop_requested = False
        
    def run(self):
        def callback(xk, *args, **kwargs):
            if self.stop_requested:
                return True
            try:
                c = cost(xk)
            except Exception:
                c = 9e9
            self.q.put(('update', xk.copy(), c))
            
        try:
            # We can run differential evolution.
            # Using workers=1 is 100% safe inside a thread context.
            result = differential_evolution(
                cost,
                self.bounds,
                strategy='best1bin',
                maxiter=10,
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
        self.root.title("🚁 Bebop Cascade Controller Gain Tuning Dashboard")
        self.root.geometry("1400x850")
        self.root.configure(bg=BG_DARK)
        
        self.updating_gui = False
        self.update_timer_id = None
        self.opt_thread = None
        self.opt_queue = None
        
        # Build layout
        self.setup_ui()
        
        # Initial simulation & plot draw
        self.update_plot_immediate()

    def setup_ui(self):
        # 1. Header Frame
        header_frame = tk.Frame(self.root, bg=BG_PANEL, height=50)
        header_frame.pack(side=tk.TOP, fill=tk.X)
        
        header_title = tk.Label(
            header_frame,
            text="🚁 BEBOP CASCADE CONTROLLER - TUNING INTERFACE",
            bg=BG_PANEL,
            fg=ACCENT_PURPLE,
            font=('Arial', 14, 'bold'),
            pady=10
        )
        header_title.pack()

        # 2. Main content container split in Left (Plots) and Right (Controls)
        main_container = tk.Frame(self.root, bg=BG_DARK)
        main_container.pack(fill=tk.BOTH, expand=True)
        
        # Configure columns weights
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

    def setup_plots(self):
        # Create dark themed Matplotlib figure
        plt.style.use('dark_background')
        self.fig, self.axs = plt.subplots(2, 2, figsize=(7, 6))
        self.fig.patch.set_facecolor(BG_DARK)
        self.axs_flat = self.axs.ravel()
        
        # Get one run to capture initial references/time vector
        initial_params = DEFAULT_POS_GAINS
        KP, KSP, KD, KSD = unpack(initial_params)
        errors, eta_hist, eta_d_hist, t_hist = simulate(KP, KSP, KD, KSD, return_history=True)
        
        self.t_hist = t_hist
        self.ref_lines = []
        self.sim_lines = []
        
        labels = ["X Coordinate", "Y Coordinate", "Z Coordinate (Altitude)", "Yaw Angle (Rotation)"]
        units = ["m", "m", "m", "deg"]
        
        for i in range(4):
            ax = self.axs_flat[i]
            ax.set_facecolor(BG_PANEL)
            ax.set_title(f"{labels[i]}", color=TEXT_MAIN, fontsize=10, fontweight='bold', pad=8)
            ax.set_xlabel("Time [s]", color=TEXT_MUTED, fontsize=8)
            ax.set_ylabel(f"[{units[i]}]", color=TEXT_MUTED, fontsize=8)
            
            # Setup grid
            ax.grid(True, color='#313244', linestyle=':', alpha=0.6)
            
            # Format spines
            for spine in ['top', 'right']:
                ax.spines[spine].set_visible(False)
            for spine in ['bottom', 'left']:
                ax.spines[spine].set_color('#45475a')
                
            # Plot reference (convert yaw to degrees)
            ref_data = eta_d_hist[:, i]
            if i == 3:
                ref_data = np.degrees(ref_data)
            ref_line, = ax.plot(t_hist, ref_data, color=ACCENT_RED, linestyle='--', linewidth=1.5, label="Reference")
            self.ref_lines.append(ref_line)
            
            # Plot simulated (convert yaw to degrees)
            sim_data = eta_hist[:, i]
            if i == 3:
                sim_data = np.degrees(sim_data)
            sim_line, = ax.plot(t_hist, sim_data, color=ACCENT_BLUE, linewidth=1.8, label="Simulated")
            self.sim_lines.append(sim_line)
            
            if i == 0:
                ax.legend(facecolor=BG_PANEL, edgecolor='#313244', fontsize=8, loc='upper right')
                
            # Set initial limits
            margin_min = min(np.min(ref_data), np.min(sim_data))
            margin_max = max(np.max(ref_data), np.max(sim_data))
            span = margin_max - margin_min
            if span < 1e-3:
                span = 1.0
            ax.set_ylim(margin_min - 0.15 * span, margin_max + 0.15 * span)
            ax.set_xlim(0, SIM_DURATION)
            ax.tick_params(colors=TEXT_MUTED, labelsize=8)
            
        self.fig.tight_layout()
        
        # Embed in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.left_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def setup_controls(self):
        # 1. Sliders Title
        sliders_title = tk.Label(
            self.right_frame,
            text="Gain Parameters Grid",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 12, 'bold')
        )
        sliders_title.pack(anchor='w', pady=(0, 10))
        
        # 2. Sliders Grid
        grid_frame = tk.Frame(self.right_frame, bg=BG_PANEL)
        grid_frame.pack(fill=tk.BOTH, expand=True)
        
        # Setup column/row weights
        for c in range(5):
            grid_frame.columnconfigure(c, weight=1)
        for r in range(5):
            grid_frame.rowconfigure(r, weight=1)
            
        # Header Labels
        cols = ["KP", "KSP", "KD", "KSD"]
        rows = ["X", "Y", "Z", "Yaw"]
        
        # Draw Column headers
        for c, col_name in enumerate(cols):
            lbl = tk.Label(
                grid_frame,
                text=col_name,
                bg=BG_PANEL,
                fg=ACCENT_PURPLE,
                font=('Arial', 10, 'bold')
            )
            lbl.grid(row=0, column=c+1, padx=4, pady=2, sticky='ew')
            
        # Draw Row headers
        for r, row_name in enumerate(rows):
            lbl = tk.Label(
                grid_frame,
                text=row_name,
                bg=BG_PANEL,
                fg=ACCENT_BLUE,
                font=('Arial', 10, 'bold')
            )
            lbl.grid(row=r+1, column=0, padx=4, pady=2, sticky='ns')
            
        # Build 16 Sliders
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
                
        # 3. Presets and Actions Frame
        presets_frame = tk.LabelFrame(
            self.right_frame,
            text="Presets & Quick Actions",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=8
        )
        presets_frame.pack(fill=tk.X, pady=(15, 10))
        
        btn_pos = create_button(
            presets_frame,
            text="📍 Load Optimal Position Gains",
            command=self.load_default_gains,
            bg_color=BG_CARD
        )
        btn_pos.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        btn_traj = create_button(
            presets_frame,
            text="📈 Load Trajectory Gains",
            command=self.load_trajectory_gains,
            bg_color=BG_CARD
        )
        btn_traj.pack(side=tk.LEFT, padx=5, fill=tk.X, expand=True)
        
        # 4. Live Statistics Cards Frame
        stats_frame = tk.LabelFrame(
            self.right_frame,
            text="Live Simulation Metrics",
            bg=BG_PANEL,
            fg=TEXT_MAIN,
            font=('Arial', 10, 'bold'),
            padx=10,
            pady=8
        )
        stats_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Stats layout inside card: 2 rows of 3 columns
        for i in range(3):
            stats_frame.columnconfigure(i, weight=1)
            
        # Cost card
        cost_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        cost_card.grid(row=0, column=0, padx=4, pady=4, sticky='nsew')
        tk.Label(cost_card, text="ISE Cost", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.cost_val_label = tk.Label(cost_card, text="0.00000", bg=BG_CARD, fg=ACCENT_PURPLE, font=('Arial', 11, 'bold'))
        self.cost_val_label.pack()
        
        # RMS X card
        rms_x_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_x_card.grid(row=0, column=1, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_x_card, text="RMS Error X", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_x_label = tk.Label(rms_x_card, text="0.00 cm", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_x_label.pack()
        
        # RMS Y card
        rms_y_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_y_card.grid(row=0, column=2, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_y_card, text="RMS Error Y", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_y_label = tk.Label(rms_y_card, text="0.00 cm", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_y_label.pack()
        
        # RMS Z card
        rms_z_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_z_card.grid(row=1, column=0, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_z_card, text="RMS Error Z", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_z_label = tk.Label(rms_z_card, text="0.00 cm", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_z_label.pack()
        
        # RMS Yaw card
        rms_yaw_card = tk.Frame(stats_frame, bg=BG_CARD, padx=5, pady=5)
        rms_yaw_card.grid(row=1, column=1, padx=4, pady=4, sticky='nsew')
        tk.Label(rms_yaw_card, text="RMS Error Yaw", bg=BG_CARD, fg=TEXT_MUTED, font=('Arial', 8)).pack()
        self.rms_yaw_label = tk.Label(rms_yaw_card, text="0.000°", bg=BG_CARD, fg=TEXT_MAIN, font=('Arial', 10, 'bold'))
        self.rms_yaw_label.pack()

        # Save Button inside stats card (row 1, col 2)
        save_btn_card = tk.Frame(stats_frame, bg=BG_PANEL)
        save_btn_card.grid(row=1, column=2, padx=4, pady=4, sticky='nsew')
        self.btn_save = create_button(
            save_btn_card,
            text="💾 Save Gains",
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
            pady=8
        )
        opt_frame.pack(fill=tk.X, pady=(0, 5))
        
        self.status_label = tk.Label(
            opt_frame,
            text="Status: Idle",
            bg=BG_PANEL,
            fg=TEXT_MUTED,
            font=('Arial', 10, 'italic'),
            anchor='w'
        )
        self.status_label.pack(fill=tk.X, pady=(0, 8))
        
        btn_box = tk.Frame(opt_frame, bg=BG_PANEL)
        btn_box.pack(fill=tk.X)
        
        self.opt_button = create_button(
            btn_box,
            text="🚀 Run Auto-Optimization",
            command=self.start_optimization,
            bg_color=ACCENT_GREEN,
            fg_color=BG_DARK
        )
        self.opt_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 5))
        
        self.stop_button = create_button(
            btn_box,
            text="🛑 Stop",
            command=self.stop_optimization,
            bg_color=BG_CARD
        )
        self.stop_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(5, 0))
        self.stop_button.config(state=tk.DISABLED)

    # ==============================================================================
    # Event Handlers & Dynamic Drawing
    # ==============================================================================
    def on_slider_change(self, val):
        if self.updating_gui:
            return
            
        # Debounce the plot updates to avoid lagging the slider drag
        if self.update_timer_id is not None:
            self.root.after_cancel(self.update_timer_id)
        self.update_timer_id = self.root.after(40, self.update_plot_immediate)

    def update_plot_immediate(self):
        self.update_timer_id = None
        
        # Get current parameters from sliders
        params = [slider.get() for slider in self.sliders]
        KP, KSP, KD, KSD = unpack(params)
        
        try:
            errors, eta_hist, eta_d_hist, t_hist = simulate(KP, KSP, KD, KSD, return_history=True)
        except Exception as e:
            print(f"Simulation error: {e}")
            return
            
        # Calculate RMS & Cost
        rms = np.sqrt(np.mean(errors ** 2, axis=0))
        current_cost = cost(params)
        
        # Update metrics labels
        self.cost_val_label.config(text=f"{current_cost:.5f}")
        self.rms_x_label.config(text=f"{rms[0]*100:.2f} cm")
        self.rms_y_label.config(text=f"{rms[1]*100:.2f} cm")
        self.rms_z_label.config(text=f"{rms[2]*100:.2f} cm")
        self.rms_yaw_label.config(text=f"{np.degrees(rms[3]):.3f}°")
        
        # Update plotted lines
        for i in range(4):
            sim_data = eta_hist[:, i]
            if i == 3:
                sim_data = np.degrees(sim_data)
            self.sim_lines[i].set_ydata(sim_data)
            
            # Autoscale limits dynamically if data runs off screen
            ax = self.axs_flat[i]
            ymin, ymax = ax.get_ylim()
            min_d, max_d = np.min(sim_data), np.max(sim_data)
            
            ref_data = eta_d_hist[:, i]
            if i == 3:
                ref_data = np.degrees(ref_data)
            min_ref, max_ref = np.min(ref_data), np.max(ref_data)
            
            # If the data exceeds the window or is way too compressed, resize
            if min_d < ymin or max_d > ymax or (ymax - ymin) > 4.0 * (max_d - min_d + 1e-2):
                margin_min = min(min_d, min_ref)
                margin_max = max(max_d, max_ref)
                span = margin_max - margin_min
                if span < 1e-3:
                    span = 1.0
                ax.set_ylim(margin_min - 0.15 * span, margin_max + 0.15 * span)
                
        # Trigger Matplotlib redraw
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
        self.status_label.config(text="Status: Loaded optimal position preset", fg=ACCENT_BLUE)

    def load_trajectory_gains(self):
        self.updating_gui = True
        for i, val in enumerate(TRAJECTORY_GAINS):
            self.sliders[i].set(val)
        self.updating_gui = False
        self.update_plot_immediate()
        self.status_label.config(text="Status: Loaded trajectory preset (clipped to bounds)", fg=ACCENT_BLUE)

    def save_gains_to_file(self):
        params = [slider.get() for slider in self.sliders]
        KP, KSP, KD, KSD = unpack(params)
        
        def fmt_mat(name, mat):
            d = np.diag(mat)
            return (f"    {name} = np.diag([{d[0]:.6f}, {d[1]:.6f}, "
                    f"{d[2]:.6f}, {d[3]:.6f}])")
                    
        lines = [
            "─" * 65,
            "OPTIMAL GAINS (to paste into extended_controller.py)",
            "─" * 65,
            fmt_mat("KP ", KP),
            fmt_mat("KSP", KSP),
            fmt_mat("KD ", KD),
            fmt_mat("KSD", KSD),
        ]
        
        out_path = "optimal_gains.txt"
        try:
            with open(out_path, "w") as f:
                f.write("# Customized gains for extended_controller.py\n")
                f.write(f"# Total ISE cost: {cost(params):.6f}\n\n")
                for l in lines[3:]:
                    f.write(l.strip() + "\n")
            messagebox.showinfo("Success", f"Gains successfully saved to: {os.path.abspath(out_path)}\n\nSaved output:\n" + "\n".join(lines[3:]))
        except Exception as e:
            messagebox.showerror("Error", f"Could not save gains to file:\n{e}")

    # ==============================================================================
    # Optimization Thread management
    # ==============================================================================
    def start_optimization(self):
        if self.opt_thread and self.opt_thread.is_alive():
            return
            
        self.opt_button.config(state=tk.DISABLED, bg=BG_CARD)
        self.stop_button.config(state=tk.NORMAL, bg=ACCENT_RED, fg=BG_DARK)
        self.status_label.config(text="Status: Optimizing (Initializing DE population)...", fg=ACCENT_GREEN)
        
        current_params = np.array([slider.get() for slider in self.sliders])
        self.opt_queue = queue.Queue()
        self.opt_thread = OptimizerWorker(self.opt_queue, current_params, BOUNDS)
        self.opt_thread.start()
        
        # Start checking queue for updates
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
                    # Update sliders silently
                    self.updating_gui = True
                    for i, val in enumerate(xk):
                        self.sliders[i].set(val)
                    self.updating_gui = False
                    
                    # Redraw plot
                    self.update_plot_immediate()
                    self.status_label.config(text=f"Status: Optimizing... (Current Best Cost: {current_cost:.5f})", fg=ACCENT_GREEN)
                    
                elif msg_type == 'done':
                    xk, final_cost = data
                    self.updating_gui = True
                    for i, val in enumerate(xk):
                        self.sliders[i].set(val)
                    self.updating_gui = False
                    
                    self.update_plot_immediate()
                    self.status_label.config(text=f"Status: Done! (Final Cost: {final_cost:.5f})", fg=ACCENT_GREEN)
                    self.cleanup_opt_thread()
                    messagebox.showinfo("Optimization Complete", f"Optimization finished successfully!\nFinal Cost: {final_cost:.6f}")
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
    
    # Configure minimum sizing
    root.minsize(1200, 750)
    
    app = GainsTuningGUI(root)
    
    # Clean matplotlib figures on close
    def on_closing():
        plt.close('all')
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
