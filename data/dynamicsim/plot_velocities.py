#!/usr/bin/env python3
"""
Script to visualize body-frame velocities and corresponding control inputs
from the drone telemetry dataset.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat

# Configuration
mat_file = "manual_log_20260517_004407.mat"

try:
    data = loadmat(mat_file)
except FileNotFoundError:
    print(f"Error: No se encontró el archivo '{mat_file}' en el directorio actual.")
    print("Asegúrate de ejecutar el script en el directorio correcto.")
    exit(1)

hz = float(data["hz"].squeeze())
dt = 1.0 / hz

# Extract signals
vx = data["vx_b"].squeeze()
vy = data["vy_b"].squeeze()
vz = data["vz_b"].squeeze()
r  = data["vpsi"].squeeze()  # Yaw rate (vpsi)
u  = data["u"]               # Control inputs [ux, uy, uz, upsi]

ux   = u[:, 0]
uy   = u[:, 1]
uz   = u[:, 2]
upsi = u[:, 3]

N = len(vx)
t = np.arange(N) * dt

# Setup premium visual layout
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Liberation Sans']

fig, axs = plt.subplots(2, 2, figsize=(15, 10))
fig.suptitle(f"Velocidades en el Marco del Cuerpo (Body Frame) & Consignas\nArchivo: {mat_file} ({hz} Hz)", 
             fontsize=16, fontweight='bold', color='#1A1A1A', y=0.96)

# Plot configurations
configs = [
    {
        "ax": axs[0, 0],
        "vel": vx,
        "input": ux,
        "title": "Velocidad Longitudinal ($V_x$)",
        "color_vel": "#008080", # Teal
        "color_in": "#E06666",  # Light Red
        "label_vel": "Velocidad $V_x$ (m/s)",
        "label_in": "Entrada $U_x$ (pitch)"
    },
    {
        "ax": axs[0, 1],
        "vel": vy,
        "input": uy,
        "title": "Velocidad Lateral ($V_y$)",
        "color_vel": "#2B6CB0", # Slate Blue
        "color_in": "#ED8936",  # Orange
        "label_vel": "Velocidad $V_y$ (m/s)",
        "label_in": "Entrada $U_y$ (roll)"
    },
    {
        "ax": axs[1, 0],
        "vel": vz,
        "input": uz,
        "title": "Velocidad Vertical ($V_z$)",
        "color_vel": "#6B46C1", # Purple
        "color_in": "#38A169",  # Green
        "label_vel": "Velocidad $V_z$ (m/s)",
        "label_in": "Entrada $U_z$ (gaz)"
    },
    {
        "ax": axs[1, 1],
        "vel": r,
        "input": upsi,
        "title": "Velocidad Angular de Guiñada ($r$ / Yaw Rate)",
        "color_vel": "#D69E2E", # Dark Yellow/Gold
        "color_in": "#319795",  # Teal Blue
        "label_vel": "Velocidad $r$ (rad/s)",
        "label_in": "Entrada $U_{\psi}$ (yaw)"
    }
]

for cfg in configs:
    ax = cfg["ax"]
    
    # Grid styling
    ax.grid(True, linestyle=':', alpha=0.6, color='#CBD5E0')
    ax.set_facecolor('#F7FAFC')
    
    # 1. Plot velocity on left axis
    line_vel, = ax.plot(t, cfg["vel"], color=cfg["color_vel"], linewidth=2, label=cfg["label_vel"])
    ax.set_ylabel("Velocidad", color=cfg["color_vel"], fontweight='semibold')
    ax.tick_params(axis='y', labelcolor=cfg["color_vel"])
    
    # 2. Plot control input on right axis (twinx)
    ax_twin = ax.twinx()
    line_in, = ax_twin.plot(t, cfg["input"], color=cfg["color_in"], linewidth=1.5, linestyle='--', alpha=0.7, label=cfg["label_in"])
    ax_twin.set_ylabel("Entrada de Control (Consigna)", color=cfg["color_in"], fontweight='semibold')
    ax_twin.tick_params(axis='y', labelcolor=cfg["color_in"])
    ax_twin.set_ylim(-1.1, 1.1)
    
    # Subplot Title & Labels
    ax.set_title(cfg["title"], fontsize=13, fontweight='bold', color='#2D3748', pad=10)
    ax.set_xlabel("Tiempo (s)", fontsize=11, color='#4A5568')
    
    # Combined legend
    lines = [line_vel, line_in]
    labels = [l.get_label() for l in lines]
    ax.legend(lines, labels, loc='upper right', frameon=True, facecolor='white', framealpha=0.9, edgecolor='#E2E8F0')

plt.tight_layout(rect=[0, 0, 1, 0.93])
plt.show()
