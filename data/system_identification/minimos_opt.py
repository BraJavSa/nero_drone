#!/usr/bin/env python3
"""
System identification for Bebop 2 drone:
- Locks Z and Yaw parameters as requested.
- Applies physically realistic bounds for translation (X and Y) based on 5 deg max tilt.
- Optimizes using normalized joint position-velocity loss with amplitude matching.
"""

import math
import os
import json
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution, minimize

def main():
    # ─── 0. Load data ────────────────────────────────────────────────────────────
    mat_file = "manual_log_20260518_172721.mat"
    if not os.path.exists(mat_file):
        print(f"[ERROR] Telemetry file '{mat_file}' not found.")
        return

    data = loadmat(mat_file)
    hz = float(data["hz"].squeeze())
    dt = 1.0 / hz

    u_full    = data["u"]
    vx_b_full = data["vx_b"].squeeze()
    vy_b_full = data["vy_b"].squeeze()
    psi_full  = np.unwrap(data["psi"].squeeze())
    z_full    = data["z_i"].squeeze()
    x_full    = data["x_i"].squeeze()
    y_full    = data["y_i"].squeeze()

    N = len(vx_b_full)
    print(f"Loaded {N} samples  |  dt = {dt*1e3:.2f} ms  |  {hz:.1f} Hz")

    # ─── 1. Filter Z and Yaw positions (Causal Moving Average W=5) ────────────
    class CausalMovingAverage:
        def __init__(self, window_size: int):
            self.W      = window_size
            self.buffer = []

        def update(self, x: float) -> float:
            self.buffer.append(x)
            if len(self.buffer) > self.W:
                self.buffer.pop(0)
            return sum(self.buffer) / len(self.buffer)

    W = 5
    z_filt   = np.zeros(N)
    psi_filt = np.zeros(N)
    filt_z   = CausalMovingAverage(W)
    filt_psi = CausalMovingAverage(W)

    for k in range(N):
        z_filt[k]   = filt_z.update(float(z_full[k]))
        psi_filt[k] = filt_psi.update(float(psi_full[k]))

    # Derive Z and Yaw velocities
    vz_b_full = np.zeros(N)
    omega_full = np.zeros(N)
    for k in range(1, N):
        vz_b_full[k]  = (z_filt[k] - z_filt[k-1]) / dt
        omega_full[k] = (psi_filt[k] - psi_filt[k-1]) / dt

    # Slice transient/boundary samples
    TRIM = 10
    u_t  = u_full[TRIM:, :]
    nu_real = np.column_stack([
        vx_b_full[TRIM:],
        vy_b_full[TRIM:],
        vz_b_full[TRIM:],
        omega_full[TRIM:]
    ])

    x_real = x_full[TRIM:]
    y_real = y_full[TRIM:]
    z_real = z_filt[TRIM:]
    psi_real = psi_filt[TRIM:]
    M_samples = u_t.shape[0]

    # ─── 2. Lock Z and Yaw parameters ─────────────────────────────────────────
    f1_z, f2_z = 1.196718, 1.362219
    f1_yaw, f2_yaw = 1.348792, 1.409996
    print(f"Locked Z (Heave): f1 = {f1_z:.6f} | f2 = {f2_z:.6f}")
    print(f"Locked Yaw:       f1 = {f1_yaw:.6f} | f2 = {f2_yaw:.6f}")

    # ─── 3. Optimize X and Y jointly (Multi-objective Loss with Amplitude Match)
    print("\nOptimizing X/Y channels jointly...")
    x0, y0 = x_real[0], y_real[0]
    
    std_vx_real = np.std(nu_real[:, 0]) if np.std(nu_real[:, 0]) > 0 else 1.0
    std_vy_real = np.std(nu_real[:, 1]) if np.std(nu_real[:, 1]) > 0 else 1.0
    std_x = np.std(x_real) if np.std(x_real) > 0 else 1.0
    std_y = np.std(y_real) if np.std(y_real) > 0 else 1.0

    # Bounds for parameters: f1_x, f2_x, f1_y, f2_y, delay_x, delay_y
    # Max tilt is 5 degrees -> g * sin(5) = 0.855 m/s^2 acceleration at max command.
    # Therefore, f1 (gain) is bounded around [0.1, 1.2].
    # Damping f2 is bounded around [0.1, 1.2] based on standard Bebop drag coefficient.
    bounds_xy = [
        (0.1, 1.2),  # f1_x
        (0.1, 1.2),  # f2_x
        (0.1, 1.2),  # f1_y
        (0.1, 1.2),  # f2_y
        (0.0, 6.0),  # delay_x
        (0.0, 6.0)   # delay_y
    ]

    def joint_loss(params):
        f1_x, f2_x, f1_y, f2_y, delay_x_val, delay_y_val = params
        dx = int(round(delay_x_val))
        dy = int(round(delay_y_val))
        
        # Apply delays
        u_x = np.zeros(M_samples)
        u_y = np.zeros(M_samples)
        if dx == 0:
            u_x[:] = u_t[:, 0]
        else:
            u_x[dx:] = u_t[:-dx, 0]
            
        if dy == 0:
            u_y[:] = u_t[:, 1]
        else:
            u_y[dy:] = u_t[:-dy, 1]
            
        v_sim_x = np.zeros(M_samples)
        v_sim_x[0] = nu_real[0, 0]
        v_sim_y = np.zeros(M_samples)
        v_sim_y[0] = nu_real[0, 1]
        
        for k in range(M_samples - 1):
            v_sim_x[k+1] = v_sim_x[k] + dt * (f1_x * u_x[k] - f2_x * v_sim_x[k])
            v_sim_y[k+1] = v_sim_y[k] + dt * (f1_y * u_y[k] - f2_y * v_sim_y[k])
            
        pos_sim = np.zeros((M_samples, 2))
        pos_sim[0] = [x0, y0]
        for k in range(M_samples - 1):
            psi_k = psi_real[k]
            pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (v_sim_x[k] * np.cos(psi_k) - v_sim_y[k] * np.sin(psi_k))
            pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (v_sim_x[k] * np.sin(psi_k) + v_sim_y[k] * np.cos(psi_k))
            
        rmse_vx = np.sqrt(np.mean((nu_real[:, 0] - v_sim_x)**2))
        rmse_vy = np.sqrt(np.mean((nu_real[:, 1] - v_sim_y)**2))
        rmse_x = np.sqrt(np.mean((x_real - pos_sim[:, 0])**2))
        rmse_y = np.sqrt(np.mean((y_real - pos_sim[:, 1])**2))
        
        # Standard deviation matching
        std_vx_sim = np.std(v_sim_x)
        std_vy_sim = np.std(v_sim_y)
        
        amp_penalty_x = abs(std_vx_sim - std_vx_real)
        amp_penalty_y = abs(std_vy_sim - std_vy_real)
        
        # Loss: position tracking + velocity tracking + amplitude matching
        loss_x = (rmse_x / std_x) + (rmse_vx / std_vx_real) + 3.0 * (amp_penalty_x / std_vx_real)
        loss_y = (rmse_y / std_y) + (rmse_vy / std_vy_real) + 3.0 * (amp_penalty_y / std_vy_real)
        
        return loss_x + loss_y

    res_xy = differential_evolution(joint_loss, bounds_xy, maxiter=30, popsize=12, disp=True)
    res_xy_polished = minimize(joint_loss, res_xy.x, method='L-BFGS-B', bounds=bounds_xy)
    
    f1_x, f2_x, f1_y, f2_y, delay_x_val, delay_y_val = res_xy_polished.x
    dx = int(round(delay_x_val))
    dy = int(round(delay_y_val))
    print(f"  X optimal: Delay = {dx} | f1_x = {f1_x:.6f}, f2_x = {f2_x:.6f}")
    print(f"  Y optimal: Delay = {dy} | f1_y = {f1_y:.6f}, f2_y = {f2_y:.6f}")

    # ─── 4. Save parameters to JSON ───────────────────────────────────────────
    results_json = {
        "identified_parameters": {
            "x": {
                "f1": float(f1_x),
                "f2": float(f2_x)
            },
            "y": {
                "f1": float(f1_y),
                "f2": float(f2_y)
            },
            "z": {
                "f1": float(f1_z),
                "f2": float(f2_z)
            },
            "yaw": {
                "f1": float(f1_yaw),
                "f2": float(f2_yaw)
            }
        },
        "delays": {
            "x": int(dx),
            "y": int(dy)
        }
    }

    json_path = "system_identification_results.json"
    with open(json_path, 'w') as f:
        json.dump(results_json, f, indent=4)
    print(f"\nSaved all parameters to {json_path}")

    # ─── 5. Simulación forward conjunta para graficación ──────────────────────
    u_delayed = u_t.copy()
    if dx > 0:
        u_delayed[dx:, 0] = u_t[:-dx, 0]
    if dy > 0:
        u_delayed[dy:, 1] = u_t[:-dy, 1]

    nu_sim = np.zeros((M_samples, 4))
    nu_sim[0] = nu_real[0]
    f1_vec = np.array([f1_x, f1_y, f1_z, f1_yaw])
    f2_vec = np.array([f2_x, f2_y, f2_z, f2_yaw])

    for k in range(M_samples - 1):
        u_k = u_delayed[k].copy()
        u_k[2] = u_t[k, 2]
        u_k[3] = u_t[k, 3]
        dnu_k = f1_vec * u_k - f2_vec * nu_sim[k]
        nu_sim[k+1] = nu_sim[k] + dt * dnu_k

    pos_sim = np.zeros((M_samples, 4))
    pos_sim[0] = [x_real[0], y_real[0], z_real[0], psi_real[0]]
    for k in range(M_samples - 1):
        psi_k = pos_sim[k, 3]
        vxb, vyb, vzb, om = nu_sim[k]
        pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (vxb * math.cos(psi_k) - vyb * math.sin(psi_k))
        pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (vxb * math.sin(psi_k) + vyb * math.cos(psi_k))
        pos_sim[k+1, 2] = pos_sim[k, 2] + 0.5 * (vzb + nu_sim[k+1, 2]) * dt
        pos_sim[k+1, 3] = pos_sim[k, 3] + 0.5 * (om + nu_sim[k+1, 3]) * dt

    t = np.arange(M_samples) * dt

    # Plot results
    fig1, axes1 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig1.suptitle("Identified Model Velocities (Forward Simulation)", fontsize=14)
    vel_names = [r"$v_{x,b}$ [m/s]", r"$v_{y,b}$ [m/s]", r"$v_{z,b}$ [m/s]", r"$\omega$ [rad/s]"]
    for i in range(4):
        axes1[i].plot(t, nu_real[:, i], label='Real', color='dodgerblue')
        axes1[i].plot(t, nu_sim[:, i], '--', label='Model', color='crimson')
        axes1[i].set_ylabel(vel_names[i])
        axes1[i].legend()
        axes1[i].grid(True, alpha=0.3)
    plt.tight_layout()
    fig1.savefig("optimized_velocities.png", dpi=150)

    fig2, axes2 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig2.suptitle("Identified Model Positions (Forward Integration)", fontsize=14)
    pos_names = ["x [m]", "y [m]", "z [m]", r"$\psi$ [rad]"]
    pos_reals = [x_real, y_real, z_real, psi_real]
    for i in range(4):
        axes2[i].plot(t, pos_reals[i], label='Real', color='dodgerblue')
        axes2[i].plot(t, pos_sim[:, i], '--', label='Model', color='crimson')
        axes2[i].set_ylabel(pos_names[i])
        axes2[i].legend()
        axes2[i].grid(True, alpha=0.3)
    plt.tight_layout()
    fig2.savefig("optimized_positions.png", dpi=150)

    print("Saved plots: optimized_velocities.png and optimized_positions.png")
    plt.show()

if __name__ == "__main__":
    main()
