#!/usr/bin/env python3
"""
Validator script that loads and evaluates the matrices F1 and F2 directly from extended_controller.py.
"""

import math
import os
import sys
import json
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt

# Add the parent folder to path so we can import CascadeController
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))
from nero_control.extended_controller import CascadeController

def rms(a, b):
    return np.sqrt(np.mean((a - b)**2))

def r2_score(y_true, y_pred):
    ss_res = np.sum((y_true - y_pred)**2)
    ss_tot = np.sum((y_true - np.mean(y_true))**2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

def fit_percentage(y_true, y_pred):
    denom = np.linalg.norm(y_true - np.mean(y_true))
    if denom == 0:
        return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y_true - y_pred) / denom)

def main():
    # Load controller matrices
    F1_raw = CascadeController.F1
    F2_raw = CascadeController.F2

    # Load delays from json if available
    json_path = "system_identification_results.json"
    delay_x, delay_y = 2, 2
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                config = json.load(f)
            delay_x = config.get("delays", {}).get("x", 2)
            delay_y = config.get("delays", {}).get("y", 2)
        except Exception:
            pass

    # Load telemetry data
    mat_file = "manual_log_20260518_172721.mat"
    if not os.path.exists(mat_file):
        print(f"[ERROR] Telemetry file '{mat_file}' not found.")
        return

    data = loadmat(mat_file)
    hz = float(data["hz"].squeeze())
    dt = 1.0 / hz

    # Classify the matrices
    # If the diagonal of F1 is close to 1, they are discrete-time matrices.
    # We must convert the full 4x4 matrices:
    # F1_cont = F2_raw * hz
    # F2_cont = (I - F1_raw) * hz
    is_discrete = F1_raw[0, 0] > 0.5 and F2_raw[0, 0] < 0.2
    
    if is_discrete:
        print("="*60)
        print("DETECTED DISCRETE-TIME MATRICES IN CONTROLLER")
        print("Converting full 4x4 matrices to continuous-time equivalents...")
        print("="*60)
        F1_cont = F2_raw * hz
        F2_cont = (np.eye(4) - F1_raw) * hz
    else:
        print("="*60)
        print("DETECTED CONTINUOUS-TIME MATRICES IN CONTROLLER")
        print("="*60)
        F1_cont = F1_raw.copy()
        F2_cont = F2_raw.copy()

    print("F1 (Continuous equivalent):")
    print(F1_cont)
    print("F2 (Continuous equivalent):")
    print(F2_cont)
    print("="*60)

    u_full    = data["u"]
    vx_b_full = data["vx_b"].squeeze()
    vy_b_full = data["vy_b"].squeeze()
    psi_full  = np.unwrap(data["psi"].squeeze())
    z_full    = data["z_i"].squeeze()
    x_full    = data["x_i"].squeeze()
    y_full    = data["y_i"].squeeze()

    N = len(vx_b_full)

    # Filter Z and Yaw
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

    vz_b_full = np.zeros(N)
    omega_full = np.zeros(N)
    for k in range(1, N):
        vz_b_full[k]  = (z_filt[k] - z_filt[k-1]) / dt
        omega_full[k] = (psi_filt[k] - psi_filt[k-1]) / dt

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

    u_delayed = u_t.copy()
    if M_samples > delay_x and delay_x > 0:
        u_delayed[delay_x:, 0] = u_t[:-delay_x, 0]
    if M_samples > delay_y and delay_y > 0:
        u_delayed[delay_y:, 1] = u_t[:-delay_y, 1]

    # Simulación forward
    nu_sim = np.zeros((M_samples, 4))
    nu_sim[0] = nu_real[0]

    for k in range(M_samples - 1):
        u_k = u_delayed[k].copy()
        u_k[2] = u_t[k, 2]
        u_k[3] = u_t[k, 3]
        dnu_k = F1_cont @ u_k - F2_cont @ nu_sim[k]
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

    # Compute metrics
    rmse_vx = rms(nu_real[:, 0], nu_sim[:, 0])
    rmse_vy = rms(nu_real[:, 1], nu_sim[:, 1])
    rmse_vz = rms(nu_real[:, 2], nu_sim[:, 2])
    rmse_om = rms(nu_real[:, 3], nu_sim[:, 3])

    rmse_x = rms(x_real, pos_sim[:, 0])
    rmse_y = rms(y_real, pos_sim[:, 1])
    rmse_z = rms(z_real, pos_sim[:, 2])
    rmse_psi = rms(psi_real, pos_sim[:, 3])

    r2_x = r2_score(x_real, pos_sim[:, 0])
    r2_y = r2_score(y_real, pos_sim[:, 1])
    r2_z = r2_score(z_real, pos_sim[:, 2])
    r2_psi = r2_score(psi_real, pos_sim[:, 3])

    fit_x = fit_percentage(x_real, pos_sim[:, 0])
    fit_y = fit_percentage(y_real, pos_sim[:, 1])
    fit_z = fit_percentage(z_real, pos_sim[:, 2])
    fit_psi = fit_percentage(psi_real, pos_sim[:, 3])

    print("\n" + "="*60)
    print("CONTROLLER VALIDATION METRICS")
    print("="*60)
    print(f"vx (Surge):   RMSE = {rmse_vx:.4f} m/s")
    print(f"vy (Sway):    RMSE = {rmse_vy:.4f} m/s")
    print(f"vz (Heave):   RMSE = {rmse_vz:.4f} m/s")
    print(f"ω  (Yaw-rate):RMSE = {rmse_om:.4f} rad/s")
    print("-"*60)
    print(f"x position:   RMSE = {rmse_x:.4f} m   | R2 = {r2_x*100:.2f}% | FIT = {fit_x:.2f}%")
    print(f"y position:   RMSE = {rmse_y:.4f} m   | R2 = {r2_y*100:.2f}% | FIT = {fit_y:.2f}%")
    print(f"z position:   RMSE = {rmse_z:.4f} m   | R2 = {r2_z*100:.2f}% | FIT = {fit_z:.2f}%")
    print(f"ψ orientation:RMSE = {rmse_psi:.4f} rad | R2 = {r2_psi*100:.2f}% | FIT = {fit_psi:.2f}%")
    print("="*60)

    # Plot
    t = np.arange(M_samples) * dt
    fig1, axes1 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig1.suptitle("Controller Velocity Validation", fontsize=14)
    vel_names = [r"$v_{x,b}$ [m/s]", r"$v_{y,b}$ [m/s]", r"$v_{z,b}$ [m/s]", r"$\omega$ [rad/s]"]
    for i in range(4):
        axes1[i].plot(t, nu_real[:, i], label='Real Telemetry', color='dodgerblue')
        axes1[i].plot(t, nu_sim[:, i], '--', label='Controller Model', color='crimson')
        axes1[i].set_ylabel(vel_names[i])
        axes1[i].legend()
        axes1[i].grid(True, alpha=0.3)
    fig1.savefig("controller_validation_velocities.png", dpi=150)

    fig2, axes2 = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig2.suptitle("Controller Position Validation", fontsize=14)
    pos_names = ["x [m]", "y [m]", "z [m]", r"$\psi$ [rad]"]
    pos_reals = [x_real, y_real, z_real, psi_real]
    for i in range(4):
        axes2[i].plot(t, pos_reals[i], label='Real Telemetry', color='dodgerblue')
        axes2[i].plot(t, pos_sim[:, i], '--', label='Controller Model', color='crimson')
        axes2[i].set_ylabel(pos_names[i])
        axes2[i].legend()
        axes2[i].grid(True, alpha=0.3)
    fig2.savefig("controller_validation_positions.png", dpi=150)
    plt.show()

if __name__ == "__main__":
    main()
