#!/usr/bin/env python3
"""
System identification for Bebop 2 drone using shape matching and inverse boost filtering.
Reconstructs the unattenuated physical velocity using a lead (inverse boost) filter,
fits the model using Least Squares, and verifies that the simulated velocity's shape
matches the original velocity without attenuation.
"""

import math
import os
import json
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt

def fit_percentage(y_true, y_pred):
    denom = np.linalg.norm(y_true - np.mean(y_true))
    if denom == 0:
        return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y_true - y_pred) / denom)

def correlation_coef(a, b):
    return np.corrcoef(a, b)[0, 1]

def causal_low_pass_filter(signals, alpha):
    out = np.zeros_like(signals)
    out[0] = signals[0]
    for k in range(1, len(signals)):
        out[k] = alpha * out[k-1] + (1.0 - alpha) * signals[k]
    return out

def boost_filter(raw_signal, alpha, gamma):
    """
    Applies an unsharp mask/lead filter: boosts high-frequency peaks (reverses LPF attenuation).
    """
    lpf = causal_low_pass_filter(raw_signal, alpha)
    boosted = (1.0 + gamma) * raw_signal - gamma * lpf
    return boosted

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
    vx_real = data["vx_b"].squeeze()
    vy_real = data["vy_b"].squeeze()
    psi_real  = np.unwrap(data["psi"].squeeze())
    x_real = data["x_i"].squeeze()
    y_real = data["y_i"].squeeze()

    N = len(vx_real)
    TRIM = 10
    u_t = u_full[TRIM:, :]
    vx_t = vx_real[TRIM:]
    vy_t = vy_real[TRIM:]
    psi_t = psi_real[TRIM:]
    x_t = x_real[TRIM:]
    y_t = y_real[TRIM:]
    M = len(u_t)

    # ─── 1. Sweep Boost Filter Parameters to find the best shape alignment ─────
    # We want a boost filter that de-attenuates the dynamics.
    # We test alpha in [0.3, 0.7] and gamma in [0.5, 3.0].
    alphas = [0.3, 0.5, 0.7]
    gammas = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    delays = [0, 1, 2, 3, 4, 5, 6]

    best_corr_x = -1.0
    best_params_x = None # (alpha, gamma, delay, f1, f2)
    
    best_corr_y = -1.0
    best_params_y = None # (alpha, gamma, delay, f1, f2)

    for alpha in alphas:
        for gamma in gammas:
            # Boost raw velocity to reconstruct the "true physical" velocity
            vx_boost = boost_filter(vx_t, alpha, gamma)
            vy_boost = boost_filter(vy_t, alpha, gamma)
            
            for delay in delays:
                # Apply delay to input command
                u_x = np.zeros(M)
                u_y = np.zeros(M)
                if delay == 0:
                    u_x[:] = u_t[:, 0]
                    u_y[:] = u_t[:, 1]
                else:
                    u_x[delay:] = u_t[:-delay, 0]
                    u_y[delay:] = u_t[:-delay, 1]
                
                # Fit discrete-time model: v[k+1] = a * v[k] + b * u[k]
                # for VX
                y_x = vx_boost[1:]
                Phi_x = np.column_stack([vx_boost[:-1], u_x[:-1]])
                theta_x, *_ = np.linalg.lstsq(Phi_x, y_x, rcond=None)
                ax, bx = float(theta_x[0]), float(theta_x[1])
                
                # Convert to continuous-time parameters
                # v_dot = f1 * u - f2 * v
                # alpha = 1 - dt * f2 -> f2 = (1 - alpha) * hz
                # beta = dt * f1 -> f1 = beta * hz
                f1_x_cand = bx * hz
                f2_x_cand = (1.0 - ax) * hz
                
                # for VY
                y_y = vy_boost[1:]
                Phi_y = np.column_stack([vy_boost[:-1], u_y[:-1]])
                theta_y, *_ = np.linalg.lstsq(Phi_y, y_y, rcond=None)
                ay, by = float(theta_y[0]), float(theta_y[1])
                
                f1_y_cand = by * hz
                f2_y_cand = (1.0 - ay) * hz
                
                # Filter out unstable or non-physical candidates
                # Gain f1 must be positive and damping f2 must be positive and stable.
                if 0.1 <= f1_x_cand <= 2.0 and 0.1 <= f2_x_cand <= 2.0:
                    # Simulate velocity response
                    v_sim_x = np.zeros(M)
                    v_sim_x[0] = vx_t[0]
                    for k in range(M - 1):
                        v_sim_x[k+1] = v_sim_x[k] + dt * (f1_x_cand * u_x[k] - f2_x_cand * v_sim_x[k])
                    
                    corr_x = correlation_coef(v_sim_x, vx_t)
                    if corr_x > best_corr_x:
                        best_corr_x = corr_x
                        best_params_x = (alpha, gamma, delay, f1_x_cand, f2_x_cand)

                if 0.1 <= f1_y_cand <= 2.0 and 0.1 <= f2_y_cand <= 2.0:
                    v_sim_y = np.zeros(M)
                    v_sim_y[0] = vy_t[0]
                    for k in range(M - 1):
                        v_sim_y[k+1] = v_sim_y[k] + dt * (f1_y_cand * u_y[k] - f2_y_cand * v_sim_y[k])
                        
                    corr_y = correlation_coef(v_sim_y, vy_t)
                    if corr_y > best_corr_y:
                        best_corr_y = corr_y
                        best_params_y = (alpha, gamma, delay, f1_y_cand, f2_y_cand)

    print("="*75)
    print("SHAPE-MATCHING IDENTIFICATION RESULTS")
    print("="*75)
    
    alpha_x, gamma_x, delay_x, f1_x, f2_x = best_params_x
    print(f"X (Surge): Delay = {delay_x} | f1 = {f1_x:.6f} | f2 = {f2_x:.6f}")
    print(f"  Boost Filter: alpha = {alpha_x:.1f}, gamma = {gamma_x:.2f}")
    print(f"  Correlation to raw: {best_corr_x*100:.2f}%")
    print(f"  Steady-state Gain K_x: {f1_x/f2_x:.3f} m/s")
    
    alpha_y, gamma_y, delay_y, f1_y, f2_y = best_params_y
    print(f"Y (Sway):  Delay = {delay_y} | f1 = {f1_y:.6f} | f2 = {f2_y:.6f}")
    print(f"  Boost Filter: alpha = {alpha_y:.1f}, gamma = {gamma_y:.2f}")
    print(f"  Correlation to raw: {best_corr_y*100:.2f}%")
    print(f"  Steady-state Gain K_y: {f1_y/f2_y:.3f} m/s")
    print("="*75)

    # ─── 2. Write to JSON ─────────────────────────────────────────────────────
    json_path = "system_identification_results.json"
    f1_z, f2_z = 1.196718, 1.362219
    f1_yaw, f2_yaw = 1.348792, 1.409996
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                old_results = json.load(f)
            f1_z = old_results["identified_parameters"]["z"]["f1"]
            f2_z = old_results["identified_parameters"]["z"]["f2"]
            f1_yaw = old_results["identified_parameters"]["yaw"]["f1"]
            f2_yaw = old_results["identified_parameters"]["yaw"]["f2"]
        except Exception:
            pass

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
            "x": int(delay_x),
            "y": int(delay_y)
        },
        "boosting_filters": {
            "x": {
                "alpha": float(alpha_x),
                "gamma": float(gamma_x)
            },
            "y": {
                "alpha": float(alpha_y),
                "gamma": float(gamma_y)
            }
        }
    }
    
    with open(json_path, 'w') as f:
        json.dump(results_json, f, indent=4)
    print(f"Saved parameters to {json_path}")

    # ─── 3. Re-simulate and plot ──────────────────────────────────────────────
    u_x = np.zeros(M)
    u_y = np.zeros(M)
    if delay_x > 0:
        u_x[delay_x:] = u_t[:-delay_x, 0]
    else:
        u_x[:] = u_t[:, 0]
        
    if delay_y > 0:
        u_y[delay_y:] = u_t[:-delay_y, 1]
    else:
        u_y[:] = u_t[:, 1]

    v_sim_x = np.zeros(M)
    v_sim_x[0] = vx_t[0]
    v_sim_y = np.zeros(M)
    v_sim_y[0] = vy_t[0]
    
    for k in range(M - 1):
        v_sim_x[k+1] = v_sim_x[k] + dt * (f1_x * u_x[k] - f2_x * v_sim_x[k])
        v_sim_y[k+1] = v_sim_y[k] + dt * (f1_y * u_y[k] - f2_y * v_sim_y[k])

    # Integrate to get position
    pos_sim = np.zeros((M, 2))
    pos_sim[0] = [x_t[0], y_t[0]]
    for k in range(M - 1):
        psi_k = psi_t[k]
        pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (v_sim_x[k] * np.cos(psi_k) - v_sim_y[k] * np.sin(psi_k))
        pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (v_sim_x[k] * np.sin(psi_k) + v_sim_y[k] * np.cos(psi_k))

    t_vec = np.arange(M) * dt

    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    
    # Velocity X
    axes[0, 0].plot(t_vec, vx_t, label='Original (Attenuated)', color='dodgerblue', alpha=0.8)
    axes[0, 0].plot(t_vec, boost_filter(vx_t, alpha_x, gamma_x), ':', label='Target (De-attenuated)', color='orange')
    axes[0, 0].plot(t_vec, v_sim_x, '--', label='Simulated Model', color='crimson')
    axes[0, 0].set_title(f"X Velocity (Surge) | Corr = {best_corr_x*100:.1f}%")
    axes[0, 0].set_ylabel("Velocity [m/s]")
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Velocity Y
    axes[0, 1].plot(t_vec, vy_t, label='Original (Attenuated)', color='dodgerblue', alpha=0.8)
    axes[0, 1].plot(t_vec, boost_filter(vy_t, alpha_y, gamma_y), ':', label='Target (De-attenuated)', color='orange')
    axes[0, 1].plot(t_vec, v_sim_y, '--', label='Simulated Model', color='crimson')
    axes[0, 1].set_title(f"Y Velocity (Sway) | Corr = {best_corr_y*100:.1f}%")
    axes[0, 1].set_ylabel("Velocity [m/s]")
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Position X
    axes[1, 0].plot(t_vec, x_t, label='Telemetry', color='dodgerblue')
    axes[1, 0].plot(t_vec, pos_sim[:, 0], '--', label='Simulated', color='crimson')
    axes[1, 0].set_title("X Position")
    axes[1, 0].set_ylabel("Position [m]")
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Position Y
    axes[1, 1].plot(t_vec, y_t, label='Telemetry', color='dodgerblue')
    axes[1, 1].plot(t_vec, pos_sim[:, 1], '--', label='Simulated', color='crimson')
    axes[1, 1].set_title("Y Position")
    axes[1, 1].set_ylabel("Position [m]")
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    fig.savefig("shape_match_fit.png", dpi=150)
    plt.show()

if __name__ == "__main__":
    main()
