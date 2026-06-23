#!/usr/bin/env python3
"""
BEST CAUSAL FILTER EVALUATOR (MOVING AVERAGE)
=============================================
Evaluates the best causal filter for Z and Yaw channels, performs system identification,
and saves the attenuation filter and identified parameters to system_identification_results.json.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.optimize import minimize
import json
import os
import sys

# ==========================================
# FILTER PARAMETERS
# ==========================================
MA_WINDOW_Z = 5
MA_WINDOW_YAW = 5

# ==========================================
# CAUSAL FILTER FUNCTION
# ==========================================

def causal_moving_average(signal, window_size):
    """
    Causal Moving Average using only current and past samples.
    """
    out = np.zeros_like(signal)
    for k in range(len(signal)):
        start_idx = max(0, k - window_size + 1)
        out[k] = np.mean(signal[start_idx : k + 1])
    return out

# ==========================================
# SYSTEM IDENTIFICATION FUNCTIONS
# ==========================================

def fit_pct(y, yhat):
    var_y = np.linalg.norm(y - np.mean(y))
    if var_y == 0:
        return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y - yhat) / var_y)

def rms(a, b):
    return np.sqrt(np.mean((a - b)**2))

def optimize_channel_two_stage(v, u, dt, pos_real=None, channel_name=""):
    """
    Identifies dynamic model parameters: v[k+1] = a * v[k] + b * u[k]
    without offset/bias c.
    """
    y = v[1:]
    Phi = np.column_stack([v[:-1], u[:-1]])
    theta, *_ = np.linalg.lstsq(Phi, y, rcond=None)
    best_a, best_b = float(theta[0]), float(theta[1])
    best_c = 0.0
    
    # Physical metrics
    tau = -dt / np.log(best_a) if best_a > 0 else 0.0
    K = best_b / (1.0 - best_a) if best_a != 1.0 else 0.0
    offset = 0.0
    
    # Simulate velocity response
    n = len(u)
    v_sim = np.zeros(n)
    v_sim[0] = v[0]
    for k in range(n - 1):
        v_sim[k+1] = best_a * v_sim[k] + best_b * u[k]
        
    final_vel_fit = fit_pct(v, v_sim)
    return 0, best_a, best_b, best_c, tau, K, offset, final_vel_fit

# ==========================================
# MAIN EXECUTION
# ==========================================

def main():
    # 1. Load telemetry data
    mat_file = "manual_log_20260518_172721.mat"
    try:
        data = loadmat(mat_file)
    except FileNotFoundError:
        print(f"[ERROR] Data file '{mat_file}' not found.")
        sys.exit(1)
        
    hz = float(data["hz"].squeeze())
    dt = 1.0 / hz
    
    u_raw = data["u"][10:]
    z_real = data["z_i"].squeeze()[10:]
    psi_real = np.unwrap(data["psi"].squeeze())[10:]
    t = np.arange(len(z_real)) * dt
    
    # 2. Raw derivatives (measured velocities)
    dz_raw = np.gradient(z_real, dt)
    dpsi_raw = np.gradient(psi_real, dt)

    # 3. Apply Causal Moving Average directly to the velocities
    vz_filt = causal_moving_average(dz_raw, MA_WINDOW_Z)
    r_filt = causal_moving_average(dpsi_raw, MA_WINDOW_YAW)
    
    # 4. Integrate Filtered Velocities to Reconstruct Positions
    z_filt = np.zeros_like(z_real)
    z_filt[0] = z_real[0]
    yaw_filt = np.zeros_like(psi_real)
    yaw_filt[0] = psi_real[0]
    
    for k in range(len(z_real) - 1):
        z_filt[k+1] = z_filt[k] + vz_filt[k] * dt
        yaw_filt[k+1] = yaw_filt[k] + r_filt[k] * dt
        
    u_vz = u_raw[:, 2]
    u_r = u_raw[:, 3]
    
    # 5. Run identification for Z and Yaw
    dz, az, bz, cz, tauz, Kz, Oz, fitz = optimize_channel_two_stage(
        vz_filt, u_vz, dt, pos_real=z_filt, channel_name="VZ"
    )
    dr, ar, br, cr, taur, Kr, Or, fitr = optimize_channel_two_stage(
        r_filt, u_r, dt, pos_real=yaw_filt, channel_name="YAW"
    )

    print("\n" + "="*55)
    print("BEST FILTER METRICS (CAUSAL MOVING AVERAGE):")
    print("="*55)
    print(f"Z Altitude (window={MA_WINDOW_Z}):  FIT = {fit_pct(z_real, z_filt):.2f}% | RMSE = {rms(z_real, z_filt):.4f} m")
    print(f"Yaw Angle (window={MA_WINDOW_YAW}):   FIT = {fit_pct(psi_real, yaw_filt):.2f}% | RMSE = {rms(psi_real, yaw_filt):.4f} rad")
    print("="*55 + "\n")
    
    print("="*55)
    print("IDENTIFIED DYNAMICS FOR Z AND YAW:")
    print("="*55)
    print(f"Z   | Delay: {dz} | tau = {tauz*1000:.1f} ms, K = {Kz:.4f} | Velocity FIT: {fitz:.2f}%")
    print(f"Yaw | Delay: {dr} | tau = {taur*1000:.1f} ms, K = {Kr:.4f} | Velocity FIT: {fitr:.2f}%")
    print("="*55 + "\n")

    # 6. Save results to JSON file
    json_path = "system_identification_results.json"
    results = {}
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                results = json.load(f)
        except Exception:
            pass
            
    if "attenuation_filters" not in results:
        results["attenuation_filters"] = {}
    if "identified_parameters" not in results:
        results["identified_parameters"] = {}
        
    # Write Z and Yaw attenuation filters
    results["attenuation_filters"]["z"] = {
        "type": "moving_average",
        "window_size": int(MA_WINDOW_Z)
    }
    results["attenuation_filters"]["yaw"] = {
        "type": "moving_average",
        "window_size": int(MA_WINDOW_YAW)
    }
    
    # Write Z and Yaw identified parameters
    results["identified_parameters"]["z"] = {
        "a": float(az),
        "b": float(bz),
        "c": float(cz),
        "delay": int(dz),
        "tau_ms": float(tauz * 1000),
        "K": float(Kz),
        "offset": float(Oz)
    }
    results["identified_parameters"]["yaw"] = {
        "a": float(ar),
        "b": float(br),
        "c": float(cr),
        "delay": int(dr),
        "tau_ms": float(taur * 1000),
        "K": float(Kr),
        "offset": float(Or)
    }
    
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=4)
    print(f"Successfully saved Z and Yaw filter parameters to {json_path}\n")

    # 7. Plotting
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    
    fig1, axs1 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig1.suptitle(f"Z POSITION & VZ VELOCITY - BEST FILTER (Moving Average, window={MA_WINDOW_Z})", fontsize=12, fontweight='bold')
    
    axs1[0].plot(t, z_real, color='lightgray', label='Measured Z (z_real)', linewidth=2.0)
    axs1[0].plot(t, z_filt, color='#2ca02c', label='Reconstructed Z (integrated from vz_filt)', linewidth=2.0)
    axs1[0].set_ylabel("Z Position (m)")
    axs1[0].legend()
    axs1[0].grid(True, alpha=0.5)
    
    axs1[1].plot(t, dz_raw, color='lightgray', label='Raw Derived VZ (with zeros)', linewidth=1.5)
    axs1[1].plot(t, vz_filt, color='#1f77b4', label='Filtered VZ (Causal Moving Average)', linewidth=2.0)
    axs1[1].set_ylabel("Vertical Velocity VZ (m/s)")
    axs1[1].set_xlabel("Time (s)")
    axs1[1].legend()
    axs1[1].grid(True, alpha=0.5)
    
    fig2, axs2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle(f"YAW POSITION & YAW RATE - BEST FILTER (Moving Average, window={MA_WINDOW_YAW})", fontsize=12, fontweight='bold')
    
    axs2[0].plot(t, psi_real, color='lightgray', label='Measured Yaw (psi_real)', linewidth=2.0)
    axs2[0].plot(t, yaw_filt, color='#2ca02c', label='Reconstructed Yaw (integrated from r_filt)', linewidth=2.0)
    axs2[0].set_ylabel("Yaw Angle (rad)")
    axs2[0].legend()
    axs2[0].grid(True, alpha=0.5)
    
    axs2[1].plot(t, dpsi_raw, color='lightgray', label='Raw Derived Yaw Rate (with zeros)', linewidth=1.5)
    axs2[1].plot(t, r_filt, color='#1f77b4', label='Filtered Yaw Rate (Causal Moving Average)', linewidth=2.0)
    axs2[1].set_ylabel("Yaw Rate (rad/s)")
    axs2[1].set_xlabel("Time (s)")
    axs2[1].legend()
    axs2[1].grid(True, alpha=0.5)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
