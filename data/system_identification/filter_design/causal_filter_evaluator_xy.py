#!/usr/bin/env python3
"""
CAUSAL FILTER EVALUATOR FOR X & Y CHANNELS
===========================================
This script evaluates the causal filtering pipeline (Derive -> Filter Body Velocity -> Integrate Position)
specifically for the horizontal axes X and Y (Surge and Sway).

It compares:
  - 1st-Order Low Pass Filter (LPF)
  - Causal Moving Average (MA)
  - 2nd-Order Causal Butterworth (initialized to initial velocity)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.signal import butter, lfilter, lfilter_zi
import sys

# ==========================================
# FILTER PARAMETERS
# ==========================================
ALPHA_X = 0.50
ALPHA_Y = 0.50

MA_WINDOW_X = 5
MA_WINDOW_Y = 5

BUTTER_CUTOFF_X = 1.0  # Hz
BUTTER_CUTOFF_Y = 1.0  # Hz

# ==========================================
# CAUSAL FILTER FUNCTIONS
# ==========================================

def causal_lpf(signals, alpha):
    """Causal first-order Low-Pass Filter (LPF)."""
    out = np.zeros_like(signals)
    out[0] = signals[0]
    for k in range(1, len(signals)):
        out[k] = alpha * out[k-1] + (1.0 - alpha) * signals[k]
    return out

def causal_moving_average(signal, window_size):
    """Causal Moving Average using only current and past samples."""
    out = np.zeros_like(signal)
    for k in range(len(signal)):
        start_idx = max(0, k - window_size + 1)
        out[k] = np.mean(signal[start_idx : k + 1])
    return out

def causal_butterworth(signal, cutoff_hz, fs):
    """2nd-order Causal Butterworth filter with state initialization to avoid startup transient."""
    nyq = 0.5 * fs
    normal_cutoff = cutoff_hz / nyq
    b, a = butter(2, normal_cutoff, btype='low', analog=False)
    zi = lfilter_zi(b, a) * signal[0]
    out, _ = lfilter(b, a, signal, zi=zi)
    return out

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
    
    x_real = data["x_i"].squeeze()[10:]
    y_real = data["y_i"].squeeze()[10:]
    psi_real = np.unwrap(data["psi"].squeeze())[10:]
    t = np.arange(len(x_real)) * dt
    
    # 2. Derive raw velocities in world-frame
    dx_raw = np.gradient(x_real, dt)
    dy_raw = np.gradient(y_real, dt)
    
    # Rotate world-frame velocities to body-frame (Surge/Sway)
    vx_raw = np.cos(psi_real) * dx_raw + np.sin(psi_real) * dy_raw
    vy_raw = -np.sin(psi_real) * dx_raw + np.cos(psi_real) * dy_raw

    # 3. Filter body velocities causally
    # LPF
    vx_lpf = causal_lpf(vx_raw, ALPHA_X)
    vy_lpf = causal_lpf(vy_raw, ALPHA_Y)
    
    # Moving Average
    vx_ma = causal_moving_average(vx_raw, MA_WINDOW_X)
    vy_ma = causal_moving_average(vy_raw, MA_WINDOW_Y)
    
    # Butterworth
    vx_butter = causal_butterworth(vx_raw, BUTTER_CUTOFF_X, fs=hz)
    vy_butter = causal_butterworth(vy_raw, BUTTER_CUTOFF_Y, fs=hz)
    
    # 4. Rotate back to world-frame and Integrate to reconstruct X & Y positions
    x_lpf = np.zeros_like(x_real)
    y_lpf = np.zeros_like(y_real)
    x_lpf[0] = x_real[0]
    y_lpf[0] = y_real[0]
    
    x_ma = np.zeros_like(x_real)
    y_ma = np.zeros_like(y_real)
    x_ma[0] = x_real[0]
    y_ma[0] = y_real[0]
    
    x_butter = np.zeros_like(x_real)
    y_butter = np.zeros_like(y_real)
    x_butter[0] = x_real[0]
    y_butter[0] = y_real[0]
    
    for k in range(len(x_real) - 1):
        # Rotate LPF velocities back
        dx_lpf_k = np.cos(psi_real[k]) * vx_lpf[k] - np.sin(psi_real[k]) * vy_lpf[k]
        dy_lpf_k = np.sin(psi_real[k]) * vx_lpf[k] + np.cos(psi_real[k]) * vy_lpf[k]
        x_lpf[k+1] = x_lpf[k] + dx_lpf_k * dt
        y_lpf[k+1] = y_lpf[k] + dy_lpf_k * dt
        
        # Rotate MA velocities back
        dx_ma_k = np.cos(psi_real[k]) * vx_ma[k] - np.sin(psi_real[k]) * vy_ma[k]
        dy_ma_k = np.sin(psi_real[k]) * vx_ma[k] + np.cos(psi_real[k]) * vy_ma[k]
        x_ma[k+1] = x_ma[k] + dx_ma_k * dt
        y_ma[k+1] = y_ma[k] + dy_ma_k * dt
        
        # Rotate Butterworth velocities back
        dx_butter_k = np.cos(psi_real[k]) * vx_butter[k] - np.sin(psi_real[k]) * vy_butter[k]
        dy_butter_k = np.sin(psi_real[k]) * vx_butter[k] + np.cos(psi_real[k]) * vy_butter[k]
        x_butter[k+1] = x_butter[k] + dx_butter_k * dt
        y_butter[k+1] = y_butter[k] + dy_butter_k * dt
        
    # 5. Calculate and print metrics
    def fit_pct(y, yhat):
        var_y = np.linalg.norm(y - np.mean(y))
        if var_y == 0:
            return 0.0
        return 100.0 * (1.0 - np.linalg.norm(y - yhat) / var_y)

    def rms(a, b):
        return np.sqrt(np.mean((a - b)**2))

    print("\n" + "="*50)
    print("X & Y POSITION RECONSTRUCTION METRICS:")
    print("="*50)
    print(f"X position (Real vs Reconstructed):")
    print(f"  - 1st LPF:       FIT = {fit_pct(x_real, x_lpf):6.2f}% | RMSE = {rms(x_real, x_lpf):.4f} m")
    print(f"  - Moving Average: FIT = {fit_pct(x_real, x_ma):6.2f}% | RMSE = {rms(x_real, x_ma):.4f} m")
    print(f"  - Butterworth:   FIT = {fit_pct(x_real, x_butter):6.2f}% | RMSE = {rms(x_real, x_butter):.4f} m")
    print(f"\nY position (Real vs Reconstructed):")
    print(f"  - 1st LPF:       FIT = {fit_pct(y_real, y_lpf):6.2f}% | RMSE = {rms(y_real, y_lpf):.4f} m")
    print(f"  - Moving Average: FIT = {fit_pct(y_real, y_ma):6.2f}% | RMSE = {rms(y_real, y_ma):.4f} m")
    print(f"  - Butterworth:   FIT = {fit_pct(y_real, y_butter):6.2f}% | RMSE = {rms(y_real, y_butter):.4f} m")
    print("="*50 + "\n")

    # 6. Plotting
    plt.style.use('seaborn-v0_8-whitegrid' if 'seaborn-v0_8-whitegrid' in plt.style.available else 'default')
    
    # Plot 1: X and VX
    fig1, axs1 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig1.suptitle("X POSITION AND VX VELOCITY (FILTER APPLIED ON VELOCITY)", fontsize=12, fontweight='bold')
    
    axs1[0].plot(t, x_real, color='lightgray', label='Raw Measured X', linewidth=2.0)
    axs1[0].plot(t, x_lpf, color='#1f77b4', label=f'X integrated from LPF (alpha={ALPHA_X})', linewidth=1.5)
    axs1[0].plot(t, x_ma, color='#2ca02c', label=f'X integrated from MovAvg (window={MA_WINDOW_X})', linewidth=1.5)
    axs1[0].plot(t, x_butter, color='#d62728', label=f'X integrated from Butter ({BUTTER_CUTOFF_X}Hz)', linewidth=1.5)
    axs1[0].set_ylabel("X Position (m)")
    axs1[0].legend()
    axs1[0].grid(True, alpha=0.5)
    
    axs1[1].plot(t, vx_raw, color='lightgray', label='Raw Derived VX (no drone velocity)', linewidth=1.5)
    axs1[1].plot(t, vx_lpf, color='#1f77b4', label='Filtered VX (LPF)', linewidth=1.8)
    axs1[1].plot(t, vx_ma, color='#2ca02c', label='Filtered VX (MovAvg)', linewidth=1.8)
    axs1[1].plot(t, vx_butter, color='#d62728', label='Filtered VX (Butter)', linewidth=1.8)
    axs1[1].set_ylabel("Body Velocity VX (m/s)")
    axs1[1].set_xlabel("Time (s)")
    axs1[1].legend()
    axs1[1].grid(True, alpha=0.5)
    
    # Plot 2: Y and VY
    fig2, axs2 = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    fig2.suptitle("Y POSITION AND VY VELOCITY (FILTER APPLIED ON VELOCITY)", fontsize=12, fontweight='bold')
    
    axs2[0].plot(t, y_real, color='lightgray', label='Raw Measured Y', linewidth=2.0)
    axs2[0].plot(t, y_lpf, color='#1f77b4', label=f'Y integrated from LPF (alpha={ALPHA_Y})', linewidth=1.5)
    axs2[0].plot(t, y_ma, color='#2ca02c', label=f'Y integrated from MovAvg (window={MA_WINDOW_Y})', linewidth=1.5)
    axs2[0].plot(t, y_butter, color='#d62728', label=f'Y integrated from Butter ({BUTTER_CUTOFF_Y}Hz)', linewidth=1.5)
    axs2[0].set_ylabel("Y Position (m)")
    axs2[0].legend()
    axs2[0].grid(True, alpha=0.5)
    
    axs2[1].plot(t, vy_raw, color='lightgray', label='Raw Derived VY (no drone velocity)', linewidth=1.5)
    axs2[1].plot(t, vy_lpf, color='#1f77b4', label='Filtered VY (LPF)', linewidth=1.8)
    axs2[1].plot(t, vy_ma, color='#2ca02c', label='Filtered VY (MovAvg)', linewidth=1.8)
    axs2[1].plot(t, vy_butter, color='#d62728', label='Filtered VY (Butter)', linewidth=1.8)
    axs2[1].set_ylabel("Body Velocity VY (m/s)")
    axs2[1].set_xlabel("Time (s)")
    axs2[1].legend()
    axs2[1].grid(True, alpha=0.5)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
