#!/usr/bin/env python3
"""
OPTIMIZE X & Y DYNAMICS (INVERSE BOOST FILTER)
==============================================
This script finds the optimal high-frequency boosting parameter (gamma) for X and Y velocities.
The objective is to minimize the RMS error between the free-run simulated positions and the 
target filtered positions (reconstructed via LPF alpha=0.50).
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
from scipy.optimize import minimize
import sys

# ==========================================
# HELPER FUNCTIONS
# ==========================================

def fit_pct(y, yhat):
    var_y = np.linalg.norm(y - np.mean(y))
    if var_y == 0:
        return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y - yhat) / var_y)

def rms(a, b):
    return np.sqrt(np.mean((a - b)**2))

def alpha_beta_filter(measurements, alpha, beta, dt):
    """
    Applies Alpha-Beta filtering to smooth measurements.
    """
    n = len(measurements)
    x_est = measurements[0]
    v_est = 0.0
    out = np.empty(n)
    for k in range(n):
        x_pred = x_est + v_est * dt
        residual = measurements[k] - x_pred
        x_est = x_pred + alpha * residual
        v_est = v_est + (beta / dt) * residual
        out[k] = x_est
    return out

def kinematics(state, v_body):
    psi = state[3]
    vx, vy, vz, r = v_body
    dx = np.cos(psi) * vx - np.sin(psi) * vy
    dy = np.sin(psi) * vx + np.cos(psi) * vy
    dz = vz
    dpsi = r
    return np.array([dx, dy, dz, dpsi])

def run_rk4(x0, v_body_seq, psi_real_seq, dt):
    n = len(v_body_seq)
    states = np.zeros((4, n))
    states[:, 0] = x0
    for k in range(n - 1):
        sk = states[:, k]
        v_k = v_body_seq[k]
        v_k1 = v_body_seq[k+1]
        v_m = 0.5 * (v_k + v_k1)
        psi0 = psi_real_seq[k]
        psi1 = psi_real_seq[k+1]
        psi_mid = 0.5 * (psi0 + psi1)
        sk_rot = sk.copy()
        sk_rot[3] = psi0
        
        def f(s, psi_val, vb):
            s_tmp = s.copy()
            s_tmp[3] = psi_val
            return kinematics(s_tmp, vb)
            
        k1 = f(sk_rot, psi0, v_k)
        k2 = f(sk_rot + 0.5 * dt * k1, psi_mid, v_m)
        k3 = f(sk_rot + 0.5 * dt * k2, psi_mid, v_m)
        k4 = f(sk_rot + dt * k3, psi1, v_k1)
        states[:, k+1] = sk + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return states

def optimize_channel_two_stage(v, u, dt, pos_real=None, other_v_real=None, psi_real=None, channel_name=""):
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
# DYNAMIC BOOST FILTER DEFINITION
# ==========================================

def causal_low_pass_filter(signals, alpha):
    out = np.zeros_like(signals)
    out[0] = signals[0]
    for k in range(1, len(signals)):
        out[k] = alpha * out[k-1] + (1.0 - alpha) * signals[k]
    return out

def boost_filter(raw_signal, alpha, gamma):
    """
    Applies an unsharp mask/lead filter: boosts high-frequency peaks.
    """
    lpf = causal_low_pass_filter(raw_signal, alpha)
    boosted = (1.0 + gamma) * raw_signal - gamma * lpf
    return boosted

# ==========================================
# MAIN EXECUTION
# ==========================================

def main():
    # 1. Load data
    mat_file = "manual_log_20260518_172721.mat"
    try:
        data = loadmat(mat_file)
    except FileNotFoundError:
        print(f"[ERROR] Data file '{mat_file}' not found.")
        sys.exit(1)
        
    hz = float(data["hz"].squeeze())
    dt = 1.0 / hz
    
    u_raw = data["u"][10:]
    psi_raw = np.unwrap(data["psi"].squeeze())[10:]
    x_raw = data["x_i"].squeeze()[10:]
    y_raw = data["y_i"].squeeze()[10:]
    t = np.arange(len(x_raw)) * dt
    
    # 2. Derive raw velocities
    dx_i = np.gradient(x_raw, dt)
    dy_i = np.gradient(y_raw, dt)
    vx_raw_deriv = np.cos(psi_raw) * dx_i + np.sin(psi_raw) * dy_i
    vy_raw_deriv = -np.sin(psi_raw) * dx_i + np.cos(psi_raw) * dy_i
    
    # 3. Ground Truth Filtered Positions (load configuration from system_identification_results.json)
    import json
    import os
    json_path = "system_identification_results.json"
    config = {}
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                config = json.load(f)
            print(f"Loaded reference filter configuration from {json_path}")
        except Exception:
            pass
            
    x_cfg = config.get("attenuation_filters", {}).get("x", {"type": "low_pass_filter", "alpha": 0.50})
    if x_cfg.get("type") == "alpha_beta":
        vx_ref = alpha_beta_filter(vx_raw_deriv, x_cfg["alpha"], x_cfg["beta"], dt)
        print(f"Applying Alpha-Beta filter to reference VX: alpha={x_cfg['alpha']:.3f}, beta={x_cfg['beta']:.3f}")
    else:
        vx_ref = causal_low_pass_filter(vx_raw_deriv, alpha=x_cfg.get("alpha", 0.50))
        print(f"Applying LPF to reference VX: alpha={x_cfg.get('alpha', 0.50):.3f}")
        
    y_cfg = config.get("attenuation_filters", {}).get("y", {"type": "low_pass_filter", "alpha": 0.50})
    if y_cfg.get("type") == "alpha_beta":
        vy_ref = alpha_beta_filter(vy_raw_deriv, y_cfg["alpha"], y_cfg["beta"], dt)
        print(f"Applying Alpha-Beta filter to reference VY: alpha={y_cfg['alpha']:.3f}, beta={y_cfg['beta']:.3f}")
    else:
        vy_ref = causal_low_pass_filter(vy_raw_deriv, alpha=y_cfg.get("alpha", 0.50))
        print(f"Applying LPF to reference VY: alpha={y_cfg.get('alpha', 0.50):.3f}")
    
    x_ref = np.zeros_like(x_raw)
    x_ref[0] = x_raw[0]
    y_ref = np.zeros_like(y_raw)
    y_ref[0] = y_raw[0]
    for k in range(len(x_raw) - 1):
        dx_k = np.cos(psi_raw[k]) * vx_ref[k] - np.sin(psi_raw[k]) * vy_ref[k]
        dy_k = np.sin(psi_raw[k]) * vx_ref[k] + np.cos(psi_raw[k]) * vy_ref[k]
        x_ref[k+1] = x_ref[k] + dx_k * dt
        y_ref[k+1] = y_ref[k] + dy_k * dt
        
    u_pitch = u_raw[:, 0]
    u_roll = u_raw[:, 1]
    
    # Sweep alpha and gamma to find the best booster
    alpha_sweep = [0.1, 0.3, 0.5, 0.7, 0.9]
    gamma_sweep = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0]
    
    best_rmse_x = 1e9
    best_gamma_x = 0.0
    best_alpha_x = 0.5
    best_model_x = None
    
    best_rmse_y = 1e9
    best_gamma_y = 0.0
    best_alpha_y = 0.5
    best_model_y = None
    
    print("="*70)
    print("SWEEPING ALPHA AND GAMMA FOR INVERSE BOOST FILTER:")
    print("="*70)
    
    for alpha in alpha_sweep:
        for gamma in gamma_sweep:
            # Boost velocity targets for identification
            vx_boost = boost_filter(vx_raw_deriv, alpha=alpha, gamma=gamma)
            vy_boost = boost_filter(vy_raw_deriv, alpha=alpha, gamma=gamma)
            
            # Optimize VX
            dx, ax, bx, cx, taux, Kx, Ox, fitx = optimize_channel_two_stage(
                vx_boost, u_pitch, dt, pos_real=x_ref, other_v_real=vy_ref, psi_real=psi_raw, channel_name="VX"
            )
            
            # Optimize VY
            dy, ay, by, cy, tauy, Ky, Oy, fity = optimize_channel_two_stage(
                vy_boost, u_roll, dt, pos_real=y_ref, other_v_real=vx_ref, psi_real=psi_raw, channel_name="VY"
            )
            
            # Simulate free-run model
            d_max = max(dx, dy)
            N_sim = len(x_raw) - d_max
            
            # Simulate X
            vsim_x = np.zeros(N_sim)
            vsim_x[0] = vx_ref[d_max]
            for k in range(N_sim - 1):
                vsim_x[k+1] = ax * vsim_x[k] + bx * u_pitch[d_max + k - dx] + cx
                
            # Simulate Y
            vsim_y = np.zeros(N_sim)
            vsim_y[0] = vy_ref[d_max]
            for k in range(N_sim - 1):
                vsim_y[k+1] = ay * vsim_y[k] + by * u_roll[d_max + k - dy] + cy
                
            # Integrate to get simulated position
            v_sim_body = np.column_stack([vsim_x, vsim_y, np.zeros(N_sim), np.zeros(N_sim)])
            psi_cut = psi_raw[d_max:]
            initial_state = np.array([x_ref[d_max], y_ref[d_max], 0.0, psi_raw[d_max]])
            pos_sim = run_rk4(initial_state, v_sim_body, psi_cut, dt)
            
            rmse_x = rms(x_ref[d_max:], pos_sim[0])
            rmse_y = rms(y_ref[d_max:], pos_sim[1])
            
            print(f"Alpha = {alpha:.1f}, Gamma = {gamma:.1f} | X RMSE = {rmse_x:.4f} m (FIT: {fit_pct(x_ref[d_max:], pos_sim[0]):.1f}%) | Y RMSE = {rmse_y:.4f} m (FIT: {fit_pct(y_ref[d_max:], pos_sim[1]):.1f}%)")
            
            if rmse_x < best_rmse_x:
                best_rmse_x = rmse_x
                best_gamma_x = gamma
                best_alpha_x = alpha
                best_model_x = (dx, ax, bx, cx, taux, Kx, Ox, fitx)
                
            if rmse_y < best_rmse_y:
                best_rmse_y = rmse_y
                best_gamma_y = gamma
                best_alpha_y = alpha
                best_model_y = (dy, ay, by, cy, tauy, Ky, Oy, fity)
                
    print("="*70)
    print("OPTIMAL INVERSE BOOST RESULTS:")
    print("="*70)
    dx, ax, bx, cx, taux, Kx, Ox, fitx = best_model_x
    dy, ay, by, cy, tauy, Ky, Oy, fity = best_model_y
    print(f"Optimal X: Alpha = {best_alpha_x:.1f}, Gamma = {best_gamma_x:.2f} | Best Position RMSE: {best_rmse_x:.4f} m")
    print(f"  Model: v[k+1] = {ax:.5f}*v[k] + {bx:.5f}*u[k] + {cx:.5f} | tau = {taux*1000:6.1f} ms, K = {Kx:7.4f}")
    print(f"Optimal Y: Alpha = {best_alpha_y:.1f}, Gamma = {best_gamma_y:.2f} | Best Position RMSE: {best_rmse_y:.4f} m")
    print(f"  Model: v[k+1] = {ay:.5f}*v[k] + {by:.5f}*u[k] + {cy:.5f} | tau = {tauy*1000:6.1f} ms, K = {Ky:7.4f}")
    print("="*70)

    # Save to JSON
    import json
    import os
    json_path = "system_identification_results.json"
    results = {}
    if os.path.exists(json_path):
        try:
            with open(json_path, 'r') as f:
                results = json.load(f)
        except Exception:
            pass
            
    if "boosting_filters" not in results:
        results["boosting_filters"] = {}
    if "identified_parameters" not in results:
        results["identified_parameters"] = {}
        
    results["boosting_filters"]["x"] = {
        "type": "high_pass_boost",
        "alpha": float(best_alpha_x),
        "gamma": float(best_gamma_x)
    }
    results["boosting_filters"]["y"] = {
        "type": "high_pass_boost",
        "alpha": float(best_alpha_y),
        "gamma": float(best_gamma_y)
    }
    
    results["identified_parameters"]["x"] = {
        "a": float(ax),
        "b": float(bx),
        "c": float(cx),
        "delay": int(dx),
        "tau_ms": float(taux * 1000),
        "K": float(Kx),
        "offset": float(Ox)
    }
    results["identified_parameters"]["y"] = {
        "a": float(ay),
        "b": float(by),
        "c": float(cy),
        "delay": int(dy),
        "tau_ms": float(tauy * 1000),
        "K": float(Ky),
        "offset": float(Oy)
    }
    
    with open(json_path, 'w') as f:
        json.dump(results, f, indent=4)
    print(f"Successfully saved X and Y boosting filters and identified parameters to {json_path}\n")

    # 4. Generate optimal simulations for plotting
    vx_boost_opt = boost_filter(vx_raw_deriv, alpha=best_alpha_x, gamma=best_gamma_x)
    vy_boost_opt = boost_filter(vy_raw_deriv, alpha=best_alpha_y, gamma=best_gamma_y)
    
    d_max = max(dx, dy)
    N_sim = len(x_raw) - d_max
    
    vsim_x_opt = np.zeros(N_sim)
    vsim_x_opt[0] = vx_ref[d_max]
    for k in range(N_sim - 1):
        vsim_x_opt[k+1] = ax * vsim_x_opt[k] + bx * u_pitch[d_max + k - dx] + cx
        
    vsim_y_opt = np.zeros(N_sim)
    vsim_y_opt[0] = vy_ref[d_max]
    for k in range(N_sim - 1):
        vsim_y_opt[k+1] = ay * vsim_y_opt[k] + by * u_roll[d_max + k - dy] + cy
        
    v_sim_body_opt = np.column_stack([vsim_x_opt, vsim_y_opt, np.zeros(N_sim), np.zeros(N_sim)])
    initial_state_opt = np.array([x_ref[d_max], y_ref[d_max], 0.0, psi_raw[d_max]])
    pos_sim_opt = run_rk4(initial_state_opt, v_sim_body_opt, psi_raw[d_max:], dt)
    
    # 5. Plotting X & Y optimized dynamics
    fig, axs = plt.subplots(2, 2, figsize=(14, 10))
    
    # X Position
    axs[0, 0].plot(t, x_ref, color='#2ca02c', linewidth=2.0, label='Real Filtered X (Target)')
    pos_sim_x_pad = np.zeros_like(t)
    pos_sim_x_pad[d_max:] = pos_sim_opt[0]
    pos_sim_x_pad[:d_max] = x_ref[:d_max]
    axs[0, 0].plot(t, pos_sim_x_pad, '--', color='#ff7f0e', linewidth=1.8, label=f'Simulated Model X (gamma={best_gamma_x})')
    axs[0, 0].set_title("X Position Comparison")
    axs[0, 0].set_ylabel("Position (m)")
    axs[0, 0].legend()
    axs[0, 0].grid(True, alpha=0.4)
    
    # VX Velocity
    axs[1, 0].plot(t, vx_ref, color='#1f77b4', linewidth=2.0, label='Real Filtered VX')
    axs[1, 0].plot(t, vx_boost_opt, ':', color='purple', alpha=0.7, label='Boosted VX for Ident.')
    vsim_x_pad = np.zeros_like(t)
    vsim_x_pad[d_max:] = vsim_x_opt
    vsim_x_pad[:d_max] = vx_ref[:d_max]
    axs[1, 0].plot(t, vsim_x_pad, '--', color='#d62728', linewidth=1.8, label='Simulated Model VX')
    axs[1, 0].set_title("VX Velocity Comparison")
    axs[1, 0].set_ylabel("Velocity (m/s)")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].legend()
    axs[1, 0].grid(True, alpha=0.4)
    
    # Y Position
    axs[0, 1].plot(t, y_ref, color='#2ca02c', linewidth=2.0, label='Real Filtered Y (Target)')
    pos_sim_y_pad = np.zeros_like(t)
    pos_sim_y_pad[d_max:] = pos_sim_opt[1]
    pos_sim_y_pad[:d_max] = y_ref[:d_max]
    axs[0, 1].plot(t, pos_sim_y_pad, '--', color='#ff7f0e', linewidth=1.8, label=f'Simulated Model Y (gamma={best_gamma_y})')
    axs[0, 1].set_title("Y Position Comparison")
    axs[0, 1].set_ylabel("Position (m)")
    axs[0, 1].legend()
    axs[0, 1].grid(True, alpha=0.4)
    
    # VY Velocity
    axs[1, 1].plot(t, vy_ref, color='#1f77b4', linewidth=2.0, label='Real Filtered VY')
    axs[1, 1].plot(t, vy_boost_opt, ':', color='purple', alpha=0.7, label='Boosted VY for Ident.')
    vsim_y_pad = np.zeros_like(t)
    vsim_y_pad[d_max:] = vsim_y_opt
    vsim_y_pad[:d_max] = vy_ref[:d_max]
    axs[1, 1].plot(t, vsim_y_pad, '--', color='#d62728', linewidth=1.8, label='Simulated Model VY')
    axs[1, 1].set_title("VY Velocity Comparison")
    axs[1, 1].set_ylabel("Velocity (m/s)")
    axs[1, 1].set_xlabel("Time (s)")
    axs[1, 1].legend()
    axs[1, 1].grid(True, alpha=0.4)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
