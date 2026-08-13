#!/usr/bin/env python3
import math
import os
import json
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import AutoMinorLocator
from scipy.optimize import differential_evolution, minimize

def apply_article_style():
    plt.rcParams.update({
        "figure.facecolor":    "white",
        "axes.facecolor":      "white",
        "axes.edgecolor":      "black",
        "axes.linewidth":      0.8,
        "xtick.direction":     "in",
        "ytick.direction":     "in",
        "xtick.major.size":    4.0,
        "ytick.major.size":    4.0,
        "xtick.minor.size":    2.5,
        "ytick.minor.size":    2.5,
        "xtick.major.width":   0.7,
        "ytick.major.width":   0.7,
        "xtick.color":         "black",
        "ytick.color":         "black",
        "xtick.labelsize":     8,
        "ytick.labelsize":     8,
        "axes.grid":           True,
        "grid.color":          "#d0d0d0",
        "grid.linewidth":      0.45,
        "grid.linestyle":      "--",
        "font.family":         "serif",
        "font.serif":          ["Times New Roman", "DejaVu Serif", "serif"],
        "mathtext.fontset":    "dejavuserif",
        "axes.labelsize":      9,
        "axes.labelcolor":     "black",
        "text.color":          "black",
        "legend.frameon":      True,
        "legend.framealpha":   1.0,
        "legend.edgecolor":    "black",
        "legend.facecolor":    "white",
        "legend.fontsize":     8,
        "legend.handlelength": 2.2,
        "lines.linewidth":     1.2,
    })

def main():
    mat_file = "manual_log_20260701_170333.mat"
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

    vz_b_full = np.zeros(N)
    omega_full = np.zeros(N)
    for k in range(1, N):
        vz_b_full[k]  = (z_full[k] - z_full[k-1]) / dt
        omega_full[k] = (psi_full[k] - psi_full[k-1]) / dt

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
    z_real = z_full[TRIM:]
    psi_real = psi_full[TRIM:]
    M_samples = u_t.shape[0]

    print("\nOptimizing X, Y, Z, and Yaw channels jointly with a SHARED delay...")
    x0, y0, z0, psi0 = x_real[0], y_real[0], z_real[0], psi_real[0]
    
    std_vx_real = np.std(nu_real[:, 0]) if np.std(nu_real[:, 0]) > 0 else 1.0
    std_vy_real = np.std(nu_real[:, 1]) if np.std(nu_real[:, 1]) > 0 else 1.0
    std_vz_real = np.std(nu_real[:, 2]) if np.std(nu_real[:, 2]) > 0 else 1.0
    std_vyaw_real = np.std(nu_real[:, 3]) if np.std(nu_real[:, 3]) > 0 else 1.0
    
    std_x = np.std(x_real) if np.std(x_real) > 0 else 1.0
    std_y = np.std(y_real) if np.std(y_real) > 0 else 1.0
    std_z = np.std(z_real) if np.std(z_real) > 0 else 1.0
    std_psi = np.std(psi_real) if np.std(psi_real) > 0 else 1.0

    bounds = [
        (0.1, 1.2),
        (0.1, 1.2),
        (0.1, 1.2),
        (0.1, 1.2),
        (0.1, 15.0),
        (0.1, 15.0),
        (0.1, 15.0),
        (0.1, 15.0),
        (0.0, 6.0)
    ]

    def joint_loss(params):
        f1_x, f2_x, f1_y, f2_y, \
        f1_z, f2_z, f1_yaw, f2_yaw, delay_val = params
        
        d = int(round(delay_val))
        
        u_x = np.zeros(M_samples)
        u_y = np.zeros(M_samples)
        u_z = np.zeros(M_samples)
        u_yaw = np.zeros(M_samples)
        
        if d == 0:
            u_x[:] = u_t[:, 0]
            u_y[:] = u_t[:, 1]
            u_z[:] = u_t[:, 2]
            u_yaw[:] = u_t[:, 3]
        else:
            u_x[d:] = u_t[:-d, 0]
            u_y[d:] = u_t[:-d, 1]
            u_z[d:] = u_t[:-d, 2]
            u_yaw[d:] = u_t[:-d, 3]
            
        v_sim_x = np.zeros(M_samples)
        v_sim_x[0] = nu_real[0, 0]
        v_sim_y = np.zeros(M_samples)
        v_sim_y[0] = nu_real[0, 1]
        v_sim_z = np.zeros(M_samples)
        v_sim_z[0] = nu_real[0, 2]
        v_sim_yaw = np.zeros(M_samples)
        v_sim_yaw[0] = nu_real[0, 3]
        
        for k in range(M_samples - 1):
            v_sim_x[k+1] = v_sim_x[k] + dt * (f1_x * u_x[k] - f2_x * v_sim_x[k])
            v_sim_y[k+1] = v_sim_y[k] + dt * (f1_y * u_y[k] - f2_y * v_sim_y[k])
            v_sim_z[k+1] = v_sim_z[k] + dt * (f1_z * u_z[k] - f2_z * v_sim_z[k])
            v_sim_yaw[k+1] = v_sim_yaw[k] + dt * (f1_yaw * u_yaw[k] - f2_yaw * v_sim_yaw[k])
            
        pos_sim = np.zeros((M_samples, 4))
        pos_sim[0] = [x0, y0, z0, psi0]
        for k in range(M_samples - 1):
            psi_k = psi_real[k]
            pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (v_sim_x[k] * np.cos(psi_k) - v_sim_y[k] * np.sin(psi_k))
            pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (v_sim_x[k] * np.sin(psi_k) + v_sim_y[k] * np.cos(psi_k))
            pos_sim[k+1, 2] = pos_sim[k, 2] + 0.5 * (v_sim_z[k] + v_sim_z[k+1]) * dt
            pos_sim[k+1, 3] = pos_sim[k, 3] + 0.5 * (v_sim_yaw[k] + v_sim_yaw[k+1]) * dt
            
        rmse_vx = np.sqrt(np.mean((nu_real[:, 0] - v_sim_x)**2))
        rmse_vy = np.sqrt(np.mean((nu_real[:, 1] - v_sim_y)**2))
        rmse_vz = np.sqrt(np.mean((nu_real[:, 2] - v_sim_z)**2))
        rmse_vyaw = np.sqrt(np.mean((nu_real[:, 3] - v_sim_yaw)**2))
        
        rmse_x = np.sqrt(np.mean((x_real - pos_sim[:, 0])**2))
        rmse_y = np.sqrt(np.mean((y_real - pos_sim[:, 1])**2))
        rmse_z = np.sqrt(np.mean((z_real - pos_sim[:, 2])**2))
        rmse_psi = np.sqrt(np.mean((psi_real - pos_sim[:, 3])**2))
        
        std_vx_sim = np.std(v_sim_x)
        std_vy_sim = np.std(v_sim_y)
        std_vz_sim = np.std(v_sim_z)
        std_vyaw_sim = np.std(v_sim_yaw)
        
        amp_penalty_x = abs(std_vx_sim - std_vx_real)
        amp_penalty_y = abs(std_vy_sim - std_vy_real)
        amp_penalty_z = abs(std_vz_sim - std_vz_real)
        amp_penalty_yaw = abs(std_vyaw_sim - std_vyaw_real)
        
        loss_x = (rmse_x / std_x) + (rmse_vx / std_vx_real) + 3.0 * (amp_penalty_x / std_vx_real)
        loss_y = (rmse_y / std_y) + (rmse_vy / std_vy_real) + 3.0 * (amp_penalty_y / std_vy_real)
        loss_z = (rmse_z / std_z) + (rmse_vz / std_vz_real) + 3.0 * (amp_penalty_z / std_vz_real)
        loss_yaw = (rmse_psi / std_psi) + (rmse_vyaw / std_vyaw_real) + 3.0 * (amp_penalty_yaw / std_vyaw_real)
        
        return loss_x + loss_y + loss_z + loss_yaw

    res = differential_evolution(joint_loss, bounds, maxiter=100, popsize=12, disp=True)
    res_polished = minimize(joint_loss, res.x, method='L-BFGS-B', bounds=bounds)
    
    f1_x, f2_x, f1_y, f2_y, \
    f1_z, f2_z, f1_yaw, f2_yaw, delay_val = res_polished.x
    
    d = int(round(delay_val))
    
    print("\n" + "="*50)
    print("4-DOF OPTIMAL SYSTEM IDENTIFICATION RESULTS (SHARED DELAY)")
    print("="*50)
    print(f"Shared Delay: {d:2d} samples ({d*dt*1e3:.1f} ms)")
    print(f"  X optimal: f1_x = {f1_x:.6f}, f2_x = {f2_x:.6f}")
    print(f"  Y optimal: f1_y = {f1_y:.6f}, f2_y = {f2_y:.6f}")
    print(f"  Z optimal: f1_z = {f1_z:.6f}, f2_z = {f2_z:.6f}")
    print(f"Yaw optimal: f1_yaw = {f1_yaw:.6f}, f2_yaw = {f2_yaw:.6f}")
    print("="*50)

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
            "x": int(d),
            "y": int(d),
            "z": int(d),
            "yaw": int(d)
        }
    }

    json_path = "system_identification_parameters_telemetry_4dof.json"
    with open(json_path, 'w') as f:
        json.dump(results_json, f, indent=4)
    print(f"Saved all parameters to {json_path}")

    u_delayed = u_t.copy()
    if d > 0:
        u_delayed[d:, 0] = u_t[:-d, 0]
        u_delayed[d:, 1] = u_t[:-d, 1]
        u_delayed[d:, 2] = u_t[:-d, 2]
        u_delayed[d:, 3] = u_t[:-d, 3]

    nu_sim = np.zeros((M_samples, 4))
    nu_sim[0] = nu_real[0]
    f1_vec = np.array([f1_x, f1_y, f1_z, f1_yaw])
    f2_vec = np.array([f2_x, f2_y, f2_z, f2_yaw])

    for k in range(M_samples - 1):
        dnu_k = f1_vec * u_delayed[k] - f2_vec * nu_sim[k]
        nu_sim[k+1] = nu_sim[k] + dt * dnu_k

    pos_sim = np.zeros((M_samples, 4))
    pos_sim[0] = [x0, y0, z0, psi0]
    for k in range(M_samples - 1):
        psi_k = pos_sim[k, 3]
        vxb, vyb, vzb, om = nu_sim[k]
        pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (vxb * math.cos(psi_k) - vyb * math.sin(psi_k))
        pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (vxb * math.sin(psi_k) + vyb * math.cos(psi_k))
        pos_sim[k+1, 2] = pos_sim[k, 2] + 0.5 * (vzb + nu_sim[k+1, 2]) * dt
        pos_sim[k+1, 3] = pos_sim[k, 3] + 0.5 * (om + nu_sim[k+1, 3]) * dt

    t = np.arange(M_samples) * dt

    apply_article_style()
    
    COLOR_MEASURED = "#000000"
    COLOR_MODEL    = "#1a56a0"
    COLOR_FILL     = "#d0e1f9"
    
    fig1, axes1 = plt.subplots(4, 1, figsize=(7.5, 9.5), sharex=True)
    vel_names = [
        r"$\nu_x$ [m/s]",
        r"$\nu_y$ [m/s]",
        r"$\nu_z$ [m/s]",
        r"$\nu_{\psi}$ [rad/s]"
    ]
    for i in range(4):
        ax = axes1[i]
        ax.plot(t, nu_real[:, i], color=COLOR_MEASURED, label='Measured', zorder=4)
        ax.plot(t, nu_sim[:, i], '--', color=COLOR_MODEL, label='Modeled', zorder=5)
        ax.fill_between(t, nu_real[:, i], nu_sim[:, i], color=COLOR_FILL, alpha=0.55, linewidth=0, zorder=2)
        
        ax.set_ylabel(vel_names[i], labelpad=6, multialignment="center")
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax.tick_params(which="both", top=True, right=True)
        ax.set_xlim(0, t[-1])
        ax.margins(x=0)
        ax.grid(True)
        
        err_patch = mpatches.Patch(facecolor=COLOR_FILL, alpha=0.6, edgecolor="none", label="Error Region")
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles + [err_patch], labels + ["Error Region"], loc="upper right")
        
        if i < 3:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("Time (s)", labelpad=3)
            
    fig1.patch.set_facecolor("white")
    plt.tight_layout()
    fig1.savefig("validation_velocities_telemetry_4dof.png", dpi=300, bbox_inches="tight")

    fig2, axes2 = plt.subplots(4, 1, figsize=(7.5, 9.5), sharex=True)
    pos_names = [
        r"$\eta_x$ [m]",
        r"$\eta_y$ [m]",
        r"$\eta_z$ [m]",
        r"$\eta_{\psi}$ [rad]"
    ]
    pos_reals = [x_real, y_real, z_real, psi_real]
    for i in range(4):
        ax = axes2[i]
        ax.plot(t, pos_reals[i], color=COLOR_MEASURED, label='Measured', zorder=4)
        ax.plot(t, pos_sim[:, i], '--', color=COLOR_MODEL, label='Modeled', zorder=5)
        ax.fill_between(t, pos_reals[i], pos_sim[:, i], color=COLOR_FILL, alpha=0.55, linewidth=0, zorder=2)
        
        ax.set_ylabel(pos_names[i], labelpad=6, multialignment="center")
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))
        ax.tick_params(which="both", top=True, right=True)
        ax.set_xlim(0, t[-1])
        ax.margins(x=0)
        ax.grid(True)
        
        err_patch = mpatches.Patch(facecolor=COLOR_FILL, alpha=0.6, edgecolor="none", label="Error Region")
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles + [err_patch], labels + ["Error Region"], loc="upper right")
        
        if i < 3:
            ax.set_xticklabels([])
        else:
            ax.set_xlabel("Time (s)", labelpad=3)
            
    fig2.patch.set_facecolor("white")
    plt.tight_layout()
    fig2.savefig("validation_positions_telemetry_4dof.png", dpi=300, bbox_inches="tight")

    print("Saved plots: validation_velocities_telemetry_4dof.png and validation_positions_telemetry_4dof.png")
    plt.show()

if __name__ == "__main__":
    main()
