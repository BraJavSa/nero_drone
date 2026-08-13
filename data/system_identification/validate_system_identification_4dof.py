#!/usr/bin/env python3
import sys
import math
import os
import json
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.ticker import AutoMinorLocator
from matplotlib.backends.backend_pdf import PdfPages

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
    json_file = sys.argv[1] if len(sys.argv) > 1 else "system_identification_parameters_telemetry_4dof.json"
    if not os.path.exists(json_file):
        print(f"[ERROR] Identified parameters file '{json_file}' not found.")
        return

    with open(json_file, 'r') as f:
        params = json.load(f)

    f1_x = params["identified_parameters"]["x"]["f1"]
    f2_x = params["identified_parameters"]["x"]["f2"]
    f1_y = params["identified_parameters"]["y"]["f1"]
    f2_y = params["identified_parameters"]["y"]["f2"]
    f1_z = params["identified_parameters"]["z"]["f1"]
    f2_z = params["identified_parameters"]["z"]["f2"]
    f1_yaw = params["identified_parameters"]["yaw"]["f1"]
    f2_yaw = params["identified_parameters"]["yaw"]["f2"]

    d_x = params["delays"]["x"]
    d_y = params["delays"]["y"]
    d_z = params["delays"]["z"]
    d_yaw = params["delays"]["yaw"]

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
    print(f"Loaded {N} samples from '{mat_file}'  |  dt = {dt*1e3:.2f} ms  |  {hz:.1f} Hz")

    vx_b_full_derived = np.zeros(N)
    vy_b_full_derived = np.zeros(N)
    for k in range(1, N):
        dxi = (x_full[k] - x_full[k-1]) / dt
        dyi = (y_full[k] - y_full[k-1]) / dt
        psi_k = psi_full[k]
        vx_b_full_derived[k] = dxi * math.cos(psi_k) + dyi * math.sin(psi_k)
        vy_b_full_derived[k] = -dxi * math.sin(psi_k) + dyi * math.cos(psi_k)

    use_derived = "derived" in json_file.lower()
    if use_derived:
        print("Using derived velocities for validation comparison.")
        vx_b_to_use = vx_b_full_derived
        vy_b_to_use = vy_b_full_derived
    else:
        print("Using EKF velocities for validation comparison.")
        vx_b_to_use = vx_b_full
        vy_b_to_use = vy_b_full

    vz_b_full = np.zeros(N)
    omega_full = np.zeros(N)
    for k in range(1, N):
        vz_b_full[k]  = (z_full[k] - z_full[k-1]) / dt
        omega_full[k] = (psi_full[k] - psi_full[k-1]) / dt

    TRIM = 10
    u_t  = u_full[TRIM:, :]
    nu_real = np.column_stack([
        vx_b_to_use[TRIM:],
        vy_b_to_use[TRIM:],
        vz_b_full[TRIM:],
        omega_full[TRIM:]
    ])

    x_real = x_full[TRIM:]
    y_real = y_full[TRIM:]
    z_real = z_full[TRIM:]
    psi_real = psi_full[TRIM:]
    M_samples = u_t.shape[0]

    x0, y0, z0, psi0 = x_real[0], y_real[0], z_real[0], psi_real[0]

    u_delayed = u_t.copy()
    if d_x > 0:
        u_delayed[d_x:, 0] = u_t[:-d_x, 0]
    if d_y > 0:
        u_delayed[d_y:, 1] = u_t[:-d_y, 1]
    if d_z > 0:
        u_delayed[d_z:, 2] = u_t[:-d_z, 2]
    if d_yaw > 0:
        u_delayed[d_yaw:, 3] = u_t[:-d_yaw, 3]

    X_state = np.zeros((M_samples, 8))
    X_state[0] = [nu_real[0, 0], nu_real[0, 1], nu_real[0, 2], nu_real[0, 3], x0, y0, z0, psi0]

    def dynamics(state, u_val):
        vx, vy, vz, om, x, y, z, psi = state
        
        dvx = f1_x * u_val[0] - f2_x * vx
        dvy = f1_y * u_val[1] - f2_y * vy
        dvz = f1_z * u_val[2] - f2_z * vz
        dom = f1_yaw * u_val[3] - f2_yaw * om
        
        dx = vx * math.cos(psi) - vy * math.sin(psi)
        dy = vx * math.sin(psi) + vy * math.cos(psi)
        dz = vz
        dpsi = om
        
        return np.array([dvx, dvy, dvz, dom, dx, dy, dz, dpsi])

    for k in range(M_samples - 1):
        u_k = u_delayed[k]
        u_kp1 = u_delayed[k+1]
        u_mid = 0.5 * (u_k + u_kp1)
        
        state_k = X_state[k]
        
        k1 = dynamics(state_k, u_k)
        k2 = dynamics(state_k + 0.5 * dt * k1, u_mid)
        k3 = dynamics(state_k + 0.5 * dt * k2, u_mid)
        k4 = dynamics(state_k + dt * k3, u_kp1)
        
        X_state[k+1] = state_k + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    nu_sim = X_state[:, 0:4]
    pos_sim = X_state[:, 4:8]

    t = np.arange(M_samples) * dt

    apply_article_style()
    
    COLOR_MEASURED = "#000000"
    COLOR_MODEL    = "#1a56a0"
    COLOR_FILL     = "#d0e1f9"

    pdf_vel_path = "validation_velocities_report_4dof.pdf"
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
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel("Time (s)", labelpad=3)
            
    fig1.patch.set_facecolor("white")
    plt.tight_layout()
    
    with PdfPages(pdf_vel_path) as pdf:
        pdf.savefig(fig1, dpi=300, bbox_inches="tight")
    plt.close(fig1)
    print(f"Generated PDF: {pdf_vel_path}")

    pdf_pos_path = "validation_positions_report_4dof.pdf"
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
            ax.tick_params(labelbottom=False)
        else:
            ax.set_xlabel("Time (s)", labelpad=3)
            
    fig2.patch.set_facecolor("white")
    plt.tight_layout()
    
    with PdfPages(pdf_pos_path) as pdf:
        pdf.savefig(fig2, dpi=300, bbox_inches="tight")
    plt.close(fig2)
    print(f"Generated PDF: {pdf_pos_path}")

    metrics = {
        "velocities": {},
        "positions": {}
    }
    
    vel_keys = ["vx", "vy", "vz", "omega"]
    for i, key in enumerate(vel_keys):
        real_val = nu_real[:, i]
        sim_val = nu_sim[:, i]
        rmse = float(np.sqrt(np.mean((real_val - sim_val)**2)))
        mae = float(np.mean(np.abs(real_val - sim_val)))
        
        mean_real = np.mean(real_val)
        ss_res = np.sum((real_val - sim_val)**2)
        ss_tot = np.sum((real_val - mean_real)**2)
        r2 = float(1.0 - (ss_res / ss_tot)) if ss_tot > 0 else 0.0
        
        metrics["velocities"][key] = {
            "rmse": rmse,
            "mae": mae,
            "r2": r2
        }
        
    pos_keys = ["x", "y", "z", "yaw"]
    pos_reals = [x_real, y_real, z_real, psi_real]
    for i, key in enumerate(pos_keys):
        real_val = pos_reals[i]
        sim_val = pos_sim[:, i]
        rmse = float(np.sqrt(np.mean((real_val - sim_val)**2)))
        mae = float(np.mean(np.abs(real_val - sim_val)))
        
        mean_real = np.mean(real_val)
        ss_res = np.sum((real_val - sim_val)**2)
        ss_tot = np.sum((real_val - mean_real)**2)
        r2 = float(1.0 - (ss_res / ss_tot)) if ss_tot > 0 else 0.0
        
        metrics["positions"][key] = {
            "rmse": rmse,
            "mae": mae,
            "r2": r2
        }

    metrics_json_path = "system_identification_validation_metrics_4dof.json"
    with open(metrics_json_path, 'w') as f:
        json.dump(metrics, f, indent=4)
        
    print(f"Generated validation metrics JSON: {metrics_json_path}")
    
    print("\n" + "="*65)
    print("VALIDATION METRICS (RMSE, MAE & R^2)")
    print("="*65)
    print(f"{'State/Channel':<15} | {'RMSE':<12} | {'MAE':<12} | {'R^2':<12}")
    print("-" * 65)
    for cat in ["velocities", "positions"]:
        print(f"--- {cat.capitalize()} ---")
        for key, val in metrics[cat].items():
            print(f"  {key:<13} | {val['rmse']:.6f}   | {val['mae']:.6f}   | {val['r2']:.6f}")
    print("="*65)

if __name__ == "__main__":
    main()
