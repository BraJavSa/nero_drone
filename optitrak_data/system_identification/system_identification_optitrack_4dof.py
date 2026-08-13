#!/usr/bin/env python3
import os
import glob
import json
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.backends.backend_pdf import PdfPages
from scipy.optimize import minimize

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

def inertial_to_body_vel(dx, dy, dz, phi, theta, psi):
    cp, sp = np.cos(psi), np.sin(psi)
    ct, st = np.cos(theta), np.sin(theta)
    cf, sf = np.cos(phi), np.sin(phi)

    vx_b = cp*ct*dx + sp*ct*dy - st*dz
    vy_b = (cp*st*sf - sp*cf)*dx + (sp*st*sf + cp*cf)*dy + ct*sf*dz
    vz_b = (cp*st*cf + sp*sf)*dx + (sp*st*cf - cp*sf)*dy + ct*cf*dz

    return vx_b, vy_b, vz_b

def load_dataset(filepath):
    TRIM_END_SEC = {
        "coupled_sine_ud_all_09_Aug_2026_11h36min_B10_15Hz_COMPLETO.csv": 30,
        "coupled_sine_ud_all_08_Aug_2026_17h15min_B10_15Hz_COMPLETO.csv": 30,
        "uncoupled_sine_udPsidot_08_Aug_2026_17h10min_B10_15Hz_COMPLETO.csv": 15,
    }
    DT_FILE = 1 / 15.0

    data = np.loadtxt(filepath, delimiter=',')
    if data.shape[0] < 20:
        return None
        
    x, y, z = data[:, 12], data[:, 13], data[:, 14]
    phi, theta, psi = data[:, 15], data[:, 16], data[:, 17]
    psi = np.unwrap(psi)

    dx, dy, dz = data[:, 18], data[:, 19], data[:, 20]
    dpsi = data[:, 23]
    
    vx_b, vy_b, vz_b = inertial_to_body_vel(dx, dy, dz, phi, theta, psi)
    u = data[:, 24:28]
    
    TRIM = 10
    if len(x) <= TRIM:
        return None

    fname = os.path.basename(filepath)
    max_sec = TRIM_END_SEC.get(fname, None)
    end_idx = (TRIM + int(max_sec / DT_FILE)) if max_sec is not None else None

    sl = slice(TRIM, end_idx)
    ds = {
        'x': x[sl], 'y': y[sl], 'z': z[sl], 'psi': psi[sl],
        'vx_b': vx_b[sl], 'vy_b': vy_b[sl], 'vz_b': vz_b[sl], 'dpsi': dpsi[sl],
        'u': u[sl],
        'filename': fname
    }
    return ds

def simulate_axis(f1, f2, delay_sec, u_channel, v_real, dt):
    M = len(u_channel)
    t = np.arange(M) * dt
    u_delayed = np.interp(t - delay_sec, t, u_channel)
    
    v_sim = np.zeros(M)
    v_sim[0] = v_real[0]
    for k in range(M - 1):
        v_sim[k+1] = v_sim[k] + dt * (f1 * u_delayed[k] - f2 * v_sim[k])
    return v_sim

def fit_axis_linear(datasets, axis_name, vel_key, u_idx, delay_sec, dt):
    A_list = []
    B_list = []
    
    for ds in datasets:
        u_ch = ds['u'][:, u_idx]
        v_real = ds[vel_key]
        M = len(u_ch)
        t = np.arange(M) * dt
        u_delayed = np.interp(t - delay_sec, t, u_ch)
        
        dv = (v_real[1:] - v_real[:-1]) / dt
        v_curr = v_real[:-1]
        u_curr = u_delayed[:-1]
        
        A_sub = np.column_stack([u_curr, -v_curr])
        A_list.append(A_sub)
        B_list.append(dv)
        
    A = np.vstack(A_list)
    B = np.concatenate(B_list)
    
    params, residuals, rank, s = np.linalg.lstsq(A, B, rcond=None)
    f1, f2 = params[0], params[1]
    
    f1 = max(0.01, f1)
    f2 = max(0.01, f2)
    return f1, f2

def evaluate_axis_delay(datasets, axis_name, vel_key, u_idx, delay_sec, dt):
    f1, f2 = fit_axis_linear(datasets, axis_name, vel_key, u_idx, delay_sec, dt)
    
    total_loss = 0.0
    for ds in datasets:
        u_ch = ds['u'][:, u_idx]
        v_real = ds[vel_key]
        v_sim = simulate_axis(f1, f2, delay_sec, u_ch, v_real, dt)
        
        rmse = np.sqrt(np.mean((v_real - v_sim)**2))
        std_real = np.std(v_real) if np.std(v_real) > 1e-5 else 1.0
        std_sim = np.std(v_sim)
        amp_penalty = abs(std_sim - std_real) / std_real
        
        total_loss += (rmse / std_real) + 1.5 * amp_penalty
        
    return total_loss, f1, f2

def optimize_axis(datasets, axis_name, vel_key, u_idx, dt):
    def obj(x):
        d_sec = x[0]
        loss, _, _ = evaluate_axis_delay(datasets, axis_name, vel_key, u_idx, d_sec, dt)
        return loss
        
    res = minimize(obj, [0.15], method='Nelder-Mead', bounds=[(0.0, 0.6)])
    best_delay = float(res.x[0])
    
    _, f1, f2 = evaluate_axis_delay(datasets, axis_name, vel_key, u_idx, best_delay, dt)
    return f1, f2, best_delay

def validate_and_plot(params, delays, val_datasets, dt, pdf_path, json_path):
    f1_x, f2_x = params['x']
    f1_y, f2_y = params['y']
    f1_z, f2_z = params['z']
    f1_yaw, f2_yaw = params['yaw']
    
    d_x = delays['x']
    d_y = delays['y']
    d_z = delays['z']
    d_yaw = delays['yaw']
    
    metrics_list = []
    apply_article_style()

    MARGIN = 0.05
    vel_keys = ['vx_b', 'vy_b', 'vz_b', 'dpsi']
    pos_keys = ['x', 'y', 'z', 'psi']
    vel_ylims, pos_ylims = [], []
    for ch in range(4):
        all_v = np.concatenate([ds[vel_keys[ch]] for ds in val_datasets])
        lo, hi = float(np.min(all_v)), float(np.max(all_v))
        pad = MARGIN * max(abs(hi - lo), 1e-6)
        vel_ylims.append((lo - pad, hi + pad))

        all_p = np.concatenate([ds[pos_keys[ch]] for ds in val_datasets])
        lo, hi = float(np.min(all_p)), float(np.max(all_p))
        pad = MARGIN * max(abs(hi - lo), 1e-6)
        if pos_keys[ch] == 'z':
            pos_ylims.append((0.0, hi + pad))
        else:
            pos_ylims.append((lo - pad, hi + pad))
    u_ylim = (-1.05, 1.05)

    with PdfPages(pdf_path) as pdf:
        for ds in val_datasets:
            M = len(ds['x'])
            u_t = ds['u']
            
            v_sim_x   = simulate_axis(f1_x,   f2_x,   d_x,   u_t[:, 0], ds['vx_b'], dt)
            v_sim_y   = simulate_axis(f1_y,   f2_y,   d_y,   u_t[:, 1], ds['vy_b'], dt)
            v_sim_z   = simulate_axis(f1_z,   f2_z,   d_z,   u_t[:, 2], ds['vz_b'], dt)
            v_sim_yaw = simulate_axis(f1_yaw, f2_yaw, d_yaw, u_t[:, 3], ds['dpsi'],  dt)
            
            psi_real = ds['psi']
            dx_dt   = dt * (v_sim_x[:-1] * np.cos(psi_real[:-1]) - v_sim_y[:-1] * np.sin(psi_real[:-1]))
            dy_dt   = dt * (v_sim_x[:-1] * np.sin(psi_real[:-1]) + v_sim_y[:-1] * np.cos(psi_real[:-1]))
            dz_dt   = 0.5 * dt * (v_sim_z[:-1]   + v_sim_z[1:])
            dyaw_dt = 0.5 * dt * (v_sim_yaw[:-1] + v_sim_yaw[1:])
            
            pos_sim_x   = ds['x'][0]   + np.concatenate(([0], np.cumsum(dx_dt)))
            pos_sim_y   = ds['y'][0]   + np.concatenate(([0], np.cumsum(dy_dt)))
            pos_sim_z   = ds['z'][0]   + np.concatenate(([0], np.cumsum(dz_dt)))
            pos_sim_yaw = ds['psi'][0] + np.concatenate(([0], np.cumsum(dyaw_dt)))

            def _ch(real, sim):
                err  = real - sim
                rmse = float(np.sqrt(np.mean(err**2)))
                mae  = float(np.mean(np.abs(err)))
                ss_r = float(np.sum(err**2))
                ss_t = float(np.sum((real - np.mean(real))**2))
                r2   = float(1.0 - ss_r / ss_t) if ss_t > 1e-12 else 0.0
                return {'rmse': rmse, 'mae': mae, 'r2': r2}

            mv = {
                'vx':  _ch(ds['vx_b'], v_sim_x),
                'vy':  _ch(ds['vy_b'], v_sim_y),
                'vz':  _ch(ds['vz_b'], v_sim_z),
                'yaw': _ch(ds['dpsi'], v_sim_yaw),
            }
            mp = {
                'x':   _ch(ds['x'],   pos_sim_x),
                'y':   _ch(ds['y'],   pos_sim_y),
                'z':   _ch(ds['z'],   pos_sim_z),
                'psi': _ch(ds['psi'], pos_sim_yaw),
            }

            metrics_list.append({
                'filename':   ds['filename'],
                'velocities': mv,
                'positions':  mp,
            })

            t = np.arange(M) * dt
            
            fig, axes = plt.subplots(4, 3, figsize=(18, 9.5), sharex='col')
            fig.suptitle(f"Validation — {ds['filename']}", fontsize=10)

            vel_keys_list = ['vx', 'vy', 'vz', 'yaw']
            pos_keys_list = ['x',  'y',  'z',  'psi']

            reals_v = [ds['vx_b'], ds['vy_b'], ds['vz_b'], ds['dpsi']]
            sims_v  = [v_sim_x, v_sim_y, v_sim_z, v_sim_yaw]
            ylabels_v = [r"Body $\nu_x$ [m/s]", r"Body $\nu_y$ [m/s]", r"Body $\nu_z$ [m/s]", r"Body $\nu_{\psi}$ [rad/s]"]
            
            reals_p = [ds['x'], ds['y'], ds['z'], ds['psi']]
            sims_p  = [pos_sim_x, pos_sim_y, pos_sim_z, pos_sim_yaw]
            ylabels_p = [r"Inertial $\eta_x$ [m]", r"Inertial $\eta_y$ [m]", r"Inertial $\eta_z$ [m]", r"Inertial $\eta_{\psi}$ [rad]"]
            
            u_channels = [u_t[:, 0], u_t[:, 1], u_t[:, 2], u_t[:, 3]]
            ylabels_u = [r"$u_x$ [cmd]", r"$u_y$ [cmd]", r"$u_z$ [cmd]", r"$u_{\psi}$ [cmd]"]
            u_colors = ["#b03a2e", "#1e8449", "#1a5276", "#7d3c98"]

            METRIC_STYLE = dict(
                fontsize=6.5,
                verticalalignment='bottom', horizontalalignment='left',
                bbox=dict(boxstyle='round,pad=0.25', facecolor='white',
                          edgecolor='#aaaaaa', alpha=0.85)
            )
            
            for i in range(4):
                ax_v = axes[i, 0]
                ax_v.plot(t, reals_v[i], color="#000000", label='Measured', zorder=4)
                ax_v.plot(t, sims_v[i],  '--', color="#1a56a0", label='Modeled', zorder=5)
                ax_v.fill_between(t, reals_v[i], sims_v[i], color="#d0e1f9", alpha=0.55, linewidth=0, zorder=2)
                ax_v.set_ylabel(ylabels_v[i], labelpad=6)
                ax_v.set_ylim(vel_ylims[i])
                ax_v.grid(True)
                ck = vel_keys_list[i]
                ax_v.text(0.02, 0.03,
                          f"RMSE={mv[ck]['rmse']:.4f}  MAE={mv[ck]['mae']:.4f}  R²={mv[ck]['r2']:.3f}",
                          transform=ax_v.transAxes, **METRIC_STYLE)
                if i == 0:
                    handles, labels_l = ax_v.get_legend_handles_labels()
                    err_patch = mpatches.Patch(facecolor="#d0e1f9", alpha=0.6, edgecolor="none")
                    ax_v.legend(handles + [err_patch], labels_l + ["Error Region"], loc="upper right")

                ax_p = axes[i, 1]
                ax_p.plot(t, reals_p[i], color="#000000", label='Measured', zorder=4)
                ax_p.plot(t, sims_p[i],  '--', color="#1a56a0", label='Modeled', zorder=5)
                ax_p.fill_between(t, reals_p[i], sims_p[i], color="#d0e1f9", alpha=0.55, linewidth=0, zorder=2)
                ax_p.set_ylabel(ylabels_p[i], labelpad=6)
                ax_p.set_ylim(pos_ylims[i])
                ax_p.grid(True)
                pk = pos_keys_list[i]
                ax_p.text(0.02, 0.03,
                          f"RMSE={mp[pk]['rmse']:.4f}  MAE={mp[pk]['mae']:.4f}  R²={mp[pk]['r2']:.3f}",
                          transform=ax_p.transAxes, **METRIC_STYLE)
                if i == 0:
                    handles, labels_l = ax_p.get_legend_handles_labels()
                    err_patch = mpatches.Patch(facecolor="#d0e1f9", alpha=0.6, edgecolor="none")
                    ax_p.legend(handles + [err_patch], labels_l + ["Error Region"], loc="upper right")

                ax_u = axes[i, 2]
                ax_u.plot(t, u_channels[i], color=u_colors[i], label='U', linewidth=1.0, zorder=4)
                ax_u.axhline(0, color='black', linewidth=0.6, linestyle='--', zorder=3)
                ax_u.set_ylabel(ylabels_u[i], labelpad=6)
                ax_u.set_ylim(u_ylim)
                ax_u.grid(True)
                if i == 0:
                    ax_u.legend(loc="upper right")

            axes[3, 0].set_xlabel("Time (s)")
            axes[3, 1].set_xlabel("Time (s)")
            axes[3, 2].set_xlabel("Time (s)")
            
            plt.tight_layout()
            pdf.savefig(fig)
            plt.close(fig)
            
    def _agg(key, cat):
        return float(np.mean([np.mean([m[cat][ch][key] for ch in m[cat]]) for m in metrics_list]))

    avg_metrics = {
        'vel_rmse': _agg('rmse', 'velocities'), 'vel_mae': _agg('mae', 'velocities'), 'vel_r2': _agg('r2', 'velocities'),
        'pos_rmse': _agg('rmse', 'positions'),  'pos_mae': _agg('mae', 'positions'),  'pos_r2': _agg('r2', 'positions'),
    }
    print(f"\n  Average over {len(metrics_list)} experiments:")
    print(f"  Velocities: RMSE={avg_metrics['vel_rmse']:.4f}  MAE={avg_metrics['vel_mae']:.4f}  R²={avg_metrics['vel_r2']:.3f}")
    print(f"  Positions:  RMSE={avg_metrics['pos_rmse']:.4f}  MAE={avg_metrics['pos_mae']:.4f}  R²={avg_metrics['pos_r2']:.3f}")
    
    results_json = {
        "identified_parameters": {
            "x":   {"f1": float(f1_x),   "f2": float(f2_x),   "delay_samples": int(round(d_x * 15.0))},
            "y":   {"f1": float(f1_y),   "f2": float(f2_y),   "delay_samples": int(round(d_y * 15.0))},
            "z":   {"f1": float(f1_z),   "f2": float(f2_z),   "delay_samples": int(round(d_z * 15.0))},
            "yaw": {"f1": float(f1_yaw), "f2": float(f2_yaw), "delay_samples": int(round(d_yaw * 15.0))}
        },
        "validation_metrics_average": avg_metrics,
        "validation_metrics_per_file": metrics_list
    }
    with open(json_path, 'w') as f:
        json.dump(results_json, f, indent=4)
    print(f"\nSaved validation plots to {pdf_path}")
    print(f"Saved validation metrics and parameters to {json_path}")

def main():
    np.random.seed(42)
    random.seed(42)
    
    base_dir = "/home/brayan/ros2_ws/src/neroControl/optitrak_data"
    d1 = os.path.join(base_dir, "identification_data_15Hz_01")
    d2 = os.path.join(base_dir, "identification_data_15Hz_02")
    
    coupled_files = sorted(glob.glob(os.path.join(d1, "coupled*.csv")) + glob.glob(os.path.join(d2, "coupled*.csv")))
    all_csv_files = sorted(glob.glob(os.path.join(d1, "*.csv")) + glob.glob(os.path.join(d2, "*.csv")))

    BLACKLIST = {
        "coupled_sine_joystick_09_Aug_2026_11h43min_B10_15Hz_COMPLETO.csv",
        "coupled_sine_joystick_08_Aug_2026_17h25min_B10_15Hz_COMPLETO.csv",
        "coupled_sine_joystick_08_Aug_2026_17h22min_B10_15Hz_COMPLETO.csv",
        "uncoupled_sine_udZdot_08_Aug_2026_17h00min_B10_15Hz_COMPLETO.csv",
        "uncoupled_sine_udZdot_08_Aug_2026_17h05min_B10_15Hz_COMPLETO.csv",
        "uncoupled_sine_udZdot_09_Aug_2026_11h25min_B10_15Hz_COMPLETO.csv",
        "uncoupled_sine_udZdot_09_Aug_2026_11h27min_B10_15Hz_COMPLETO.csv",
        "uncoupled_sine_udZdot_09_Aug_2026_11h29min_B10_15Hz_COMPLETO.csv",
        "uncoupled_sine_udPhi_08_Aug_2026_16h56min_B10_15Hz_COMPLETO.csv",
    }
    coupled_files = [f for f in coupled_files if os.path.basename(f) not in BLACKLIST]
    all_csv_files  = [f for f in all_csv_files  if os.path.basename(f) not in BLACKLIST]

    print(f"Found {len(all_csv_files)} total experiments (after blacklist).")
    print(f"Found {len(coupled_files)} coupled experiments (after blacklist).")
    
    train_datasets = []
    for f in coupled_files:
        ds = load_dataset(f)
        if ds is not None:
            train_datasets.append(ds)
            
    val_datasets = []
    for f in all_csv_files:
        ds = load_dataset(f)
        if ds is not None:
            val_datasets.append(ds)
            
    random.shuffle(train_datasets)
    n_train = int(0.8 * len(train_datasets))
    train_sub = train_datasets[:n_train]
    
    dt = 1.0 / 15.0
    
    print("\n" + "="*50)
    print("RUNNING LEAST SQUARES SYSTEM IDENTIFICATION")
    print("="*50)
    
    params = {}
    delays = {}
    
    axes_info = [
        ('x', 'vx_b', 0),
        ('y', 'vy_b', 1),
        ('z', 'vz_b', 2),
        ('yaw', 'dpsi', 3)
    ]
    
    for ax_name, vel_key, u_idx in axes_info:
        f1, f2, delay_sec = optimize_axis(train_sub, ax_name, vel_key, u_idx, dt)
        params[ax_name] = (f1, f2)
        delays[ax_name] = delay_sec
        print(f"  {ax_name.upper():3s} optimal: f1 = {f1:.6f}, f2 = {f2:.6f}, Delay = {delay_sec*1000:.1f} ms")
        
    print("="*50)
    print("\nRunning Validation and Generating Report...")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pdf_out = os.path.join(script_dir, "validation_report_optitrack_4dof.pdf")
    json_out = os.path.join(script_dir, "system_identification_parameters_optitrack_4dof.json")
    
    validate_and_plot(params, delays, val_datasets, dt, pdf_out, json_out)

if __name__ == "__main__":
    main()
