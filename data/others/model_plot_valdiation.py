#!/usr/bin/env python3
"""
Validación paso a paso del modelo Bebop.

En cada paso k:
  - Condición inicial = medición REAL de vel[k] y pos[k]
  - Se aplica UN paso de Euler (mismo orden que el simulador)
  - Se compara vel[k+1] y pos[k+1] predichos vs medidos

Esquema del simulador (timer_cb):
    xddot      = F(psi_k) @ Ku @ u_k  -  Kv @ xdot_k
    x_{k+1}    = x_k    + xdot_k * dt    ← posición con velocidad VIEJA
    xdot_{k+1} = xdot_k + xddot  * dt    ← velocidad después

Uso:
    python3 validate_stepbystep.py [ruta.pkl]
"""

import sys
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec

Ku = np.diag([0.8417,  0.8354,  3.9660,  9.8524])
Kv = np.diag([0.18227, 0.17095, 4.00100, 4.72950])

PHASE_DUR  = 4.0
N_PHASES   = 5
TOTAL_DUR  = PHASE_DUR * N_PHASES

VEL_LABELS = [r"$\nu_x$ [m/s]", r"$\nu_y$ [m/s]", r"$\nu_z$ [m/s]", r"$\omega$ [rad/s]"]
POS_LABELS = [r"$x$ [m]",       r"$y$ [m]",        r"$z$ [m]",        r"$\psi$ [rad]"]
U_LABELS   = [r"$u_{v_x}$",     r"$u_{v_y}$",      r"$u_{v_z}$",      r"$u_{\dot\psi}$"]

PHASE_COLORS = ["#EEEDFE", "#E1F5EE", "#FAECE7", "#FAEEDA", "#F1EFE8"]
PHASE_NAMES  = ["lin_x solo", "lin_y solo", "lin_z solo", "ang_z solo", "todos"]


def rotation_matrix(psi: float) -> np.ndarray:
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0, 0],
                     [s,  c, 0, 0],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]])
                     

def rmse(a, b): return float(np.sqrt(np.mean((a - b) ** 2)))
def r2(a, b):
    ss_res = np.sum((a - b) ** 2)
    ss_tot = np.sum((a - np.mean(a)) ** 2)
    return float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan")


def load(pkl_path: str):
    with open(pkl_path, "rb") as f:
        data = pickle.load(f)
    t   = np.array([d["t"]   for d in data])
    u   = np.array([d["u"]   for d in data])
    pos = np.array([d["pos"] for d in data])
    vel = np.array([d["vel"] for d in data])
    return t, u, pos, vel


def simulate_stepbystep(t, u, pos, vel):
    N = len(t)
    vel_pred = np.full_like(vel, np.nan)
    pos_pred = np.full_like(pos, np.nan)
    a=0.2
    for k in range(N - 1):
        dt     = t[k + 1] - t[k]
        xdot_k = vel[k].copy()
        x_k    = pos[k].copy()
        u_k    = u[k].copy()
        psi_k  = x_k[3]

        F     = rotation_matrix(psi_k)
        xddot = F @ (Ku @ u_k) - Kv @ xdot_k
        
        xddot[0] = np.clip(xddot[0], -a, a)
        xddot[1] = np.clip(xddot[1], -a, a)
        xddot[2] = np.clip(xddot[2], -a, a)

        vel_next = xdot_k + xddot * dt
        vel_next[:2] = np.clip(vel_next[:2], -5.0, 5.0)
        vel_next[2]  = np.clip(vel_next[2],   -5.0,  5.0)
        vel_next[3]  = np.clip(vel_next[3],   -7.0,  7.0)
        # ───────────────────────────────────────────────────────────

        pos_pred[k + 1] = x_k    + xdot_k * dt
        vel_pred[k + 1] = vel_next

    return vel_pred, pos_pred


def _add_phase_bg(ax):
    for p in range(N_PHASES):
        ax.axvspan(p * PHASE_DUR, (p + 1) * PHASE_DUR,
                   color=PHASE_COLORS[p], alpha=0.55, zorder=0)


def plot_and_metrics(t, u, pos, vel, vel_pred, pos_pred, out_base):
    vel[:, [0, 1]]      = vel[:, [1, 0]]
    vel_pred[:, [0, 1]] = vel_pred[:, [1, 0]]
    pos[:, [0, 1]]      = pos[:, [1, 0]]
    pos_pred[:, [0, 1]] = pos_pred[:, [1, 0]]
    
    patches = [mpatches.Patch(facecolor=PHASE_COLORS[p], label=PHASE_NAMES[p])
               for p in range(N_PHASES)]

    valid = ~np.isnan(vel_pred[:, 0])

    for (measured, predicted), labels, title_str, fname_tag in [
        ((vel, vel_pred), VEL_LABELS,
         "Velocity — one-step prediction  (CI = measured)", "_vel_sbs"),
        ((pos, pos_pred), POS_LABELS,
         "Position — one-step prediction  (CI = measured)", "_pos_sbs"),
    ]:
        fig = plt.figure(figsize=(18, 11))
        fig.suptitle(title_str, fontsize=13, y=0.98)
        gs = GridSpec(4, 1, figure=fig, hspace=0.5)

        for i in range(4):
            ax = fig.add_subplot(gs[i])
            _add_phase_bg(ax)

            ax.plot(t,        measured[:, i],      lw=1.2, color="tab:blue",   label="measured $k+1$")
            ax.plot(t[valid], predicted[valid, i], lw=1.0, color="tab:orange", ls="--", label="1-step pred")

            ax2 = ax.twinx()
            ax2.step(t, u[:, i], lw=0.9, color="gray", alpha=0.5, label="u", where="post")
            ax2.set_ylabel(U_LABELS[i], fontsize=9, color="gray")
            ax2.tick_params(axis="y", labelcolor="gray", labelsize=8)
            ax2.set_ylim(-1.5, 1.5)

            ax.set_ylabel(labels[i], fontsize=10)
            ax.set_xlim(0, TOTAL_DUR)
            ax.grid(True, lw=0.3, alpha=0.5)
            ax.tick_params(labelsize=8)

            rm  = rmse(measured[valid, i], predicted[valid, i])
            r2v = r2(measured[valid, i],   predicted[valid, i])
            ax.set_title(f"{labels[i]}    RMSE={rm:.5f}    R²={r2v:.5f}",
                         fontsize=9, loc="left")

            if i == 0:
                l1, lb1 = ax.get_legend_handles_labels()
                l2, lb2 = ax2.get_legend_handles_labels()
                ax.legend(l1+l2, lb1+lb2, fontsize=8, loc="upper right", framealpha=0.7)

        fig.axes[-1].set_xlabel("time [s]", fontsize=9)
        fig.legend(handles=patches, loc="lower center", ncol=5, fontsize=8, framealpha=0.8)
        fig.tight_layout(rect=[0, 0.04, 1, 0.97])
        #path = out_base + fname_tag + ".png"
        #fig.savefig(path, dpi=150, bbox_inches="tight")
        #print(f"Saved → {path}")

    # ── Métricas en consola ───────────────────────────────────────────────────
    print("\n── One-step prediction metrics ─────────────────────────────────")
    print(f"  {'canal':>8s}  {'RMSE_vel':>12s}  {'R2_vel':>8s}  {'RMSE_pos':>12s}  {'R2_pos':>8s}")
    for i, name in enumerate(["x", "y", "z", "psi"]):
        rv  = rmse(vel[valid, i], vel_pred[valid, i])
        r2v = r2(vel[valid, i],   vel_pred[valid, i])
        rp  = rmse(pos[valid, i], pos_pred[valid, i])
        r2p = r2(pos[valid, i],   pos_pred[valid, i])
        print(f"  {name:>8s}  {rv:12.6f}  {r2v:8.5f}  {rp:12.6f}  {r2p:8.5f}")

    print("\n── Diagnóstico vz ──────────────────────────────────────────────")
    mask_z = np.abs(u[:, 2]) > 0.05
    if mask_z.any():
        err_z = vel[mask_z, 2] - vel_pred[mask_z, 2]
        print(f"  Muestras con |u_z| > 0.05  : {mask_z.sum()}")
        print(f"  Error medio  vz (bias)      : {err_z.mean():+.6f} m/s")
        print(f"  Error std    vz             : {err_z.std():.6f} m/s")
        print(f"  Error max abs vz            : {np.abs(err_z).max():.6f} m/s")
        gs_est = Ku[2, 2] / Kv[2, 2]
        print(f"  Ganancia estática modelo    : {gs_est:.4f} m/s / u")
        print(f"  vz medida max (u_z activo)  : {vel[mask_z, 2].max():.4f} m/s")
        print(f"  vz esperada   (estado est.) : {gs_est * np.abs(u[mask_z, 2]).max():.4f} m/s")

    print()
    plt.show()


DEFAULT_PKL = "/home/brayan/ros2_ws/src/neroControl/data/sysid_20260512_153207.pkl"

if __name__ == "__main__":
    pkl_path = sys.argv[1] if len(sys.argv) == 2 else DEFAULT_PKL
    out_base = pkl_path.replace(".pkl", "")

    print(f"Loading: {pkl_path}")
    t, u, pos, vel = load(pkl_path)
    dts = np.diff(t)
    print(f"  {len(t)} samples")
    print(f"  dt: mean={dts.mean()*1000:.2f} ms  std={dts.std()*1000:.2f} ms"
          f"  min={dts.min()*1000:.2f} ms  max={dts.max()*1000:.2f} ms")

    vel_pred, pos_pred = simulate_stepbystep(t, u, pos, vel)
    plot_and_metrics(t, u, pos, vel, vel_pred, pos_pred, out_base)