#!/usr/bin/env python3
"""
Offline Closed-Loop Simulator for the Cascade Controller.

Replicates:
  - The identified 4-DOF plant dynamics (f1, f2) from system_identification_optitrack_4dof.py
  - The cascade controller logic from gt_extended_controller.py
  - The step-based position reference from gt_position_ref.py

Produces diagnostic plots showing position tracking, velocity, control signal,
and overshoot analysis for each axis.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ─────────────────────── Plant Model (identified) ───────────────────────
# Body-frame 1st-order model per axis: ν̇_i = f1_i * u_i  -  f2_i * ν_i
F1_DIAG = np.array([0.921527, 1.053286, 4.173221, 8.772786])
F2_DIAG = np.array([0.247044, 0.395160, 1.975836, 6.101834])

# ─────────────────────── Controller Gains (opt == 1) ────────────────────
# KP  = np.diag([4.000000, 4.000000, 8.000000, 12.152000])
# KSP = np.diag([0.300000, 0.300000, 0.100000, 0.500000])
# KD  = np.diag([6.411000, 6.235000, 1.204000, 2.951000])
# KSD = np.diag([1.976000, 2.000000, 1.000000, 0.419000])


KP  = np.diag([5.000000, 1.058000, 7.374000, 6.730000])
KSP = np.diag([0.201000, 0.201000, 0.200000, 0.492000])
KD  = np.diag([2.197000, 8.009000, 1.479000, 0.300000])
KSD = np.diag([0.118000, 0.737000, 0.004000, 0.873000])

# KP  = np.diag([4.000000, 4.000000, 8.000000, 10.940000])
# KSP = np.diag([0.201000, 0.200000, 0.100000, 0.200000])
# KD  = np.diag([6.161000, 5.881000, 3.457000, 1.530000])
# KSD = np.diag([1.924000, 2.000000, 0.100000, 0.326000])

U_MAX = np.ones(4)

# ─────────────────────── Reference Sequence (from gt_position_ref.py) ───
HOLD_TIME = 25.0  # seconds per waypoint
L = 1.0
WAYPOINTS = np.array([
    [0.0, 0.0, 1.8, np.deg2rad(0)],
    [-L,  -L,  1.6, np.deg2rad(45)],
    [ L,  -L,  2.2, np.deg2rad(-45)],
    [-L,   L,  1.7, np.deg2rad(90)],
    [ L,   L,  2.3, np.deg2rad(-90)],
])

# ─────────────────────── Simulation Parameters ─────────────────────────
RATE_HZ = 15.0
DT = 1.0 / RATE_HZ
T_TOTAL = 125.0  # seconds (5 waypoints * 25s)

# Scaling factors applied in _publish_cmd (as in the current broken code)
#CMD_SCALE = np.array([0.3, 0.3, 0.5, 0.0])  # <-- the problem!
CMD_SCALE = np.array([1.0, 1.0, 1.0, 1.0])  # <-- what it should be


# ─────────────────────── Helper Functions ───────────────────────────────

def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def jacobian(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c, -s, 0.0, 0.0],
        [ s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_inv(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c,  s, 0.0, 0.0],
        [-s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def jacobian_dot(psi, psi_dot):
    c, s = np.cos(psi), np.sin(psi)
    return psi_dot * np.array([
        [-s, -c, 0.0, 0.0],
        [ c, -s, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0],
    ])


def get_reference(t):
    """Return (eta_d, nu_d, alpha_d) for a given time, replicating gt_position_ref.py."""
    cycle_t = t % (HOLD_TIME * len(WAYPOINTS))
    idx = int(cycle_t // HOLD_TIME)
    idx = min(idx, len(WAYPOINTS) - 1)
    eta_d = WAYPOINTS[idx].copy()
    nu_d = np.zeros(4)
    alpha_d = np.zeros(4)
    return eta_d, nu_d, alpha_d


def plant_step(nu_body, u_cmd, dt):
    """
    Simulate one step of the identified body-frame plant.
    ν̇_i = f1_i * u_i  -  f2_i * ν_i
    """
    nu_dot = F1_DIAG * u_cmd - F2_DIAG * nu_body
    nu_body_next = nu_body + dt * nu_dot
    return nu_body_next


def controller_step(eta, nu_body, eta_d, nu_d, alpha_d, X_dot_ref_prev, X_ddot_ref_filt, dt):
    """
    One step of the cascade controller (exact copy of gt_extended_controller.py logic).
    Returns: U_body_saturated, X_dot_ref (for memory), X_ddot_ref_filt (for memory)
    """
    psi = eta[3]
    r = nu_body[3]  # yaw rate is the same in body and inertial

    f1 = np.diag(F1_DIAG)
    f2 = np.diag(F2_DIAG)

    J = jacobian(psi)
    J_d = jacobian_dot(psi, r)
    J_i = jacobian_inv(psi)

    F1 = J @ f1
    F2 = J @ f2 @ J_i - J_d @ J_i
    F1_inv = np.linalg.inv(F1)

    nu = nu_body.copy()
    X_dot = J @ nu

    X_tilde = eta_d - eta
    X_tilde[3] = wrap_angle(X_tilde[3])

    X_dot_ref = nu_d + KSP @ np.tanh(KP @ X_tilde)

    X_dot_tilde = X_dot_ref - X_dot
    X_ddot_ref_raw = (X_dot_ref - X_dot_ref_prev) / dt
    X_ddot_ref_filt_new = 0.7 * X_ddot_ref_filt + 0.3 * X_ddot_ref_raw

    Ud = F1_inv @ (
        alpha_d
        + X_ddot_ref_filt_new
        + KSD @ np.tanh(KD @ X_dot_tilde)
        + F2 @ X_dot
    )

    U_body = np.clip(Ud, -U_MAX, U_MAX)

    return U_body, X_dot_ref.copy(), X_ddot_ref_filt_new.copy()


def integrate_pose(eta, nu_body, dt):
    """Integrate body velocities into inertial pose."""
    psi = eta[3]
    J = jacobian(psi)
    eta_dot = J @ nu_body
    eta_new = eta + dt * eta_dot
    eta_new[3] = wrap_angle(eta_new[3])
    return eta_new


# ─────────────────────── Main Simulation Loop ──────────────────────────

def run_simulation(cmd_scale, label=""):
    N = int(T_TOTAL / DT)

    # State histories
    t_hist = np.zeros(N)
    eta_hist = np.zeros((N, 4))
    nu_hist = np.zeros((N, 4))
    u_hist = np.zeros((N, 4))
    ref_hist = np.zeros((N, 4))

    # Initial conditions
    eta = np.array([0.0, 0.0, 1.5, 0.0])  # start at origin, on the ground
    nu_body = np.zeros(4)

    # Controller memory
    X_dot_ref_prev = np.zeros(4)
    X_ddot_ref_filt = np.zeros(4)

    for k in range(N):
        t = k * DT
        t_hist[k] = t

        # Get reference
        eta_d, nu_d, alpha_d = get_reference(t)
        ref_hist[k] = eta_d

        # Store state
        eta_hist[k] = eta
        nu_hist[k] = nu_body

        # Controller
        U_body, X_dot_ref_prev, X_ddot_ref_filt = controller_step(
            eta, nu_body, eta_d, nu_d, alpha_d,
            X_dot_ref_prev, X_ddot_ref_filt, DT
        )

        # Apply _publish_cmd scaling (this is the bug we're testing)
        U_applied = U_body * cmd_scale
        u_hist[k] = U_applied  # store actual applied control action

        # Plant dynamics
        nu_body = plant_step(nu_body, U_applied, DT)

        # Integrate pose
        eta = integrate_pose(eta, nu_body, DT)

    return t_hist, eta_hist, nu_hist, u_hist, ref_hist


# ─────────────────────── Plotting ──────────────────────────────────────

def plot_results(results_list):
    """Plot comparison of position tracking and control action."""
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "axes.edgecolor": "black",
        "axes.linewidth": 0.8,
        "axes.grid": True,
        "grid.color": "#d0d0d0",
        "grid.linewidth": 0.45,
        "grid.linestyle": "--",
        "font.family": "serif",
        "font.size": 9,
        "legend.frameon": True,
        "legend.framealpha": 1.0,
        "legend.edgecolor": "black",
        "legend.facecolor": "white",
        "legend.fontsize": 8,
        "lines.linewidth": 1.2,
    })

    axis_labels = ['X [m]', 'Y [m]', 'Z [m]', 'Yaw [rad]']
    vel_labels = ['v_x Cuerpo [m/s]', 'v_y Cuerpo [m/s]', 'v_z Cuerpo [m/s]', 'r Cuerpo [rad/s]']
    u_labels = ['ux [cmd]', 'uy [cmd]', 'uz [cmd]', 'uψ [cmd]']

    fig, axes = plt.subplots(4, 3, figsize=(17, 10), sharex=True)
    fig.suptitle('Cascade Controller: Sin Escala vs Con Escala', fontsize=13, fontweight='bold')

    for sim_idx, (t, eta, nu, u, ref, label, color) in enumerate(results_list):
        for i in range(4):
            # Position tracking (Col 0)
            ax = axes[i, 0]
            if sim_idx == 0:
                ax.plot(t, ref[:, i], 'k--', linewidth=1.5, label='Referencia', zorder=10)
            ax.plot(t, eta[:, i], color=color, label=label, zorder=5)
            ax.set_ylabel(axis_labels[i])
            if i == 0:
                ax.set_title('Posición (Tracking)')
                ax.legend(loc='upper right', fontsize=8)

            # Velocity (Col 1)
            ax = axes[i, 1]
            ax.plot(t, nu[:, i], color=color, label=label)
            ax.set_ylabel(vel_labels[i])
            if i == 0:
                ax.set_title('Velocidad (Marco Cuerpo / Body Frame)')
                ax.legend(loc='upper right', fontsize=8)

            # Control action (Col 2)
            ax = axes[i, 2]
            ax.plot(t, u[:, i], color=color, label=label)
            ax.axhline(1.0, color='red', linestyle=':', linewidth=0.8, alpha=0.5)
            ax.axhline(-1.0, color='red', linestyle=':', linewidth=0.8, alpha=0.5)
            ax.set_ylabel(u_labels[i])
            if i == 0:
                ax.set_title('Acción de Control Aplicada')
                ax.legend(loc='upper right', fontsize=8)

    for j in range(3):
        axes[3, j].set_xlabel('Tiempo [s]')

    plt.tight_layout()
    plt.savefig('/home/brayan/ros2_ws/src/neroControl/optitrak_data/system_identification/simulation_results.pdf',
                dpi=150, bbox_inches='tight')
    plt.savefig('/home/brayan/ros2_ws/src/neroControl/optitrak_data/system_identification/simulation_results.png',
                dpi=150, bbox_inches='tight')
    print("Saved: simulation_results.pdf / .png")
    plt.show()


def compute_overshoot_stats(t, eta, ref):
    """Compute overshoot percentage and settling time per axis."""
    labels = ['X', 'Y', 'Z', 'Yaw']
    print("\n" + "=" * 60)
    print("OVERSHOOT & SETTLING ANALYSIS")
    print("=" * 60)

    for i in range(4):
        # Find step transitions in reference
        ref_changes = np.where(np.abs(np.diff(ref[:, i])) > 0.01)[0]
        if len(ref_changes) == 0:
            print(f"  {labels[i]}: No step changes detected")
            continue

        overshoots = []
        settling_times = []

        for change_idx in ref_changes:
            if change_idx + 1 >= len(ref):
                continue
            target = ref[change_idx + 1, i]
            initial = eta[change_idx, i]
            step_size = target - initial

            if abs(step_size) < 0.01:
                continue

            # Look at response after the step
            end_idx = min(change_idx + int(HOLD_TIME / DT), len(eta))
            response = eta[change_idx:end_idx, i]

            if step_size > 0:
                peak = np.max(response)
                overshoot_pct = (peak - target) / abs(step_size) * 100.0
            else:
                peak = np.min(response)
                overshoot_pct = (target - peak) / abs(step_size) * 100.0

            overshoots.append(max(0, overshoot_pct))

            # Settling time (2% band)
            band = 0.02 * abs(step_size)
            settled = np.abs(response - target) < band
            settling_idx = None
            for j in range(len(settled) - 1, -1, -1):
                if not settled[j]:
                    settling_idx = j + 1
                    break
            if settling_idx is not None and settling_idx < len(settled):
                settling_times.append(settling_idx * DT)
            else:
                settling_times.append(0.0)

        if overshoots:
            print(f"  {labels[i]:4s}: Avg Overshoot = {np.mean(overshoots):6.1f}%  "
                  f"Max Overshoot = {np.max(overshoots):6.1f}%  "
                  f"Avg Settling = {np.mean(settling_times):5.2f}s")
        else:
            print(f"  {labels[i]:4s}: No valid steps to analyze")

    print("=" * 60)


# ─────────────────────── Entry Point ───────────────────────────────────

if __name__ == '__main__':
    print(f"Running simulation WITH CMD_SCALE = {CMD_SCALE}...")
    t1, eta1, nu1, u1, ref1 = run_simulation(
        cmd_scale=CMD_SCALE
    )
    compute_overshoot_stats(t1, eta1, ref1)

    print("\nRunning baseline simulation WITHOUT SCALING (cmd_scale = [1.0, 1.0, 1.0, 1.0])...")
    t2, eta2, nu2, u2, ref2 = run_simulation(
        cmd_scale=np.array([1.0, 1.0, 1.0, 1.0])
    )
    compute_overshoot_stats(t2, eta2, ref2)

    plot_results([
        (t1, eta1, nu1, u1, ref1, f'Cmd Scale ({CMD_SCALE})', '#d35400'),
        (t2, eta2, nu2, u2, ref2, 'Sin escala (×1.0)', '#1a56a0'),
    ])
