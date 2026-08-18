#!/usr/bin/env python3
"""
RK4 Dual-Mode Simulation for GT Cascade Controller on Bebop Drone.
Simulates both:
  1. Position Mode (opt = 2): Nominal velocity reference is ZERO (nu_d_body = 0).
  2. Trajectory Mode (opt = 1): Dynamic velocity reference transformed to BODY frame.

Plots body frame velocities (vx, vy, vz, wyaw) for both reference and simulation.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

# --- Model Identification Matrices (OptiTrack 4DOF) ---
F1 = np.diag([0.921527, 1.053286, 4.173221, 8.772786])
F2 = np.diag([0.247044, 0.395160, 1.975836, 6.101834])

# --- Optimized Gains (from optimal_gt_gains.txt) ---
# Position Gains (opt = 2)
KP_pos  = np.diag([3.375229, 1.746591, 8.000000, 8.186915])
KSP_pos = np.diag([1.200000, 1.200000, 1.000000, 1.000000])
KD_pos  = np.diag([5.480436, 1.572563, 6.000000, 4.000000])
KSD_pos = np.diag([2.000000, 2.000000, 1.200000, 1.000000])

# Trajectory Gains (opt = 1)
KP_traj  = np.diag([2.284778, 2.124041, 7.978605, 8.623905])
KSP_traj = np.diag([0.885532, 1.200000, 0.804755, 1.000000])
KD_traj  = np.diag([8.570068, 8.348064, 5.141405, 4.000000])
KSD_traj = np.diag([1.171520, 0.655947, 1.200000, 1.000000])

U_MAX  = np.ones(4)
NU_MAX = np.array([0.9, 0.9, 0.8, 0.8])

CTRL_HZ = 15.0
CTRL_DT = 1.0 / CTRL_HZ

# --- Kinematic Functions ---
def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def J(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c, -s, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

def J_inv(psi):
    c, s = np.cos(psi), np.sin(psi)
    return np.array([
        [ c,  s, 0., 0.],
        [-s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

def J_dot(psi, r):
    c, s = np.cos(psi), np.sin(psi)
    return r * np.array([
        [-s, -c, 0., 0.],
        [ c, -s, 0., 0.],
        [0., 0., 0., 0.],
        [0., 0., 0., 0.],
    ])

# --- Position Setpoint Generator (Velocity Ref = 0) ---
POINTS_POS = np.array([
    [0.0, 0.0, 1.8],
    [-1.5, -1.5, 1.6],
    [1.5, -1.5, 2.2],
    [-1.5, 1.5, 1.7],
    [1.5, 1.5, 2.3]
])
YAWS_POS = np.deg2rad([0, 45, -45, 90, -90])
HOLD_TIME = 10.0

def get_position_ref(t):
    idx = int(t / HOLD_TIME) % len(POINTS_POS)
    eta_d = np.array([POINTS_POS[idx, 0], POINTS_POS[idx, 1], POINTS_POS[idx, 2], YAWS_POS[idx]])
    nu_d_world = np.zeros(4)      # World velocity reference is ZERO
    alpha_d_world = np.zeros(4)   # World acceleration reference is ZERO
    return eta_d, nu_d_world, alpha_d_world

# --- Continuous Trajectory Generator ---
amplitude_x = 1.8
amplitude_y = 1.8
z_base = 2.0
z_amp = 0.4
w = 0.1

def get_trajectory_ref(t):
    x = amplitude_x * np.sin(w * t)
    y = amplitude_y * np.sin(w * t) * np.cos(w * t)
    z = z_base + z_amp * np.sin(0.5 * w * t)

    dx = amplitude_x * w * np.cos(w * t)
    dy = amplitude_y * w * (np.cos(w * t)**2 - np.sin(w * t)**2)
    dz = z_amp * 0.5 * w * np.cos(0.5 * w * t)

    ddx = -amplitude_x * (w**2) * np.sin(w * t)
    ddy = -4.0 * amplitude_y * (w**2) * np.sin(w * t) * np.cos(w * t)
    ddz = -z_amp * (0.5 * w)**2 * np.sin(0.5 * w * t)

    yaw = wrap_angle(np.arctan2(dy, dx))

    dx_next = amplitude_x * w * np.cos(w * (t + CTRL_DT))
    dy_next = amplitude_y * w * (np.cos(w * (t + CTRL_DT))**2 - np.sin(w * (t + CTRL_DT))**2)
    yaw_next = wrap_angle(np.arctan2(dy_next, dx_next))
    wyaw = wrap_angle(yaw_next - yaw) / CTRL_DT

    eta_d = np.array([x, y, z, yaw])
    nu_d_world = np.array([dx, dy, dz, wyaw])
    alpha_d_world = np.array([ddx, ddy, ddz, 0.0])
    return eta_d, nu_d_world, alpha_d_world

# --- RK4 Integration ---
def rk4_step(eta, nu, u_body, dt):
    def f_nu(nu_curr):
        return F1 @ u_body - F2 @ nu_curr

    def f_eta(eta_curr, nu_curr):
        return J(eta_curr[3]) @ nu_curr

    k1_v = f_nu(nu)
    k2_v = f_nu(nu + 0.5 * dt * k1_v)
    k3_v = f_nu(nu + 0.5 * dt * k2_v)
    k4_v = f_nu(nu + dt * k3_v)
    nu_next = nu + (dt / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
    nu_next = np.clip(nu_next, -NU_MAX, NU_MAX)

    k1_e = f_eta(eta, nu)
    k2_e = f_eta(eta + 0.5 * dt * k1_e, nu + 0.5 * dt * k1_v)
    k3_e = f_eta(eta + 0.5 * dt * k2_e, nu + 0.5 * dt * k2_v)
    k4_e = f_eta(eta + dt * k3_e, nu + dt * k3_v)
    eta_next = eta + (dt / 6.0) * (k1_e + 2.0 * k2_e + 2.0 * k3_e + k4_e)
    eta_next[3] = wrap_angle(eta_next[3])
    
    if eta_next[2] < 0.1:
        eta_next[2] = 0.1
        nu_next[2] = 0.0

    return eta_next, nu_next

def run_simulation(is_trajectory=False, duration=60.0):
    t_vec = np.arange(0.0, duration, CTRL_DT)
    
    eta = np.array([0.0, 0.0, 1.8, 0.0])
    nu = np.zeros(4)
    X_dot_ref_prev = np.zeros(4)

    KP  = KP_traj if is_trajectory else KP_pos
    KSP = KSP_traj if is_trajectory else KSP_pos
    KD  = KD_traj if is_trajectory else KD_pos
    KSD = KSD_traj if is_trajectory else KSD_pos

    eta_hist = []
    eta_ref_hist = []
    nu_hist = []
    nu_ref_body_hist = []
    u_hist = []

    for t in t_vec:
        if is_trajectory:
            eta_d, nu_d_world, alpha_d_world = get_trajectory_ref(t)
        else:
            eta_d, nu_d_world, alpha_d_world = get_position_ref(t)

        psi = eta[3]
        r = nu[3]

        Jmat = J(psi)
        Jdot = J_dot(psi, r)
        Jinv = J_inv(psi)

        # Convert reference velocity to Body Frame for plotting and comparison
        nu_d_body = J_inv(eta_d[3]) @ nu_d_world

        F1_eff = Jmat @ F1
        F2_eff = Jmat @ F2 @ Jinv - Jdot @ Jinv
        F1_eff_inv = np.linalg.inv(F1_eff)

        X_dot = Jmat @ nu

        X_tilde = eta_d - eta
        X_tilde[3] = wrap_angle(X_tilde[3])

        X_dot_ref = nu_d_world + KSP @ np.tanh(KP @ X_tilde)
        X_dot_tilde = X_dot_ref - X_dot
        X_ddot_ref = (X_dot_ref - X_dot_ref_prev) / CTRL_DT
        X_dot_ref_prev = X_dot_ref.copy()

        Ud = F1_eff_inv @ (
            alpha_d_world
            + X_ddot_ref
            + KSD @ np.tanh(KD @ X_dot_tilde)
            + F2_eff @ X_dot
        )
        u_body = np.clip(Ud, -U_MAX, U_MAX)

        eta_hist.append(eta.copy())
        eta_ref_hist.append(eta_d.copy())
        nu_hist.append(nu.copy())
        nu_ref_body_hist.append(nu_d_body.copy())
        u_hist.append(u_body.copy())

        eta, nu = rk4_step(eta, nu, u_body, CTRL_DT)

    return (
        t_vec,
        np.array(eta_hist),
        np.array(eta_ref_hist),
        np.array(nu_hist),
        np.array(nu_ref_body_hist),
        np.array(u_hist),
    )

def plot_mode(t, eta, eta_ref, nu, nu_ref_body, u_cmd, mode_title, output_filename):
    fig, axs = plt.subplots(3, 4, figsize=(18, 11), dpi=120)
    fig.suptitle(f"RK4 Simulation - {mode_title}", fontsize=16, fontweight='bold')

    titles_pos = ['Position X', 'Position Y', 'Position Z (Altitude)', 'Yaw Heading']
    units_pos = ['m', 'm', 'm', 'deg']

    # 1. Position
    for i in range(4):
        ax = axs[0, i]
        ref_val = eta_ref[:, i]
        sim_val = eta[:, i]
        if i == 3:
            ref_val = np.degrees(ref_val)
            sim_val = np.degrees(sim_val)

        ax.plot(t, ref_val, 'r--', linewidth=1.5, label='Reference')
        ax.plot(t, sim_val, 'b-', linewidth=1.8, label='RK4 Sim')
        ax.set_title(f"{titles_pos[i]} [{units_pos[i]}]", fontsize=11, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel(f"[{units_pos[i]}]", fontsize=9)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', fontsize=8)

    # 2. Body Frame Velocities (Ref Body vs RK4 Sim Body)
    titles_vel = ['Body Velocity vx', 'Body Velocity vy', 'Body Velocity vz', 'Yaw Rate wyaw']
    units_vel = ['m/s', 'm/s', 'm/s', 'rad/s']

    for i in range(4):
        ax = axs[1, i]
        ax.plot(t, nu_ref_body[:, i], 'r--', linewidth=1.5, label='Ref Body Vel (0 in Pos Mode)' if 'Position' in mode_title else 'Ref Body Vel')
        ax.plot(t, nu[:, i], 'g-', linewidth=1.8, label='RK4 Body Vel')
        ax.set_title(f"{titles_vel[i]} [{units_vel[i]}]", fontsize=11, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel(f"[{units_vel[i]}]", fontsize=9)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', fontsize=8)

    # 3. Control Commands (Autoscaled for legibility)
    titles_cmd = ['Control Input ux', 'Control Input uy', 'Control Input uz', 'Control Input uyaw']
    colors_cmd = ['#d62728', '#2ca02c', '#1f77b4', '#9467bd']

    for i in range(4):
        ax = axs[2, i]
        ax.plot(t, u_cmd[:, i], color=colors_cmd[i], linewidth=1.5, label=f'u_{i}')
        ax.axhline(1.0, color='gray', linestyle=':', alpha=0.5)
        ax.axhline(-1.0, color='gray', linestyle=':', alpha=0.5)
        
        # Smart dynamic limits for legibility
        u_max_val = max(0.2, np.max(np.abs(u_cmd[:, i])) * 1.2)
        ax.set_ylim([-u_max_val, u_max_val])
        
        ax.set_title(f"{titles_cmd[i]}", fontsize=11, fontweight='bold')
        ax.set_xlabel('Time (s)', fontsize=9)
        ax.set_ylabel('Control Signal [-1, 1]', fontsize=9)
        ax.grid(True, linestyle=':', alpha=0.6)
        ax.legend(loc='upper right', fontsize=8)

    plt.tight_layout()
    output_path = os.path.join("/home/brayan/ros2_ws/src/neroControl/data", output_filename)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, dpi=120)
    print(f"Saved simulation plot: {output_path}")

def main():
    print("1. Running Position Mode (Setpoints, Body Ref Velocity = 0)...")
    t_p, eta_p, eta_ref_p, nu_p, nu_ref_p, u_p = run_simulation(is_trajectory=False, duration=50.0)
    plot_mode(t_p, eta_p, eta_ref_p, nu_p, nu_ref_p, u_p, "Position Setpoint Control Mode (v_ref_body = 0)", "gt_rk4_position_simulation.png")

    print("\n2. Running Continuous Trajectory Mode (Lissajous Curve)...")
    t_t, eta_t, eta_ref_t, nu_t, nu_ref_t, u_t = run_simulation(is_trajectory=True, duration=60.0)
    plot_mode(t_t, eta_t, eta_ref_t, nu_t, nu_ref_t, u_t, "Continuous Trajectory Control Mode (Lissajous)", "gt_rk4_trajectory_simulation.png")

if __name__ == "__main__":
    main()
