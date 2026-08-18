#!/usr/bin/env python3
"""
GT Controller Gain Optimizer using Scipy Differential Evolution.
Optimizes KP, KSP, KD, KSD for both Position Setpoints and Continuous Lissajous Trajectory.
Uses OptiTrack identified model parameters:
  f1 = diag([0.921527, 1.053286, 4.173221, 8.772786])
  f2 = diag([0.247044, 0.395160, 1.975836, 6.101834])

Run directly with:
  ros2 run neroControl gt_gains_optimizer
or:
  python3 /home/brayan/ros2_ws/src/neroControl/nero_control/gt/gt_gains_optimizer.py
"""

import numpy as np
from scipy.optimize import differential_evolution
import time
import os

# --- Model Parameters (Identified from OptiTrack) ---
F1 = np.diag([0.921527, 1.053286, 4.173221, 8.772786])
F2 = np.diag([0.247044, 0.395160, 1.975836, 6.101834])

F1_inv = np.linalg.inv(F1)

DYN_DT  = 0.064        # 64 ms physical dynamics integration
CTRL_DT = 1.0 / 15.0  # 15 Hz control loop

U_MAX  = np.ones(4)
NU_MAX = np.array([0.9, 0.9, 0.8, 0.8])

# --- Helper Kinematic Functions ---
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

def unpack(params):
    KP  = np.diag(params[0:4])
    KSP = np.diag(params[4:8])
    KD  = np.diag(params[8:12])
    KSD = np.diag(params[12:16])
    return KP, KSP, KD, KSD

# --- 1. Position Reference Generator (5 Setpoints, 10s hold each) ---
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
    nu_d = np.zeros(4)
    alpha_d = np.zeros(4)
    return eta_d, nu_d, alpha_d

# --- 2. Continuous Trajectory Generator (Lissajous) ---
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
    nu_d = np.array([dx, dy, dz, wyaw])
    alpha_d = np.array([ddx, ddy, ddz, 0.0])
    return eta_d, nu_d, alpha_d

# --- Simulation Engine ---
def simulate(KP, KSP, KD, KSD, is_trajectory=False):
    t = 0.0
    t_end = 25.0  # Fast evaluation window for quick convergence
    
    eta = np.array([0.0, 0.0, 1.8, 0.0])
    nu = np.zeros(4)
    X_dot_ref_prev = np.zeros(4)
    
    errors = []
    inputs = []
    
    sim_steps = max(1, round(CTRL_DT / DYN_DT))

    while t < t_end:
        if is_trajectory:
            eta_d, nu_d, alpha_d = get_trajectory_ref(t)
        else:
            eta_d, nu_d, alpha_d = get_position_ref(t)

        psi = eta[3]
        r = nu[3]

        Jmat = J(psi)
        Jdot = J_dot(psi, r)
        Jinv = J_inv(psi)
        
        F1_eff = Jmat @ F1
        F2_eff = Jmat @ F2 @ Jinv - Jdot @ Jinv
        F1_eff_inv = np.linalg.inv(F1_eff)

        X_dot = Jmat @ nu

        X_tilde = eta_d - eta
        X_tilde[3] = wrap_angle(X_tilde[3])

        X_dot_ref = nu_d + KSP @ np.tanh(KP @ X_tilde)
        X_dot_tilde = X_dot_ref - X_dot
        X_ddot_ref = (X_dot_ref - X_dot_ref_prev) / CTRL_DT
        X_dot_ref_prev = X_dot_ref.copy()

        Ud = F1_eff_inv @ (
            alpha_d
            + X_ddot_ref
            + KSD @ np.tanh(KD @ X_dot_tilde)
            + F2_eff @ X_dot
        )
        U_body = np.clip(Ud, -U_MAX, U_MAX)

        errors.append(X_tilde.copy())
        inputs.append(U_body.copy())

        for _ in range(sim_steps):
            nu_dot = F1 @ U_body - F2 @ nu
            nu = nu + DYN_DT * nu_dot
            nu = np.clip(nu, -NU_MAX, NU_MAX)

            eta_dot = J(eta[3]) @ nu
            eta = eta + DYN_DT * eta_dot
            eta[3] = wrap_angle(eta[3])
            if eta[2] < 0.1:
                eta[2] = 0.1
                nu[2] = 0.0

        t += CTRL_DT

    return np.array(errors), np.array(inputs)

def cost_pos(params):
    KP, KSP, KD, KSD = unpack(params)
    try:
        errors, inputs = simulate(KP, KSP, KD, KSD, is_trajectory=False)
    except Exception:
        return 1e9

    if not np.all(np.isfinite(errors)):
        return 1e9

    w_weights = np.array([1.0, 1.0, 2.5, 2.5])
    ise = np.mean((errors ** 2) * w_weights)
    
    u_diff = np.diff(inputs, axis=0)
    chattering = 0.05 * np.mean(u_diff ** 2)
    return ise + chattering

def cost_traj(params):
    KP, KSP, KD, KSD = unpack(params)
    try:
        errors, inputs = simulate(KP, KSP, KD, KSD, is_trajectory=True)
    except Exception:
        return 1e9

    if not np.all(np.isfinite(errors)):
        return 1e9

    w_weights = np.array([1.5, 1.5, 2.5, 3.0])
    ise = np.mean((errors ** 2) * w_weights)
    
    u_diff = np.diff(inputs, axis=0)
    chattering = 0.05 * np.mean(u_diff ** 2)
    return ise + chattering

BOUNDS = [
    (0.5, 6.0), (0.5, 6.0), (1.0, 8.0), (1.0, 12.0),  # KP: x, y, z, yaw
    (0.2, 1.2), (0.2, 1.2), (0.2, 1.0), (0.1, 1.0),   # KSP: x, y, z, yaw
    (1.0, 10.0), (1.0, 10.0), (0.5, 6.0), (0.3, 4.0),  # KD: x, y, z, yaw
    (0.2, 2.0), (0.2, 2.0), (0.1, 1.2), (0.1, 1.0),   # KSD: x, y, z, yaw
]

def format_matrix(name, mat):
    d = np.diag(mat)
    return f"        {name:3s} = np.diag([{d[0]:.6f}, {d[1]:.6f}, {d[2]:.6f}, {d[3]:.6f}])"

def main():
    print("=" * 70)
    print("🚀 GT CONTROLLER GAIN OPTIMIZATION (FAST PARALLEL MODE)")
    print("=" * 70)

    print("\n--- [1/2] Fast Optimizing POSITION Gains (Step Setpoints) ---")
    t0 = time.time()
    res_pos = differential_evolution(
        cost_pos,
        BOUNDS,
        strategy='best1bin',
        maxiter=8,
        popsize=8,
        tol=1e-3,
        mutation=(0.5, 1.0),
        recombination=0.8,
        seed=42,
        workers=-1,
        disp=True
    )
    t_pos = time.time() - t0
    KP_p, KSP_p, KD_p, KSD_p = unpack(res_pos.x)
    print(f"✅ Position Gains Optimization Done in {t_pos:.2f}s (Cost: {res_pos.fun:.6f})")

    print("\n--- [2/2] Fast Optimizing TRAJECTORY Gains (Lissajous Curve) ---")
    t0 = time.time()
    res_traj = differential_evolution(
        cost_traj,
        BOUNDS,
        strategy='best1bin',
        maxiter=8,
        popsize=8,
        tol=1e-3,
        mutation=(0.5, 1.0),
        recombination=0.8,
        seed=42,
        workers=-1,
        disp=True
    )
    t_traj = time.time() - t0
    KP_t, KSP_t, KD_t, KSD_t = unpack(res_traj.x)
    print(f"✅ Trajectory Gains Optimization Done in {t_traj:.2f}s (Cost: {res_traj.fun:.6f})")

    print("\n" + "=" * 70)
    print("🎯 OPTIMIZED POSITION GAINS (opt = 2):")
    print("=" * 70)
    print(format_matrix("KP", KP_p))
    print(format_matrix("KSP", KSP_p))
    print(format_matrix("KD", KD_p))
    print(format_matrix("KSD", KSD_p))

    print("\n" + "=" * 70)
    print("📈 OPTIMIZED TRAJECTORY GAINS (opt = 1):")
    print("=" * 70)
    print(format_matrix("KP", KP_t))
    print(format_matrix("KSP", KSP_t))
    print(format_matrix("KD", KD_t))
    print(format_matrix("KSD", KSD_t))

    out_file = "/home/brayan/ros2_ws/src/neroControl/data/optimal_gt_gains.txt"
    os.makedirs(os.path.dirname(out_file), exist_ok=True)
    with open(out_file, "w") as f:
        f.write("# Optimal GT Controller Gains for OptiTrack Model\n\n")
        f.write("# Position Gains (opt = 2):\n")
        f.write(format_matrix("KP", KP_p) + "\n")
        f.write(format_matrix("KSP", KSP_p) + "\n")
        f.write(format_matrix("KD", KD_p) + "\n")
        f.write(format_matrix("KSD", KSD_p) + "\n\n")

        f.write("# Trajectory Gains (opt = 1):\n")
        f.write(format_matrix("KP", KP_t) + "\n")
        f.write(format_matrix("KSP", KSP_t) + "\n")
        f.write(format_matrix("KD", KD_t) + "\n")
        f.write(format_matrix("KSD", KSD_t) + "\n")

    print(f"\nSaved gains summary to: {out_file}")

if __name__ == "__main__":
    main()
