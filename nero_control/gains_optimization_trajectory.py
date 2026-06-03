#!/usr/bin/env python3

import numpy as np
from scipy.optimize import differential_evolution
import time

_F1 = np.diag([0.988324, 0.986558, 0.802580, 0.853101])
_F2 = np.diag([0.018878, 0.025773, 0.122009, 0.122507])

DYN_DT  = 0.064
CTRL_DT = 1.0 / 15.0

CTRL_F1 = np.array([
    [ 0.988324,  0.014589,  0.000000, -0.009546],
    [-0.019977,  0.986558,  0.000000,  0.005055],
    [ 0.000000,  0.000000,  0.802580,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.853101],
])

CTRL_F2 = np.array([
    [ 0.018878, -0.002861,  0.000000,  0.003410],
    [-0.002938,  0.025773,  0.000000, -0.008888],
    [ 0.000000,  0.000000,  0.122009,  0.000000],
    [ 0.000000,  0.000000,  0.000000,  0.122507],
])

NU_MAX = np.array([0.9, 0.9, 0.8, 0.8])
U_MAX  = np.ones(4)

SIM_DURATION = 90.0

AMP_X = 1.5
AMP_Y = 1.5
Z_BASE = 1.5
Z_AMP = 0.5
W = 0.1

def wrap_angle(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi

def J(psi):
    c = np.cos(psi)
    s = np.sin(psi)

    return np.array([
        [ c, -s, 0., 0.],
        [ s,  c, 0., 0.],
        [0., 0., 1., 0.],
        [0., 0., 0., 1.],
    ])

def J_dot(psi, r):
    c = np.cos(psi)
    s = np.sin(psi)

    return r * np.array([
        [-s, -c, 0., 0.],
        [ c, -s, 0., 0.],
        [0., 0., 0., 0.],
        [0., 0., 0., 0.],
    ])

def get_ref(t):
    x = AMP_X * np.sin(W * t)

    y = AMP_Y * np.sin(W * t) * np.cos(W * t)

    z = Z_BASE + Z_AMP * np.sin(0.5 * W * t)

    dx = AMP_X * W * np.cos(W * t)

    dy = AMP_Y * W * (
        np.cos(W * t)**2 - np.sin(W * t)**2
    )

    dz = Z_AMP * 0.5 * W * np.cos(0.5 * W * t)

    ddx = -AMP_X * (W**2) * np.sin(W * t)

    ddy = -4.0 * AMP_Y * (W**2) * \
          np.sin(W * t) * np.cos(W * t)

    ddz = -Z_AMP * (0.5 * W)**2 * np.sin(0.5 * W * t)

    yaw = np.arctan2(dy, dx)

    vx = dx
    vy = dy
    vz = dz

    wyaw = 0.0

    ax = ddx
    ay = ddy
    az = ddz

    ayaw = 0.0

    eta_d = np.array([x, y, z, yaw])
    nu_d = np.array([vx, vy, vz, wyaw])
    alpha_d = np.array([ax, ay, az, ayaw])

    return eta_d, nu_d, alpha_d

def simulate(KP, KSP, KD, KSD):
    eta = np.array([0., 0., 1.5, 0.])
    nu = np.zeros(4)

    X_dot_ref_prev = np.zeros(4)

    errors = []
    vel_errors = []
    acc_cmd_hist = []
    jerk_hist = []

    last_u = np.zeros(4)

    t = 0.0

    sim_steps = max(1, round(CTRL_DT / DYN_DT))

    while t < SIM_DURATION:

        eta_d, nu_d, alpha_d = get_ref(t)

        psi = eta[3]
        r = nu[3]

        Jmat = J(psi)
        Jdotmat = J_dot(psi, r)

        F1_eff = Jmat @ CTRL_F1
        F2_eff = Jmat @ CTRL_F2 - Jdotmat

        F1_inv = np.linalg.inv(F1_eff)

        X_dot = Jmat @ nu

        X_tilde = eta_d - eta
        X_tilde[3] = wrap_angle(X_tilde[3])

        X_dot_ref = nu_d + KSP @ np.tanh(KP @ X_tilde)

        X_dot_tilde = X_dot_ref - X_dot

        X_ddot_ref = (X_dot_ref - X_dot_ref_prev) / CTRL_DT
        X_dot_ref_prev = X_dot_ref.copy()

        Ud = F1_inv @ (
            alpha_d
            + X_ddot_ref
            + KSD @ np.tanh(KD @ X_dot_tilde)
            + F2_eff @ nu
        )

        U_body = np.clip(Ud, -U_MAX, U_MAX)

        jerk = (U_body - last_u) / CTRL_DT
        last_u = U_body.copy()

        errors.append(X_tilde.copy())
        vel_errors.append(X_dot_tilde.copy())
        acc_cmd_hist.append(U_body.copy())
        jerk_hist.append(jerk.copy())

        for _ in range(sim_steps):

            nu_dot = _F1 @ U_body - _F2 @ nu

            nu = nu + DYN_DT * nu_dot

            nu = np.clip(nu, -NU_MAX, NU_MAX)

            eta_dot = J(eta[3]) @ nu

            eta = eta + DYN_DT * eta_dot

            eta[3] = wrap_angle(eta[3])

            if eta[2] < 0.05:
                eta[2] = 0.05
                nu[2] = 0.0

        t += CTRL_DT

    return (
        np.array(errors),
        np.array(vel_errors),
        np.array(acc_cmd_hist),
        np.array(jerk_hist),
    )

def unpack(params):

    KP  = np.diag(params[0:4])
    KSP = np.diag(params[4:8])
    KD  = np.diag(params[8:12])
    KSD = np.diag(params[12:16])

    return KP, KSP, KD, KSD

def cost(params):

    KP, KSP, KD, KSD = unpack(params)

    try:
        pos_err, vel_err, u_hist, jerk_hist = simulate(
            KP,
            KSP,
            KD,
            KSD
        )

    except Exception:
        return 1e9

    if not np.all(np.isfinite(pos_err)):
        return 1e9

    w_pos = np.array([1.0, 1.0, 2.5, 2.0])
    w_vel = np.array([0.8, 0.8, 1.5, 1.0])

    pos_cost = np.mean((pos_err ** 2) * w_pos)

    vel_cost = np.mean((vel_err ** 2) * w_vel)

    jerk_cost = 4.0 * np.mean(jerk_hist ** 2)

    control_cost = 1.5 * np.mean(u_hist ** 2)

    smooth_vel_cost = 3.0 * np.mean(np.diff(vel_err, axis=0) ** 2)

    transient_len = len(pos_err) // 5

    transient_cost = 0.7 * np.mean(
        (pos_err[:transient_len] ** 2) * w_pos
    )

    steady_cost = 1.8 * np.mean(
        (pos_err[-transient_len:] ** 2) * w_pos
    )

    total = (
        pos_cost
        + vel_cost
        + jerk_cost
        + control_cost
        + smooth_vel_cost
        + transient_cost
        + steady_cost
    )

    return total

BOUNDS = [

    (0.2, 2.5),
    (0.2, 2.5),
    (2.0, 5.0),
    (4.0, 10.0),

    (0.1, 0.6),
    (0.1, 0.6),
    (0.05, 0.5),
    (0.05, 0.5),

    (0.5, 4.0),
    (0.5, 4.0),
    (0.5, 3.0),
    (0.3, 2.0),

    (0.05, 1.0),
    (0.05, 1.0),
    (0.05, 0.8),
    (0.05, 0.6),
]

def fmt_mat(name, mat):

    d = np.diag(mat)

    return (
        f"{name} = np.diag(["
        f"{d[0]:.6f}, "
        f"{d[1]:.6f}, "
        f"{d[2]:.6f}, "
        f"{d[3]:.6f}])"
    )

def main():

    print("=" * 70)
    print("OPTIMIZACIÓN PARA TRACKING SUAVE DE TRAYECTORIA")
    print("=" * 70)

    t0 = time.time()

    result = differential_evolution(
        cost,
        BOUNDS,
        strategy='best1bin',
        maxiter=100,
        popsize=20,
        mutation=(0.4, 1.0),
        recombination=0.8,
        tol=1e-7,
        polish=True,
        seed=42,
        workers=-1,
        updating='deferred',
        disp=True
    )

    elapsed = time.time() - t0

    KP, KSP, KD, KSD = unpack(result.x)

    print()
    print("=" * 70)
    print("RESULTADO")
    print("=" * 70)

    print(f"Tiempo total: {elapsed:.2f} s")
    print(f"Coste final : {result.fun:.8f}")

    print()
    print(fmt_mat("KP ", KP))
    print(fmt_mat("KSP", KSP))
    print(fmt_mat("KD ", KD))
    print(fmt_mat("KSD", KSD))

    pos_err, vel_err, u_hist, jerk_hist = simulate(
        KP,
        KSP,
        KD,
        KSD
    )

    rms_pos = np.sqrt(np.mean(pos_err**2, axis=0))
    rms_vel = np.sqrt(np.mean(vel_err**2, axis=0))
    rms_jerk = np.sqrt(np.mean(jerk_hist**2, axis=0))

    print()
    print("=" * 70)
    print("MÉTRICAS")
    print("=" * 70)

    print()
    print("RMS posición")
    print(f"x    : {rms_pos[0]*100:.2f} cm")
    print(f"y    : {rms_pos[1]*100:.2f} cm")
    print(f"z    : {rms_pos[2]*100:.2f} cm")
    print(f"yaw  : {np.degrees(rms_pos[3]):.2f} deg")

    print()
    print("RMS velocidad")
    print(f"vx   : {rms_vel[0]:.4f} m/s")
    print(f"vy   : {rms_vel[1]:.4f} m/s")
    print(f"vz   : {rms_vel[2]:.4f} m/s")
    print(f"yaw  : {rms_vel[3]:.4f} rad/s")

    print()
    print("RMS jerk comando")
    print(f"jx   : {rms_jerk[0]:.4f}")
    print(f"jy   : {rms_jerk[1]:.4f}")
    print(f"jz   : {rms_jerk[2]:.4f}")
    print(f"jyaw : {rms_jerk[3]:.4f}")

    with open("optimal_gains_trajectory.txt", "w") as f:

        f.write("Ganancias óptimas tracking trayectoria suave\n\n")

        f.write(fmt_mat("KP ", KP) + "\n")
        f.write(fmt_mat("KSP", KSP) + "\n")
        f.write(fmt_mat("KD ", KD) + "\n")
        f.write(fmt_mat("KSD", KSD) + "\n")

        f.write("\n")

        f.write(f"Coste final: {result.fun:.8f}\n")

        f.write("\nRMS posición\n")
        f.write(f"x    : {rms_pos[0]*100:.2f} cm\n")
        f.write(f"y    : {rms_pos[1]*100:.2f} cm\n")
        f.write(f"z    : {rms_pos[2]*100:.2f} cm\n")
        f.write(f"yaw  : {np.degrees(rms_pos[3]):.2f} deg\n")

        f.write("\nRMS velocidad\n")
        f.write(f"vx   : {rms_vel[0]:.4f} m/s\n")
        f.write(f"vy   : {rms_vel[1]:.4f} m/s\n")
        f.write(f"vz   : {rms_vel[2]:.4f} m/s\n")
        f.write(f"yaw  : {rms_vel[3]:.4f} rad/s\n")

    print()
    print("Archivo guardado: optimal_gains_trajectory.txt")

if __name__ == "__main__":
    main()