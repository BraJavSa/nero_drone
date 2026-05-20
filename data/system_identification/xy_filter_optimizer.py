#!/usr/bin/env python3

# Optimizes Alpha-Beta filter parameters for the X and Y axes using a grid search.
"""
Identificación dinámica Bebop
=================================

Ahora NO se usan vx_b_full ni vy_b_full del Bebop.

Las velocidades body se reconstruyen así:

    posicion mundo
        ↓
    derivada numerica
        ↓
    velocidad mundo
        ↓
    rotacion mundo → body
        ↓
    filtro Alpha-Beta
        ↓
    identificacion

Luego se busca automáticamente el mejor filtro
(alpha_x, beta_x, alpha_y, beta_y)
maximizando el FIT de posición.

"""

import numpy as np
import matplotlib.pyplot as plt

from scipy.io import loadmat
from scipy.signal import savgol_filter


mat_file = "manual_log_20260518_172721.mat"

data = loadmat(mat_file)

hz = float(data["hz"].squeeze())
dt = 1.0 / hz

DELAY = 2
MIN_INPUT = 0.0

u_full = data["u"]

vz_b_full = data["vz_b"].squeeze()

psi_full = data["psi"].squeeze()
r_b_full = data["vpsi"].squeeze()

x_real_full = data["x_i"].squeeze()
y_real_full = data["y_i"].squeeze()
z_real_full = data["z_i"].squeeze()


def alpha_beta_filter(measurements, alpha, beta, dt):

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



def identify_coupled(vx, vy, r, ux, uy, upsi, min_input):

    Phi = np.column_stack([
        vx[:-1],
        vy[:-1],
        r[:-1],

        ux[:-1],
        uy[:-1],
        upsi[:-1],
    ])

    mask = (
        (np.abs(ux[:-1]) > min_input)
        |
        (np.abs(uy[:-1]) > min_input)
    )

    theta_x, *_ = np.linalg.lstsq(
        Phi[mask],
        vx[1:][mask],
        rcond=None
    )

    theta_y, *_ = np.linalg.lstsq(
        Phi[mask],
        vy[1:][mask],
        rcond=None
    )

    return theta_x, theta_y


def identify_decoupled(v, u_in, min_input):

    y = v[1:]

    mask = np.abs(u_in[:-1]) > min_input

    Phi = np.column_stack([
        v[:-1][mask],
        u_in[:-1][mask]
    ])

    theta, *_ = np.linalg.lstsq(
        Phi,
        y[mask],
        rcond=None
    )

    return theta[0], theta[1]


def kinematics(state, v_body):

    psi_k = state[3]

    vx_b, vy_b, vz_b, r_b = v_body

    return np.array([

        np.cos(psi_k) * vx_b
        -
        np.sin(psi_k) * vy_b,

        np.sin(psi_k) * vx_b
        +
        np.cos(psi_k) * vy_b,

        vz_b,

        r_b,
    ])


def fit_pct(y, yhat):

    var_y = np.linalg.norm(y - np.mean(y))

    if var_y == 0:
        return 0.0

    return 100.0 * (
        1.0 - np.linalg.norm(y - yhat) / var_y
    )


def rms(a, b):

    return np.sqrt(np.mean((a - b) ** 2))



print("\nReconstruyendo velocidades body...")

x_s = savgol_filter(x_real_full, 11, 3)
y_s = savgol_filter(y_real_full, 11, 3)

vx_w = np.gradient(x_s, dt)
vy_w = np.gradient(y_s, dt)

c = np.cos(psi_full)
s = np.sin(psi_full)

vx_body_raw = c * vx_w + s * vy_w
vy_body_raw = -s * vx_w + c * vy_w


ALPHA_Z = 0.0387
BETA_Z  = 0.4066

ALPHA_YAW = 0.2471
BETA_YAW  = 0.3516

vz_f = alpha_beta_filter(
    vz_b_full,
    ALPHA_Z,
    BETA_Z,
    dt
)

r_f = alpha_beta_filter(
    r_b_full,
    ALPHA_YAW,
    BETA_YAW,
    dt
)


alphas = np.linspace(0.02, 0.4, 8)
betas  = np.linspace(0.02, 0.8, 8)

best_score = -1e9
best_data = None

total = len(alphas)**4
counter = 0

print("\nBuscando filtro óptimo...\n")

for ax in alphas:
    for bx in betas:
        for ay in alphas:
            for by in betas:

                counter += 1

                print(
                    f"\r[{counter}/{total}] "
                    f"ax={ax:.3f} "
                    f"bx={bx:.3f} "
                    f"ay={ay:.3f} "
                    f"by={by:.3f}",
                    end=""
                )


                vx_f = alpha_beta_filter(
                    vx_body_raw,
                    ax,
                    bx,
                    dt
                )

                vy_f = alpha_beta_filter(
                    vy_body_raw,
                    ay,
                    by,
                    dt
                )


                delay = DELAY

                u = u_full[:-delay, :]

                vx = vx_f[delay:]
                vy = vy_f[delay:]

                vz = vz_f[delay:]
                r  = r_f[delay:]

                psi = psi_full[delay:]

                x_real = x_real_full[delay:]
                y_real = y_real_full[delay:]
                z_real = z_real_full[delay:]

                ux, uy, uz, upsi = (
                    u[:,0],
                    u[:,1],
                    u[:,2],
                    u[:,3]
                )

                N = len(ux)


                theta_x, theta_y = identify_coupled(
                    vx,
                    vy,
                    r,
                    ux,
                    uy,
                    upsi,
                    MIN_INPUT
                )

                alpha_z, beta_z = identify_decoupled(
                    vz,
                    uz,
                    MIN_INPUT
                )

                alpha_r, beta_r = identify_decoupled(
                    r,
                    upsi,
                    MIN_INPUT
                )

                Ad = np.array([

                    [
                        theta_x[0],
                        theta_x[1],
                        0.0,
                        theta_x[2]
                    ],

                    [
                        theta_y[0],
                        theta_y[1],
                        0.0,
                        theta_y[2]
                    ],

                    [
                        0.0,
                        0.0,
                        alpha_z,
                        0.0
                    ],

                    [
                        0.0,
                        0.0,
                        0.0,
                        alpha_r
                    ],
                ])

                Bd = np.array([

                    [
                        theta_x[3],
                        theta_x[4],
                        0.0,
                        theta_x[5]
                    ],

                    [
                        theta_y[3],
                        theta_y[4],
                        0.0,
                        theta_y[5]
                    ],

                    [
                        0.0,
                        0.0,
                        beta_z,
                        0.0
                    ],

                    [
                        0.0,
                        0.0,
                        0.0,
                        beta_r
                    ],
                ])


                X_sim = np.zeros((4, N))

                X_sim[:,0] = [
                    vx[0],
                    vy[0],
                    vz[0],
                    r[0]
                ]

                U = np.vstack([
                    ux,
                    uy,
                    uz,
                    upsi
                ])

                for k in range(N - 1):

                    X_sim[:,k+1] = (
                        Ad @ X_sim[:,k]
                        +
                        Bd @ U[:,k]
                    )

                vx_hat, vy_hat, vz_hat, r_hat = X_sim


                x_sim = np.zeros(N)
                y_sim = np.zeros(N)
                z_sim = np.zeros(N)
                psi_sim = np.zeros(N)

                x_sim[0] = x_real[0]
                y_sim[0] = y_real[0]
                z_sim[0] = z_real[0]
                psi_sim[0] = psi[0]

                for k in range(N - 1):

                    sk = np.array([
                        x_sim[k],
                        y_sim[k],
                        z_sim[k],
                        psi_sim[k]
                    ])

                    v_k = np.array([
                        vx_hat[k],
                        vy_hat[k],
                        vz_hat[k],
                        r_hat[k]
                    ])

                    v_k1 = np.array([
                        vx_hat[k+1],
                        vy_hat[k+1],
                        vz_hat[k+1],
                        r_hat[k+1]
                    ])

                    v_m = 0.5 * (v_k + v_k1)

                    k1 = kinematics(sk, v_k)

                    k2 = kinematics(
                        sk + 0.5 * dt * k1,
                        v_m
                    )

                    k3 = kinematics(
                        sk + 0.5 * dt * k2,
                        v_m
                    )

                    k4 = kinematics(
                        sk + dt * k3,
                        v_k1
                    )

                    sk1 = sk + (
                        dt / 6.0
                    ) * (
                        k1
                        + 2*k2
                        + 2*k3
                        + k4
                    )

                    x_sim[k+1] = sk1[0]
                    y_sim[k+1] = sk1[1]
                    z_sim[k+1] = sk1[2]
                    psi_sim[k+1] = sk1[3]


                fit_x = fit_pct(x_real, x_sim)
                fit_y = fit_pct(y_real, y_sim)

                score = fit_x + fit_y

                if score > best_score:

                    best_score = score

                    best_data = {

                        "ax": ax,
                        "bx": bx,

                        "ay": ay,
                        "by": by,

                        "fit_x": fit_x,
                        "fit_y": fit_y,

                        "vx": vx,
                        "vy": vy,

                        "vx_hat": vx_hat,
                        "vy_hat": vy_hat,

                        "x_real": x_real,
                        "y_real": y_real,

                        "x_sim": x_sim,
                        "y_sim": y_sim,

                        "Ad": Ad,
                        "Bd": Bd,
                    }

print("\n")
print("\n================================================")
print("MEJOR FILTRO ENCONTRADO")
print("================================================")

print(f"ALPHA VX : {best_data['ax']:.6f}")
print(f"BETA  VX : {best_data['bx']:.6f}")

print(f"ALPHA VY : {best_data['ay']:.6f}")
print(f"BETA  VY : {best_data['by']:.6f}")

print("\n================================================")
print("FIT")
print("================================================")

print(f"FIT X : {best_data['fit_x']:.2f}%")
print(f"FIT Y : {best_data['fit_y']:.2f}%")

print("\n================================================")
print("Ad")
print("================================================")
print(best_data["Ad"])

print("\n================================================")
print("Bd")
print("================================================")
print(best_data["Bd"])


fig, axs = plt.subplots(2, 2, figsize=(14, 8))

axs[0,0].plot(
    best_data["vx"],
    label="VX filtrada"
)

axs[0,0].plot(
    best_data["vx_hat"],
    '--',
    label="VX modelo"
)

axs[0,0].set_title("VX")
axs[0,0].grid(True)
axs[0,0].legend()

axs[0,1].plot(
    best_data["vy"],
    label="VY filtrada"
)

axs[0,1].plot(
    best_data["vy_hat"],
    '--',
    label="VY modelo"
)

axs[0,1].set_title("VY")
axs[0,1].grid(True)
axs[0,1].legend()

axs[1,0].plot(
    best_data["x_real"],
    label="X real"
)

axs[1,0].plot(
    best_data["x_sim"],
    '--',
    label="X sim"
)

axs[1,0].set_title(
    f"X FIT={best_data['fit_x']:.2f}%"
)

axs[1,0].grid(True)
axs[1,0].legend()

axs[1,1].plot(
    best_data["y_real"],
    label="Y real"
)

axs[1,1].plot(
    best_data["y_sim"],
    '--',
    label="Y sim"
)

axs[1,1].set_title(
    f"Y FIT={best_data['fit_y']:.2f}%"
)

axs[1,1].grid(True)
axs[1,1].legend()

plt.tight_layout()
plt.show()