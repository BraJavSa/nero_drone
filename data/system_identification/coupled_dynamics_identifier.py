#!/usr/bin/env python3

# Identifies coupled dynamic matrices (Ad, Bd) using Least Squares over flight data.
"""
Identificación dinámica Bebop.
Filtros Alpha-Beta fijos para Z y YAW.
X e Y sin filtrar.
Bd ACOPLADO: ux, uy y upsi influyen en vx y vy.
"""

import numpy as np
from scipy.io import loadmat


mat_file = "manual_log_20260518_172721.mat"
data     = loadmat(mat_file)
hz       = float(data["hz"].squeeze())
dt       = 1.0 / hz

DELAY     = 2
MIN_INPUT = 0.0

u_full      = data["u"]
vx_b_full   = data["vx_b"].squeeze()
vy_b_full   = data["vy_b"].squeeze()
vz_b_full   = data["vz_b"].squeeze()
psi_full    = data["psi"].squeeze()
r_b_full    = data["vpsi"].squeeze()

x_real_full = data["x_i"].squeeze()
y_real_full = data["y_i"].squeeze()
z_real_full = data["z_i"].squeeze()



def alpha_beta_filter(measurements, alpha, beta, dt):
    n     = len(measurements)
    x_est = measurements[0]
    v_est = 0.0
    out   = np.empty(n)
    for k in range(n):
        x_pred  = x_est + v_est * dt
        residuo = measurements[k] - x_pred
        x_est   = x_pred + alpha * residuo
        v_est   = v_est  + (beta / dt) * residuo
        out[k]  = x_est
    return out



ALPHA_Z   = 0.0387;  BETA_Z   = 0.4066
ALPHA_YAW = 0.2471;  BETA_YAW = 0.3516

vx_f = vx_b_full.copy()
vy_f = vy_b_full.copy()
vz_f = alpha_beta_filter(vz_b_full, ALPHA_Z,   BETA_Z,   dt)
r_f  = alpha_beta_filter(r_b_full,  ALPHA_YAW, BETA_YAW, dt)



def identify_coupled(vx, vy, r, ux, uy, upsi, min_input):
    """
    Bd acoplado: cada ecuacion (vx, vy) usa las tres entradas (ux, uy, upsi).

    vx[k+1] = θx1·vx[k] + θx2·vy[k] + θx3·r[k]
             + θx4·ux[k] + θx5·uy[k] + θx6·upsi[k]

    vy[k+1] = θy1·vx[k] + θy2·vy[k] + θy3·r[k]
             + θy4·ux[k] + θy5·uy[k] + θy6·upsi[k]
    """
    Phi = np.column_stack([
        vx[:-1], vy[:-1], r[:-1],
        ux[:-1], uy[:-1], upsi[:-1],
    ])

    mask = (np.abs(ux[:-1]) > min_input) | (np.abs(uy[:-1]) > min_input)

    theta_x, *_ = np.linalg.lstsq(Phi[mask], vx[1:][mask], rcond=None)
    theta_y, *_ = np.linalg.lstsq(Phi[mask], vy[1:][mask], rcond=None)

    return theta_x, theta_y


def identify_decoupled(v, u_in, min_input):
    y    = v[1:]
    mask = np.abs(u_in[:-1]) > min_input
    Phi  = np.column_stack([v[:-1][mask], u_in[:-1][mask]])
    theta, *_ = np.linalg.lstsq(Phi, y[mask], rcond=None)
    return theta[0], theta[1]


def kinematics(state, v_body):
    psi_k = state[3]
    vx_b, vy_b, vz_b, r_b = v_body
    return np.array([
        np.cos(psi_k) * vx_b - np.sin(psi_k) * vy_b,
        np.sin(psi_k) * vx_b + np.cos(psi_k) * vy_b,
        vz_b,
        r_b,
    ])


def rms(a, b):
    return np.sqrt(np.mean((a - b) ** 2))


def fit_pct(y, yhat):
    var_y = np.linalg.norm(y - np.mean(y))
    if var_y == 0:
        return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y - yhat) / var_y)



delay = DELAY
u      = u_full[:-delay, :]
vx = vx_f[delay:]; vy = vy_f[delay:]
vz = vz_f[delay:]; r  = r_f[delay:]
psi    = psi_full[delay:]
x_real = x_real_full[delay:]
y_real = y_real_full[delay:]
z_real = z_real_full[delay:]

ux, uy, uz, upsi = u[:, 0], u[:, 1], u[:, 2], u[:, 3]
N = len(ux)

theta_x, theta_y = identify_coupled(vx, vy, r, ux, uy, upsi, MIN_INPUT)
alpha_z, beta_z  = identify_decoupled(vz, uz,   MIN_INPUT)
alpha_r, beta_r  = identify_decoupled(r,  upsi, MIN_INPUT)


Ad = np.array([
    [theta_x[0], theta_x[1], 0.0,     theta_x[2]],
    [theta_y[0], theta_y[1], 0.0,     theta_y[2]],
    [0.0,        0.0,        alpha_z, 0.0        ],
    [0.0,        0.0,        0.0,     alpha_r    ],
])

Bd = np.array([
    [theta_x[3], theta_x[4], 0.0,    theta_x[5]],
    [theta_y[3], theta_y[4], 0.0,    theta_y[5]],
    [0.0,        0.0,        beta_z, 0.0        ],
    [0.0,        0.0,        0.0,    beta_r     ],
])

X_sim       = np.zeros((4, N))
X_sim[:, 0] = [vx[0], vy[0], vz[0], r[0]]
U           = np.vstack([ux, uy, uz, upsi])
for k in range(N - 1):
    X_sim[:, k + 1] = Ad @ X_sim[:, k] + Bd @ U[:, k]

vx_hat, vy_hat, vz_hat, r_hat = X_sim

x_sim   = np.zeros(N); x_sim[0]   = x_real[0]
y_sim   = np.zeros(N); y_sim[0]   = y_real[0]
z_sim   = np.zeros(N); z_sim[0]   = z_real[0]
psi_sim = np.zeros(N); psi_sim[0] = psi[0]

for k in range(N - 1):
    sk   = np.array([x_sim[k], y_sim[k], z_sim[k], psi_sim[k]])
    v_k  = np.array([vx_hat[k],     vy_hat[k],     vz_hat[k],     r_hat[k]])
    v_k1 = np.array([vx_hat[k + 1], vy_hat[k + 1], vz_hat[k + 1], r_hat[k + 1]])
    v_m  = 0.5 * (v_k + v_k1)
    k1   = kinematics(sk,              v_k)
    k2   = kinematics(sk + 0.5*dt*k1, v_m)
    k3   = kinematics(sk + 0.5*dt*k2, v_m)
    k4   = kinematics(sk + dt*k3,     v_k1)
    sk1  = sk + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    x_sim[k+1]   = sk1[0]
    y_sim[k+1]   = sk1[1]
    z_sim[k+1]   = sk1[2]
    psi_sim[k+1] = sk1[3]

t = np.arange(N) * dt

np.set_printoptions(formatter={'float': lambda x: f"{x:0.6f}"})

print("\n================================================")
print("FILTROS ALPHA-BETA APLICADOS")
print("================================================")
print(f"  X   -> sin filtro")
print(f"  Y   -> sin filtro")
print(f"  Z   -> Alpha: {ALPHA_Z:.4f},  Beta: {BETA_Z:.4f}")
print(f"  YAW -> Alpha: {ALPHA_YAW:.4f},  Beta: {BETA_YAW:.4f}")

print("\n================================================")
print("MATRIZ A DISCRETA (Ad)")
print("================================================")
print(Ad)

print("\n================================================")
print("MATRIZ B DISCRETA (Bd)  [Bd ACOPLADO XY]")
print("================================================")
print(Bd)
print("\n  Columnas: [ux,  uy,  uz,  upsi]")
print("  Filas   : [vx,  vy,  vz,  r   ]")
print(f"\n  vx <- ux:{theta_x[3]:+.6f}  uy:{theta_x[4]:+.6f}  upsi:{theta_x[5]:+.6f}")
print(f"  vy <- ux:{theta_y[3]:+.6f}  uy:{theta_y[4]:+.6f}  upsi:{theta_y[5]:+.6f}")

print("\n================================================")
print("VELOCITY FIT")
print("================================================")
print(f"  FIT VX  : {fit_pct(vx,  vx_hat):.2f}%")
print(f"  FIT VY  : {fit_pct(vy,  vy_hat):.2f}%")
print(f"  FIT VZ  : {fit_pct(vz,  vz_hat):.2f}%")
print(f"  FIT YAW : {fit_pct(r,   r_hat ):.2f}%")

print("\n================================================")
print("POSITION FIT  &  RMS")
print("================================================")
print(f"  FIT X   : {fit_pct(x_real, x_sim):.2f}%   RMS: {rms(x_real, x_sim):.4f} m")
print(f"  FIT Y   : {fit_pct(y_real, y_sim):.2f}%   RMS: {rms(y_real, y_sim):.4f} m")
print(f"  FIT Z   : {fit_pct(z_real, z_sim):.2f}%   RMS: {rms(z_real, z_sim):.4f} m")
print(f"  FIT PSI : {fit_pct(psi,  psi_sim):.2f}%   RMS: {rms(psi,  psi_sim):.4f} rad")


import matplotlib.pyplot as plt

fig, axs = plt.subplots(2, 4, figsize=(18, 9))
fig.suptitle(
    "Identificacion Dinamica Bebop — Bd ACOPLADO XY  |  Z y YAW filtrados",
    fontsize=14, fontweight='bold'
)

pairs_v = [
    (vx,  vx_hat, "VX (BODY)"),
    (vy,  vy_hat, "VY (BODY)"),
    (vz,  vz_hat, "VZ"),
    (r,   r_hat,  "YAW RATE"),
]
pairs_p = [
    (x_real, x_sim,   "Posicion X",   "m"),
    (y_real, y_sim,   "Posicion Y",   "m"),
    (z_real, z_sim,   "Posicion Z",   "m"),
    (psi,    psi_sim, "Posicion YAW", "rad"),
]

for i, (sig, hat, name) in enumerate(pairs_v):
    ax = axs[0, i]
    ax.plot(t, sig, color='blue',  alpha=0.8, label='Medido')
    ax.plot(t, hat, '--', color='red', label='Modelo')
    ax.set_title(f"{name}  FIT: {fit_pct(sig, hat):.2f}%")
    ax.legend(loc='upper right', fontsize=8); ax.grid(True)

for i, (real, sim, name, unit) in enumerate(pairs_p):
    ax = axs[1, i]
    ax.plot(t, real, color='gray', alpha=0.6, label='Real')
    ax.plot(t, sim,  '--', color='blue', label='Simulado')
    ax.set_title(f"{name}  FIT: {fit_pct(real, sim):.2f}%"
                 f"  RMS: {rms(real, sim):.4f} {unit}")
    ax.legend(loc='upper right', fontsize=8); ax.grid(True)

plt.tight_layout()
plt.show()