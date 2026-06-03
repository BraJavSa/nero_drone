#!/usr/bin/env python3

# Identifies diagonal dynamic matrices (Ad, Bd) using Least Squares over flight data.
"""
Identificación dinámica Bebop — Modelo DIAGONAL.
Filtros Alpha-Beta fijos para Z y YAW.
X e Y sin filtrar.
Ad y Bd estrictamente diagonales: cada canal solo depende de sí mismo.

IMPORTANTE:
- El yaw real se desenvuelve con np.unwrap()
- El yaw simulado NO se limita a [-pi, pi]
- El yaw simulado sigue creciendo continuamente
"""

import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt


mat_file = "manual_log_20260518_172721.mat"
data     = loadmat(mat_file)

hz = float(data["hz"].squeeze())
dt = 1.0 / hz

DELAY     = 2
MIN_INPUT = 0.0

u_full    = data["u"]

vx_b_full = data["vx_b"].squeeze()
vy_b_full = data["vy_b"].squeeze()
vz_b_full = data["vz_b"].squeeze()

# ── IMPORTANTE: unwrap del yaw real ──────────────────────────────────────────
psi_full = np.unwrap(data["psi"].squeeze())

r_b_full = data["vpsi"].squeeze()

x_real_full = data["x_i"].squeeze()
y_real_full = data["y_i"].squeeze()
z_real_full = data["z_i"].squeeze()


# ── Alpha-Beta filter ────────────────────────────────────────────────────────

def alpha_beta_filter(measurements, alpha, beta, dt):
    n = len(measurements)

    x_est = measurements[0]
    v_est = 0.0

    out = np.empty(n)

    for k in range(n):
        x_pred  = x_est + v_est * dt
        residuo = measurements[k] - x_pred

        x_est = x_pred + alpha * residuo
        v_est = v_est  + (beta / dt) * residuo

        out[k] = x_est

    return out


# ── Filter parameters ────────────────────────────────────────────────────────

ALPHA_Z   = 0.0387
BETA_Z    = 0.4066

ALPHA_YAW = 0.2471
BETA_YAW  = 0.3516

vx_f = vx_b_full.copy()
vy_f = vy_b_full.copy()

vz_f = alpha_beta_filter(vz_b_full, ALPHA_Z,   BETA_Z,   dt)
r_f  = alpha_beta_filter(r_b_full,  ALPHA_YAW, BETA_YAW, dt)


# ── Identification helpers ───────────────────────────────────────────────────

def identify_decoupled(v, u_in, min_input):

    """
    v[k+1] = alpha * v[k] + beta * u[k]
    """

    y = v[1:]

    mask = np.abs(u_in[:-1]) > min_input

    Phi = np.column_stack([
        v[:-1][mask],
        u_in[:-1][mask]
    ])

    theta, *_ = np.linalg.lstsq(Phi, y[mask], rcond=None)

    return float(theta[0]), float(theta[1])


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


# ── Apply delay and slice ────────────────────────────────────────────────────

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
    u[:, 0],
    u[:, 1],
    u[:, 2],
    u[:, 3]
)

N = len(ux)


# ── Diagonal identification ──────────────────────────────────────────────────

alpha_x, beta_x = identify_decoupled(vx, ux,   MIN_INPUT)
alpha_y, beta_y = identify_decoupled(vy, uy,   MIN_INPUT)
alpha_z, beta_z = identify_decoupled(vz, uz,   MIN_INPUT)
alpha_r, beta_r = identify_decoupled(r,  upsi, MIN_INPUT)


# ── Build diagonal Ad and Bd ─────────────────────────────────────────────────

Ad = np.array([
    [alpha_x, 0.0,     0.0,     0.0],
    [0.0,     alpha_y, 0.0,     0.0],
    [0.0,     0.0,     alpha_z, 0.0],
    [0.0,     0.0,     0.0,     alpha_r],
])

Bd = np.array([
    [beta_x, 0.0,    0.0,    0.0],
    [0.0,    beta_y, 0.0,    0.0],
    [0.0,    0.0,    beta_z, 0.0],
    [0.0,    0.0,    0.0,    beta_r],
])


# ── Velocity simulation ──────────────────────────────────────────────────────

X_sim = np.zeros((4, N))

X_sim[:, 0] = [
    vx[0],
    vy[0],
    vz[0],
    r[0]
]

U = np.vstack([ux, uy, uz, upsi])

for k in range(N - 1):

    X_sim[:, k + 1] = (
        Ad @ X_sim[:, k]
        + Bd @ U[:, k]
    )

vx_hat, vy_hat, vz_hat, r_hat = X_sim


# ── Position simulation (RK4) ────────────────────────────────────────────────

x_sim = np.zeros(N)
y_sim = np.zeros(N)
z_sim = np.zeros(N)

# IMPORTANTE:
# yaw continuo SIN wrap
psi_sim = np.zeros(N)

x_sim[0]   = x_real[0]
y_sim[0]   = y_real[0]
z_sim[0]   = z_real[0]
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
        vx_hat[k + 1],
        vy_hat[k + 1],
        vz_hat[k + 1],
        r_hat[k + 1]
    ])

    v_m = 0.5 * (v_k + v_k1)

    k1 = kinematics(sk,              v_k)
    k2 = kinematics(sk + 0.5*dt*k1, v_m)
    k3 = kinematics(sk + 0.5*dt*k2, v_m)
    k4 = kinematics(sk + dt*k3,     v_k1)

    sk1 = sk + (dt / 6.0) * (
        k1 + 2*k2 + 2*k3 + k4
    )

    x_sim[k + 1]   = sk1[0]
    y_sim[k + 1]   = sk1[1]
    z_sim[k + 1]   = sk1[2]

    # yaw acumulativo continuo
    psi_sim[k + 1] = sk1[3]


t = np.arange(N) * dt


# ── Print results ────────────────────────────────────────────────────────────

np.set_printoptions(
    formatter={'float': lambda x: f"{x:0.6f}"}
)

print("\n================================================")
print("FILTROS ALPHA-BETA APLICADOS")
print("================================================")
print(f"  X   -> sin filtro")
print(f"  Y   -> sin filtro")
print(f"  Z   -> Alpha: {ALPHA_Z:.4f},  Beta: {BETA_Z:.4f}")
print(f"  YAW -> Alpha: {ALPHA_YAW:.4f},  Beta: {BETA_YAW:.4f}")

print("\n================================================")
print("MATRIZ A DISCRETA (Ad)  [DIAGONAL]")
print("================================================")
print(Ad)

print(f"\n  alpha_x : {alpha_x:+.6f}")
print(f"  alpha_y : {alpha_y:+.6f}")
print(f"  alpha_z : {alpha_z:+.6f}")
print(f"  alpha_r : {alpha_r:+.6f}")

print("\n================================================")
print("MATRIZ B DISCRETA (Bd)  [DIAGONAL]")
print("================================================")
print(Bd)

print("\n  Columnas: [ux, uy, uz, upsi]")
print("  Filas   : [vx, vy, vz, r]")

print(f"\n  vx <- ux   : {beta_x:+.6f}")
print(f"  vy <- uy   : {beta_y:+.6f}")
print(f"  vz <- uz   : {beta_z:+.6f}")
print(f"  r  <- upsi : {beta_r:+.6f}")

print("\n================================================")
print("VELOCITY FIT")
print("================================================")

print(f"  FIT VX  : {fit_pct(vx, vx_hat):.2f}%")
print(f"  FIT VY  : {fit_pct(vy, vy_hat):.2f}%")
print(f"  FIT VZ  : {fit_pct(vz, vz_hat):.2f}%")
print(f"  FIT YAW : {fit_pct(r,  r_hat):.2f}%")

print("\n================================================")
print("POSITION FIT  &  RMS")
print("================================================")

print(f"  FIT X   : {fit_pct(x_real, x_sim):.2f}%   RMS: {rms(x_real, x_sim):.4f} m")
print(f"  FIT Y   : {fit_pct(y_real, y_sim):.2f}%   RMS: {rms(y_real, y_sim):.4f} m")
print(f"  FIT Z   : {fit_pct(z_real, z_sim):.2f}%   RMS: {rms(z_real, z_sim):.4f} m")
print(f"  FIT PSI : {fit_pct(psi, psi_sim):.2f}%   RMS: {rms(psi, psi_sim):.4f} rad")


# ── Plots ────────────────────────────────────────────────────────────────────

fig, axs = plt.subplots(2, 4, figsize=(18, 9))

fig.suptitle(
    "Identificacion Dinamica Bebop — Modelo DIAGONAL | YAW CONTINUO",
    fontsize=14,
    fontweight='bold'
)

pairs_v = [
    (vx, vx_hat, "VX (BODY)"),
    (vy, vy_hat, "VY (BODY)"),
    (vz, vz_hat, "VZ"),
    (r,  r_hat,  "YAW RATE"),
]

pairs_p = [
    (x_real, x_sim,   "Posicion X",   "m"),
    (y_real, y_sim,   "Posicion Y",   "m"),
    (z_real, z_sim,   "Posicion Z",   "m"),
    (psi,    psi_sim, "Posicion YAW", "rad"),
]

for i, (sig, hat, name) in enumerate(pairs_v):

    ax = axs[0, i]

    ax.plot(
        t,
        sig,
        color='blue',
        alpha=0.8,
        label='Medido'
    )

    ax.plot(
        t,
        hat,
        '--',
        color='red',
        label='Modelo'
    )

    ax.set_title(
        f"{name}  FIT: {fit_pct(sig, hat):.2f}%"
    )

    ax.set_xlabel("t [s]")
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=8)


for i, (real, sim, name, unit) in enumerate(pairs_p):

    ax = axs[1, i]

    ax.plot(
        t,
        real,
        color='gray',
        alpha=0.6,
        label='Real'
    )

    ax.plot(
        t,
        sim,
        '--',
        color='blue',
        label='Simulado'
    )

    ax.set_title(
        f"{name}  FIT: {fit_pct(real, sim):.2f}%"
        f"  RMS: {rms(real, sim):.4f} {unit}"
    )

    ax.set_xlabel("t [s]")
    ax.grid(True)
    ax.legend(loc='upper right', fontsize=8)

plt.tight_layout()
plt.show()