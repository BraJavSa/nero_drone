#!/usr/bin/env python3

import numpy as np
from scipy.io import loadmat

mat_file = "manual_log_20260517_004407.mat"
data = loadmat(mat_file)
hz = float(data["hz"].squeeze())
dt = 1.0 / hz

DELAY = 2
MIN_INPUT = 0.0

u_full   = data["u"]
vx_full  = data["vx_b"].squeeze()   # ya en marco del cuerpo
vy_full  = data["vy_b"].squeeze()   # ya en marco del cuerpo
vz_full  = data["vz_b"].squeeze()
psi_full = data["psi"].squeeze()
r_full   = data["vpsi"].squeeze()

x_real_full = data["x_i"].squeeze()
y_real_full = data["y_i"].squeeze()
z_real_full = data["z_i"].squeeze()

if DELAY > 0:
    u      = u_full[:-DELAY, :]
    vx     = vx_full[DELAY:]
    vy     = vy_full[DELAY:]
    vz     = vz_full[DELAY:]
    psi    = psi_full[DELAY:]
    r      = r_full[DELAY:]
    x_real = x_real_full[DELAY:]
    y_real = y_real_full[DELAY:]
    z_real = z_real_full[DELAY:]
else:
    u      = u_full
    vx     = vx_full
    vy     = vy_full
    vz     = vz_full
    psi    = psi_full
    r      = r_full
    x_real = x_real_full
    y_real = y_real_full
    z_real = z_real_full

ux   = u[:, 0]
uy   = u[:, 1]
uz   = u[:, 2]
upsi = u[:, 3]

N = len(ux)


# ─── IDENTIFICATION FUNCTIONS ───────────────────────────────────────────────

def identify_coupled(vx, vy, r, ux, uy, min_input):
    """
    Identifies coupled XY dynamics:
      vx[k+1] = ax*vx[k] + axy*vy[k] + ar_x*r[k] + bx*ux[k]
      vy[k+1] = ayx*vx[k] + ay*vy[k] + ar_y*r[k] + by*uy[k]
    """
    y_x   = vx[1:]
    Phi_x = np.column_stack([vx[:-1], vy[:-1], r[:-1], ux[:-1]])

    y_y   = vy[1:]
    Phi_y = np.column_stack([vx[:-1], vy[:-1], r[:-1], uy[:-1]])

    mask  = (np.abs(ux[:-1]) > min_input) | (np.abs(uy[:-1]) > min_input)

    theta_x, *_ = np.linalg.lstsq(Phi_x[mask], y_x[mask], rcond=None)
    theta_y, *_ = np.linalg.lstsq(Phi_y[mask], y_y[mask], rcond=None)
    return theta_x, theta_y


def identify_decoupled(v, u_in, min_input):
    """
    Identifies single-axis dynamics:
      v[k+1] = alpha*v[k] + beta*u[k]
    """
    y    = v[1:]
    vk   = v[:-1]
    uk   = u_in[:-1]
    mask = np.abs(uk) > min_input
    Phi  = np.column_stack([vk[mask], uk[mask]])
    theta, *_ = np.linalg.lstsq(Phi, y[mask], rcond=None)
    return theta[0], theta[1]


def fit(y, yhat):
    var_y = np.linalg.norm(y - np.mean(y))
    if var_y == 0:
        return 0.0
    return 100.0 * (1.0 - np.linalg.norm(y - yhat) / var_y)


def kinematics(state, v_body):
    psi_k = state[3]
    vx_b, vy_b, vz_b, r_b = v_body
    xdot   = np.cos(psi_k) * vx_b - np.sin(psi_k) * vy_b
    ydot   = np.sin(psi_k) * vx_b + np.cos(psi_k) * vy_b
    zdot   = vz_b
    psidot = r_b
    return np.array([xdot, ydot, zdot, psidot])


# ─── PARAMETER IDENTIFICATION ────────────────────────────────────────────────

theta_x, theta_y = identify_coupled(vx, vy, r, ux, uy, MIN_INPUT)
alpha_z, beta_z  = identify_decoupled(vz, uz, MIN_INPUT)
alpha_r, beta_r  = identify_decoupled(r,  upsi, MIN_INPUT)

Ad = np.array([
    [theta_x[0], theta_x[1], 0.0,     theta_x[2]],
    [theta_y[0], theta_y[1], 0.0,     theta_y[2]],
    [0.0,        0.0,        alpha_z, 0.0        ],
    [0.0,        0.0,        0.0,     alpha_r    ]
])

Bd = np.array([
    [theta_x[3], 0.0,        0.0,    0.0    ],
    [0.0,        theta_y[3], 0.0,    0.0    ],
    [0.0,        0.0,        beta_z, 0.0    ],
    [0.0,        0.0,        0.0,    beta_r ]
])

np.set_printoptions(formatter={'float': lambda x: f"{x:0.6f}"})

print("\n================================================")
print("MATRIZ A DISCRETA (Ad)  — sin filtrado")
print("================================================")
print(Ad)

print("\n================================================")
print("MATRIZ B DISCRETA (Bd)  — sin filtrado")
print("================================================")
print(Bd)


# ─── VELOCITY SIMULATION ─────────────────────────────────────────────────────

X_sim = np.zeros((4, N))
X_sim[0, 0] = vx[0]
X_sim[1, 0] = vy[0]
X_sim[2, 0] = vz[0]
X_sim[3, 0] = r[0]

U = np.vstack([ux, uy, uz, upsi])

for k in range(N - 1):
    X_sim[:, k + 1] = Ad @ X_sim[:, k] + Bd @ U[:, k]

vx_hat = X_sim[0, :]
vy_hat = X_sim[1, :]
vz_hat = X_sim[2, :]
r_hat  = X_sim[3, :]

print("\n================================================")
print("VELOCITY FIT")
print("================================================")
print(f"FIT VX  : {fit(vx, vx_hat):.2f}%")
print(f"FIT VY  : {fit(vy, vy_hat):.2f}%")
print(f"FIT VZ  : {fit(vz, vz_hat):.2f}%")
print(f"FIT YAW : {fit(r,  r_hat ):.2f}%")


# ─── POSITION SIMULATION (RK4 + kinematics) ──────────────────────────────────

x_sim   = np.zeros(N)
y_sim   = np.zeros(N)
z_sim   = np.zeros(N)
psi_sim = np.zeros(N)

x_sim[0]   = x_real[0]
y_sim[0]   = y_real[0]
z_sim[0]   = z_real[0]
psi_sim[0] = psi[0]

for k in range(N - 1):
    state_k = np.array([x_sim[k], y_sim[k], z_sim[k], psi_sim[k]])

    v_k   = np.array([vx_hat[k],     vy_hat[k],     vz_hat[k],     r_hat[k]    ])
    v_kp1 = np.array([vx_hat[k + 1], vy_hat[k + 1], vz_hat[k + 1], r_hat[k + 1]])
    v_mid = 0.5 * (v_k + v_kp1)

    k1 = kinematics(state_k,                   v_k  )
    k2 = kinematics(state_k + 0.5 * dt * k1,  v_mid)
    k3 = kinematics(state_k + 0.5 * dt * k2,  v_mid)
    k4 = kinematics(state_k + dt * k3,         v_kp1)

    state_kp1 = state_k + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    x_sim[k + 1]   = state_kp1[0]
    y_sim[k + 1]   = state_kp1[1]
    z_sim[k + 1]   = state_kp1[2]
    psi_sim[k + 1] = state_kp1[3]

print("\n================================================")
print("POSITION FIT")
print("================================================")
print(f"FIT X   : {fit(x_real, x_sim):.2f}%")
print(f"FIT Y   : {fit(y_real, y_sim):.2f}%")
print(f"FIT Z   : {fit(z_real, z_sim):.2f}%")
print(f"FIT PSI : {fit(psi,    psi_sim):.2f}%")

# ─── RECONSTRUCT RAW VELOCITIES FOR PLOTTING ─────────────────────────────────
w_vx_raw = np.zeros_like(x_real_full)
w_vy_raw = np.zeros_like(y_real_full)
w_vx_raw[1:] = np.diff(x_real_full) / dt
w_vy_raw[1:] = np.diff(y_real_full) / dt

cos_psi = np.cos(psi_full)
sin_psi = np.sin(psi_full)

vx_raw_full = cos_psi * w_vx_raw + sin_psi * w_vy_raw
vy_raw_full = -sin_psi * w_vx_raw + cos_psi * w_vy_raw

vz_raw_full = np.zeros_like(z_real_full)
vz_raw_full[1:] = np.diff(z_real_full) / dt

dpsi = np.zeros_like(psi_full)
for i in range(1, len(psi_full)):
    d = psi_full[i] - psi_full[i-1]
    while d > np.pi:
        d -= 2.0 * np.pi
    while d < -np.pi:
        d += 2.0 * np.pi
    dpsi[i] = d / dt
r_raw_full = dpsi

if DELAY > 0:
    vx_raw = vx_raw_full[DELAY:]
    vy_raw = vy_raw_full[DELAY:]
    vz_raw = vz_raw_full[DELAY:]
    r_raw  = r_raw_full[DELAY:]
else:
    vx_raw = vx_raw_full
    vy_raw = vy_raw_full
    vz_raw = vz_raw_full
    r_raw  = r_raw_full

# ─── PLOTTING ────────────────────────────────────────────────────────────────
import matplotlib.pyplot as plt

# Filter parameters for labels
OPT_ALPHA_X, OPT_BETA_X = 0.9900, 0.0001
OPT_ALPHA_Y, OPT_BETA_Y = 0.8963, 0.2252
OPT_ALPHA_Z, OPT_BETA_Z = 0.3677, 0.2694
OPT_ALPHA_R, OPT_BETA_R = 0.3960, 0.3000

t = np.arange(N) * dt

fig, axs = plt.subplots(2, 4, figsize=(18, 9))
fig.suptitle("Identificación Dinámica Bebop: Velocidades del Cuerpo Grabadas y Filtro en Tiempo Real", fontsize=15, fontweight='bold')

# VX Subplot
axs[0, 0].plot(t, vx_raw, label='Medido (raw)', color='lightblue', alpha=0.5)
axs[0, 0].plot(t, vx, label=f'Filtrado (a={OPT_ALPHA_X:.2f}, b={OPT_BETA_X:.3f})', color='blue', alpha=0.8)
axs[0, 0].plot(t, vx_hat, '--', label='Modelo', color='red')
axs[0, 0].set_title(f"VX (BODY) - FIT: {fit(vx, vx_hat):.2f}%")
axs[0, 0].legend(loc='upper right')
axs[0, 0].grid(True)

# VY Subplot
axs[0, 1].plot(t, vy_raw, label='Medido (raw)', color='lightblue', alpha=0.5)
axs[0, 1].plot(t, vy, label=f'Filtrado (a={OPT_ALPHA_Y:.2f}, b={OPT_BETA_Y:.3f})', color='blue', alpha=0.8)
axs[0, 1].plot(t, vy_hat, '--', label='Modelo', color='red')
axs[0, 1].set_title(f"VY (BODY) - FIT: {fit(vy, vy_hat):.2f}%")
axs[0, 1].legend(loc='upper right')
axs[0, 1].grid(True)

# VZ Subplot
axs[0, 2].plot(t, vz_raw, label='Medido (raw)', color='lightblue', alpha=0.5)
axs[0, 2].plot(t, vz, label=f'Filtrado (a={OPT_ALPHA_Z:.2f}, b={OPT_BETA_Z:.3f})', color='blue', alpha=0.8)
axs[0, 2].plot(t, vz_hat, '--', label='Modelo', color='red')
axs[0, 2].set_title(f"VZ - FIT: {fit(vz, vz_hat):.2f}%")
axs[0, 2].legend(loc='upper right')
axs[0, 2].grid(True)

# YAW RATE Subplot
axs[0, 3].plot(t, r_raw, label='Medido (raw)', color='lightblue', alpha=0.5)
axs[0, 3].plot(t, r, label=f'Filtrado (a={OPT_ALPHA_R:.2f}, b={OPT_BETA_R:.3f})', color='blue', alpha=0.8)
axs[0, 3].plot(t, r_hat, '--', label='Modelo', color='red')
axs[0, 3].set_title(f"YAW RATE - FIT: {fit(r, r_hat):.2f}%")
axs[0, 3].legend(loc='upper right')
axs[0, 3].grid(True)

# X Position Subplot
axs[1, 0].plot(t, x_real, label='Real', color='gray', alpha=0.6)
axs[1, 0].plot(t, x_sim, '--', label='Simulado', color='blue')
axs[1, 0].set_title(f"Posición X - FIT: {fit(x_real, x_sim):.2f}%")
axs[1, 0].legend(loc='upper right')
axs[1, 0].grid(True)

# Y Position Subplot
axs[1, 1].plot(t, y_real, label='Real', color='gray', alpha=0.6)
axs[1, 1].plot(t, y_sim, '--', label='Simulado', color='blue')
axs[1, 1].set_title(f"Posición Y - FIT: {fit(y_real, y_sim):.2f}%")
axs[1, 1].legend(loc='upper right')
axs[1, 1].grid(True)

# Z Position Subplot
axs[1, 2].plot(t, z_real, label='Real', color='gray', alpha=0.6)
axs[1, 2].plot(t, z_sim, '--', label='Simulado', color='blue')
axs[1, 2].set_title(f"Posición Z - FIT: {fit(z_real, z_sim):.2f}%")
axs[1, 2].legend(loc='upper right')
axs[1, 2].grid(True)

# YAW Position Subplot
axs[1, 3].plot(t, psi, label='Real', color='gray', alpha=0.6)
axs[1, 3].plot(t, psi_sim, '--', label='Simulado', color='blue')
axs[1, 3].set_title(f"Posición YAW - FIT: {fit(psi, psi_sim):.2f}%")
axs[1, 3].legend(loc='upper right')
axs[1, 3].grid(True)

plt.tight_layout()
plt.show()