#!/usr/bin/env python3

# Validates the identified dynamic model against raw, unfiltered telemetry data.
"""
VALIDACION DEL MODELO
=====================

SIN FILTROS

Se valida directamente el modelo usando:

    - velocidades RAW del dataset
    - inputs RAW
    - sin alpha-beta
    - sin suavizado

Dataset:
    manual_log_20260518_001410.mat
"""

import numpy as np
import matplotlib.pyplot as plt

from scipy.io import loadmat


Ad = np.array([

    [ 9.63895373e-01, -6.18407625e-02, 0.0,  1.60376855e-02],

    [-8.14310627e-05,  9.99449376e-01, 0.0,  5.21436812e-03],

    [ 0.0,             0.0,            8.57642038e-01, 0.0],

    [ 0.0,             0.0,            0.0,  8.22489970e-01],
])

Bd = np.array([

    [ 4.68509248e-02,  9.89534630e-03, 0.0, -1.41951368e-02],

    [ 2.04387648e-05,  3.40358601e-02, 0.0, -8.51778748e-03],

    [ 0.0,             0.0,            1.80292391e-01, 0.0],

    [ 0.0,             0.0,            0.0,  1.84677304e-01],
])


DELAY = 2


mat_file = "manual_log_20260518_002252.mat"

data = loadmat(mat_file)

hz = float(data["hz"].squeeze())

dt = 1.0 / hz

u_full = data["u"]

vx_b_full = data["vx_b"].squeeze()
vy_b_full = data["vy_b"].squeeze()
vz_b_full = data["vz_b"].squeeze()

psi_full = data["psi"].squeeze()

r_b_full = data["vpsi"].squeeze()

x_real_full = data["x_i"].squeeze()
y_real_full = data["y_i"].squeeze()
z_real_full = data["z_i"].squeeze()


delay = DELAY

u = u_full[:-delay, :]

vx_real = vx_b_full[delay:]
vy_real = vy_b_full[delay:]
vz_real = vz_b_full[delay:]

r_real = r_b_full[delay:]

psi_real = psi_full[delay:]

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

t = np.arange(N) * dt


X_sim = np.zeros((4, N))

X_sim[:,0] = [
    vx_real[0],
    vy_real[0],
    vz_real[0],
    r_real[0]
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


x_sim = np.zeros(N)
y_sim = np.zeros(N)
z_sim = np.zeros(N)
psi_sim = np.zeros(N)

x_sim[0] = x_real[0]
y_sim[0] = y_real[0]
z_sim[0] = z_real[0]

psi_sim[0] = psi_real[0]

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


def fit_pct(y, yhat):

    var_y = np.linalg.norm(
        y - np.mean(y)
    )

    if var_y == 0:
        return 0.0

    return 100.0 * (
        1.0
        -
        np.linalg.norm(y - yhat) / var_y
    )

def rms(a, b):

    return np.sqrt(
        np.mean((a - b)**2)
    )


print("\n================================================")
print("VALIDACION DEL MODELO")
print("================================================")

print("\n================================================")
print("FIT VELOCIDADES")
print("================================================")

print(f"FIT VX   : {fit_pct(vx_real, vx_hat):.2f}%")
print(f"FIT VY   : {fit_pct(vy_real, vy_hat):.2f}%")
print(f"FIT VZ   : {fit_pct(vz_real, vz_hat):.2f}%")
print(f"FIT YAW  : {fit_pct(r_real,  r_hat ):.2f}%")

print("\n================================================")
print("FIT POSICIONES")
print("================================================")

print(
    f"FIT X    : "
    f"{fit_pct(x_real, x_sim):.2f}%"
    f"    RMS: {rms(x_real, x_sim):.4f} m"
)

print(
    f"FIT Y    : "
    f"{fit_pct(y_real, y_sim):.2f}%"
    f"    RMS: {rms(y_real, y_sim):.4f} m"
)

print(
    f"FIT Z    : "
    f"{fit_pct(z_real, z_sim):.2f}%"
    f"    RMS: {rms(z_real, z_sim):.4f} m"
)

print(
    f"FIT PSI  : "
    f"{fit_pct(psi_real, psi_sim):.2f}%"
    f"    RMS: {rms(psi_real, psi_sim):.4f} rad"
)


fig1, axs = plt.subplots(2, 2, figsize=(15, 8))

fig1.suptitle(
    "VALIDACION VELOCIDADES",
    fontsize=14,
    fontweight='bold'
)

velocity_pairs = [

    (vx_real, vx_hat, "VX BODY"),

    (vy_real, vy_hat, "VY BODY"),

    (vz_real, vz_hat, "VZ BODY"),

    (r_real, r_hat, "YAW RATE"),
]

for i, (real, sim, title) in enumerate(velocity_pairs):

    ax = axs[i // 2, i % 2]

    ax.plot(
        t,
        real,
        label="Real",
        linewidth=1.2
    )

    ax.plot(
        t,
        sim,
        '--',
        label="Modelo",
        linewidth=1.5
    )

    ax.set_title(
        f"{title}   "
        f"FIT={fit_pct(real, sim):.2f}%"
    )

    ax.grid(True)
    ax.legend()


fig2, axs2 = plt.subplots(2, 2, figsize=(15, 8))

fig2.suptitle(
    "VALIDACION POSICIONES",
    fontsize=14,
    fontweight='bold'
)

position_pairs = [

    (x_real, x_sim, "POS X", "m"),

    (y_real, y_sim, "POS Y", "m"),

    (z_real, z_sim, "POS Z", "m"),

    (psi_real, psi_sim, "YAW", "rad"),
]

for i, (real, sim, title, unit) in enumerate(position_pairs):

    ax = axs2[i // 2, i % 2]

    ax.plot(
        t,
        real,
        label="Real",
        linewidth=1.2
    )

    ax.plot(
        t,
        sim,
        '--',
        label="Modelo",
        linewidth=1.5
    )

    ax.set_title(
        f"{title}   "
        f"FIT={fit_pct(real, sim):.2f}%   "
        f"RMS={rms(real, sim):.4f} {unit}"
    )

    ax.grid(True)
    ax.legend()


plt.figure(figsize=(8,8))

plt.plot(
    x_real,
    y_real,
    label="Trayectoria Real",
    linewidth=2
)

plt.plot(
    x_sim,
    y_sim,
    '--',
    label="Trayectoria Modelo",
    linewidth=2
)

plt.xlabel("X [m]")
plt.ylabel("Y [m]")

plt.title("TRAYECTORIA XY")

plt.axis('equal')

plt.grid(True)

plt.legend()

plt.tight_layout()

plt.show()