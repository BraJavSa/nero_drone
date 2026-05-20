#!/usr/bin/env python3

# Optimizes Alpha-Beta filter parameters for Altitude (Z) and Yaw independently.
"""
BUSQUEDA DEL MEJOR FILTRO ALPHA-BETA
PARA VZ Y VYAW
====================================

Ahora Z y YAW se optimizan por separado
porque el modelo esta desacoplado:

    vz[k+1] = az * vz[k] + bz * uz[k]

    r[k+1]  = ar * r[k]  + br * upsi[k]

Entonces NO se hace una búsqueda 4D.

Se hacen DOS búsquedas 2D independientes:

    (alpha_z, beta_z)

    (alpha_r, beta_r)

Resolución:
    0.02

Valores:
    0.00
    0.02
    0.04
    ...
    0.98
"""

import numpy as np
import matplotlib.pyplot as plt

from scipy.io import loadmat


Ad = np.array(

[[ 9.63895373e-01, -6.18407625e-02,  0.00000000e+00,  1.60376855e-02],
 [-8.14310627e-05,  9.99449376e-01,  0.00000000e+00,  5.21436812e-03],
 [ 0.00000000e+00,  0.00000000e+00,  8.57642038e-01,  0.00000000e+00],
 [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  8.22489970e-01],
])

Bd = np.array([

    [ 4.68509248e-02,  9.89534630e-03,  0.00000000e+00, -1.41951368e-02],
    [ 2.04387648e-05,  3.40358601e-02,  0.00000000e+00, -8.51778748e-03],
    [ 0.00000000e+00, 0.00000000e+00,  1.80292391e-01,  0.00000000e+00],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.84677304e-01],
])




DELAY = 2


mat_file = "manual_log_20260517_205200.mat"

data = loadmat(mat_file)

hz = float(data["hz"].squeeze())
dt = 1.0 / hz

u_full = data["u"]

vz_b_full = data["vz_b"].squeeze()
r_b_full  = data["vpsi"].squeeze()


delay = DELAY

u = u_full[:-delay, :]

vz_raw = vz_b_full[delay:]
r_raw  = r_b_full[delay:]

uz    = u[:,2]
upsi  = u[:,3]

N = len(uz)

t = np.arange(N) * dt


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


def fit_pct(y, yhat):

    var_y = np.linalg.norm(y - np.mean(y))

    if var_y == 0:
        return 0.0

    return 100.0 * (
        1.0 - np.linalg.norm(y - yhat) / var_y
    )

def rms(a, b):

    return np.sqrt(np.mean((a - b)**2))


alphas = np.arange(0.00, 1.00, 0.02)
betas  = np.arange(0.00, 1.00, 0.02)


print("\n================================================")
print("BUSQUEDA FILTRO VZ")
print("================================================")

best_score_z = -1e9
best_z = None

total = len(alphas) * len(betas)
counter = 0

for az in alphas:
    for bz in betas:

        counter += 1

        print(
            f"\r[{counter}/{total}] "
            f"alpha_z={az:.2f} "
            f"beta_z={bz:.2f}",
            end=""
        )

        vz_f = alpha_beta_filter(
            vz_raw,
            az,
            bz,
            dt
        )

        vz_hat = np.zeros(N)

        vz_hat[0] = vz_f[0]

        for k in range(N - 1):

            vz_hat[k+1] = (
                Ad[2,2] * vz_hat[k]
                +
                Bd[2,2] * uz[k]
            )

        fit_z = fit_pct(vz_f, vz_hat)

        if fit_z > best_score_z:

            best_score_z = fit_z

            best_z = {

                "alpha": az,
                "beta": bz,

                "fit": fit_z,

                "vz_f": vz_f,
                "vz_hat": vz_hat,
            }

print("\n")


print("\n================================================")
print("BUSQUEDA FILTRO VYAW")
print("================================================")

best_score_r = -1e9
best_r = None

counter = 0

for ar in alphas:
    for br in betas:

        counter += 1

        print(
            f"\r[{counter}/{total}] "
            f"alpha_r={ar:.2f} "
            f"beta_r={br:.2f}",
            end=""
        )

        r_f = alpha_beta_filter(
            r_raw,
            ar,
            br,
            dt
        )

        r_hat = np.zeros(N)

        r_hat[0] = r_f[0]

        for k in range(N - 1):

            r_hat[k+1] = (
                Ad[3,3] * r_hat[k]
                +
                Bd[3,3] * upsi[k]
            )

        fit_r = fit_pct(r_f, r_hat)

        if fit_r > best_score_r:

            best_score_r = fit_r

            best_r = {

                "alpha": ar,
                "beta": br,

                "fit": fit_r,

                "r_f": r_f,
                "r_hat": r_hat,
            }

print("\n")


print("\n================================================")
print("MEJOR FILTRO VZ")
print("================================================")

print(f"ALPHA_Z : {best_z['alpha']:.2f}")
print(f"BETA_Z  : {best_z['beta']:.2f}")

print(f"FIT_Z   : {best_z['fit']:.2f}%")

print(
    f"RMS_Z   : "
    f"{rms(best_z['vz_f'], best_z['vz_hat']):.6f}"
)

print("\n================================================")
print("MEJOR FILTRO VYAW")
print("================================================")

print(f"ALPHA_R : {best_r['alpha']:.2f}")
print(f"BETA_R  : {best_r['beta']:.2f}")

print(f"FIT_R   : {best_r['fit']:.2f}%")

print(
    f"RMS_R   : "
    f"{rms(best_r['r_f'], best_r['r_hat']):.6f}"
)


fig, axs = plt.subplots(2, 1, figsize=(14, 8))


axs[0].plot(
    t,
    vz_raw,
    alpha=0.4,
    label="VZ raw"
)

axs[0].plot(
    t,
    best_z["vz_f"],
    label="VZ filtrada",
    linewidth=1.5
)

axs[0].plot(
    t,
    best_z["vz_hat"],
    '--',
    label="VZ modelo",
    linewidth=1.5
)

axs[0].set_title(
    f"VZ   "
    f"FIT={best_z['fit']:.2f}%"
)

axs[0].grid(True)
axs[0].legend()


axs[1].plot(
    t,
    r_raw,
    alpha=0.4,
    label="VYAW raw"
)

axs[1].plot(
    t,
    best_r["r_f"],
    label="VYAW filtrada",
    linewidth=1.5
)

axs[1].plot(
    t,
    best_r["r_hat"],
    '--',
    label="VYAW modelo",
    linewidth=1.5
)

axs[1].set_title(
    f"VYAW   "
    f"FIT={best_r['fit']:.2f}%"
)

axs[1].grid(True)
axs[1].legend()

plt.tight_layout()
plt.show()



