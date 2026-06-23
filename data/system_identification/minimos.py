#!/usr/bin/env python3
"""
System identification for the Bebop 2 body-frame model:

    ν̇ = f1 · U  -  f2 · ν          (eq. model_body)

Reformulación discreta (Euler explícito) para evitar derivadas numéricas:

    ν[k+1] = (1 - dt·f2)·ν[k]  +  dt·f1·U[k]
           = a·ν[k]  +  b·U[k]         con  a = 1 - dt·f2,  b = dt·f1

El regresor es  Φ = [ν[k]  U[k]]  y la salida es  ν[k+1].
LS sobre velocidades → sin doble derivación → mucho más robusto al ruido.

Tras identificar a y b:
    f1 = b / dt
    f2 = (1 - a) / dt
"""

import math
import os
import numpy as np
from scipy.io import loadmat
import matplotlib.pyplot as plt

OUT_DIR = os.getcwd()

def savefig(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=150, bbox_inches='tight')
    print(f"  saved → {path}")


# ─── 0. Load data ────────────────────────────────────────────────────────────
mat_file = "manual_log_20260518_172721.mat"
data     = loadmat(mat_file)

hz = float(data["hz"].squeeze())
dt = 1.0 / hz

u_full    = data["u"]                           # (N, 4): [uvx, uvy, uvz, uψ̇]
vx_b_full = data["vx_b"].squeeze()
vy_b_full = data["vy_b"].squeeze()
psi_full  = np.unwrap(data["psi"].squeeze())
z_full    = data["z_i"].squeeze()
x_full    = data["x_i"].squeeze()
y_full    = data["y_i"].squeeze()

N = len(vx_b_full)
print(f"Loaded {N} samples  |  dt = {dt*1e3:.2f} ms  |  {hz:.1f} Hz")


# ─── 1. Causal moving-average filter  ────────────────────────────────────────
class CausalMovingAverage:
    def __init__(self, window_size: int):
        self.W      = window_size
        self.buffer = []

    def update(self, x: float) -> float:
        self.buffer.append(x)
        if len(self.buffer) > self.W:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)

W = 5
z_filt_arr   = np.zeros(N)
psi_filt_arr = np.zeros(N)

filt_z   = CausalMovingAverage(W)
filt_psi = CausalMovingAverage(W)

for k in range(N):
    z_filt_arr[k]   = filt_z.update(float(z_full[k]))
    psi_filt_arr[k] = filt_psi.update(float(psi_full[k]))  # psi unwrapped → escalar


# ─── 2. Velocidades vz y ω por diferencia finita sobre posición filtrada ──────
vz_b_full   = np.zeros(N)
vyaw_b_full = np.zeros(N)

for k in range(1, N):
    vz_b_full[k]   = (z_filt_arr[k]   - z_filt_arr[k-1]) / dt
    vyaw_b_full[k] = (psi_filt_arr[k] - psi_filt_arr[k-1]) / dt


# ─── 3. Trim transiente del filtro ───────────────────────────────────────────
TRIM = W

u_t  = u_full[TRIM:, :]                        # (M, 4)
nu_t = np.column_stack([                        # (M, 4)  velocidades en el cuerpo
    vx_b_full[TRIM:],
    vy_b_full[TRIM:],
    vz_b_full[TRIM:],
    vyaw_b_full[TRIM:],
])

M_samples = u_t.shape[0]
print(f"Using {M_samples} samples (trimmed {TRIM})")


# ─── 4. Mínimos cuadrados en forma discreta  ─────────────────────────────────
#
#   ν[k+1] = a·ν[k] + b·U[k]
#
#   Φ[k] = [ν[k]  U[k]]   ∈ R^(1×2)
#   y[k] = ν[k+1]
#
#   Apilando k = 0…M-2:
#       Y = Φ · θ,   θ = [a, b]ᵀ
#
#   Recuperación:
#       f2 = (1 - a) / dt
#       f1 =  b / dt
#

channel_names  = ["vx_b (surge)", "vy_b (sway)", "vz_b (heave)", "yaw-rate ω"]
channel_labels = [r"$v_{x,b}$",   r"$v_{y,b}$",  r"$v_{z,b}$",  r"$\omega$"]
results = {}

# ── Formulación en Δν para capturar dinámica real ────────────────────────────────────
#
#   ν[k+1] - ν[k] = dt·f1·U[k] - dt·f2·ν[k]
#        Δν[k]    =    b·U[k]   -    c·ν[k]
#
#   Φ = [U[k]  -ν[k]],   y = Δν[k] = ν[k+1] - ν[k]
#
#   Recuperación:  f1 = b/dt,  f2 = c/dt
#
#   R² mide qué fracción del CAMBIO de velocidad explica el modelo,
#   no el valor absoluto → métrica honesta de la dinámica identificada.
#

for i, name in enumerate(channel_names):
    nu_k  = nu_t[:-1, i]
    u_k   = u_t[:-1, i]
    dnu_k = nu_t[1:, i] - nu_t[:-1, i]   # Δν[k] = ν[k+1] - ν[k]  ← salida

    Phi   = np.column_stack([u_k, -nu_k])   # (M-1, 2)
    theta, *_ = np.linalg.lstsq(Phi, dnu_k, rcond=None)

    b, c  = theta
    f1i   = b / dt
    f2i   = c / dt

    dnu_hat = Phi @ theta
    ss_res  = np.sum((dnu_k - dnu_hat)**2)
    ss_tot  = np.sum((dnu_k - dnu_k.mean())**2)
    R2      = 1.0 - ss_res / ss_tot if ss_tot > 0 else float('nan')

    results[name] = dict(f1=f1i, f2=f2i, b=b, c=c, R2=R2,
                         nu_meas=nu_t[:, i], u=u_t[:, i])
    print(f"\n[{name}]  f1 = {f1i:+.6f}   f2 = {f2i:+.6f}   R²(Δν) = {R2:.4f}")

print("\nf1 = diag(", ", ".join(f"{results[n]['f1']:.6f}" for n in channel_names), ")")
print("f2 = diag(", ", ".join(f"{results[n]['f2']:.6f}" for n in channel_names), ")")


# ─── 5. Simulación forward ───────────────────────────────────────────────────
f1_vec = np.array([results[n]['f1'] for n in channel_names])
f2_vec = np.array([results[n]['f2'] for n in channel_names])

nu_model = np.zeros((M_samples, 4))
nu_model[0] = nu_t[0]

for k in range(M_samples - 1):
    dnu_k         = f1_vec * u_t[k] - f2_vec * nu_model[k]
    nu_model[k+1] = nu_model[k] + dt * dnu_k

# Posiciones: integración con Jacobian
pos_model    = np.zeros((M_samples, 4))   # [x, y, z, ψ]
pos_model[0] = [x_full[TRIM], y_full[TRIM], z_filt_arr[TRIM], psi_filt_arr[TRIM]]

for k in range(M_samples - 1):
    psi_k = pos_model[k, 3]
    vxb, vyb, vzb, om = nu_model[k]
    pos_model[k+1, 0] = pos_model[k, 0] + dt * (vxb * math.cos(psi_k) - vyb * math.sin(psi_k))
    pos_model[k+1, 1] = pos_model[k, 1] + dt * (vxb * math.sin(psi_k) + vyb * math.cos(psi_k))
    pos_model[k+1, 2] = pos_model[k, 2] + dt * vzb
    pos_model[k+1, 3] = pos_model[k, 3] + dt * om

x_real   = x_full[TRIM:]
y_real   = y_full[TRIM:]
z_real   = z_filt_arr[TRIM:]
psi_real = psi_filt_arr[TRIM:]

t = np.arange(M_samples) * dt


# ═══════════════════════════════════════════════════════════════════════════════
# FIGURA 1 – Velocidades: real vs modelada (simulación forward)
# ═══════════════════════════════════════════════════════════════════════════════
vel_labels = [r"$v_{x,b}$ [m/s]", r"$v_{y,b}$ [m/s]", r"$v_{z,b}$ [m/s]", r"$\omega$ [rad/s]"]
nu_ref = np.column_stack([
    vx_b_full[TRIM:],
    vy_b_full[TRIM:],
    vz_b_full[TRIM:],
    vyaw_b_full[TRIM:],
])
ref_labels = [
    'measured (sensor)',
    'measured (sensor)',
    r'$\Delta z_{filt}\ /\ dt$',
    r'$\Delta\psi_{filt}\ /\ dt$',
]

fig1, axes1 = plt.subplots(4, 1, figsize=(13, 14), sharex=True)
fig1.suptitle("Body-frame velocities: measured vs model (forward simulation)", fontsize=14)

for i, (lbl, ax) in enumerate(zip(vel_labels, axes1)):
    name = channel_names[i]
    ax.plot(t, nu_ref[:, i],   lw=1.2, alpha=0.75, label=ref_labels[i])
    ax.plot(t, nu_model[:, i], lw=1.5, ls='--', color='crimson',
            label=f"model  f1={results[name]['f1']:.4f}  f2={results[name]['f2']:.4f}  R²(Δν)={results[name]['R2']:.3f}")
    ax.set_ylabel(lbl)
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)

axes1[-1].set_xlabel("Time [s]")
plt.tight_layout()


# ═══════════════════════════════════════════════════════════════════════════════
# FIGURA 2 – Posiciones: real vs modelada
# ═══════════════════════════════════════════════════════════════════════════════
pos_labels     = ["x [m]", "y [m]", "z [m]", r"$\psi$ [rad]"]
pos_real_list  = [x_real, y_real, z_real, psi_real]
pos_model_list = [pos_model[:, j] for j in range(4)]

fig2, axes2 = plt.subplots(4, 1, figsize=(13, 14), sharex=True)
fig2.suptitle("Inertial positions: measured vs model (forward integration)", fontsize=14)

for i, (lbl, ax) in enumerate(zip(pos_labels, axes2)):
    ax.plot(t, pos_real_list[i],  lw=1.2, alpha=0.75, label='measured (filtered)')
    ax.plot(t, pos_model_list[i], lw=1.5, ls='--', color='crimson', label='model (fwd. sim.)')
    ax.set_ylabel(lbl)
    ax.legend(fontsize=9, loc='upper right')
    ax.grid(True, alpha=0.3)

axes2[-1].set_xlabel("Time [s]")
plt.tight_layout()

plt.show()
print("\nAll done.")