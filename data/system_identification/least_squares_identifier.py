# Implements standard Least Squares estimation for linear system identification.
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import sys


mat_file = "manual_log_20260518_165334.mat"   # <-- cambia si es necesario

try:
    data = sio.loadmat(mat_file)
except FileNotFoundError:
    print(f"[ERROR] No se encontro el archivo: {mat_file}")
    sys.exit(1)

hz   = float(data["hz"].squeeze())
Ts   = 1.0 / hz

u     = data["u"]             # (N, 4)
x_i   = data["x_i"].squeeze()
y_i   = data["y_i"].squeeze()
z_i   = data["z_i"].squeeze()
psi   = data["psi"].squeeze()
vx_b  = data["vx_b"].squeeze()
vy_b  = data["vy_b"].squeeze()
vz_b  = data["vz_b"].squeeze()
vpsi  = data["vpsi"].squeeze()

N_total = len(x_i)


k0    = 5
k_end = N_total - int(5.0 * hz)

if k_end <= k0:
    print("[ERROR] El dataset es demasiado corto para el recorte solicitado.")
    sys.exit(1)

u    = u   [k0:k_end]
x_i  = x_i[k0:k_end]
y_i  = y_i[k0:k_end]
z_i  = z_i[k0:k_end]
psi  = psi [k0:k_end]
vx_b = vx_b[k0:k_end]
vy_b = vy_b[k0:k_end]
vz_b = vz_b[k0:k_end]
vpsi = vpsi[k0:k_end]

N = len(x_i)
t = np.arange(N) * Ts

print(f"Muestras totales : {N_total}")
print(f"Frecuencia       : {hz} Hz  ->  Ts = {Ts:.4f} s")
print(f"Muestras usadas  : {N}  ({N*Ts:.2f} s)")


u_labels = ["u₁  (vx — lon)", "u₂  (vy — lat)", "u₃  (vz — alt)", "u₄  (vpsi — yaw)"]
colors_u  = ["#2a7ae2", "#e06c2a", "#2aad58", "#9b2ae0"]

fig1, axs1 = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
fig1.suptitle("Acciones de control", fontsize=13, fontweight="bold")

for i in range(4):
    axs1[i].plot(t, u[:, i], color=colors_u[i], linewidth=0.9)
    axs1[i].set_ylabel(u_labels[i], fontsize=9)
    axs1[i].grid(True, linewidth=0.4, alpha=0.6)
    axs1[i].axhline(0, color="gray", linewidth=0.5, linestyle="--")

axs1[-1].set_xlabel("Tiempo [s]")
fig1.tight_layout()


vel_data   = [vx_b, vy_b, vz_b, vpsi]
vel_labels = ["vx_b  [m/s]", "vy_b  [m/s]", "vz_b  [m/s]", "vpsi  [rad/s]"]
colors_v   = ["#2a7ae2", "#e06c2a", "#2aad58", "#9b2ae0"]

fig2, axs2 = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
fig2.suptitle("Velocidades medidas (body frame)", fontsize=13, fontweight="bold")

for i in range(4):
    axs2[i].plot(t, vel_data[i], color=colors_v[i], linewidth=0.9)
    axs2[i].set_ylabel(vel_labels[i], fontsize=9)
    axs2[i].grid(True, linewidth=0.4, alpha=0.6)
    axs2[i].axhline(0, color="gray", linewidth=0.5, linestyle="--")

axs2[-1].set_xlabel("Tiempo [s]")
fig2.tight_layout()


pos_data   = [x_i, y_i, z_i, psi]
pos_labels = ["x  [m]", "y  [m]", "z  [m]", "psi  [rad]"]
colors_p   = ["#2a7ae2", "#e06c2a", "#2aad58", "#9b2ae0"]

fig3, axs3 = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
fig3.suptitle("Posiciones medidas (frame inercial)", fontsize=13, fontweight="bold")

for i in range(4):
    axs3[i].plot(t, pos_data[i], color=colors_p[i], linewidth=0.9)
    axs3[i].set_ylabel(pos_labels[i], fontsize=9)
    axs3[i].grid(True, linewidth=0.4, alpha=0.6)

axs3[-1].set_xlabel("Tiempo [s]")
fig3.tight_layout()


fig4, ax4 = plt.subplots(figsize=(7, 7))
ax4.plot(x_i, y_i, color="#2a7ae2", linewidth=1.2, label="Trayectoria")
ax4.scatter(x_i[0],  y_i[0],  color="green", zorder=5, s=60, label="Inicio")
ax4.scatter(x_i[-1], y_i[-1], color="red",   zorder=5, s=60, label="Fin")
ax4.set_xlabel("x [m]")
ax4.set_ylabel("y [m]")
ax4.set_title("Trayectoria XY", fontsize=12, fontweight="bold")
ax4.axis("equal")
ax4.grid(True, linewidth=0.4, alpha=0.6)
ax4.legend()
fig4.tight_layout()

plt.show()