#!/usr/bin/env python3
import sys
import os

# --- PARCHE DE ENTORNO VIRTUAL ---
sys.path = [p for p in sys.path if 'dist-packages' not in p]

# --- PARCHE mpl_toolkits ---
import types
venv_site = next(p for p in sys.path if 'venv-ardupilot' in p and 'site-packages' in p)
mpl_toolkits_path = os.path.join(venv_site, 'mpl_toolkits')
mpl_pkg = types.ModuleType('mpl_toolkits')
mpl_pkg.__path__ = [mpl_toolkits_path]
mpl_pkg.__package__ = 'mpl_toolkits'
sys.modules['mpl_toolkits'] = mpl_pkg

# --- IMPORTS ---
import casadi as ca
import numpy as np
import scipy.io
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 1. CONFIGURACIÓN DE RUTAS
base_path = os.path.dirname(os.path.abspath(__file__))
SO_PATH   = os.path.join(base_path, "drone_model_v2.so")
JSON_PATH = os.path.join(base_path, "drone_metadata_v2.json")
MAT_PATH  = os.path.join(base_path, "real_val.mat")

if not os.path.exists(SO_PATH) or not os.path.exists(JSON_PATH):
    raise FileNotFoundError("No se encontró el archivo .so o el .json de metadatos.")

with open(JSON_PATH, 'r') as f:
    meta = json.load(f)

window_size = meta['window_size']
dt          = meta['dt']

f_mlp_c = ca.external('f_mlp', SO_PATH)

# 2. CARGAR DATOS REALES (descartando las primeras 10 muestras)
SKIP = 10
data = scipy.io.loadmat(MAT_PATH)

v_real   = np.vstack((data['vx_b'].flatten(), data['vy_b'].flatten(),
                      data['vz_b'].flatten(), data['vpsi'].flatten())).T[SKIP:]
u_real   = data['u'][SKIP:]
pos_real = np.vstack((data['x_i'].flatten(), data['y_i'].flatten(),
                      data['z_i'].flatten(), data['psi'].flatten())).T[SKIP:]

# 3. PREPARACIÓN DE LA SIMULACIÓN
num_samples = len(v_real)
v_sim   = np.zeros_like(v_real)
pos_sim = np.zeros((num_samples, 4))

v_sim[:window_size]   = v_real[:window_size]
pos_sim[:window_size] = pos_real[:window_size]

print(f"Simulando {num_samples} pasos (datos desde muestra {SKIP})...")
print(f"Formato de entrada: v_now(4) + u_hist({window_size}×4) + u_now(4) = {4*(window_size+2)} features")

# 4. BUCLE DE SIMULACIÓN (RK4 + INTEGRACIÓN CINEMÁTICA)
for t in range(window_size - 1, num_samples - 1):
    u_window = u_real[t - window_size + 1 : t + 1]   # (W, 4)
    u_actual = u_real[t + 1]                          # (4,)

    def f_derivada(v_state):
        # Nuevo formato: v_now(4) + u_hist(W*4) + u_now(4)
        x_input = np.hstack((v_state.flatten(),
                              u_window.flatten(),
                              u_actual.flatten())).reshape(1, -1)
        return f_mlp_c(x_input).full().flatten()

    y_n = v_sim[t]
    k1  = f_derivada(y_n)
    k2  = f_derivada(y_n + 0.5 * dt * k1)
    k3  = f_derivada(y_n + 0.5 * dt * k2)
    k4  = f_derivada(y_n + dt * k3)

    v_sim[t + 1] = y_n + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)

    vx_b, vy_b, vz_b, vpsi = v_sim[t + 1]
    psi_prev = pos_sim[t, 3]

    x_dot = vx_b * np.cos(psi_prev) - vy_b * np.sin(psi_prev)
    y_dot = vx_b * np.sin(psi_prev) + vy_b * np.cos(psi_prev)

    pos_sim[t + 1, 0] = pos_sim[t, 0] + x_dot * dt
    pos_sim[t + 1, 1] = pos_sim[t, 1] + y_dot * dt
    pos_sim[t + 1, 2] = pos_sim[t, 2] + vz_b * dt
    pos_sim[t + 1, 3] = pos_sim[t, 3] + vpsi * dt

# 5. VISUALIZACIÓN
t_axis = np.arange(num_samples) * dt

# Gráfica 1: Velocidades (Body Frame)
fig1, axs1 = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
v_labels = ['Vx_b (m/s)', 'Vy_b (m/s)', 'Vz_b (m/s)', 'Vpsi (rad/s)']
for i in range(4):
    axs1[i].plot(t_axis, v_real[:, i], color='gray', alpha=0.5, label='Real')
    axs1[i].plot(t_axis, v_sim[:, i], label='Simulado', linewidth=1.5)
    axs1[i].set_ylabel(v_labels[i])
    axs1[i].legend(loc='upper right')
    axs1[i].grid(True, alpha=0.3)
axs1[-1].set_xlabel('Tiempo (s)')
fig1.suptitle('Validación de Velocidades (Marco del Cuerpo)', fontsize=14)

# Gráfica 2: Trayectoria Inercial (Posiciones)
fig2, axs2 = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
p_labels = ['X Inercial (m)', 'Y Inercial (m)', 'Z Inercial (m)', 'Psi (rad)']
for i in range(4):
    axs2[i].plot(t_axis, pos_real[:, i], color='black', linestyle='--', alpha=0.6, label='Real')
    axs2[i].plot(t_axis, pos_sim[:, i], color='red', label='Simulado')
    axs2[i].set_ylabel(p_labels[i])
    axs2[i].legend(loc='upper right')
    axs2[i].grid(True, alpha=0.3)
axs2[-1].set_xlabel('Tiempo (s)')
fig2.suptitle('Evolución de la Trayectoria Inercial', fontsize=14)

# Gráfica 3: Trayectoria 3D
fig3 = plt.figure(figsize=(10, 7))
ax3d = fig3.add_subplot(111, projection='3d')
ax3d.plot(pos_real[:, 0], pos_real[:, 1], pos_real[:, 2], 'k--', alpha=0.5, label='Real')
ax3d.plot(pos_sim[:, 0], pos_sim[:, 1], pos_sim[:, 2], 'r', label='Simulado')
ax3d.set_xlabel('X (m)')
ax3d.set_ylabel('Y (m)')
ax3d.set_zlabel('Z (m)')
ax3d.legend()
plt.title('Trayectoria 3D: Reconstrucción Inercial')

plt.tight_layout()
plt.show()

# Error Final
rmse_pos = np.sqrt(np.mean((pos_real[window_size:] - pos_sim[window_size:])**2))
print(f"\nRMSE Posición Final: {rmse_pos:.6f}")