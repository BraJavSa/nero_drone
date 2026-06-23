import numpy as np
import scipy.io as sio
from scipy.optimize import differential_evolution

data = sio.loadmat("manual_log_20260518_172721.mat")
hz = float(data["hz"].squeeze())
dt = 1.0 / hz

u = data["u"]
vx_b = data["vx_b"].squeeze()
vy_b = data["vy_b"].squeeze()
x_real = data["x_i"].squeeze()
y_real = data["y_i"].squeeze()
psi = np.unwrap(data["psi"].squeeze())

N = len(vx_b)

# Slice boundary transient
TRIM = 10
u_t = u[TRIM:]
x_t = x_real[TRIM:]
y_t = y_real[TRIM:]
psi_t = psi[TRIM:]
vx_t = vx_b[TRIM:]
vy_t = vy_b[TRIM:]
M = len(u_t)

x0 = x_t[0]
y0 = y_t[0]

# Optimize: f1_x, f2_x, f1_y, f2_y, delay
# Bounds: f1 ∈ [0.01, 5.0], f2 ∈ [0.01, 5.0], delay ∈ [0.0, 4.0]
bounds = [
    (0.01, 5.0), # f1_x
    (0.01, 5.0), # f2_x
    (0.01, 5.0), # f1_y
    (0.01, 5.0), # f2_y
    (0.0, 4.0)   # delay
]

def joint_loss(params):
    f1_x, f2_x, f1_y, f2_y, delay_val = params
    delay = int(round(delay_val))
    
    # Delayed inputs
    u_x = np.zeros(M)
    u_y = np.zeros(M)
    if delay == 0:
        u_x[:] = u_t[:, 0]
        u_y[:] = u_t[:, 1]
    else:
        u_x[delay:] = u_t[:-delay, 0]
        u_y[delay:] = u_t[:-delay, 1]
        
    v_sim_x = np.zeros(M)
    v_sim_x[0] = vx_t[0]
    v_sim_y = np.zeros(M)
    v_sim_y[0] = vy_t[0]
    
    for k in range(M - 1):
        v_sim_x[k+1] = v_sim_x[k] + dt * (f1_x * u_x[k] - f2_x * v_sim_x[k])
        v_sim_y[k+1] = v_sim_y[k] + dt * (f1_y * u_y[k] - f2_y * v_sim_y[k])
        
    pos_sim = np.zeros((M, 2))
    pos_sim[0] = [x0, y0]
    for k in range(M - 1):
        psi_k = psi_t[k]
        pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (v_sim_x[k] * np.cos(psi_k) - v_sim_y[k] * np.sin(psi_k))
        pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (v_sim_x[k] * np.sin(psi_k) + v_sim_y[k] * np.cos(psi_k))
        
    rmse_x = np.sqrt(np.mean((x_t - pos_sim[:, 0])**2))
    rmse_y = np.sqrt(np.mean((y_t - pos_sim[:, 1])**2))
    return rmse_x + rmse_y

res = differential_evolution(joint_loss, bounds, maxiter=30, popsize=15, disp=True)

# Print results
f1_x, f2_x, f1_y, f2_y, delay_val = res.x
delay = int(round(delay_val))

print("\n" + "="*60)
print("OPTIMIZATION RESULTS (MINIMIZING POSITION RMSE)")
print("="*60)
print(f"Delay: {delay}")
print(f"X (Surge): f1 = {f1_x:.6f} | f2 = {f2_x:.6f}")
print(f"Y (Sway):  f1 = {f1_y:.6f} | f2 = {f2_y:.6f}")
print(f"Joint Position RMSE: {res.fun:.4f} m")

# Calculate R2 for positions
u_x = np.zeros(M)
u_y = np.zeros(M)
if delay == 0:
    u_x[:] = u_t[:, 0]
    u_y[:] = u_t[:, 1]
else:
    u_x[delay:] = u_t[:-delay, 0]
    u_y[delay:] = u_t[:-delay, 1]
    
v_sim_x = np.zeros(M)
v_sim_x[0] = vx_t[0]
v_sim_y = np.zeros(M)
v_sim_y[0] = vy_t[0]
for k in range(M - 1):
    v_sim_x[k+1] = v_sim_x[k] + dt * (f1_x * u_x[k] - f2_x * v_sim_x[k])
    v_sim_y[k+1] = v_sim_y[k] + dt * (f1_y * u_y[k] - f2_y * v_sim_y[k])
    
pos_sim = np.zeros((M, 2))
pos_sim[0] = [x0, y0]
for k in range(M - 1):
    psi_k = psi_t[k]
    pos_sim[k+1, 0] = pos_sim[k, 0] + dt * (v_sim_x[k] * np.cos(psi_k) - v_sim_y[k] * np.sin(psi_k))
    pos_sim[k+1, 1] = pos_sim[k, 1] + dt * (v_sim_x[k] * np.sin(psi_k) + v_sim_y[k] * np.cos(psi_k))

ss_tot_x = np.sum((x_t - np.mean(x_t))**2)
r2_x = 1.0 - np.sum((x_t - pos_sim[:, 0])**2) / ss_tot_x
ss_tot_y = np.sum((y_t - np.mean(y_t))**2)
r2_y = 1.0 - np.sum((y_t - pos_sim[:, 1])**2) / ss_tot_y
print(f"Position R2 X: {r2_x*100:.2f}%")
print(f"Position R2 Y: {r2_y*100:.2f}%")
print("="*60)
