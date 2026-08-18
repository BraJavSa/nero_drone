#!/usr/bin/env python3

# Evolutionary Optimizer for the Feedback Linearization Velocity Controller.
# Uses Differential Evolution to find optimal tracking gains K_v and K_sv.
import os
import time
import numpy as np
from scipy.optimize import differential_evolution

# Model Parameters (system_identification_parameters_optitrack_4dof.json)
F1_DIAG = np.array([0.921527, 1.053286, 4.173221, 8.772786])
F2_DIAG = np.array([0.247044, 0.395160, 1.975836, 6.101834])

DYN_DT = 1.0 / 15.0    # 15 Hz dynamics integration step (matching controller rate)
CTRL_DT = 1.0 / 15.0   # 15 Hz controller frequency
REF_TAU = np.array([0.18, 0.18, 0.10, 0.25])

# Optimization bounds
# kv_x, kv_y, kv_z, kv_yaw, ksv_x, ksv_y, ksv_z, ksv_yaw
BOUNDS = [
    (0.5, 15.0),
    (0.5, 15.0),
    (0.1, 10.0),
    (0.1, 10.0),
    (0.05, 3.0),
    (0.05, 3.0),
    (0.05, 3.0),
    (0.05, 3.0)
]

DEFAULT_GAINS = np.array([5.0, 5.0, 0.7, 0.7, 0.4, 0.4, 0.5, 0.8])

def get_ref_velocity(t):
    t_mod = t % 20.0
    nu_d = np.zeros(4)
    # Sequentially test steps in all 4 axes, repeating every 20s
    if 2.0 <= t_mod < 6.0:
        nu_d[0] = 0.5
    elif 6.0 <= t_mod < 10.0:
        nu_d[1] = 0.5
    elif 10.0 <= t_mod < 14.0:
        nu_d[2] = 0.4
    elif 14.0 <= t_mod < 18.0:
        nu_d[3] = 0.5
    return nu_d

def simulate(K_v, K_sv, return_history=False):
    t = 0.0
    t_end = 80.0
    
    nu = np.zeros(4)
    ref_filt = np.zeros(4)
    ref_prev = np.zeros(4)
    
    errors = []
    inputs = []
    t_history = []
    ref_history = []
    nu_history = []
    
    while t < t_end:
        nu_d_raw = get_ref_velocity(t)
        alpha_filt = CTRL_DT / (REF_TAU + CTRL_DT)
        ref_filt = ref_filt + alpha_filt * (nu_d_raw - ref_filt)
        
        nu_d_dot = (ref_filt - ref_prev) / CTRL_DT
        ref_prev = ref_filt.copy()
        
        # Tracking error
        nu_tilde = ref_filt - nu
        # Wrap yaw error
        nu_tilde[3] = (nu_tilde[3] + np.pi) % (2.0 * np.pi) - np.pi
        
        # Control law
        alpha_control = nu_d_dot + K_sv * np.tanh(K_v * nu_tilde)
        Ud = (1.0 / F1_DIAG) * (alpha_control + F2_DIAG * nu)
        u_body = np.clip(Ud, -1.0, 1.0)
        
        # Euler Integration of dynamics: v_dot = F1*u - F2*v
        nu_dot = F1_DIAG * u_body - F2_DIAG * nu
        nu = nu + CTRL_DT * nu_dot
        
        # Constraints
        nu[0:2] = np.clip(nu[0:2], -4.5, 4.5)
        nu[2] = np.clip(nu[2], -1.0, 1.0)
        nu[3] = np.clip(nu[3], -np.deg2rad(100.0), np.deg2rad(100.0))
        
        errors.append(ref_filt - nu)
        inputs.append(u_body.copy())
        if return_history:
            t_history.append(t)
            ref_history.append(ref_filt.copy())
            nu_history.append(nu.copy())
            
        t += CTRL_DT
        
    if return_history:
        return np.array(errors), np.array(inputs), np.array(t_history), np.array(ref_history), np.array(nu_history)
    return np.array(errors), np.array(inputs)

def cost(params):
    K_v = params[0:4]
    K_sv = params[4:8]
    
    try:
        errors, inputs = simulate(K_v, K_sv)
    except Exception:
        return 1e9
        
    if not np.all(np.isfinite(errors)):
        return 1e9
        
    weights = np.array([1.0, 1.0, 1.2, 1.0])
    mse = np.mean(errors**2, axis=0)
    weighted_mse = np.sum(mse * weights)
    
    # Input rate of change penalty to avoid chattering
    input_diff = np.diff(inputs, axis=0)
    chattering_penalty = 0.05 * np.sum(np.mean(input_diff**2, axis=0))
    
    return weighted_mse + chattering_penalty

def main():
    print("=" * 60)
    print("🏎️  STARTING EVOLUTIONARY GAIN OPTIMIZER FOR VELOCITY LOOP")
    print("=" * 60)
    
    t0 = time.time()
    
    # Run Scipy Differential Evolution
    res = differential_evolution(
        cost,
        BOUNDS,
        strategy='best1bin',
        maxiter=30,
        popsize=15,
        tol=1e-5,
        mutation=(0.5, 1.0),
        recombination=0.7,
        seed=42,
        disp=True,
        workers=1
    )
    
    duration = time.time() - t0
    print("\n" + "=" * 60)
    print(f"✅ OPTIMIZATION COMPLETED IN {duration:.2f} SECONDS")
    print("=" * 60)
    
    opt_params = res.x
    opt_kv = opt_params[0:4]
    opt_ksv = opt_params[4:8]
    
    print("\n--- Optimized Gain Configurations ---")
    print(f"kv_x:   {opt_kv[0]:.6f} (init: 5.0)")
    print(f"kv_y:   {opt_kv[1]:.6f} (init: 5.0)")
    print(f"kv_z:   {opt_kv[2]:.6f} (init: 0.7)")
    print(f"kv_yaw: {opt_kv[3]:.6f} (init: 0.7)")
    print("")
    print(f"ksv_x:   {opt_ksv[0]:.6f} (init: 0.4)")
    print(f"ksv_y:   {opt_ksv[1]:.6f} (init: 0.4)")
    print(f"ksv_z:   {opt_ksv[2]:.6f} (init: 0.5)")
    print(f"ksv_yaw: {opt_ksv[3]:.6f} (init: 0.8)")
    print("=" * 60)
    
    # Save optimized gains to file
    os.makedirs('/home/brayan/ros2_ws/src/neroControl/data', exist_ok=True)
    gains_file = '/home/brayan/ros2_ws/src/neroControl/data/optimal_velocity_gains.txt'
    with open(gains_file, 'w') as f:
        f.write("# Optimal Velocity Loop Gains (Feedback Linearization)\n")
        f.write(f"# Best Cost: {res.fun:.6f}\n\n")
        f.write("kv_x = %.6f\n" % opt_kv[0])
        f.write("kv_y = %.6f\n" % opt_kv[1])
        f.write("kv_z = %.6f\n" % opt_kv[2])
        f.write("kv_yaw = %.6f\n" % opt_kv[3])
        f.write("\n")
        f.write("ksv_x = %.6f\n" % opt_ksv[0])
        f.write("ksv_y = %.6f\n" % opt_ksv[1])
        f.write("ksv_z = %.6f\n" % opt_ksv[2])
        f.write("ksv_yaw = %.6f\n" % opt_ksv[3])
    print(f"Saved optimized gains to: {gains_file}")
    
    # Plot Before/After comparison
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    
    # Simulate initial
    _, _, t_h, ref_h, nu_init = simulate(DEFAULT_GAINS[0:4], DEFAULT_GAINS[4:8], return_history=True)
    # Simulate optimal
    _, _, _, _, nu_opt = simulate(opt_kv, opt_ksv, return_history=True)
    
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    labels = ['vx (Forward) [m/s]', 'vy (Lateral) [m/s]', 'vz (Vertical) [m/s]', 'vyaw (Yaw Rate) [rad/s]']
    
    for i in range(4):
        axs[i].plot(t_h, ref_h[:, i], 'r--', lw=1.5, label='Reference')
        axs[i].plot(t_h, nu_init[:, i], 'orange', lw=1.5, label='Initial (Default)')
        axs[i].plot(t_h, nu_opt[:, i], 'g-', lw=1.8, label='Optimized')
        axs[i].set_ylabel(labels[i])
        axs[i].grid(True, linestyle=':', alpha=0.6)
        axs[i].legend()
        
    axs[3].set_xlabel('Time (s)')
    plt.tight_layout()
    plot_path = '/home/brayan/ros2_ws/src/neroControl/data/velocity_optimization_results.png'
    plt.savefig(plot_path)
    print(f"Saved comparison plot to: {plot_path}")

if __name__ == '__main__':
    main()
