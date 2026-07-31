# Bebop 2 System Identification (4-DOF Joint Optimization with Shared Delay)

This document summarizes the system identification process implemented in `minimos_opt_raw_4dof.py` and details the parameter values obtained in `system_identification_results_raw_4dof.json`.

---

## 1. Overview of the Identification Process

The script performs a **joint 4-DOF (Degrees of Freedom) parameter identification** for a Bebop 2 drone using raw, unfiltered telemetry data. It models the velocity dynamics of each channel as a first-order linear system with a shared input command delay.

### Dynamic Model
For each channel $i \in \{x, y, z, \psi\}$, the velocity state $\nu_i$ evolves according to:
$$\dot{\nu}_i(t) = f_{1}^i u_i(t - \tau) - f_{2}^i \nu_i(t)$$

Where:
*   $\nu_x, \nu_y$ are the body longitudinal and lateral translational velocities.
*   $\nu_z$ is the vertical velocity.
*   $\nu_{\psi}$ is the yaw rate.
*   $u_i$ is the control command for the respective channel.
*   $\tau$ is the shared input delay.
*   $f_{1}^i$ is the control input gain for channel $i$.
*   $f_{2}^i$ is the drag/damping coefficient for channel $i$.

---

## 2. Optimization Methodology

The script optimizes **9 parameters** simultaneously:
*   $f_1^x, f_2^x$ (Surge/X parameters)
*   $f_1^y, f_2^y$ (Sway/Y parameters)
*   $f_1^z, f_2^z$ (Heave/Z parameters)
*   $f_1^{\text{yaw}}, f_2^{\text{yaw}}$ (Yaw rate/Yaw parameters)
*   $\tau$ (Shared input delay, represented as an integer sample delay $d$)

### Step-by-Step Execution
1.  **Velocity Estimation**:
    *   $\nu_x$ and $\nu_y$ are directly obtained from the `.mat` log file.
    *   $\nu_z$ and $\nu_{\psi}$ are numerically differentiated from raw positions using backward differences:
        $$\nu_z[k] = \frac{z[k] - z[k-1]}{dt}$$
        $$\nu_{\psi}[k] = \frac{\psi[k] - \psi[k-1]}{dt}$$
2.  **Simulation & Integration**:
    *   The velocities are simulated using Euler integration.
    *   The inertial positions ($x$, $y$, $z$, and yaw angle $\psi$) are simulated by integrating the velocities. Translation velocities $\nu_x, \nu_y$ are rotated to the inertial frame using the measured yaw angle $\psi[k]$.
3.  **Cost Function (Joint Loss)**:
    For each channel, the loss is defined as a sum of normalized RMSEs and an amplitude matching penalty:
    $$\text{Loss}_i = \frac{\text{RMSE}(\eta_i)}{\sigma(\eta_{i, \text{real}})} + \frac{\text{RMSE}(\nu_i)}{\sigma(\nu_{i, \text{real}})} + 3.0 \cdot \frac{|\sigma(\nu_{i, \text{sim}}) - \sigma(\nu_{i, \text{real}})|}{\sigma(\nu_{i, \text{real}})}$$
    Where $\eta$ represents position, $\nu$ represents velocity, and $\sigma(\cdot)$ is the standard deviation. The total loss is:
    $$\text{Total Loss} = \text{Loss}_x + \text{Loss}_y + \text{Loss}_z + \text{Loss}_{\text{yaw}}$$
4.  **Optimization Algorithm**:
    *   **Global Search**: Differential Evolution (`differential_evolution`) runs for up to 100 iterations to find a good global minimum.
    *   **Local Refinement**: An L-BFGS-B optimizer (`minimize`) polishes the solution starting from the best candidate.

---

## 3. Identification Results

Below are the identified parameter values from `system_identification_results_raw_4dof.json`:

### Shared Input Delay
*   **Delay ($d$)**: $6 \text{ samples}$
    *(Note: The corresponding time delay in milliseconds is $6 \times dt$, depending on the telemetry rate $hz$)*

### Identified Coefficients

| Channel | Input Gain ($f_1$) | Damping ($f_2$) |
| :--- | :---: | :---: |
| **X (Surge)** | $0.864684$ | $1.193358$ |
| **Y (Sway)** | $0.614802$ | $0.522566$ |
| **Z (Heave)** | $2.984807$ | $2.519493$ |
| **Yaw** | $2.963938$ | $2.571065$ |

These parameters are saved in JSON format as follows:
```json
{
    "identified_parameters": {
        "x": {
            "f1": 0.8646835429680272,
            "f2": 1.1933576532723789
        },
        "y": {
            "f1": 0.6148021360314251,
            "f2": 0.5225661148208318
        },
        "z": {
            "f1": 2.984806878084027,
            "f2": 2.5194931238243967
        },
        "yaw": {
            "f1": 2.9639382522459274,
            "f2": 2.5710654927237298
        }
    },
    "delays": {
        "x": 6,
        "y": 6,
        "z": 6,
        "yaw": 6
    }
}
```
