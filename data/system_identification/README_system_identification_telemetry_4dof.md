# Bebop 2 System Identification (4-DOF Joint Optimization with Shared Delay)

This document summarizes the system identification process implemented in `system_identification_telemetry_4dof.py` and details the parameter values obtained in `system_identification_parameters_telemetry_4dof.json`.

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

---

## 3. Identification Results

Below are the identified parameter values from `system_identification_parameters_telemetry_4dof.json`:

### Shared Input Delay
*   **Delay ($d$)**: $6 \text{ samples}$

### Identified Coefficients

| Channel | Input Gain ($f_1$) | Damping ($f_2$) |
| :--- | :---: | :---: |
| **X (Surge)** | $0.864684$ | $1.193358$ |
| **Y (Sway)** | $0.614802$ | $0.522566$ |
| **Z (Heave)** | $2.984807$ | $2.519493$ |
| **Yaw** | $2.963938$ | $2.571065$ |

These parameters are saved in JSON format in `system_identification_parameters_telemetry_4dof.json`.
