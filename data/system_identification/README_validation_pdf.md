# Bebop 2 Model Validation & PDF Generator (`validate_model_pdf.py`)

This document summarizes the purpose and implementation details of the `validate_model_pdf.py` script.

---

## 1. Overview of the Validation Process

The script is a headless utility designed to **validate the identified Bebop 2 dynamics** by comparing a forward-simulated state trajectory against experimental telemetry logs. It generates publication-ready vector PDF figures comparing measured and simulated states.

### Core Functions
1.  **Loads Parameters**: Reads the identified input gains ($f_1$), drag coefficients ($f_2$), and input delays ($d$) from a JSON file (by default `system_identification_results_raw_4dof.json`).
2.  **Loads Telemetry Log**: Reads the `.mat` experimental dataset (`manual_log_20260701_170333.mat`).
3.  **Applies Input Delays**: Shifts each of the control inputs ($u_x, u_y, u_z, u_{\text{yaw}}$) by their respective channel delays.
4.  **Integrates Coupled Dynamics**: Runs a joint forward simulation of both velocities and positions.
5.  **Plots & Saves**: Outputs vector graphics PDFs for both velocities and positions using a professional academic paper aesthetic.

---

## 2. Mathematical Model & Integration (RK4)

Instead of integrating velocity and position independently, the script sets up a coupled **8-state space vector**:
$$X(t) = \begin{bmatrix} v_x(t) & v_y(t) & v_z(t) & \omega(t) & x(t) & y(t) & z(t) & \psi(t) \end{bmatrix}^T$$

### Differential Equations
The dynamics function $\dot{X} = f(X, u)$ is defined as:

*   **Body Velocities**:
    $$\dot{v}_x = f_1^x u_x(t - d_x) - f_2^x v_x$$
    $$\dot{v}_y = f_1^y u_y(t - d_y) - f_2^y v_y$$
    $$\dot{v}_z = f_1^z u_z(t - d_z) - f_2^z v_z$$
    $$\dot{\omega} = f_1^{\text{yaw}} u_{\text{yaw}}(t - d_{\text{yaw}}) - f_2^{\text{yaw}} \omega$$
*   **Inertial Positions**:
    $$\dot{x} = v_x \cos\psi - v_y \sin\psi$$
    $$\dot{y} = v_x \sin\psi + v_y \cos\psi$$
    $$\dot{z} = v_z$$
    $$\dot{\psi} = \omega$$

### Numerical Integration Method
The script uses a high-fidelity **4th-Order Runge-Kutta (RK4)** integration scheme to step the state forward from the initial condition:
$$X_{k+1} = X_k + \frac{dt}{6}(k_1 + 2k_2 + 2k_3 + k_4)$$
Where:
*   $k_1 = f(X_k, u_k)$
*   $k_2 = f(X_k + \frac{dt}{2}k_1, u_{\text{mid}})$
*   $k_3 = f(X_k + \frac{dt}{2}k_2, u_{\text{mid}})$
*   $k_4 = f(X_k + dt \cdot k_3, u_{k+1})$
*   $u_{\text{mid}} = \frac{1}{2}(u_k + u_{k+1})$

---

## 3. Key Implementation Details

*   **Adaptive Velocity Source**:
    *   If the JSON file path contains the word `"derived"`, the script compares the simulated trajectory against **numerically derived velocities** calculated from positions rotated to the body frame.
    *   Otherwise, it compares the simulation against the **onboard EKF-estimated velocities** ($v_x, v_y$).
*   **Headless PDF Output**:
    *   Forces the `matplotlib` backend to `"Agg"` (headless/non-interactive mode) via `matplotlib.use("Agg")` so the script can run in automated pipelines without requiring an X-server or displaying windows.
    *   Saves the results directly into vector format PDF files:
        *   `validation_velocities.pdf`
        *   `validation_positions.pdf`
*   **Plot Styling**:
    *   Implements an academic style configuration (serif font, Times New Roman, fine grids, inside ticks, customized legends).
    *   Highlights the tracking error by plotting a **light blue shaded region** (`#d0e1f9`) between the measured and modeled curves.
