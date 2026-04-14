# Lyapunov-based Control of a Two-Link Planar Robot Manipulator

<p align="center">
  <img src="animations/robot_motion.gif" alt="Manipulator Animation" width="600"/>
</p>

<p align="center">
  <em>Lyapunov-based controller driving the manipulator from initial configuration to the target position</em>
</p>

---

## 1. Project Overview

This project implements and compares two control strategies for a **two-link planar robot manipulator**:

- **Lyapunov-based nonlinear controller** — model-based control law derived from Lyapunov stability theory, guaranteeing asymptotic stability of the tracking error.
- **PID controller** — classical linear feedback controller used as a baseline for comparison.

The dynamic model is derived using the **Euler–Lagrange formulation**. Simulation results demonstrate that the Lyapunov-based controller provides superior tracking performance, faster convergence, and bounded control effort compared to PID.

---

## 2. Manipulator Model

<p align="center">
  <img src="figures/manipulator_model.png" alt="Two-Link Planar Robot Manipulator" width="600"/>
</p>

<p align="center">
  <em>Figure 1: A simplified model of a two-link planar robot manipulator</em>
</p>

The system consists of two rigid links of lengths $L_1$ and $L_2$ with point masses $M_1$ and $M_2$ concentrated at the end of each link. The generalized coordinates are the joint angles $\theta_1$ and $\theta_2$.

### 2.1. Forward Kinematics

The Cartesian coordinates of the joint masses are:

$$x_1 = L_1 \cos\theta_1, \qquad y_1 = L_1 \sin\theta_1$$

$$x_2 = L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2), \qquad y_2 = L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)$$

### 2.2. Euler–Lagrange Dynamics

The equations of motion are derived from the Lagrangian $\mathcal{L} = T - U$ where $T$ is the kinetic energy and $U$ is the potential energy:

$$T = \frac{1}{2}(M_1 + M_2)L_1^2\,\dot\theta_1^2 + \frac{1}{2}M_2 L_2^2(\dot\theta_1 + \dot\theta_2)^2 + M_2 L_1 L_2 \dot\theta_1(\dot\theta_1 + \dot\theta_2)\cos\theta_2$$

$$U = (M_1 + M_2)\,g\,L_1\sin\theta_1 + M_2\,g\,L_2\sin(\theta_1 + \theta_2)$$

The resulting standard manipulator equation is:

$$M(\theta)\,\ddot\theta + C(\theta,\dot\theta)\,\dot\theta + G(\theta) = \tau$$

where:

**Inertia matrix** $M(\theta)$:

$$M(\theta) = \begin{bmatrix} (M_1 + M_2)L_1^2 + M_2 L_2^2 + 2M_2 L_1 L_2 \cos\theta_2 & M_2 L_2^2 + M_2 L_1 L_2 \cos\theta_2 \\ M_2 L_2^2 + M_2 L_1 L_2 \cos\theta_2 & M_2 L_2^2 \end{bmatrix}$$

**Coriolis and centrifugal matrix** $C(\theta, \dot\theta)$:

$$C(\theta,\dot\theta) = \begin{bmatrix} -M_2 L_1 L_2 \sin\theta_2\;\dot\theta_2 & -M_2 L_1 L_2 \sin\theta_2\;(\dot\theta_1 + \dot\theta_2) \\ M_2 L_1 L_2 \sin\theta_2\;\dot\theta_1 & 0 \end{bmatrix}$$

**Gravity vector** $G(\theta)$:

$$G(\theta) = \begin{bmatrix} (M_1 + M_2)\,g\,L_1\cos\theta_1 + M_2\,g\,L_2\cos(\theta_1 + \theta_2) \\ M_2\,g\,L_2\cos(\theta_1 + \theta_2) \end{bmatrix}$$

---

## 3. Control Design

### 3.1. Lyapunov-based Controller

Define the tracking error and its derivative:

$$e = \theta - \theta_d, \qquad \dot{e} = \dot\theta - \dot\theta_d$$

Introduce the composite error variable:

$$s = \dot{e} + \Lambda\, e$$

where $\Lambda = \text{diag}(\lambda_1, \lambda_2)$ is a positive definite diagonal matrix.

The **Lyapunov function** is chosen as:

$$L(t) = \frac{1}{2}\,s^T M(\theta)\,s + \frac{1}{2}\,e^T K_P\, e$$

where $K_P = \text{diag}(k_{p1}, k_{p2})$ is a positive definite gain matrix.

The control law that guarantees $\dot{L}(t) \leq 0$ is:

$$\tau = M(\theta)\,(\ddot\theta_d - \Lambda\,\dot{e}) + C(\theta,\dot\theta)\,\dot\theta + G(\theta) - K_D\,s - K_P\,e$$

where $K_D = \text{diag}(k_{d1}, k_{d2})$ is a positive definite damping gain matrix.

**Stability proof.** Taking the time derivative of $L(t)$ and substituting the control law yields:

$$\dot{L}(t) = -s^T K_D\, s \leq 0$$

Since $\dot{L}(t)$ is negative semi-definite and $L(t)$ is positive definite and radially unbounded, the system is **stable in the sense of Lyapunov**. By LaSalle's invariance principle, the tracking error converges asymptotically to zero:

$$e(t) \to 0, \quad \dot{e}(t) \to 0 \quad \text{as} \quad t \to \infty$$

### 3.2. PID Controller

The baseline PID controller is defined as:

$$\tau_{\text{PID}} = K_P\,e + K_I\int_0^t e(\sigma)\,d\sigma + K_D\,\dot{e}$$

This controller does not exploit the manipulator dynamics and serves as a comparison baseline.

---

## 4. Simulation Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Link 1 length | $L_1$ | 1.0 m |
| Link 2 length | $L_2$ | 0.7 m |
| Mass 1 | $M_1$ | 3.0 kg |
| Mass 2 | $M_2$ | 2.0 kg |
| Gravity | $g$ | 9.81 m/s² |
| Desired joint 1 | $\theta_{1d}$ | $\pi/2$ rad |
| Desired joint 2 | $\theta_{2d}$ | 0 rad |
| Initial joint 1 | $\theta_1(0)$ | $\pi$ rad |
| Initial joint 2 | $\theta_2(0)$ | 0.25 rad |
| Simulation time | $T$ | 10 s |

---

## 5. Results

### 5.1. Lyapunov vs PID — Performance Comparison

<p align="center">
  <img src="figures/comparison_plots.png" alt="Comparison: Lyapunov vs PID" width="900"/>
</p>

<p align="center">
  <em>Figure 2: Comparison of Lyapunov-based and PID controllers — joint angles, tracking errors, and control torques</em>
</p>

Key observations from the comparison:

- **Joint angles.** The Lyapunov controller drives both $\theta_1$ and $\theta_2$ to their desired values ($\theta_{1d} = \pi/2$, $\theta_{2d} = 0$) within approximately 4–5 seconds with moderate overshoot. The PID controller exhibits significantly larger oscillations and fails to converge within the simulation window.
- **Tracking error.** The Lyapunov-based tracking errors $e_1$ and $e_2$ converge monotonically to zero after initial transients. The PID errors remain oscillatory and grow over time, indicating instability for the chosen gains.
- **Control action.** The Lyapunov controller produces bounded torques with a peak of approximately 200 N·m, decaying to zero at steady state. The PID controller demands extremely high torques (exceeding 2500 N·m), which is impractical for real actuators.

### 5.2. Lyapunov Function and Its Derivative

<p align="center">
  <img src="figures/lyapunov_function.png" alt="Lyapunov Function" width="700"/>
</p>

<p align="center">
  <em>Figure 3: Evolution of the Lyapunov function $L(t)$ (top) and its time derivative $\dot{L}(t)$ (bottom)</em>
</p>

The plots confirm the theoretical stability guarantees:

- $L(t) > 0$ for all $t > 0$ and $L(t) \to 0$ as $t \to \infty$, which verifies the positive definiteness of the Lyapunov function.
- $\dot{L}(t) \leq 0$ for all $t$, confirming that the energy-like function is monotonically non-increasing. This is the direct numerical validation of the inequality $\dot{L} = -s^T K_D\,s \leq 0$.

### 5.3. Phase Portraits

<p align="center">
  <img src="figures/phase_portrait.png" alt="Phase Portraits" width="850"/>
</p>

<p align="center">
  <em>Figure 4: Phase portraits for Joint 1 (left) and Joint 2 (right) under the Lyapunov controller</em>
</p>

The phase portraits show the state-space trajectories $(\theta_i, \dot\theta_i)$ for each joint:

- **Joint 1** starts at $(\pi, 0)$ (green dot) and spirals inward toward the target $(\pi/2, 0)$ (red star), confirming asymptotic convergence to the desired equilibrium.
- **Joint 2** starts at $(0.25, 0)$ and follows a similar convergent spiral toward $(0, 0)$.

The inward-spiraling trajectories are characteristic of a **stable focus**, consistent with the underdamped but stable behavior predicted by the Lyapunov analysis.

---

## 6. Project Structure

```
project_1_lyapunov_control_two_link_manipulator/
├── README.md                 
├── requirements.txt          
├── main.py                   
├── configs/
│   └── params.yaml            
├── src/                       
│   ├── __init__.py
│   ├── system.py              
│   ├── lyapunov_controller.py 
│   ├── pid_controller.py      
│   ├── simulation.py          
│   └── visualization.py       
├── figures/                   
│   ├── comparison_plots.png   
│   ├── lyapunov_function.png  
│   ├── manipulator_model.png  
│   └── phase_portrait.png     
└── animations/               
    └── robot_motion.gif       
```

---

## 7. How to Run

```bash
# Install dependencies
pip install numpy scipy matplotlib

# Run simulation
python main.py
```

---

## 8. References

1. Baccouch M., Dodds S. A two-link robot manipulator: Simulation and control design //International Journal of Robotic Engineering. – 2020. – V. 5. – №. 2. – P. 1-17.
