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

- **Lyapunov-based nonlinear controller** — model-based PD controller with gravity compensation and a formal Lyapunov stability guarantee.
- **PID controller** — classical decentralized controller used as a baseline.

The goal is **stabilization (regulation)**: drive the manipulator from an arbitrary initial state to a constant desired configuration \( \theta_d \).

---

## 2. Manipulator Model

<p align="center">
  <img src="figures/manipulator_model.png" alt="Two-Link Planar Robot Manipulator" width="600"/>
</p>

<p align="center">
  <em>Figure 1: A simplified model of a two-link planar robot manipulator</em>
</p>

The system consists of two rigid links of lengths \( l_1 \) and \( l_2 \) with point masses \( m_1 \) and \( m_2 \) concentrated at the end of each link.

---

### 2.1. Forward Kinematics

\[
x_1 = l_1 \cos\theta_1, \qquad y_1 = l_1 \sin\theta_1
\]

\[
x_2 = l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2), \qquad
y_2 = l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2)
\]

---

### 2.2. Dynamics (Euler–Lagrange Form)

\[
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta) = a
\]

---

### Inertia Matrix

\[
M(\theta) =
\begin{bmatrix}
(m_1+m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 \cos\theta_2 &
m_2 l_2^2 + m_2 l_1 l_2 \cos\theta_2 \\
m_2 l_2^2 + m_2 l_1 l_2 \cos\theta_2 &
m_2 l_2^2
\end{bmatrix}
\]

---

### Coriolis and Centrifugal Matrix

\[
h = -m_2 l_1 l_2 \sin\theta_2
\]

\[
C(\theta,\dot{\theta}) =
\begin{bmatrix}
h\,\dot{\theta}_2 & h\,(\dot{\theta}_1 + \dot{\theta}_2) \\
-h\,\dot{\theta}_1 & 0
\end{bmatrix}
\]

---

### Gravity Vector

\[
G(\theta) =
\begin{bmatrix}
(m_1 + m_2)\,g\,l_1\,\cos\theta_1 + m_2\,g\,l_2\,\cos(\theta_1 + \theta_2) \\
m_2\,g\,l_2\,\cos(\theta_1 + \theta_2)
\end{bmatrix}
\]

---

## 3. Control Design

### 3.1. Lyapunov-Based Controller

Define the position error:

\[
e = \theta - \theta_d
\]

The control law is:

\[
a = -k_1\,e - k_2\,\dot{\theta} + G(\theta)
\]

**Interpretation:**
- \( -k_1 e \): restoring (spring-like) term  
- \( -k_2 \dot{\theta} \): damping  
- \( +G(\theta) \): gravity compensation  

---

### Closed-Loop Dynamics

\[
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + k_2\,\dot{\theta} + k_1\,e = 0
\]

---

### Lyapunov Function

\[
L = \frac{1}{2}\,\dot{\theta}^T M(\theta)\,\dot{\theta} + \frac{1}{2}\,k_1\,e^T e
\]

---

### Time Derivative

\[
\dot{L} = -k_2\,\|\dot{\theta}\|^2 \leq 0
\]

---

### Stability Result

By LaSalle’s invariance principle:

\[
e(t) \to 0, \quad \dot{\theta}(t) \to 0 \quad \text{as} \quad t \to \infty
\]

The equilibrium \( (\theta, \dot{\theta}) = (\theta_d, 0) \) is **asymptotically stable**.

---

### 3.2. PID Controller

\[
a_{\text{PID}} = K_P\,e + K_I \int_0^t e(\sigma)\,d\sigma + K_D\,\dot{e}
\]

- Independent per joint  
- Does not compensate nonlinear dynamics  

---

## 4. Simulation Parameters

| Parameter | Symbol | Value |
|----------|--------|-------|
| Link 1 length | \( l_1 \) | 1.0 m |
| Link 2 length | \( l_2 \) | 1.0 m |
| Mass 1 | \( m_1 \) | 1.0 kg |
| Mass 2 | \( m_2 \) | 2.0 kg |
| Gravity | \( g \) | 9.81 m/s² |
| Desired joint 1 | \( \theta_{1d} \) | \( \pi/2 \) rad |
| Desired joint 2 | \( \theta_{2d} \) | 0 rad |
| Initial joint 1 | \( \theta_1(0) \) | \( \pi \) rad |
| Initial joint 2 | \( \theta_2(0) \) | 0 rad |
| Simulation time | \( T \) | 10 s |

---

## 5. Results

### 5.1. Lyapunov vs PID — Performance Comparison

<p align="center">
  <img src="figures/comparison_plots.png" alt="Comparison: Lyapunov vs PID" width="900"/>
</p>

<p align="center">
  <em>Figure 2: Comparison of Lyapunov-based and PID controllers — joint angles, tracking errors, and control torques</em>
</p>

Key observations:

- **Lyapunov controller**
  - Fast convergence (< 2 s)
  - No steady-state error
  - Smooth response

- **PID controller**
  - Oscillations due to coupling
  - Slower convergence
  - High control effort

---

### 5.2. Lyapunov Function and Its Derivative

<p align="center">
  <img src="figures/lyapunov_function.png" alt="Lyapunov Function" width="700"/>
</p>

<p align="center">
  <em>Figure 3: Evolution of the Lyapunov function $L(t)$ (top) and its time derivative $\dot{L}(t)$ (bottom)</em>
</p>

- \( L(t) \to 0 \)
- \( \dot{L}(t) \leq 0 \)

---

### 5.3. Phase Portraits

<p align="center">
  <img src="figures/phase_portrait.png" alt="Phase Portraits" width="850"/>
</p>

<p align="center">
  <em>Figure 4: Phase portraits for Joint 1 (left) and Joint 2 (right) under the Lyapunov controller</em>
</p>

- Converging spirals → asymptotic stability  
- Stable focus behavior  

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
pip install -r requirements.txt
python main.py

---

## 8. References

1. Baccouch M., Dodds S. A two-link robot manipulator: Simulation and control design //International Journal of Robotic Engineering. – 2020. – V. 5. – №. 2. – P. 1-17.
