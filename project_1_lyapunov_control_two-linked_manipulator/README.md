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

- **Lyapunov-based nonlinear controller** — model-based PD controller with gravity compensation and a formal stability guarantee  
- **PID controller** — classical decentralized baseline  

The goal is **stabilization (regulation)** to a constant target configuration $\theta_d$.

---

## 2. Notation Table (All Symbols Explained)

| Symbol | Meaning | Units |
|--------|--------|------|
| $\theta = [\theta_1, \theta_2]^T$ | Joint angles vector | rad |
| $\dot{\theta}$ | Joint velocities | rad/s |
| $\ddot{\theta}$ | Joint accelerations | rad/s² |
| $\theta_d$ | Desired joint configuration | rad |
| $e = \theta - \theta_d$ | Position error | rad |
| $a = [\tau_1, \tau_2]^T$ | Control input (torques) | N·m |
| $M(\theta)$ | Inertia matrix | — |
| $C(\theta,\dot{\theta})$ | Coriolis and centrifugal matrix | — |
| $G(\theta)$ | Gravity vector | N·m |
| $k_1$ | Proportional gain ($k_1 > 0$) | — |
| $k_2$ | Derivative gain ($k_2 > 0$) | — |
| $L$ | Lyapunov function | — |
| $\dot{L}$ | Time derivative of Lyapunov function | — |
| $m_1, m_2$ | Link masses | kg |
| $l_1, l_2$ | Link lengths | m |
| $g$ | Gravitational acceleration | m/s² |

---

## 3. Manipulator Model

<p align="center">
  <img src="figures/manipulator_model.png" alt="Two-Link Planar Robot Manipulator" width="600"/>
</p>

<p align="center">
  <em>Figure 1: A simplified model of a two-link planar robot manipulator</em>
</p>

---

### 3.1 Forward Kinematics

$$
x_1 = l_1 \cos\theta_1, \quad y_1 = l_1 \sin\theta_1
$$

$$
x_2 = l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2)
$$

$$
y_2 = l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2)
$$

---

### 3.2 Dynamics

$$
M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + G(\theta) = a
$$

---

### Inertia Matrix

$$
M(\theta) =
\begin{bmatrix}
(m_1+m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 \cos\theta_2 & m_2 l_2^2 + m_2 l_1 l_2 \cos\theta_2 \\
m_2 l_2^2 + m_2 l_1 l_2 \cos\theta_2 & m_2 l_2^2
\end{bmatrix}
$$

---

### Coriolis Matrix

First define the auxiliary scalar:

$$
h = -m_2 l_1 l_2 \sin\theta_2
$$

Then:

$$
C(\theta,\dot{\theta}) =
\begin{bmatrix}
h\dot{\theta}_2 & h(\dot{\theta}_1 + \dot{\theta}_2) \\
-h\dot{\theta}_1 & 0
\end{bmatrix}
$$

---

### Gravity Vector

$$
G(\theta) =
\begin{bmatrix}
(m_1 + m_2)g l_1 \cos\theta_1 + m_2 g l_2 \cos(\theta_1 + \theta_2) \\
m_2 g l_2 \cos(\theta_1 + \theta_2)
\end{bmatrix}
$$

---

## 4. Control Design

---

### 4.1 Error Definition

First define the tracking error:

$$
e = \theta - \theta_d
$$

---

### 4.2 Control Law

$$
a = -k_1 e - k_2 \dot{\theta} + G(\theta)
$$

**Interpretation:**
- $-k_1 e$ — proportional restoring term (acts like a spring)  
- $-k_2 \dot{\theta}$ — damping term (removes energy)  
- $G(\theta)$ — gravity compensation  

---

### 4.3 Closed-loop Dynamics

Substituting the control law into the dynamics:

$$
M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + k_2 \dot{\theta} + k_1 e = 0
$$

---

## 5. Lyapunov Stability Analysis (Detailed)

---

### 5.1 Choice of Lyapunov Function

Consider the candidate:

$$
L = \frac{1}{2}\dot{\theta}^T M(\theta)\dot{\theta} + \frac{1}{2}k_1 e^T e
$$

**Justification:**
- First term represents kinetic energy  
- Second term penalizes position error  
- Both terms are positive definite  

---

### 5.2 Time Derivative

Differentiate:

$$
\dot{L} =
\frac{1}{2}\dot{\theta}^T \dot{M} \dot{\theta}
+ \dot{\theta}^T M \ddot{\theta}
+ k_1 e^T \dot{\theta}
$$

---

### 5.3 Key Identity

From robot dynamics:

$$
\dot{\theta}^T \dot{M} \dot{\theta} = 2 \dot{\theta}^T C \dot{\theta}
$$

Thus:

$$
\dot{L} =
\dot{\theta}^T C \dot{\theta}
+ \dot{\theta}^T M \ddot{\theta}
+ k_1 e^T \dot{\theta}
$$

---

### 5.4 Substitution of Dynamics

Using closed-loop system:

$$
M\ddot{\theta} + C\dot{\theta} = -k_1 e - k_2 \dot{\theta}
$$

Substitute:

$$
\dot{L} =
\dot{\theta}^T (-k_1 e - k_2 \dot{\theta})
+ k_1 e^T \dot{\theta}
$$

---

### 5.5 Simplification

$$
\dot{L} =
- k_1 \dot{\theta}^T e
- k_2 \dot{\theta}^T \dot{\theta}
+ k_1 e^T \dot{\theta}
$$

Since:

$$
\dot{\theta}^T e = e^T \dot{\theta}
$$

They cancel:

$$
\dot{L} = -k_2 \|\dot{\theta}\|^2
$$

---

### 5.6 Final Result

$$
\dot{L} \le 0
$$


The system is **stable**.

## 6. Simulation Parameters

| Parameter | Value |
|----------|------|
| $l_1$ | 1.0 m |
| $l_2$ | 1.0 m |
| $m_1$ | 1.0 kg |
| $m_2$ | 2.0 kg |
| $g$ | 9.81 m/s² |
| $\theta_d$ | $(\pi/2, 0)$ |
| Initial state | $(\pi, 0, 0, 0)$ |
| Time | 10 s |

---

## 7. Results

### Lyapunov vs PID

<p align="center">
  <img src="figures/comparison_plots.png" width="900"/>
</p>

---

### Lyapunov Function

<p align="center">
  <img src="figures/lyapunov_function.png" width="700"/>
</p>

---

### Phase Portrait

<p align="center">
  <img src="figures/phase_portrait.png" width="850"/>
</p>

---

## 8. Project Structure


```
project_1_lyapunov_control_two-linked_manipulator/
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

## 9. How to Run

```bash
pip install -r requirements.txt
python main.py
```
---

## 10. References

1. Baccouch M., Dodds S. A two-link robot manipulator: Simulation and control design //International Journal of Robotic Engineering. – 2020. – V. 5. – №. 2. – P. 1-17.
