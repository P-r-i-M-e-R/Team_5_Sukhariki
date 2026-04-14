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

## 2. Manipulator Model

<p align="center">
  <img src="figures/manipulator_model.png" alt="Two-Link Planar Robot Manipulator" width="600"/>
</p>

<p align="center">
  <em>Figure 1: A simplified model of a two-link planar robot manipulator</em>
</p>

The system consists of two rigid links with lengths $l_1, l_2$ and masses $m_1, m_2$.

---

### 2.1 Forward Kinematics

$x_1 = l_1 \cos\theta_1, \quad y_1 = l_1 \sin\theta_1$

$x_2 = l_1 \cos\theta_1 + l_2 \cos(\theta_1 + \theta_2)$  
$y_2 = l_1 \sin\theta_1 + l_2 \sin(\theta_1 + \theta_2)$

---

### 2.2 Dynamics

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

$$
h = -m_2 l_1 l_2 \sin\theta_2
$$

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

## 3. Control Design

### 3.1 Lyapunov Controller

Error definition:

$$
e = \theta - \theta_d
$$

Control law:

$$
a = -k_1 e - k_2 \dot{\theta} + G(\theta)
$$

---

### Closed-loop Dynamics

$$
M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + k_2 \dot{\theta} + k_1 e = 0
$$

---

### Lyapunov Function

$$
L = \frac{1}{2}\dot{\theta}^T M(\theta)\dot{\theta} + \frac{1}{2}k_1 e^T e
$$

---

### Time Derivative

$$
\dot{L} = -k_2 \|\dot{\theta}\|^2 \le 0
$$

---

### Stability Result

$$
e(t) \to 0, \quad \dot{\theta}(t) \to 0 \quad \text{as } t \to \infty
$$

---

### 3.2 PID Controller

$$
a_{\text{PID}} = K_P e + K_I \int_0^t e(\sigma)d\sigma + K_D \dot{e}
$$

---

## 4. Simulation Parameters

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

## 5. Results

### 5.1 Lyapunov vs PID — Performance Comparison

<p align="center">
  <img src="figures/comparison_plots.png" alt="Comparison: Lyapunov vs PID" width="900"/>
</p>

<p align="center">
  <em>Figure 2: Comparison of Lyapunov-based and PID controllers — joint angles, tracking errors, and control torques</em>
</p>

---

### 5.2 Lyapunov Function

<p align="center">
  <img src="figures/lyapunov_function.png" alt="Lyapunov Function" width="700"/>
</p>

<p align="center">
  <em>Figure 3: Evolution of the Lyapunov function $L(t)$ and its derivative</em>
</p>

---

### 5.3 Phase Portraits

<p align="center">
  <img src="figures/phase_portrait.png" alt="Phase Portraits" width="850"/>
</p>

<p align="center">
  <em>Figure 4: Phase portraits under the Lyapunov controller</em>
</p>

--- 

---

## 6. Project Structure


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

## 7. How to Run

```bash
pip install -r requirements.txt
python main.py
```
---

## 8. References

1. Baccouch M., Dodds S. A two-link robot manipulator: Simulation and control design //International Journal of Robotic Engineering. – 2020. – V. 5. – №. 2. – P. 1-17.
