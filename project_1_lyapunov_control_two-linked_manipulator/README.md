# Project 1: Lyapunov-Based Control of a Two-Link Robot Manipulator

<p align="center">
  <img src="animations/robot_motion.gif" alt="Manipulator Animation" width="600"/>
</p>

<p align="center">
  <em>Figure 0: Animation of the two-link manipulator stabilizing from an initial configuration (\(\theta_1=\pi, \theta_2=0\)) to the target position (\(\theta_d=[\pi/2, 0]\)) using the Lyapunov-based controller.</em>
</p>

---

## 1. Problem Definition

**Control Problem:** Stabilization (regulation) of a planar two-link robot manipulator. The objective is to drive both joint angles from an arbitrary initial configuration to a desired fixed target configuration \(\theta_d\) and hold them there with zero steady-state error.

**Plant:** A nonlinear, coupled two-degree-of-freedom robotic arm operating in a vertical plane. The system exhibits complex dynamics due to inertial coupling, Coriolis/centrifugal forces, and gravity.

**Method Class:** Model-based nonlinear control using **Lyapunov Stability Analysis**. Specifically, we implement a PD controller with exact gravity compensation. The asymptotic stability of the closed-loop system is formally proven using a Lyapunov function candidate and LaSalle's Invariance Principle.

**Comparison:** The proposed Lyapunov-based controller is compared against a standard decentralized **PID controller**. The PID baseline ignores the dynamic coupling between links and does not explicitly compensate for nonlinearities, serving as a benchmark to demonstrate the superiority of model-based control for this system.

---

## 2. System Description

### Physical Setup

The system consists of two rigid links connected by revolute joints rotating in a vertical plane under the influence of gravity \(g\). Link 1 is attached to a fixed base; Link 2 is attached to the end of Link 1.

<p align="center">
  <img src="figures/manipulator_model.png" alt="Two-Link Planar Robot Manipulator Model" width="500"/>
</p>

<p align="center">
  <em>Figure 1: Schematic of the two-link planar robot manipulator.</em>
</p>

### State Variables

The state vector \(x \in \mathbb{R}^4\) is defined as:

\[
x = [\theta_1, \theta_2, \dot{\theta}_1, \dot{\theta}_2]^T
\]

| Symbol | Meaning | Units |
|---|---|---|
| \(\theta_1, \theta_2\) | Joint angles (Link 1 relative to horizontal, Link 2 relative to Link 1) | rad |
| \(\dot{\theta}_1, \dot{\theta}_2\) | Joint angular velocities | rad/s |
| \(\ddot{\theta}_1, \ddot{\theta}_2\) | Joint angular accelerations | rad/s² |

### Control Input

The control input is the vector of applied joint torques:

\[
a = [\tau_1, \tau_2]^T \in \mathbb{R}^2
\]

> Note: In this project, \(a\) denotes the physical torque vector applied to the joints.

### Dynamic Parameters

| Symbol | Meaning | Value | Units |
|---|---|---:|---|
| \(m_1\) | Mass of Link 1 | 1.0 | kg |
| \(m_2\) | Mass of Link 2 | 2.0 | kg |
| \(l_1\) | Length of Link 1 | 1.0 | m |
| \(l_2\) | Length of Link 2 | 1.0 | m |
| \(g\) | Gravitational acceleration | 9.81 | m/s² |

---

## 3. Mathematical Specification

### Equations of Motion

The dynamics are derived via Lagrangian mechanics [Baccouch & Dodds, 2020] and take the standard form:

\[
M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + G(\theta) = a \tag{1}
\]

where:

- \(\theta = [\theta_1, \theta_2]^T\)
- \(M(\theta) \in \mathbb{R}^{2\times2}\) is the symmetric, positive-definite inertia matrix
- \(C(\theta,\dot{\theta}) \in \mathbb{R}^{2\times2}\) is the Coriolis and centrifugal matrix
- \(G(\theta) \in \mathbb{R}^2\) is the gravity vector

#### Inertia Matrix \(M(\theta)\)

\[
M(\theta) =
\begin{bmatrix}
M_{11} & M_{12} \\
M_{12} & M_{22}
\end{bmatrix}
\]

with elements:

\[
\begin{aligned}
M_{11} &= (m_1 + m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 \cos\theta_2 \\
M_{12} &= m_2 l_2^2 + m_2 l_1 l_2 \cos\theta_2 \\
M_{22} &= m_2 l_2^2
\end{aligned}
\]

#### Coriolis Matrix \(C(\theta, \dot{\theta})\)

Using the auxiliary term \(h = -m_2 l_1 l_2 \sin\theta_2\):

\[
C(\theta,\dot{\theta}) =
\begin{bmatrix}
h\dot{\theta}_2 & h(\dot{\theta}_1 + \dot{\theta}_2) \\
-h\dot{\theta}_1 & 0
\end{bmatrix}
\]

**Key Property:** The matrix \(\dot{M}(\theta) - 2C(\theta,\dot{\theta})\) is skew-symmetric. This implies that for any vector \(x \in \mathbb{R}^2\),

\[
x^T (\dot{M} - 2C) x = 0
\;\Longrightarrow\;
x^T \dot{M} x = 2 x^T C x \tag{2}
\]

#### Gravity Vector \(G(\theta)\)

\[
G(\theta) =
\begin{bmatrix}
(m_1 + m_2)g l_1 \cos\theta_1 + m_2 g l_2 \cos(\theta_1 + \theta_2) \\
m_2 g l_2 \cos(\theta_1 + \theta_2)
\end{bmatrix}
\]

---

## 4. Method Description and Stability Proof

### 4.1 Control Law

We define the tracking error as \(e = \theta - \theta_d\), where \(\theta_d\) is the constant target configuration. Since \(\theta_d\) is constant, \(\dot{e} = \dot{\theta}\) and \(\ddot{e} = \ddot{\theta}\).

The proposed control law is a **PD controller with gravity compensation**:

\[
a = -k_1 e - k_2 \dot{\theta} + G(\theta) \tag{3}
\]

where \(k_1 > 0\) and \(k_2 > 0\) are scalar gain coefficients (or diagonal matrices).

- \(-k_1 e\): proportional feedback (restoring force)
- \(-k_2 \dot{\theta}\): derivative feedback (damping)
- \(+G(\theta)\): exact cancellation of gravitational torques

### 4.2 Closed-Loop Dynamics

Substituting (3) into (1):

\[
M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + G(\theta)
= -k_1 e - k_2 \dot{\theta} + G(\theta)
\]

Canceling \(G(\theta)\) and rearranging:

\[
M(\theta)\ddot{\theta} + C(\theta,\dot{\theta})\dot{\theta} + k_2 \dot{\theta} + k_1 e = 0 \tag{4}
\]

### 4.3 Lyapunov Stability Analysis

To prove asymptotic stability, we use Lyapunov's direct method.

**Step 1: Lyapunov Function Candidate**

Consider the energy-like function \(L(e, \dot{\theta})\):

\[
L(e, \dot{\theta})
=
\underbrace{\frac{1}{2}\dot{\theta}^T M(\theta)\dot{\theta}}_{\text{Kinetic Energy}}
+
\underbrace{\frac{1}{2}k_1 e^T e}_{\text{Potential Energy}} \tag{5}
\]

Since \(M(\theta)\) is positive definite and \(k_1 > 0\), \(L(e, \dot{\theta}) > 0\) for all \((e, \dot{\theta}) \neq (0,0)\), and \(L(0,0)=0\). Thus, \(L\) is positive definite.

**Step 2: Time Derivative of \(L\)**

Differentiating \(L\) with respect to time:

\[
\dot{L}
=
\frac{1}{2}\dot{\theta}^T \dot{M} \dot{\theta}
+ \dot{\theta}^T M \ddot{\theta}
+ k_1 e^T \dot{e}
\]

Using \(\dot{e} = \dot{\theta}\) and the skew-symmetry property (2), namely
\(\frac{1}{2}\dot{\theta}^T \dot{M} \dot{\theta} = \dot{\theta}^T C \dot{\theta}\), we obtain:

\[
\dot{L}
=
\dot{\theta}^T C \dot{\theta}
+ \dot{\theta}^T M \ddot{\theta}
+ k_1 e^T \dot{\theta}
\]

Factoring out \(\dot{\theta}^T\):

\[
\dot{L}
=
\dot{\theta}^T (M \ddot{\theta} + C \dot{\theta}) + k_1 e^T \dot{\theta} \tag{6}
\]

**Step 3: Substitution and Simplification**

From the closed-loop dynamics (4), we have:

\[
M \ddot{\theta} + C \dot{\theta} = -k_2 \dot{\theta} - k_1 e
\]

Substituting this into (6):

\[
\dot{L}
=
\dot{\theta}^T (-k_2 \dot{\theta} - k_1 e) + k_1 e^T \dot{\theta}
\]

\[
\dot{L}
=
-k_2 \dot{\theta}^T \dot{\theta}
- k_1 \dot{\theta}^T e
+ k_1 e^T \dot{\theta}
\]

Since \(\dot{\theta}^T e\) is a scalar, it equals its transpose \(e^T \dot{\theta}\). These terms cancel:

\[
\dot{L} = -k_2 \| \dot{\theta} \|^2 \tag{7}
\]

**Conclusion**

Since \(k_2 > 0\), \(\dot{L} \leq 0\) for all states. Moreover, \(\dot{L} = 0\) implies \(\dot{\theta} = 0\).

If \(\dot{\theta} \equiv 0\), then \(\ddot{\theta} \equiv 0\). Substituting into (4) gives \(k_1 e = 0 \Rightarrow e = 0\).

By **LaSalle's Invariance Principle**, the equilibrium point \((e, \dot{\theta}) = (0,0)\) is **globally asymptotically stable**.

---

## 5. Algorithm Listing

The control algorithm executed at each time step \(t\):

1. **Read State:** Obtain current \(\theta(t)\) and \(\dot{\theta}(t)\).
2. **Compute Error:** \(e = \theta_d - \theta(t)\).
3. **Compute Dynamics Matrices:** Calculate \(M(\theta)\), \(C(\theta, \dot{\theta})\), and \(G(\theta)\) using the current state.
4. **Compute Control Input:**
   \[
   a(t) = -k_1 e - k_2 \dot{\theta}(t) + G(\theta)
   \]
5. **Apply Torque:** Apply \(a(t)\) to the plant.
6. **Integrate Dynamics:** Solve
   \[
   \ddot{\theta} = M^{-1}(a - C\dot{\theta} - G)
   \]
   to update the state for \(t+\Delta t\).

---

## 6. Experimental Setup

| Parameter | Value | Description |
|---|---|---|
| **Initial State** | \(\theta=[\pi, 0], \dot{\theta}=[0,0]\) | Arm starts pointing left, stationary. |
| **Target State** | \(\theta_d=[\pi/2, 0]\) | Target is vertical (upright). |
| **Simulation Time** | 10 seconds | Duration of the experiment. |
| **Controller Gains** | \(k_1=100, k_2=20\) | High stiffness and damping for fast response. |
| **PID Gains** | \(K_P=30, K_I=20, K_D=15\) | Baseline gains per joint. |

---

## 7. Results and Discussion

### 7.1 Comparison: Lyapunov vs. PID

<p align="center">
  <img src="figures/comparison_plots.png" alt="Comparison Plots" width="900"/>
</p>

<p align="center">
  <em>Figure 2: Comparison of joint angles, tracking errors, and control torques for the Lyapunov-based controller (left) and PID controller (right).</em>
</p>

**Interpretation of Figure 2**

- **Joint Angles and Errors (Top/Middle Rows):** The Lyapunov controller achieves fast convergence (~2–3 s) with minimal overshoot. The error decays exponentially to zero. In contrast, the PID controller exhibits significant oscillations and slower settling time (~5–7 s) because it fails to compensate for the inertial coupling (\(M_{12}\)) and Coriolis forces.
- **Control Action (Bottom Row):** This is the most critical difference. The Lyapunov controller produces smooth, physically realizable torques (peaking around \(\pm 100\) N·m). The PID controller generates massive initial torque spikes (reaching **-2500 N·m**) to overcome the unmodeled dynamics. Such high torques would saturate real-world actuators, making the PID approach impractical for high-performance tasks.

### 7.2 Lyapunov Function Evolution

<p align="center">
  <img src="figures/lyapunov_function.png" alt="Lyapunov Function Plot" width="700"/>
</p>

<p align="center">
  <em>Figure 3: Time evolution of the Lyapunov function \(L(t)\) (blue) and its derivative \(\dot{L}(t)\) (red).</em>
</p>

**Interpretation of Figure 3**

- **\(L(t)\) (Blue):** The function decreases monotonically from its initial value (~120) to zero, confirming the theoretical proof that the system energy dissipates over time.
- **\(\dot{L}(t)\) (Red):** The derivative remains non-positive (\(\dot{L} \leq 0\)) throughout the simulation, validating the stability condition derived in Section 4.3. The initial sharp drop corresponds to the rapid application of control torque to stabilize the arm from the unstable initial position.

### 7.3 Phase Portrait

<p align="center">
  <img src="figures/phase_portrait.png" alt="Phase Portrait" width="850"/>
</p>

<p align="center">
  <em>Figure 4: Phase portraits for Joint 1 (left) and Joint 2 (right), showing trajectories converging to the target (red star).</em>
</p>

**Interpretation of Figure 4**

- The trajectories spiral inward toward the target state (marked with a red star), which is characteristic of a stable, underdamped second-order system.
- The smooth convergence without limit cycles confirms the absence of sustained oscillations, further verifying the effectiveness of the damping term \(-k_2 \dot{\theta}\) in the control law.

---

## 8. Project Structure

```text
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

1. Baccouch, M., & Dodds, S. (2020). *A two-link robot manipulator: Simulation and control design*. International Journal of Robotic Engineering, 5(2), 1–17.
