# Project 1: Lyapunov-Based Control of a Two-Link Robot Manipulator

---

## 4.1 Problem Definition

**Control problem.**
Stabilization (regulation) of a planar two-link robot manipulator: starting from an arbitrary initial configuration, drive both joint angles to a desired fixed target configuration and hold them there.

**Plant.**
A nonlinear, coupled two-degree-of-freedom robotic arm operating in a vertical plane. The arm is subject to inertial coupling between links, Coriolis and centrifugal effects, and gravitational torques.

**Assumptions.**
- Both links are rigid bodies with point masses concentrated at the end of each link.
- Joint friction is neglected.
- The dynamic model of the plant is known exactly (no model uncertainty).
- There are no external disturbances.
- Joint torques are unbounded (no actuator saturation).

**Class of methods.**
Model-based nonlinear control using Lyapunov stability analysis. Specifically, a PD controller with gravity compensation, whose asymptotic stability is formally proven via a Lyapunov function candidate and LaSalle's invariance principle.

**Comparison.**
The proposed Lyapunov-based controller is compared against a standard decentralized PID controller that ignores the coupling between links and does not compensate for the nonlinear dynamics.

---

## 4.2 System Description

### Physical setup

The system consists of two rigid links connected by revolute joints rotating in a vertical plane. Link 1 is attached to a fixed base; link 2 is attached to the end of link 1. Gravity acts downward.

### State variables

The full state of the system is a vector of dimension 4:

$$
x = [\theta_1,\; \theta_2,\; \dot{\theta}_1,\; \dot{\theta}_2]^T
$$

| Symbol | Meaning | Units |
|---|---|---|
| $\theta_1$ | Angle of link 1, measured from the positive horizontal axis | rad |
| $\theta_2$ | Angle of link 2 relative to link 1 | rad |
| $\dot{\theta}_1$ | Angular velocity of link 1 | rad/s |
| $\dot{\theta}_2$ | Angular velocity of link 2 | rad/s |

### Control input

$$
a = [\tau_1,\; \tau_2]^T
$$

where $\tau_1$ and $\tau_2$ are the torques applied at joint 1 and joint 2, respectively. Units: N·m. In this project, $a$ denotes the control action (generalized force vector) applied to the system.

### Physical parameters

| Symbol | Meaning | Value | Units |
|---|---|---|---|
| $m_1$ | Mass of link 1 | 1.0 | kg |
| $m_2$ | Mass of link 2 | 2.0 | kg |
| $l_1$ | Length of link 1 | 1.0 | m |
| $l_2$ | Length of link 2 | 1.0 | m |
| $g$ | Gravitational acceleration | 9.81 | m/s² |

### Control bounds

No torque limits are imposed in this project. The actuators are assumed ideal.

### Equations of state dynamics

The dynamics of the two-link manipulator, derived via Lagrangian mechanics (see Baccouch & Dodds, 2020), have the standard form:

$$
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta) = a \tag{1}
$$

where:

- $\theta = [\theta_1, \theta_2]^T \in \mathbb{R}^2$ — joint angle vector.
- $\dot{\theta} = [\dot{\theta}_1, \dot{\theta}_2]^T \in \mathbb{R}^2$ — joint velocity vector.
- $\ddot{\theta} = [\ddot{\theta}_1, \ddot{\theta}_2]^T \in \mathbb{R}^2$ — joint acceleration vector.
- $M(\theta) \in \mathbb{R}^{2\times2}$ — inertia matrix (symmetric, positive definite).
- $C(\theta,\dot{\theta}) \in \mathbb{R}^{2\times2}$ — Coriolis and centrifugal matrix.
- $G(\theta) \in \mathbb{R}^2$ — gravity vector.
- $a \in \mathbb{R}^2$ — control action (applied joint torques).

**Inertia matrix $M(\theta)$:**

$$
M(\theta) = \begin{bmatrix} M_{11} & M_{12} \\ M_{12} & M_{22} \end{bmatrix}
$$

$$
M_{11} = (m_1 + m_2)\,l_1^2 + m_2\,l_2^2 + 2\,m_2\,l_1\,l_2\,\cos\theta_2
$$

$$
M_{12} = m_2\,l_2^2 + m_2\,l_1\,l_2\,\cos\theta_2
$$

$$
M_{22} = m_2\,l_2^2
$$

The matrix $M(\theta)$ is symmetric ($M_{21} = M_{12}$) and positive definite for all $\theta$, which means it is always invertible and $x^T M\,x > 0$ for any nonzero vector $x$.

**Coriolis and centrifugal matrix $C(\theta, \dot{\theta})$:**

Define the auxiliary quantity:

$$
h = -m_2\,l_1\,l_2\,\sin\theta_2
$$

Then:

$$
C(\theta,\dot{\theta}) = \begin{bmatrix} h\,\dot{\theta}_2 & h\,(\dot{\theta}_1 + \dot{\theta}_2) \\ -h\,\dot{\theta}_1 & 0 \end{bmatrix}
$$

This particular form of $C$ is constructed using Christoffel symbols so that the matrix $\dot{M}(\theta) - 2\,C(\theta,\dot{\theta})$ is skew-symmetric (see Section 4.3).

**Gravity vector $G(\theta)$:**

$$
G_1 = (m_1 + m_2)\,g\,l_1\,\cos\theta_1 + m_2\,g\,l_2\,\cos(\theta_1 + \theta_2)
$$

$$
G_2 = m_2\,g\,l_2\,\cos(\theta_1 + \theta_2)
$$

The gravity vector is the gradient of the gravitational potential energy $U_g(\theta)$ with respect to $\theta$:

$$
G(\theta) = \frac{\partial U_g}{\partial \theta}, \qquad U_g = (m_1 + m_2)\,g\,l_1\,\sin\theta_1 + m_2\,g\,l_2\,\sin(\theta_1 + \theta_2)
$$

---

## 4.3 Mathematical Specification

### Complete notation table

All symbols used in this project are listed below. No symbol is reused with a different meaning.

| Symbol | Type | Meaning |
|---|---|---|
| $\theta$ | $\mathbb{R}^2$ | Joint angle vector $[\theta_1, \theta_2]^T$ |
| $\dot{\theta}$ | $\mathbb{R}^2$ | Joint angular velocity vector |
| $\ddot{\theta}$ | $\mathbb{R}^2$ | Joint angular acceleration vector |
| $\theta_d$ | $\mathbb{R}^2$ | Desired (target) joint angle vector (constant) |
| $e$ | $\mathbb{R}^2$ | Position error: $e = \theta - \theta_d$ |
| $a$ | $\mathbb{R}^2$ | Control action (applied torque vector) |
| $M(\theta)$ | $\mathbb{R}^{2\times2}$ | Inertia matrix (symmetric, positive definite) |
| $\dot{M}(\theta)$ | $\mathbb{R}^{2\times2}$ | Time derivative of the inertia matrix |
| $C(\theta,\dot{\theta})$ | $\mathbb{R}^{2\times2}$ | Coriolis and centrifugal matrix |
| $G(\theta)$ | $\mathbb{R}^2$ | Gravity vector |
| $k_1$ | $\mathbb{R},\; k_1 > 0$ | Proportional gain (position feedback) |
| $k_2$ | $\mathbb{R},\; k_2 > 0$ | Derivative gain (velocity feedback) |
| $L$ | $\mathbb{R}_{\geq 0}$ | Lyapunov function candidate |
| $\dot{L}$ | $\mathbb{R}$ | Time derivative of the Lyapunov function |
| $h$ | $\mathbb{R}$ | Auxiliary quantity $h = -m_2 l_1 l_2 \sin\theta_2$ |
| $m_1, m_2$ | $\mathbb{R}_{>0}$ | Link masses |
| $l_1, l_2$ | $\mathbb{R}_{>0}$ | Link lengths |
| $g$ | $\mathbb{R}_{>0}$ | Gravitational acceleration |
| $t$ | $\mathbb{R}_{\geq 0}$ | Time |
| $\|\cdot\|$ | — | Euclidean norm |

### Key property of the dynamic model

The Coriolis matrix $C(\theta, \dot{\theta})$ is constructed (via Christoffel symbols) so that the matrix $\dot{M}(\theta) - 2\,C(\theta, \dot{\theta})$ is **skew-symmetric**. This means that for any vector $x \in \mathbb{R}^2$:

$$
x^T \bigl(\dot{M}(\theta) - 2\,C(\theta, \dot{\theta})\bigr)\,x = 0 \tag{2}
$$

An equivalent and useful rewriting:

$$
x^T \dot{M}(\theta)\,x = 2\,x^T C(\theta, \dot{\theta})\,x \tag{2'}
$$

This property is a direct consequence of the Lagrangian structure of the dynamics and is essential for the stability proof in Section 4.4.

---

## 4.4 Method Description

### Control law

We begin with the simplest case: stabilization at the origin $\theta = 0$. A natural choice for the control action is a linear state feedback with gravity compensation:

$$
a = -k_1\,\theta - k_2\,\dot{\theta} + G(\theta)
$$

This law drives the manipulator to the zero configuration $\theta = 0$. However, in practice we want to choose an arbitrary target position $\theta_d \neq 0$ for the end-effector. To achieve this, we replace $\theta$ with the position error $e = \theta - \theta_d$, obtaining the general form:

$$
a = -k_1\,(\theta - \theta_d) - k_2\,\dot{\theta} + G(\theta) \tag{3}
$$

or equivalently, using the error $e = \theta - \theta_d$:

$$
a = -k_1\,e - k_2\,\dot{\theta} + G(\theta) \tag{3'}
$$

**Interpretation of each term:**
- $-k_1\,e$: a restoring force proportional to the position error, pulling the system toward $\theta_d$. Analogous to a spring.
- $-k_2\,\dot{\theta}$: a damping force proportional to velocity, dissipating kinetic energy. Analogous to a viscous damper.
- $+G(\theta)$: exact compensation of the gravitational torque. Without this term, gravity would prevent convergence to an arbitrary target and the Lyapunov proof would not close (see derivation below).

### Why gravity compensation is necessary

If we used $a = -k_1\,e - k_2\,\dot{\theta}$ without the $+G(\theta)$ term, the derivative of the Lyapunov function would contain the residual $-\dot{\theta}^T G(\theta)$, which can be positive or negative depending on the configuration. This means we could not guarantee $\dot{L} \leq 0$, and the stability proof would fail.

### Closed-loop dynamics

Substituting the control law (3') into the plant dynamics (1):

$$
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta) = -k_1\,e - k_2\,\dot{\theta} + G(\theta)
$$

The gravity vectors $G(\theta)$ cancel:

$$
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} = -k_1\,e - k_2\,\dot{\theta}
$$

Rearranging:

$$
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + k_2\,\dot{\theta} + k_1\,e = 0 \tag{4}
$$

Note that since $\theta_d$ is constant, $\dot{e} = \dot{\theta}$ and $\ddot{e} = \ddot{\theta}$.

**Important:** the closed-loop system (4) remains **nonlinear** — the matrices $M(\theta)$ and $C(\theta, \dot{\theta})$ still depend on the state. This is not feedback linearization (Computed Torque Control). The stability of this nonlinear system is proven directly via a Lyapunov argument below.

### Lyapunov stability proof

#### Step 1. Lyapunov function candidate

We choose the following scalar function $L(e, \dot{\theta})$:

$$
L = \frac{1}{2}\,\dot{\theta}^T M(\theta)\,\dot{\theta} \;+\; \frac{1}{2}\,k_1\,e^T e \tag{5}
$$

- The first term $\frac{1}{2}\,\dot{\theta}^T M(\theta)\,\dot{\theta}$ is the kinetic energy of the system. Since $M(\theta)$ is positive definite, this term is strictly positive whenever $\dot{\theta} \neq 0$ and zero only when $\dot{\theta} = 0$.
- The second term $\frac{1}{2}\,k_1\,e^T e$ is a quadratic penalty on the position error. Since $k_1 > 0$, this term is strictly positive whenever $e \neq 0$ and zero only when $e = 0$.

Therefore:

$$
L(e, \dot{\theta}) > 0 \quad \forall\; (e, \dot{\theta}) \neq (0, 0), \qquad L(0, 0) = 0
$$

The function $L$ is **positive definite** — a valid Lyapunov function candidate.

#### Step 2. Time derivative of $L$

Differentiate $L$ with respect to time $t$ along the trajectories of the system:

$$
\dot{L} = \frac{d}{dt}\!\left(\frac{1}{2}\,\dot{\theta}^T M\,\dot{\theta}\right) + \frac{d}{dt}\!\left(\frac{1}{2}\,k_1\,e^T e\right)
$$

Expand using the product rule (note that $M = M(\theta(t))$ depends on time through $\theta$):

$$
\dot{L} = \frac{1}{2}\,\dot{\theta}^T \dot{M}\,\dot{\theta} + \dot{\theta}^T M\,\ddot{\theta} + k_1\,e^T \dot{e} \tag{6}
$$

Since $\dot{e} = \dot{\theta}$, the last term becomes $k_1\,e^T \dot{\theta}$.

Now apply the skew-symmetry property (2') with $x = \dot{\theta}$:

$$
\frac{1}{2}\,\dot{\theta}^T \dot{M}\,\dot{\theta} = \dot{\theta}^T C\,\dot{\theta}
$$

Substitute into (6):

$$
\dot{L} = \dot{\theta}^T C\,\dot{\theta} + \dot{\theta}^T M\,\ddot{\theta} + k_1\,e^T \dot{\theta}
$$

Factor out $\dot{\theta}^T$:

$$
\dot{L} = \dot{\theta}^T \!\left(M\,\ddot{\theta} + C\,\dot{\theta}\right) + k_1\,e^T \dot{\theta} \tag{7}
$$

#### Step 3. Substitute the closed-loop dynamics

From the closed-loop equation (4), extract:

$$
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} = -k_1\,e - k_2\,\dot{\theta}
$$

Substitute into (7):

$$
\dot{L} = \dot{\theta}^T \!\left(-k_1\,e - k_2\,\dot{\theta}\right) + k_1\,e^T \dot{\theta}
$$

Expand:

$$
\dot{L} = -k_1\,\dot{\theta}^T e - k_2\,\dot{\theta}^T \dot{\theta} + k_1\,e^T \dot{\theta}
$$

The scalar $\dot{\theta}^T e$ equals its transpose $e^T \dot{\theta}$, so the first and third terms cancel:

$$
\boxed{\dot{L} = -k_2\,\dot{\theta}^T \dot{\theta} = -k_2\,\|\dot{\theta}\|^2} \tag{8}
$$

Since $k_2 > 0$, we have $\dot{L} \leq 0$ for all $(e, \dot{\theta})$, and $\dot{L} = 0$ only when $\dot{\theta} = 0$. Furthermore, if $\dot{\theta}(t) \equiv 0$, then $\ddot{\theta} \equiv 0$, and substituting into the closed-loop equation (4) gives $k_1\,e = 0$, hence $e = 0$. By LaSalle's invariance principle, the equilibrium $(\theta, \dot{\theta}) = (\theta_d, 0)$ is **asymptotically stable**:

$$
e(t) \to 0 \quad \text{and} \quad \dot{\theta}(t) \to 0 \quad \text{as} \quad t \to \infty
$$

---

## 4.5 Algorithm Listing

### Control pipeline (executed at every simulation time step)

```
Input: current state x = [θ₁, θ₂, dθ₁, dθ₂], target θ_d = [θ₁_d, θ₂_d]

1. Compute the position error:
       e = θ − θ_d

2. Compute the gravity vector G(θ) from the current joint angles:
       G₁ = (m₁+m₂)·g·l₁·cos(θ₁) + m₂·g·l₂·cos(θ₁+θ₂)
       G₂ = m₂·g·l₂·cos(θ₁+θ₂)

3. Compute the control action:
       a = −k₁·e − k₂·dθ + G(θ)

4. Apply torque a to the plant.

5. Integrate the plant dynamics forward by one time step:
       M(θ)·ddθ = a − C(θ, dθ)·dθ − G(θ)
       ddθ = M(θ)⁻¹ · (a − C(θ, dθ)·dθ − G(θ))
       Integrate [dθ, ddθ] using RK45 to obtain the new state.

6. Record state, control, error, and Lyapunov function value.

Output: updated state x_new, applied torque a
```

### Simulation pipeline (full experiment)

```
1.  Load parameters from configs/params.yaml.
2.  Initialize the TwoLinkManipulator plant with (m₁, m₂, l₁, l₂, g).
3.  Initialize the LyapunovController with (k₁, k₂).
4.  Initialize the PIDController with (kp, ki, kd) for comparison.
5.  Set initial state x₀ and target θ_d.
6.  FOR t = 0 to t_end with step dt:
        Compute control a(t) from controller.
        Integrate dynamics one step via solve_ivp (RK45).
        Store [t, θ, dθ, a, e, L, dL].
7.  Repeat step 6 for PID controller.
8.  Generate comparison plots → figures/
9.  Generate animation → animations/
10. Print summary metrics (settling time, final error, max torque).
```

---

## 4.6 Experimental Setup

| Parameter | Value |
|---|---|
| Initial state $x_0$ | $\theta_1 = \pi,\; \theta_2 = 0,\; \dot{\theta}_1 = 0,\; \dot{\theta}_2 = 0$ |
| Target configuration $\theta_d$ | $\theta_{1d} = \pi/2,\; \theta_{2d} = 0$ |
| Simulation time | $0$ to $10$ seconds |
| Time step $dt$ | $0.01$ s |
| Integration method | `scipy.integrate.solve_ivp` with RK45, rtol=$10^{-6}$, atol=$10^{-9}$ |
| Lyapunov controller gains | $k_1 = 100,\; k_2 = 20$ |
| PID controller gains (baseline) | $k_p = 30,\; k_i = 20,\; k_d = 15$ (per joint, independent) |
| Model mismatch | None (controller uses exact plant model) |
| Disturbances | None |
| Actuator limits | None (unbounded torques) |

**Choice of initial state:** the arm starts in a configuration far from the target ($\theta_1 = \pi$ means pointing left, while the target $\theta_1 = \pi/2$ means pointing up), producing a large initial error that tests the controller's ability to stabilize from a challenging starting point.

---

## 4.7 Reproducibility

### Prerequisites

Python 3.8 or higher, pip.

### Install dependencies

```bash
pip install -r requirements.txt
```

Contents of `requirements.txt`:
```
numpy
scipy
matplotlib
pyyaml
```

### Run the project

```bash
python main.py
```

### Produced outputs

| Output | Location | Description |
|---|---|---|
| Comparison plots | `figures/comparison_plots.png` | Angles, errors, and torques for both controllers |
| Lyapunov function plot | `figures/lyapunov_function.png` | $L(t)$ and $\dot{L}(t)$ over time |
| Phase portrait | `figures/phase_portrait.png` | $(\theta_i, \dot{\theta}_i)$ trajectories |
| Animation | `animations/robot_motion.gif` | Animated manipulator with target overlay |

---

## 4.8 Results Summary

### What works

- The Lyapunov-based controller (PD + gravity compensation) drives both joints to the target configuration with zero steady-state error.
- Convergence is fast (settling time < 2 s) and smooth, with minimal overshoot.
- The Lyapunov function $L(t)$ decreases monotonically, confirming the theoretical guarantee $\dot{L} \leq 0$.
- The controller handles the strong coupling between links correctly because the stability proof holds for the full nonlinear dynamics — no linearization or decoupling assumptions are made.

### What does not work / limitations

- The controller requires exact knowledge of the gravity vector $G(\theta)$, which in a real system may contain model errors.
- No robustness to parameter uncertainty: if the true masses or lengths differ from the model, the gravity compensation becomes imprecise, and steady-state error may appear.
- The gains $k_1, k_2$ are scalar (same for both joints). Per-joint diagonal gains $K_1 = \mathrm{diag}(k_{1,1},\; k_{1,2})$, $K_2 = \mathrm{diag}(k_{2,1},\; k_{2,2})$ would allow finer tuning.
- Actuator torque limits are not modeled. In a real robot, the large initial torques produced by high gains could saturate the motors.

### Comparison with PID baseline

The decentralized PID controller treats each joint independently and does not compensate for gravity or inertial coupling. As a result:
- When link 1 moves, it disturbs link 2 via the off-diagonal terms in $M(\theta)$ and $C(\theta, \dot{\theta})$, causing oscillations.
- Gravity creates a persistent load that the integrator must slowly compensate, leading to longer settling times (5–7 s) and noticeable overshoot.
- The PID controller has no formal stability guarantee for the nonlinear coupled system.

---

## References

1. Baccouch M., Dodds S. (2020). A Two-Link Robot Manipulator: Simulation and Control Design. *International Journal of Robotics Engineering*, 5:028.
