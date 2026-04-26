# Project 2: Adaptive Control of a Two-Link Manipulator with Unknown Payload Mass

<p align="center">
  <img src="animations/robot_motion.gif" alt="Manipulator Animation" width="600"/>
</p>

<p align="center">
  <em>Two-link manipulator moving to a target configuration while holding an object whose mass is unknown to the controller.</em>
</p>

---

## 1. Problem Definition

**Control problem:** stabilize a planar two-link robot manipulator at a desired joint configuration while the robot is carrying an object of unknown mass. The controller knows the link parameters, but it does not know the payload mass in the gripper.

**Plant:** a nonlinear two-degree-of-freedom arm in a vertical plane. The known robot links contribute the usual inertia, Coriolis/centrifugal, and gravity terms. The carried object is modeled as a point mass at the end effector; its true mass is constant but unknown.

**Method:** model-based adaptive control with certainty equivalence and a Lyapunov proof. The controller estimates the scalar payload mass online and uses the estimate in the dynamic compensation term.

**Comparison:** the adaptive controller is compared with a non-adaptive PD + known-link gravity baseline. The baseline compensates only the robot links and therefore treats the object as an unmodeled load.

---

## 2. System Description

### Physical Setup

The robot has two rigid links of lengths $l_1,l_2$ and known masses $m_1,m_2$. A payload with unknown mass $m_p>0$ is held at the end effector.

<p align="center">
  <img src="figures/manipulator_model.png" alt="Two-Link Planar Robot Manipulator Model" width="500"/>
</p>

<p align="center">
  <em>Figure 1: two-link planar manipulator carrying a point payload of unknown mass $m_p$.</em>
</p>

### State and Input

```math
x = [\theta_1,\theta_2,\dot{\theta}_1,\dot{\theta}_2]^T,
\qquad
a = [\tau_1,\tau_2]^T .
```

The target is a constant configuration

```math
\theta_d = [\theta_{1d},\theta_{2d}]^T .
```

The unknown parameter and its estimate are

```math
m_p > 0,\qquad \hat m_p(t),\qquad \tilde m_p = \hat m_p - m_p .
```

### Parameters Used in the Simulation

| Symbol | Meaning | Value |
|---|---|---:|
| $m_1$ | Link 1 mass | 1.0 kg |
| $m_2$ | Link 2 mass | 2.0 kg |
| $l_1$ | Link 1 length | 1.0 m |
| $l_2$ | Link 2 length | 1.0 m |
| $g$ | Gravity acceleration | 9.81 m/s² |
| $m_p$ | True payload mass | 3.0 kg, unknown to controller |

---

## 3. Mathematical Specification

### 3.1 Equations of Motion

The dynamics are derived via Lagrangian mechanics and take the standard form:

```math
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + G(\theta) = a \qquad \text{(1)}
```

where:

- $\theta = [\theta_1, \theta_2]^T$
- $M(\theta) \in \mathbb{R}^{2\times2}$ is the symmetric, positive-definite inertia matrix
- $C(\theta,\dot{\theta}) \in \mathbb{R}^{2\times2}$ is the Coriolis and centrifugal matrix
- $G(\theta) \in \mathbb{R}^2$ is the gravity vector

### 3.2 Effect of the Unknown Payload

The payload mass $m_u$ at the tip of Link 2 contributes to all three terms **linearly**. We decompose each term as:

```math
M(\theta) = M_0(\theta) + m_u\, M_p(\theta)
```
```math
C(\theta,\dot{\theta}) = C_0(\theta,\dot{\theta}) + m_u\, C_p(\theta,\dot{\theta})
```
```math
G(\theta) = G_0(\theta) + m_u\, G_p(\theta)
```

where the subscript $0$ denotes the **known nominal part** (identical to Project 1), and the $p$ terms are **known regressor matrices** that encode how a unit point mass at the tip affects the dynamics.

#### Nominal Inertia Matrix $M_0(\theta)$

```math
M_0(\theta) =
\begin{bmatrix}
M_{11} & M_{12} \\
M_{12} & M_{22}
\end{bmatrix}
```
```math
\begin{aligned}
M_{11} &= (m_1 + m_2)l_1^2 + m_2 l_2^2 + 2 m_2 l_1 l_2 \cos\theta_2 \\
M_{12} &= m_2 l_2^2 + m_2 l_1 l_2 \cos\theta_2 \\
M_{22} &= m_2 l_2^2
\end{aligned}
```

The matrix $M_0(\theta)$ is symmetric ($M_{21} = M_{12}$) and positive definite for all $\theta$.

#### Payload Inertia Regressor $M_p(\theta)$

A point mass at the tip of Link 2 contributes:

```math
M_p(\theta) =
\begin{bmatrix}
l_1^2 + l_2^2 + 2 l_1 l_2 \cos\theta_2 & l_2^2 + l_1 l_2 \cos\theta_2 \\
l_2^2 + l_1 l_2 \cos\theta_2 & l_2^2
\end{bmatrix}
```

#### Nominal Coriolis Matrix $C_0(\theta,\dot{\theta})$

Using the auxiliary term $h = -m_2 l_1 l_2 \sin\theta_2$:

```math
C_0(\theta,\dot{\theta}) =
\begin{bmatrix}
h\dot{\theta}_2 & h(\dot{\theta}_1 + \dot{\theta}_2) \\
-h\dot{\theta}_1 & 0
\end{bmatrix}
```

#### Payload Coriolis Regressor $C_p(\theta,\dot{\theta})$

Using $h_p = -l_1 l_2 \sin\theta_2$:

```math
C_p(\theta,\dot{\theta}) =
\begin{bmatrix}
h_p\dot{\theta}_2 & h_p(\dot{\theta}_1 + \dot{\theta}_2) \\
-h_p\dot{\theta}_1 & 0
\end{bmatrix}
```

This particular form of $C_0$ (and identically $C_p$) is constructed using Christoffel symbols so that $\dot{M}_0 - 2C_0$ is skew-symmetric. Since $m_u$ is a constant scalar, the same property holds for the full matrices $M = M_0 + m_u M_p$ and $C = C_0 + m_u C_p$. That is, for any vector $x \in \mathbb{R}^2$:

```math
x^T \bigl(\dot{M}(\theta) - 2C(\theta,\dot{\theta})\bigr) x = 0 \qquad \text{(2)}
```

An equivalent and useful rewriting:

```math
x^T \dot{M} x = 2 x^T C x \qquad \text{(2')}
```

#### Nominal Gravity Vector $G_0(\theta)$

```math
G_0(\theta) =
\begin{bmatrix}
(m_1 + m_2)g l_1 \cos\theta_1 + m_2 g l_2 \cos(\theta_1 + \theta_2) \\
m_2 g l_2 \cos(\theta_1 + \theta_2)
\end{bmatrix}
```

#### Payload Gravity Regressor $G_p(\theta)$

```math
G_p(\theta) =
\begin{bmatrix}
g l_1 \cos\theta_1 + g l_2 \cos(\theta_1 + \theta_2) \\
g l_2 \cos(\theta_1 + \theta_2)
\end{bmatrix}
```

### 3.3 Regressor Form of the Dynamics

Substituting the decompositions into (1) and separating known and unknown parts:

```math
M_0(\theta)\,\ddot{\theta} + C_0(\theta,\dot{\theta})\,\dot{\theta} + G_0(\theta) = a - m_u\,\underbrace{\bigl(M_p(\theta)\,\ddot{\theta} + C_p(\theta,\dot{\theta})\,\dot{\theta} + G_p(\theta)\bigr)}_{\displaystyle Y(\theta,\dot{\theta},\ddot{\theta})\;\in\;\mathbb{R}^2}
\qquad \text{(3)}
```

The vector $Y(\theta,\dot\theta,\ddot\theta)$ is the **regressor** — it depends only on the state and acceleration, both of which are measurable. The unknown parameter $m_u$ enters equation (3) **linearly** and **through the same channel as the control input $a$**. This structural property is what makes it possible to cancel the unknown term exactly through the control law.

---

## 4. Method Description and Stability Proof

### 4.1 Non-Adaptive Controller (Project 1)

For reference and comparison, we first state the non-adaptive controller designed for the payload-free system ($m_u = 0$). The **PD controller with gravity compensation** is:

```math
a = -k_1 e - k_2 \dot{\theta} + G_0(\theta) \qquad \text{(4)}
```

where $e = \theta - \theta_d$ is the joint angle error and $k_1, k_2 > 0$ are scalar gains.

- $-k_1 e$: Proportional feedback — a restoring force pulling the system toward $\theta_d$.
- $-k_2\dot{\theta}$: Derivative feedback — a damping force dissipating kinetic energy.
- $+G_0(\theta)$: Exact cancellation of gravitational torques (for the known part only).

When $m_u = 0$, substituting (4) into (1) gives the closed-loop dynamics:

```math
M_0(\theta)\,\ddot{\theta} + C_0(\theta,\dot{\theta})\,\dot{\theta} + k_2\dot{\theta} + k_1 e = 0 \qquad \text{(5)}
```

#### Lyapunov Function for the Non-Adaptive Controller

Consider the energy-like function $L(e, \dot{\theta})$:

```math
L(e, \dot{\theta}) = \underbrace{\frac{1}{2} \dot{\theta}^T M_0(\theta) \dot{\theta}}_{\text{Kinetic Energy}} + \underbrace{\frac{1}{2}k_1 e^T e}_{\text{Potential Energy}} \qquad \text{(6)}
```

**Positive Definiteness of $L$:**

Since $M_0(\theta)$ is symmetric and positive definite for all $\theta$, there exists $\lambda_{\min} > 0$ such that $\dot{\theta}^T M_0 \dot{\theta} \geq \lambda_{\min}\|\dot{\theta}\|^2 > 0$ for all $\dot{\theta} \neq 0$. The second term satisfies $\frac{1}{2}k_1 e^T e > 0$ for all $e \neq 0$ since $k_1 > 0$. Therefore $L(e, \dot{\theta}) > 0$ for all $(e, \dot{\theta}) \neq (0,0)$, and $L(0,0) = 0$. Thus, $L$ is **positive definite**.

**Step 2: Time Derivative of $L$**

**Given:**

```math
L(e, \dot{\theta}) = \frac{1}{2} \dot{\theta}^T M_0(\theta) \dot{\theta} + \frac{1}{2}k_1 e^T e
```

Differentiate each term with respect to time.

**Term 1:** $\frac{1}{2}\dot{\theta}^T M_0(\theta)\dot{\theta}$

Apply the product rule for three factors:

```math
\frac{d}{dt}\left(\frac{1}{2} \dot{\theta}^T M_0 \dot{\theta}\right) = \frac{1}{2} \ddot{\theta}^T M_0 \dot{\theta} + \frac{1}{2} \dot{\theta}^T \dot{M}_0 \dot{\theta} + \frac{1}{2} \dot{\theta}^T M_0 \ddot{\theta}
```

Since $M_0$ is symmetric, $\ddot{\theta}^T M_0 \dot{\theta} = \dot{\theta}^T M_0 \ddot{\theta}$, therefore the first and third terms combine:

```math
= \dot{\theta}^T M_0 \ddot{\theta} + \frac{1}{2} \dot{\theta}^T \dot{M}_0 \dot{\theta}
```

**Term 2:** $\frac{1}{2}k_1 e^T e$

```math
\frac{d}{dt}\left(\frac{1}{2}k_1 e^T e\right) = k_1 e^T \dot{e}
```

Since $\theta_d$ is constant, $\dot{e} = \dot{\theta}$, therefore:

```math
= k_1 e^T \dot{\theta}
```

**Total:**

```math
\dot{L} = \dot{\theta}^T M_0 \ddot{\theta} + \frac{1}{2} \dot{\theta}^T \dot{M}_0 \dot{\theta} + k_1 e^T \dot{\theta}
```

Use the skew-symmetry property of $(\dot{M}_0 - 2C_0)$: for any vector $x$, $x^T(\dot{M}_0 - 2C_0)x = 0$, which implies $\frac{1}{2}x^T \dot{M}_0 x = x^T C_0 x$. Substituting $x = \dot{\theta}$:

```math
\frac{1}{2} \dot{\theta}^T \dot{M}_0 \dot{\theta} = \dot{\theta}^T C_0 \dot{\theta}
```

We obtain:

```math
\dot{L} = \dot{\theta}^T M_0 \ddot{\theta} + \dot{\theta}^T C_0 \dot{\theta} + k_1 e^T \dot{\theta} = \dot{\theta}^T\underbrace{(M_0\ddot{\theta} + C_0\dot{\theta})}_{\text{from (5)}} + k_1 e^T \dot{\theta}
```

From the closed-loop dynamics (5): $M_0\ddot{\theta} + C_0\dot{\theta} = -k_1 e - k_2\dot{\theta}$. Substitute:

```math
\dot{L} = \dot{\theta}^T(-k_1 e - k_2 \dot{\theta}) + k_1 e^T \dot{\theta}
```

```math
\dot{L} = -k_1\underbrace{\dot{\theta}^T e}_{=\, e^T\dot{\theta}} - k_2\dot{\theta}^T\dot{\theta} + k_1 e^T\dot{\theta}
```

The first and third terms cancel:

```math
\dot{L} = -k_2\|\dot{\theta}\|^2 \qquad \text{(7)}
```

Since $k_2 > 0$, we have $\dot{L} \leq 0$ for all $(e, \dot{\theta})$, and $\dot{L} = 0$ only when $\dot{\theta} = 0$. Furthermore, if $\dot{\theta}(t) \equiv 0$, then $\ddot{\theta} \equiv 0$, and substituting into the closed-loop equation (5) gives $k_1 e = 0$, hence $e = 0$. By LaSalle's invariance principle, the equilibrium $(\theta, \dot{\theta}) = (\theta_d, 0)$ is **asymptotically stable**:

```math
e(t) \to 0 \quad \text{and} \quad \dot{\theta}(t) \to 0 \quad \text{as} \quad t \to \infty \qquad \text{(8)}
```

#### Failure of the Non-Adaptive Controller When $m_u \neq 0$

When the payload is present but the controller does not know it, the gravity compensation $G_0(\theta)$ is incomplete — it underestimates the gravitational torques by $m_u G_p(\theta)$. The closed-loop dynamics become:

```math
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + k_2\dot{\theta} + k_1 e = m_u\, Y(\theta,\dot{\theta},\ddot{\theta}) \qquad \text{(9)}
```

where the right-hand side is a **persistent disturbance** caused by the uncompensated payload. At any steady state where $\dot{\theta} = \ddot{\theta} = 0$, equation (9) reduces to:

```math
k_1 e^* = m_u\, G_p(\theta^*) \qquad \Rightarrow \qquad e^* = \frac{m_u}{k_1} G_p(\theta_d) \neq 0
```

This shows that the non-adaptive controller converges to a **wrong equilibrium** that is offset from $\theta_d$ by an amount proportional to $m_u$. No choice of gains $k_1$ and $k_2$ can remove this error — it is structural, not a tuning problem.

---

### 4.2 Adaptive Controller

#### Control Law

The **adaptive PD controller with estimated gravity compensation** replaces the unknown $m_u$ with its current estimate $\hat{m}_u(t)$:

```math
a = -k_1 e - k_2 \dot{\theta} + G_0(\theta) + \hat{m}_u\, Y(\theta,\dot{\theta},\ddot{\theta}) \qquad \text{(10)}
```

where $Y$ is the regressor defined in (3). The estimate $\hat{m}_u$ is updated online by an adaptation law derived below.

#### Closed-Loop Dynamics

Define the estimation error:

```math
\tilde{m}_u := \hat{m}_u - m_u \qquad \text{(11)}
```

Substituting (10) into (1) and using the full decomposition:

```math
(M_0 + m_u M_p)\ddot{\theta} + (C_0 + m_u C_p)\dot{\theta} + G_0 + m_u G_p = -k_1 e - k_2\dot{\theta} + G_0 + \hat{m}_u Y
```

The $G_0$ terms cancel. Using $\hat{m}_u Y = m_u Y + \tilde{m}_u Y$ and collecting:

```math
M(\theta)\,\ddot{\theta} + C(\theta,\dot{\theta})\,\dot{\theta} + k_2\dot{\theta} + k_1 e = \tilde{m}_u\, Y(\theta,\dot{\theta},\ddot{\theta}) \qquad \text{(12)}
```

When $\tilde{m}_u = 0$, equation (12) reduces exactly to (5). The term $\tilde{m}_u Y$ is the residual disturbance driven by the estimation error alone.

#### Augmented Lyapunov Function

To design the adaptation law and prove stability, we augment $L$ with a quadratic penalty on the estimation error:

```math
L_c(e, \dot{\theta}, \tilde{m}_u) := \underbrace{\frac{1}{2} \dot{\theta}^T M(\theta) \dot{\theta} + \frac{1}{2}k_1 e^T e}_{L(e,\dot\theta)} + \underbrace{\frac{1}{2\gamma}\tilde{m}_u^2}_{\text{estimation penalty}} \qquad \text{(13)}
```

where $\gamma > 0$ is the adaptation gain.

**Positive Definiteness of $L_c$:**

The first two terms have the same structure as (6) but with the full matrix $M = M_0 + m_u M_p$. Since $M_0$ is positive definite and $M_p$ is positive semi-definite (it is the inertia contribution of a non-negative mass), $M$ is positive definite for all $m_u \geq 0$. The third term is non-negative. Therefore:

```math
L_c(e, \dot{\theta}, \tilde{m}_u) > 0 \quad \text{for all } (e, \dot{\theta}, \tilde{m}_u) \neq (0,0,0), \qquad L_c(0,0,0) = 0
```

Thus, $L_c$ is **positive definite**.

**Time Derivative of $L_c$:**

**Term 1:** $\frac{1}{2}\dot{\theta}^T M(\theta)\dot{\theta}$

Apply the product rule:

```math
\frac{d}{dt}\left(\frac{1}{2} \dot{\theta}^T M \dot{\theta}\right) = \frac{1}{2} \ddot{\theta}^T M \dot{\theta} + \frac{1}{2} \dot{\theta}^T \dot{M} \dot{\theta} + \frac{1}{2} \dot{\theta}^T M \ddot{\theta}
```

Since $M$ is symmetric, the first and third terms combine:

```math
= \dot{\theta}^T M \ddot{\theta} + \frac{1}{2} \dot{\theta}^T \dot{M} \dot{\theta}
```

Using the skew-symmetry property (2') for the full matrix $M$: $\frac{1}{2}\dot{\theta}^T \dot{M} \dot{\theta} = \dot{\theta}^T C \dot{\theta}$:

```math
\frac{d}{dt}\left(\frac{1}{2} \dot{\theta}^T M \dot{\theta}\right) = \dot{\theta}^T M \ddot{\theta} + \dot{\theta}^T C \dot{\theta}
```

**Term 2:** $\frac{1}{2}k_1 e^T e$

```math
\frac{d}{dt}\left(\frac{1}{2}k_1 e^T e\right) = k_1 e^T \dot{\theta}
```

**Term 3:** $\frac{1}{2\gamma}\tilde{m}_u^2$

Since $m_u$ is a constant, $\dot{\tilde{m}}_u = \dot{\hat{m}}_u$:

```math
\frac{d}{dt}\left(\frac{1}{2\gamma}\tilde{m}_u^2\right) = \frac{1}{\gamma}\tilde{m}_u\,\dot{\hat{m}}_u
```

**Combining all three terms:**

```math
\dot{L}_c = \dot{\theta}^T(M\ddot{\theta} + C\dot{\theta}) + k_1 e^T\dot{\theta} + \frac{1}{\gamma}\tilde{m}_u\,\dot{\hat{m}}_u
```

Substituting from the closed-loop dynamics (12) — $M\ddot{\theta} + C\dot{\theta} = -k_1 e - k_2\dot{\theta} + \tilde{m}_u Y$:

```math
\dot{L}_c = \dot{\theta}^T(-k_1 e - k_2\dot{\theta} + \tilde{m}_u Y) + k_1 e^T\dot{\theta} + \frac{1}{\gamma}\tilde{m}_u\,\dot{\hat{m}}_u
```

```math
\dot{L}_c = -k_1\underbrace{\dot{\theta}^T e}_{=\, e^T\dot{\theta}} - k_2\|\dot{\theta}\|^2 + \tilde{m}_u\,\dot{\theta}^T Y + k_1 e^T\dot{\theta} + \frac{1}{\gamma}\tilde{m}_u\,\dot{\hat{m}}_u
```

The first and fourth terms cancel:

```math
\dot{L}_c = -k_2\|\dot{\theta}\|^2 + \tilde{m}_u\!\left(\dot{\theta}^T Y + \frac{1}{\gamma}\dot{\hat{m}}_u\right) \qquad \text{(14)}
```

The first term provides the desired dissipation. The second term is the **cross-term** that couples the state dynamics to the parameter estimation error.

#### Adaptation Law

To cancel the cross-term in (14), we choose $\dot{\hat{m}}_u$ to make the parenthesis zero:

```math
\dot{\theta}^T Y + \frac{1}{\gamma}\dot{\hat{m}}_u = 0 \qquad \Rightarrow \qquad \boxed{\dot{\hat{m}}_u = -\gamma\,\dot{\theta}^T Y(\theta,\dot{\theta},\ddot{\theta})} \qquad \text{(15)}
```

Substituting (15) into (14):

```math
\dot{L}_c = -k_2\|\dot{\theta}\|^2 + \tilde{m}_u\!\left(\dot{\theta}^T Y - \dot{\theta}^T Y\right) = -k_2\|\dot{\theta}\|^2 \qquad \text{(16)}
```

#### Stability by LaSalle's Invariance Principle

**Step 1: All signals are bounded.**

From (16): $\dot{L}_c = -k_2\|\dot\theta\|^2 \leq 0$ for all time. Therefore $L_c(t) \leq L_c(0) < \infty$, which implies that $e(t)$, $\dot\theta(t)$, and $\tilde{m}_u(t)$ are all **bounded** for all $t \geq 0$.

**Step 2: $\dot{\theta}(t) \to 0$.**

Integrating (16) from $0$ to $\infty$:

```math
\int_0^\infty k_2\|\dot{\theta}(t)\|^2\,dt \leq L_c(0) < \infty
```

so $\|\dot{\theta}\| \in L^2$. Since $e$, $\dot\theta$, and $\hat{m}_u$ are all bounded, the closed-loop equation (12) shows that $\ddot\theta$ is bounded as well. Therefore $\dot\theta$ is **uniformly continuous**. By **Barbalat's Lemma**:

```math
\dot{\theta}(t) \to 0 \quad \text{as} \quad t \to \infty
```

**Step 3: $e(t) \to 0$ by LaSalle's Invariance Principle.**

Consider the set:

```math
\mathcal{S} = \left\{(e, \dot{\theta}, \tilde{m}_u) : \dot{L}_c = 0\right\} = \left\{(e, \dot{\theta}, \tilde{m}_u) : \dot{\theta} = 0\right\}
```

On any trajectory that stays in $\mathcal{S}$, we have $\dot\theta \equiv 0$ and hence $\ddot\theta \equiv 0$. From the adaptation law (15): $\dot{\hat{m}}_u = -\gamma\,\dot\theta^T Y = 0$, so $\tilde{m}_u$ is constant on $\mathcal{S}$. Substituting $\dot\theta = \ddot\theta = 0$ into (12):

```math
k_1 e = \tilde{m}_u\, G_p(\theta)
```

Taking the time derivative of both sides (with $\dot\theta = 0$ and $\tilde{m}_u$ constant):

```math
k_1\dot{e} = k_1\dot\theta = 0
```

which is already satisfied. However, the equation $k_1 e = \tilde{m}_u G_p(\theta)$ with $\dot\theta \equiv 0$ and $\dot{\tilde{m}}_u = 0$ represents a static balance. For this to be an invariant trajectory, the velocity must remain zero at all future times. But if $e \neq 0$, the PD term $-k_1 e$ would accelerate the arm, generating $\dot\theta \neq 0$ — a contradiction. Therefore the only solution consistent with $\dot\theta \equiv 0$ is $e \equiv 0$, which from $k_1 e = \tilde{m}_u G_p(\theta_d)$ implies $\tilde{m}_u = 0$. The largest invariant set in $\mathcal{S}$ is therefore $\{(0, 0, 0)\}$. By **LaSalle's Invariance Principle**:

```math
e(t) \to 0, \quad \dot{\theta}(t) \to 0, \quad \tilde{m}_u(t) \to 0 \quad \text{as} \quad t \to \infty \qquad \blacksquare
```

**Remark on parameter convergence:** LaSalle's argument shows $\hat{m}_u \to m_u$ in the limit. The rate of convergence depends on how much the regressor $Y$ is excited during the motion. If the arm settles quickly, $\hat{m}_u$ may converge slowly to the true value, though the state convergence $e \to 0$ is guaranteed regardless.

---
## 5. Algorithm Listing

The control algorithm executed at each time step $t$:

1. **Read State:** Obtain current $\theta(t)$ and $\dot{\theta}(t)$.
2. **Compute Error:** $e = \theta - \theta_d$.
3. **Compute Nominal Dynamics Matrices:** Calculate $M_0(\theta)$, $C_0(\theta,\dot{\theta})$, $G_0(\theta)$.
4. **Compute Regressor Matrices:** Calculate $M_p(\theta)$, $C_p(\theta,\dot{\theta})$, $G_p(\theta)$.
5. **Compute Regressor Vector:**

```math
Y = M_p(\theta)\,\ddot{\theta} + C_p(\theta,\dot{\theta})\,\dot{\theta} + G_p(\theta)
```

6. **Update Estimate:**

```math
\hat{m}_u(t + \Delta t) = \hat{m}_u(t) - \Delta t \cdot \gamma\,\dot{\theta}^T Y
```

7. **Compute Control Input:**

```math
a(t) = -k_1 e - k_2 \dot{\theta} + G_0(\theta) + \hat{m}_u\, Y
```

8. **Apply Torque:** Apply $a(t)$ to the plant.
9. **Integrate Dynamics:** Solve

```math
\ddot{\theta} = M(\theta)^{-1}\bigl(a - C(\theta,\dot{\theta})\dot{\theta} - G(\theta)\bigr)
```

to update the state for $t + \Delta t$, where $M = M_0 + m_u M_p$ and $G = G_0 + m_u G_p$ use the **true** (unknown) mass in the simulation only.

---

## 6. Experimental Setup

| Parameter | Value | Description |
|---|---|---|
| **Initial State** | $\theta=[\pi, 0],\; \dot{\theta}=[0,0]$ | Arm starts pointing left, stationary. |
| **Target State** | $\theta_d=[\pi/2, 0]$ | Target is vertical (upright). |
| **Grasp Time** | $t_{\text{grasp}} = 1.0$ s | Payload attached at $t=1$ s. |
| **True Payload Mass** | $m_u = 1.5$ kg | Unknown to the controller. |
| **Simulation Time** | 15 seconds | Longer to capture adaptation transient. |
| **Controller Gains** | $k_1=100,\; k_2=20$ | Same as Project 1 for fair comparison. |
| **Adaptation Gain** | $\gamma = 5.0$ | Controls speed of parameter estimation. |
| **Initial Estimate** | $\hat{m}_u(0) = 0$ | Controller starts with no payload knowledge. |

**Choice of initial state:** the arm starts far from the target ($\theta_1=\pi$ means pointing left, target is $\theta_1=\pi/2$ pointing up), producing a large initial error that tests the controller on a challenging configuration. The payload is introduced at $t = 1$ s after a brief initial transient, to clearly isolate the adaptation response from the initial stabilization phase.

---

## 7. Results and Discussion

### 7.1 Comparison: Adaptive vs. Non-Adaptive

<p align="center">
  <img src="figures/comparison_plots.png" alt="Comparison Plots" width="900"/>
</p>

<p align="center">
  <em>Figure 2: Comparison of joint angles, tracking errors, control torques, and payload estimate for the adaptive controller (green) and the non-adaptive Lyapunov controller from Project 1 (blue). Dashed vertical line at t=1 s marks the grasp event.</em>
</p>

**Interpretation of Figure 2**

- **Joint Angles and Errors (Top/Middle Rows):** Both controllers perform identically before $t=1$ s, as expected — the plant is the same. After grasping, the non-adaptive controller converges to a wrong equilibrium: the error settles at a constant nonzero value that matches the prediction $e^* = \frac{m_u}{k_1}G_p(\theta_d)$ from Section 4.1. The adaptive controller shows a brief transient as the estimator adjusts, then fully recovers and converges to $\theta_d$ with zero error.
- **Control Torques (Third Row):** The adaptive controller requires slightly larger torques during the adaptation transient. The non-adaptive controller produces smooth torques but holds the arm at the wrong position permanently.
- **Payload Estimate (Bottom Row):** $\hat{m}_u(t)$ starts at zero, remains zero until grasping (the arm is at rest so $Y = G_p$ and $\dot\theta^T Y \approx 0$), then increases as the arm responds to the load and converges toward the true value $m_u = 1.5$ kg.

### 7.2 Lyapunov Function Evolution

<p align="center">
  <img src="figures/lyapunov_function.png" alt="Lyapunov Function" width="700"/>
</p>

<p align="center">
  <em>Figure 3: Time evolution of the augmented Lyapunov function L_c(t) (blue) and its derivative L̇_c(t) (red) for the adaptive controller.</em>
</p>

**Interpretation of Figure 3**

- **$L_c(t)$ (Blue):** Decreases monotonically throughout the simulation, including after the grasp event at $t=1$ s. This confirms the theoretical result that $\dot{L}_c \leq 0$ at all times. The brief plateau at $t=1$ s reflects the sudden increase in $\tilde{m}_u^2$ when the payload is attached and $\hat{m}_u$ still equals zero; $L_c$ resumes decreasing immediately as adaptation begins.
- **$\dot{L}_c(t)$ (Red):** Remains non-positive at all times, consistent with equation (16). The initial large negative value corresponds to the rapid dissipation of kinetic energy from the initial configuration; the smaller dip at $t=1$ s corresponds to the new transient driven by the payload.

### 7.3 Phase Portrait

<p align="center">
  <img src="figures/phase_portrait.png" alt="Phase Portrait" width="850"/>
</p>

<p align="center">
  <em>Figure 4: Phase portraits for Joint 1 (left) and Joint 2 (right), showing trajectories for the adaptive controller (green) and the non-adaptive controller (blue). Red star marks the true target; blue cross marks the wrong equilibrium of the non-adaptive controller.</em>
</p>

**Interpretation of Figure 4**

- The adaptive controller (green) spirals inward and converges to the true target $\theta_d$ (red star), characteristic of a stable, well-damped second-order system.
- The non-adaptive controller (blue) also spirals inward but converges to a **displaced equilibrium** (blue cross) that is offset from the target by the uncompensated gravity load. The offset is visible as a separate attractor in both phase portraits.

---

## 8. Key Findings and Conclusions

#### 8.1 Necessity of Adaptation

The non-adaptive Lyapunov controller from Project 1 fails in a structural way when the payload is unknown: it converges to the wrong position, and the steady-state error grows linearly with $m_u$. This failure cannot be fixed by retuning $k_1$ or $k_2$. The adaptive controller eliminates this failure by learning $m_u$ online and correcting the gravity compensation term in real time.

#### 8.2 What the Adaptive Controller Guarantees

The adaptive scheme provides:

- **Lyapunov stability:** $L_c(t)$ is non-increasing, so $e$, $\dot\theta$, and $\tilde{m}_u$ remain bounded for all time.
- **Asymptotic state convergence:** $e(t) \to 0$ and $\dot\theta(t) \to 0$ as $t \to \infty$, proven via Barbalat's Lemma.
- **Asymptotic parameter convergence:** $\hat{m}_u(t) \to m_u$ as $t \to \infty$, by LaSalle's Invariance Principle.

#### 8.3 Practical Implications

1. **Model Structure vs. Model Parameters:** The adaptive controller requires knowing the structure of how $m_u$ enters the dynamics (the regressors $M_p$, $C_p$, $G_p$) but not the value of $m_u$ itself. This is a strictly weaker requirement than what Project 1 needed.
2. **Adaptation Gain $\gamma$:** A larger $\gamma$ speeds up estimation but can amplify sensor noise in the regressor signal. In practice, $\gamma$ must be tuned against noise levels.
3. **Regressor Computation:** The adaptation law (15) requires computing $\ddot\theta$, typically obtained by differentiating $\dot\theta$. Noise in this signal is the main practical limitation and motivates the use of observers or low-pass filtering.

---

## 9. Project Structure

```text
project_2_mass/
├── README_project2.md
├── requirements.txt
├── main.py
├── configs/
│   └── params.yaml
├── scripts/
│   └── random_search_params.py
├── src/
│   ├── system.py
│   ├── adaptive_controller.py
│   ├── controller.py
│   ├── lyapunov_controller.py
│   ├── simulation.py
│   └── visualization.py
├── figures/
│   ├── comparison_plots.png
│   ├── lyapunov_function.png
│   ├── manipulator_model.png
│   ├── parameter_estimation.png
│   ├── payload_compensation.png
│   └── phase_portrait.png
└── animations/
    └── robot_motion.gif
```

---

## 10. How to Run

```bash
pip install -r requirements.txt
python3 main.py
```

This runs both controllers on the same loaded manipulator and regenerates all figures and the animation.

---

## 11. References

1. Slotine, J.-J. E., & Li, W. (1991). *Applied Nonlinear Control*. Prentice Hall.
2. Baccouch, M., & Dodds, S. (2020). *A two-link robot manipulator: Simulation and control design*. International Journal of Robotic Engineering, 5(2), 1-17.
3. Osinenko, P. (2026). *Essentials of Adaptive Control*. Lecture Notes.
