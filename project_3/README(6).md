# Project 3: Fault-Tolerant Backstepping Control of a Vertical Rocket

## Motivation

This project implements and compares two backstepping controllers for a vertically flying rocket subject to a partial thruster failure. The nominal backstepping controller assumes full thruster effectiveness at all times. The adaptive fault-tolerant controller estimates the unknown fault parameter online and compensates for it, maintaining the target altitude despite the failure.

---

## 1. Problem Definition

**Control problem:** stabilize a vertically flying rocket at a desired altitude $z_d$ when the thruster experiences a partial loss of effectiveness — an unknown fraction $\rho \in (0, 1]$ of the commanded thrust is actually delivered.

**Plant:** a 1D rocket modeled as a point mass moving along the vertical axis. The single control input is the commanded thrust $T_c$. Gravity and the actuator fault act on the vehicle.

**Assumptions:**
- Motion is purely vertical (1D model)
- Actuator dynamics are negligible (thrust is applied instantaneously)
- The fault parameter $\rho$ is constant but unknown to the controller
- The mass $m$ is known

**Class of methods:** adaptive backstepping based on a Control Lyapunov Function (CLF). A stabilization triple $(\pi_0, L_0, K_0)$ is first designed for the base plant, then the Lyapunov function is augmented by the squared backstepping error and a parameter estimation error term to obtain a new triple $(\pi_1, L_{\text{aug}}, K_1)$ for the full adaptive system.

**Comparison:** two controllers sharing the same backstepping structure are compared. The nominal controller fixes $\hat{\rho} = 1$ for all time. The adaptive FTC updates $\hat{\rho}$ online. Under a thruster fault $\rho = 0.5$, the nominal controller settles at a permanent altitude error. The adaptive FTC detects and compensates the fault, recovering the target altitude.

---

## 2. System Description

### Physical Setup

The rocket moves vertically. Two forces act on it: the actual thrust $T_{\text{real}}$ upward and gravity $mg$ downward.

```
        ↑  T_real = ρ · T_c
        |
      [ROCKET]
        |
        ↓  mg
```

The fault model: the thruster delivers only a fraction $\rho$ of the commanded thrust:

$$T_{\text{real}} = \rho \cdot T_c, \quad \rho \in (0, 1], \quad \rho \text{ unknown}$$

### State Variables

$$s = [z,\ \dot{z}]^\top \in \mathbb{R}^2$$

| Symbol | Meaning | Units |
|---|---|---|
| $z$ | Altitude | m |
| $\dot{z}$ | Vertical velocity | m/s |

### Control Input

$$a = T_c \in \mathbb{R}, \quad T_c \geq 0$$

The actual thrust applied is $T_{\text{real}} = \rho \cdot T_c$.

### Unknown Parameter

| Symbol | Meaning | Range |
|---|---|---|
| $\rho$ | Thruster effectiveness | $(0, 1]$ |

### Reference Trajectory

$$z_d(t) = \begin{cases} 0\ \text{m} & t < 2\ \text{s} \\ 10\ \text{m} & t \geq 2\ \text{s} \end{cases}$$

### Fault Scenario

$$\rho(t) = \begin{cases} 1.0 & t < 5\ \text{s} \\ 0.5 & t \geq 5\ \text{s} \end{cases}$$

The 3-second gap between the reference step at $t = 2$ s and the fault onset at $t = 5$ s is intentional: it allows the system to fully stabilize at $z_d = 10$ m before the disturbance occurs, making the fault response clearly visible in the plots. The controller does not know when or how much the fault occurs.

### Physical Parameters

| Symbol | Meaning | Value | Units |
|---|---|---:|---|
| $m$ | Mass | 2.0 | kg |
| $g$ | Gravitational acceleration | 9.81 | m/s² |
| $\rho$ | Fault level (unknown to controller) | 0.5 | — |

### Equation of Motion

$$m\ddot{z} = \rho \cdot T_c - mg \tag{1}$$

---

## 3. Mathematical Specification

All variables are defined at first use. No symbol is reused with a different meaning.

| Symbol | Meaning |
|---|---|
| $z$ | Altitude |
| $z_d$ | Desired altitude |
| $e$ | Altitude error: $e = z - z_d$ |
| $v_e$ | Velocity error: $v_e = \dot{z} - \dot{z}_d$ |
| $\delta$ | Backstepping error: $\delta = v_e - \pi_0(e)$ |
| $\pi_0$ | Virtual control policy for the base plant |
| $\hat{\rho}$ | Online estimate of $\rho$ |
| $\tilde{\rho}$ | Estimation error: $\tilde{\rho} = \hat{\rho} - \rho$ |
| $L_0,\ L_{\text{aug}}$ | Lyapunov function candidates |
| $k_1,\ K,\ \gamma$ | Controller and adaptation gains |
| $T_{\text{nom}}$ | Nominal thrust term (defined in Section 4, Step 3) |

### Error Coordinates

$$e = z - z_d, \qquad v_e = \dot{z} - \dot{z}_d \tag{2}$$

Error dynamics:

$$\dot{e} = v_e \tag{3}$$

$$\dot{v}_e = \frac{\rho}{m} T_c - g - \ddot{z}_d \tag{4}$$

This is a cascade system: equation (3) describes altitude error driven by velocity error, and equation (4) describes velocity error driven by the control input.

### Backstepping Error Variable

$$\delta := v_e - \pi_0(e) = v_e + k_1 e \tag{5}$$

where $\pi_0(e) = -k_1 e$ is the virtual desired velocity chosen in Step 1 below.

---

## 4. Common Backstepping Foundation

Both controllers share the same backstepping structure. They differ in one respect only: how the unknown thruster effectiveness $\rho$ is handled. This section develops the shared foundation.

### Step 1 — Base Plant and Triple $(\pi_0, L_0, K_0)$

Consider only equation (3), treating $v_e$ as a free input:

$$\dot{e} = v_e$$

Choose the virtual control policy:

$$\pi_0(e) = -k_1 e, \quad k_1 > 0 \tag{6}$$

CLF candidate:

$$L_0 = \frac{1}{2} e^2 \tag{7}$$

**Verification:**

*Positive definiteness:* $L_0 \geq 0$, and $L_0 = 0 \iff e = 0$. ✓

*Radial unboundedness:* $L_0 \to \infty$ as $|e| \to \infty$. ✓

*Decay condition:* under $v_e = \pi_0(e)$:

$$\dot{L}_0 = e \dot{e} = e(-k_1 e) = -k_1 e^2 \leq 0 \tag{8}$$

By LaSalle's invariance principle, the only invariant set where $\dot{L}_0 = 0$ is $\{e = 0\}$, so $e(t) \to 0$.

The triple $(\pi_0, L_0, K_0)$ with $K_0(r) = k_1 r^2$ is valid for the base plant.

### Step 2 — Augmented Lyapunov Function

The Lyapunov function is augmented by two additional terms: the squared backstepping error penalizing the deviation of $v_e$ from its virtual target, and the squared estimation error penalizing the fault parameter mismatch:

$$L_{\text{aug}} := \frac{1}{2}e^2 + \frac{1}{2}\delta^2 + \frac{1}{2\gamma}\tilde{\rho}^2 \tag{9}$$

Each term is non-negative, and $L_{\text{aug}} = 0$ iff $(e, \delta, \tilde{\rho}) = (0, 0, 0)$. Since $L_{\text{aug}} \to \infty$ as $\|(e, \delta, \tilde{\rho})\| \to \infty$, it is radially unbounded.

### Step 3 — Nominal Thrust Term

Define the nominal thrust term used by both controllers:

$$T_{\text{nom}} := m\bigl(g + \ddot{z}_d - k_1 v_e - K\delta - e\bigr) \tag{10}$$

This is the thrust required to achieve the desired closed-loop dynamics if $\rho = 1$. The two controllers differ only in how $T_{\text{nom}}$ is converted into the actual commanded thrust $T_c$.

---

## 5. Controller A — Nominal Backstepping (fixed $\hat{\rho} = 1$)

### Control Law

The nominal controller assumes the thruster is always fully effective. It sets $\hat{\rho} = 1$ for all time and commands:

$$T_c^{\text{nom}} = T_{\text{nom}} = m\bigl(g + \ddot{z}_d - k_1 v_e - K\delta - e\bigr) \tag{11a}$$

No adaptation law is used.

### Lyapunov Analysis — No Fault ($\rho = 1$)

When $\rho = 1$ the dynamics match the design assumption. Substituting (11a) into (4):

$$\dot{v}_e = \frac{1}{m}T_{\text{nom}} - g - \ddot{z}_d = -k_1 v_e - K\delta - e$$

Computing $\dot{\delta} = \dot{v}_e + k_1 v_e$:

$$\dot{\delta} = -K\delta - e$$

The Lyapunov derivative:

$$\dot{L}_{\text{aug}}\big|_{\rho=1} = e\dot{e} + \delta\dot{\delta} = (e\delta - k_1 e^2) + (-K\delta^2 - e\delta) = -k_1 e^2 - K\delta^2 \leq 0$$

Decay is guaranteed. By LaSalle's invariance principle $e(t) \to 0$ and $\delta(t) \to 0$. ✓

### Lyapunov Analysis — Under Fault ($\rho = 0.5$)

After fault onset, the actual dynamics become:

$$\dot{v}_e = \frac{0.5}{m}T_c^{\text{nom}} - g - \ddot{z}_d$$

Substituting $T_c^{\text{nom}} = m(g + \ddot{z}_d - k_1 v_e - K\delta - e)$:

$$\dot{v}_e = 0.5(g + \ddot{z}_d - k_1 v_e - K\delta - e) - g - \ddot{z}_d$$
$$= -0.5\,k_1 v_e - 0.5\,K\delta - 0.5\,e - 0.5\,g - 0.5\,\ddot{z}_d$$

Computing $\dot{\delta} = \dot{v}_e + k_1 v_e$:

$$\dot{\delta} = 0.5\,k_1 v_e - 0.5\,K\delta - 0.5\,e - 0.5\,g - 0.5\,\ddot{z}_d$$

The effective gain on $\delta$ is halved from $-K$ to $-0.5K$, and uncompensated terms involving $g$ and $\ddot{z}_d$ remain. The decay condition $\dot{L}_{\text{aug}} \leq -k_1 e^2 - K\delta^2$ **no longer holds**. ✗

### Steady-State Error

At steady state: $\dot{e} = \dot{v}_e = 0$, $\ddot{z}_d = 0$, $v_e = 0$, $\delta = k_1 e$. Setting $\dot{v}_e = 0$:

$$0 = \frac{0.5}{m}T_c^{\text{nom}} - g \implies T_c^{\text{nom}} = 2mg$$

From the control law with $v_e = 0$ and $\delta = k_1 e$:

$$T_c^{\text{nom}} = m\bigl(g - (Kk_1 + 1)\,e\bigr)$$

Setting equal to $2mg$:

$$e^* = \frac{-g}{Kk_1 + 1}$$

With $K = 6$, $k_1 = 2$, $g = 9.81$ m/s²:

$$\boxed{e^* = \frac{-9.81}{13} \approx -0.75\ \text{m}}$$

The rocket settles permanently **0.75 m below** the target. This error cannot be removed without adaptation.

---

## 6. Controller B — Adaptive FTC (online $\hat{\rho}$)

### Control Law

The adaptive controller divides the nominal thrust term by the current estimate $\hat{\rho}$:

$$T_c = \frac{T_{\text{nom}}}{\hat{\rho}}, \quad T_c \geq 0 \tag{11b}$$

### Lyapunov Decay Derivation

Differentiating $L_{\text{aug}}$ along system trajectories:

$$\dot{L}_{\text{aug}} = e\dot{e} + \delta\dot{\delta} + \frac{1}{\gamma}\tilde{\rho}\dot{\hat{\rho}}$$

*First term.* Using $\dot{e} = v_e = \delta - k_1 e$:

$$e\dot{e} = e\delta - k_1 e^2$$

*Second term.* Differentiating $\delta = v_e + k_1 e$, substituting (4), and using $\dfrac{\rho}{\hat{\rho}} = 1 - \dfrac{\tilde{\rho}}{\hat{\rho}}$:

$$\dot{\delta} = -K\delta - e - \frac{\tilde{\rho}}{\hat{\rho}}\cdot\frac{T_{\text{nom}}}{m}$$

$$\delta\dot{\delta} = -K\delta^2 - e\delta - \frac{\tilde{\rho}}{\hat{\rho}}\cdot\frac{\delta\,T_{\text{nom}}}{m}$$

*Assembling before choosing the adaptation law:*

$$\dot{L}_{\text{aug}} = -k_1 e^2 - K\delta^2 - \frac{\tilde{\rho}}{\hat{\rho}}\cdot\frac{\delta\,T_{\text{nom}}}{m} + \frac{1}{\gamma}\tilde{\rho}\dot{\hat{\rho}}$$

Choosing the adaptation law to cancel the cross-term exactly:

$$\dot{\hat{\rho}} = \frac{\gamma\,\delta\,T_{\text{nom}}}{m\hat{\rho}} \tag{12}$$

gives $\dfrac{1}{\gamma}\tilde{\rho}\dot{\hat{\rho}} = \dfrac{\tilde{\rho}}{\hat{\rho}}\cdot\dfrac{\delta\,T_{\text{nom}}}{m}$, and the cross-term cancels:

$$\boxed{\dot{L}_{\text{aug}} = -k_1 e^2 - K\delta^2 \leq 0} \tag{13}$$

This holds for any fault level $\rho \in (0, 1]$. ✓

### LaSalle's Invariance Principle

$L_{\text{aug}}$ is continuously differentiable, radially unbounded, and $\dot{L}_{\text{aug}} \leq 0$ everywhere. Every sublevel set $\Omega_c = \{L_{\text{aug}} \leq c\}$ is positively invariant.

Define:

$$\mathcal{S} = \bigl\{(e,\delta,\tilde{\rho})\ :\ \dot{L}_{\text{aug}} = 0\bigr\} = \{e = 0,\ \delta = 0\}$$

On the largest invariant set $\mathcal{M} \subseteq \mathcal{S}$: $e \equiv 0$, $\delta \equiv 0$, $v_e \equiv 0$. The adaptation law gives $\dot{\hat{\rho}} = 0$, so $\hat{\rho}$ is constant:

$$\mathcal{M} = \{e = 0,\ \delta = 0,\ \hat{\rho} = \text{const}\}$$

By LaSalle's invariance principle, $e(t) \to 0$ and $v_e(t) \to 0$ as $t \to \infty$.

**Note on parameter convergence:** $\hat{\rho}$ converges to a constant but not necessarily to the true $\rho$. The adaptation law drives $\dot{\hat{\rho}} \to 0$ once $\delta \to 0$, even if $\hat{\rho} \neq \rho$. Exact parameter identification would additionally require a persistent excitation condition.

---

## 7. Side-by-Side Comparison

| | Nominal Backstepping | Adaptive FTC |
|---|---|---|
| **Estimate** | $\hat{\rho} = 1$ fixed | $\hat{\rho}$ updated online via (12) |
| **Control law** | $T_c = T_{\text{nom}}$ | $T_c = T_{\text{nom}} / \hat{\rho}$ |
| **Adaptation law** | none | $\dot{\hat{\rho}} = \gamma\,\delta\,T_{\text{nom}} / (m\hat{\rho})$ |
| **Lyapunov decay, $\rho = 1$** | $-k_1 e^2 - K\delta^2 \leq 0$ ✓ | $-k_1 e^2 - K\delta^2 \leq 0$ ✓ |
| **Lyapunov decay, $\rho = 0.5$** | not guaranteed ✗ | $-k_1 e^2 - K\delta^2 \leq 0$ ✓ |
| **Steady-state error after fault** | $e^* \approx -0.75$ m (permanent) | $e \to 0$ (recovered) |
| **Convergence guarantee** | asymptotic, fault-free only | asymptotic, any $\rho \in (0,1]$ |
| **Parameter convergence** | — | neighborhood of $\rho$ (no PE required) |

The two controllers are identical before $t = 5$ s. The difference becomes visible only after fault onset, making the comparison clean and unambiguous.

---

## 8. Algorithm

```
Algorithm 1: Nominal Backstepping
──────────────────────────────────────────────────
Input:  state (z, ż), reference (z_d, ż_d, z̈_d)
Output: thrust T_c

1.  e   = z  − z_d
    v_e = ż  − ż_d
2.  δ   = v_e + k₁ · e
3.  T_c = m · (g + z̈_d − k₁·v_e − K·δ − e)      [eq. 11a]
    T_c = max(T_c, 0)
──────────────────────────────────────────────────

Algorithm 2: Adaptive Backstepping FTC
──────────────────────────────────────────────────
Input:  state (z, ż), reference (z_d, ż_d, z̈_d),
        current estimate ρ̂
Output: thrust T_c, updated estimate ρ̂

1.  e   = z  − z_d
    v_e = ż  − ż_d
2.  δ   = v_e + k₁ · e
3.  T_nom = m · (g + z̈_d − k₁·v_e − K·δ − e)     [eq. 10]
4.  T_c   = T_nom / ρ̂                               [eq. 11b]
    T_c   = max(T_c, 0)
5.  ρ̂_dot = (γ / (m · ρ̂)) · δ · T_nom              [eq. 12]
    ρ̂ ← ρ̂ + ρ̂_dot · Δt
    ρ̂ = clip(ρ̂, 0.01, 2.0)
──────────────────────────────────────────────────
```

---

## 9. Experimental Setup

| Parameter | Value |
|---|---|
| Simulation time | 20 s |
| Time step $\Delta t$ | 0.01 s |
| Initial altitude | $z_0 = 0$ m |
| Initial velocity | $\dot{z}_0 = 0$ m/s |
| Initial estimate | $\hat{\rho}_0 = 1.0$ |
| Reference step | $0 \to 10$ m at $t = 2$ s |
| Fault onset | $t = 5$ s |
| Fault level | $\rho = 0.5$ |

### Controller Gains

| Gain | Adaptive FTC | Nominal backstepping |
|---|---:|---:|
| $k_1$ — virtual control | 2.0 | 2.0 |
| $K$ — backstepping gain | 6.0 | 6.0 |
| $\gamma$ — adaptation rate | 0.5 | — |

---

## 10. Reproducibility

```bash
pip install -r requirements.txt
python main.py
```

All figures and animations are saved automatically to `figures/` and `animations/`.

| Output file | Content |
|---|---|
| `figures/altitude_tracking.png` | $z(t)$ vs $z_d(t)$, both controllers; shows fault recovery of adaptive FTC |
| `figures/altitude_error.png` | Error $e(t)$; shows permanent $e^* \approx -0.75$ m of nominal vs convergence of adaptive |
| `figures/rho_estimation.png` | $\hat{\rho}(t)$ vs true $\rho = 0.5$; shows convergence of estimation after fault |
| `figures/lyapunov.png` | $L_{\text{aug}}(t)$; confirms monotone decrease predicted by eq. (13) |
| `figures/thrust.png` | $T_c(t)$, both controllers; shows compensation spike at fault onset |
| `figures/phase_portrait.png` | Phase plot $(e, \delta)$; adaptive spirals to origin, nominal converges off-origin |
| `animations/rocket_flight.gif` | Animated rocket with reference line and live $\hat{\rho}$ readout |

All plots include labeled axes with units, legends, and captions stating what conclusion should be drawn.

---

## 11. Results Summary

### 11.1 Altitude Tracking

**What is plotted:** $z(t)$ and $z_d(t)$ for both controllers alongside the error $e(t)$.

**Conclusion:** before $t = 5$ s both controllers track identically. After the fault, the nominal controller drifts to $e^* \approx -0.75$ m and does not recover. The adaptive FTC briefly deviates then converges back to $z_d$, confirming the decay bound (13).

### 11.2 Fault Estimation

**What is plotted:** $\hat{\rho}(t)$ and the true value $\rho = 0.5$ (dashed horizontal line).

**Conclusion:** $\hat{\rho}$ starts at 1.0, begins decreasing after $t = 5$ s, and converges toward 0.5. Convergence speed is controlled by $\gamma$. The estimate stabilizes close to but not necessarily exactly at $\rho$, consistent with the LaSalle analysis in Section 6.

### 11.3 Lyapunov Function

**What is plotted:** $L_{\text{aug}}(t) = \frac{1}{2}e^2 + \frac{1}{2}\delta^2 + \frac{1}{2\gamma}\tilde{\rho}^2$.

**Conclusion:** $L_{\text{aug}}$ decreases monotonically before the fault. After the fault it rises briefly then decreases again, directly confirming equation (13).

### 11.4 Phase Portrait

**What is plotted:** trajectories in the $(e, \delta)$ plane for both controllers.

**Conclusion:** the adaptive FTC trajectory spirals into the origin $(0, 0)$. The nominal controller trajectory converges to a point with $e \approx -0.75$ m, visually confirming the steady-state error derived in Section 5.

### 11.5 What Works and What Does Not

**Works:**
- altitude tracking without fault — asymptotic stability confirmed via LaSalle for both controllers
- recovery after thruster fault — adaptive law drives $e, \delta \to 0$ for any $\rho \in (0,1]$
- clear, quantifiable outperformance: nominal settles at $e^* \approx -0.75$ m, adaptive converges to $e = 0$

**Does not work / limitations:**
- $\hat{\rho}$ converges to a neighborhood of $\rho$, not necessarily to $\rho$ exactly; full parameter convergence requires persistent excitation
- model is 1D: attitude dynamics and aerodynamic drag are not modeled
- adaptation gain $\gamma$ must be tuned; too large causes oscillations in $\hat{\rho}$

---

## 12. Project Structure

```
project_3_fault_tolerant_control_rocket/
├── README.md
├── requirements.txt
├── main.py
├── configs/
│   └── params.yaml
├── src/
│   ├── __init__.py
│   ├── system.py          # rocket dynamics, fault model
│   ├── controller.py      # adaptive FTC + nominal backstepping
│   ├── simulation.py      # integration loop
│   └── visualization.py   # all plotting and animation
├── figures/
│   ├── altitude_tracking.png
│   ├── altitude_error.png
│   ├── rho_estimation.png
│   ├── lyapunov.png
│   ├── thrust.png
│   └── phase_portrait.png
└── animations/
    └── rocket_flight.gif
```

---

## 13. References

1. Fonte, A., dos Santos, P., Oliveira, P. (2025). *Control and Navigation of a 2-D Electric Rocket*. arXiv:2509.19970.
2. Ren, Z. (2023). *A Survey of Modularized Backstepping Control Design*. arXiv:2305.02066.
3. Khalil, H.K. (2002). *Nonlinear Systems*, 3rd ed. Prentice Hall. Section 14.3.
4. Krstic, M., Kanellakopoulos, I., Kokotovic, P. (1995). *Nonlinear and Adaptive Control Design*. Wiley.

---

## 14. Note on AI Usage

AI (Claude, Anthropic) was used to assist with structuring the mathematical derivations, organizing the documentation, and explaining concepts during the learning process. All control design decisions, stability analysis steps, and simulation results were produced and verified by the team.
