# Project 4: Torque-Based MPC for Dynamic Obstacle Avoidance

![MPC vs APF comparison](results/map_1_comparison.gif)

**Animation.** Representative comparison on map 1. The MPC robot is blue, the APF baseline is magenta, static obstacles are gray, and moving obstacles are orange.

---

## 1. Problem Statement

**Control problem:** drive a differential-drive mobile robot from a start position to a desired goal position in a 2D workspace with static and moving circular obstacles. The obstacle motion is known and predicted over the MPC horizon. The controller must explicitly handle dynamic obstacle avoidance, map boundary constraints, and actuator limits while maintaining efficient and smooth motion.

**Plant:** a differential-drive mobile robot moving in a planar workspace. The robot state includes planar position, heading angle, linear velocity, and angular velocity. The control inputs are left and right wheel torques. The robot is affected by actuator limits, damping, map boundaries, and collision constraints with static and dynamic obstacles.

**Class of methods:** torque-based Model Predictive Control. At each time step, the controller predicts future robot states over a finite horizon, evaluates predicted obstacle positions, rejects infeasible trajectories that violate safety constraints, and selects a torque sequence minimizing goal-tracking error, path length, control effort, smoothness, and energy-related cost. Only the first torque input is applied, and the optimization is repeated at the next step.

**Comparison:** the MPC controller is compared with an Artificial Potential Field baseline. APF uses attractive forces toward the goal and repulsive forces from obstacles. Unlike MPC, APF is reactive and does not optimize a future torque sequence over a horizon. The comparison evaluates goal reaching, obstacle clearance, near-obstacle events, path length, path efficiency, smoothness, heading aggressiveness, control effort, and boundary violations.

---

## 2. System Model

![Differential-drive robot schematic](results/robot_model_schematic.png)

**Figure 1.** Differential-drive robot model with center position, heading, linear velocity, angular velocity, and wheel torque inputs.

The robot state is

```math
x_k = [p_{x,k},\;p_{y,k},\;\theta_k,\;v_k,\;\omega_k]^T \qquad\text{(1)}
```

where $x_k$ is the robot state at discrete step $k$, $p_{x,k}$ and $p_{y,k}$ are the robot coordinates in meters, $\theta_k$ is the heading angle in radians, $v_k$ is the linear velocity, and $\omega_k$ is the angular velocity.

The MPC control input is

```math
u_k = [\tau_{R,k},\;\tau_{L,k}]^T \qquad\text{(2)}
```

where $u_k$ is the control input, $\tau_{R,k}$ is the right wheel torque, and $\tau_{L,k}$ is the left wheel torque.

The discrete robot model used in `robot_model.py` is

```math
p_{x,k+1} = p_{x,k} + v_k \cos(\theta_k)\Delta t \qquad\text{(3)}
```

where $p_{x,k}$ is the current x-coordinate, $v_k$ is the linear velocity, $\theta_k$ is the heading angle, and $\Delta t$ is the simulation time step.

```math
p_{y,k+1} = p_{y,k} + v_k \sin(\theta_k)\Delta t \qquad\text{(4)}
```

where $p_{y,k}$ is the current y-coordinate, $v_k$ is the linear velocity, $\theta_k$ is the heading angle, and $\Delta t$ is the simulation time step.

```math
\theta_{k+1} = \theta_k + \omega_k\Delta t \qquad\text{(5)}
```

where $\theta_k$ is the current heading angle, $\omega_k$ is the angular velocity, and $\Delta t$ is the simulation time step. The implementation wraps the updated angle to the interval $[-\pi,\pi]$.

```math
v_{k+1} = v_k + \left(k_v(\tau_{R,k}+\tau_{L,k}) - c_v v_k\right)\Delta t \qquad\text{(6)}
```

where $v_k$ is the current linear velocity, $k_v=0.85$ is the longitudinal acceleration gain, $\tau_{R,k}$ and $\tau_{L,k}$ are wheel torques, $c_v=0.55$ is the linear damping coefficient, and $\Delta t$ is the simulation time step.

```math
\omega_{k+1} = \omega_k + \left(k_\omega(\tau_{R,k}-\tau_{L,k}) - c_\omega\omega_k\right)\Delta t \qquad\text{(7)}
```

where $\omega_k$ is the current angular velocity, $k_\omega=1.25$ is the angular acceleration gain, $\tau_{R,k}-\tau_{L,k}$ is the torque difference, $c_\omega=0.65$ is the angular damping coefficient, and $\Delta t$ is the simulation time step.

---

## 3. Obstacle Model and Constraints

Map boundary constraints are enforced with the robot radius included:

```math
x_{\min}+r_{\mathrm{robot}} \le p_x \le x_{\max}-r_{\mathrm{robot}} \qquad\text{(8)}
```

where $x_{\min}$ and $x_{\max}$ are the map bounds along the x-axis, $p_x$ is the robot x-coordinate, and $r_{\mathrm{robot}}$ is the robot radius.

```math
y_{\min}+r_{\mathrm{robot}} \le p_y \le y_{\max}-r_{\mathrm{robot}} \qquad\text{(9)}
```

where $y_{\min}$ and $y_{\max}$ are the map bounds along the y-axis, $p_y$ is the robot y-coordinate, and $r_{\mathrm{robot}}$ is the robot radius.

The predicted obstacle clearance constraint is

```math
\|p_k - c_{j,k}\| \ge r_{\mathrm{robot}} + r_j + r_{\mathrm{margin}} \qquad\text{(10)}
```

where $p_k=[p_{x,k},p_{y,k}]^T$ is the robot position, $c_{j,k}$ is the center of obstacle $j$ at step $k$, $r_j$ is the obstacle radius, and $r_{\mathrm{margin}}=0.12$ m is the additional safety margin.

The velocity constraints are

```math
0 \le v_k \le v_{\max} \qquad\text{(11)}
```

where $v_k$ is the robot linear velocity and $v_{\max}=1.35$ m/s is the maximum allowed forward velocity.

```math
-\omega_{\max} \le \omega_k \le \omega_{\max} \qquad\text{(12)}
```

where $\omega_k$ is the robot angular velocity and $\omega_{\max}=1.9$ rad/s is the maximum allowed absolute angular velocity.

The wheel torque constraints are

```math
-\tau_{\max} \le \tau_{R,k} \le \tau_{\max} \qquad\text{(13)}
```

where $\tau_{R,k}$ is the right wheel torque and $\tau_{\max}=1.35$ is the maximum allowed torque magnitude.

```math
-\tau_{\max} \le \tau_{L,k} \le \tau_{\max} \qquad\text{(14)}
```

where $\tau_{L,k}$ is the left wheel torque and $\tau_{\max}=1.35$ is the maximum allowed torque magnitude.

Moving obstacles have deterministic constant-velocity motion:

```math
c_j(t) = c_{j,0} + v_j t \qquad\text{(15)}
```

where $c_j(t)$ is the center of moving obstacle $j$ at time $t$, $c_{j,0}$ is its initial position, and $v_j$ is its constant velocity vector.

The implementation validates every map before simulation by sampling the full time interval and checking static-static, static-moving, and moving-moving disk separation.

---

## 4. MPC Design

The finite-horizon objective is

```math
J_N(x_k,U_k)=\sum_{i=0}^{N-1}\ell(x_{i|k},u_{i|k}) + J_f(x_{N|k}) \qquad\text{(16)}
```

where $J_N$ is the finite-horizon MPC cost, $x_k$ is the current state, $U_k$ is the candidate control sequence, $\ell$ is the stage cost, $J_f$ is the terminal cost, and $N=50$ is the prediction horizon.

The stage cost is written compactly as

```math
\ell(x_{i|k},u_{i|k}) = w_g J_g + w_o J_o + w_e J_e + w_s J_s + w_p J_p \qquad\text{(17)}
```

where $J_g$ is the goal-tracking term, $J_o$ is the obstacle and boundary shaping term, $J_e$ is the energy-related term, $J_s$ is the smoothness term, $J_p$ is the path-efficiency term, and $w_g,w_o,w_e,w_s,w_p$ are positive weights. Hard safety, input, velocity, and terminal feasibility checks are applied before this cost is used for comparison.

Goal tracking is

```math
J_g = \|p_{i|k}-p_{\mathrm{goal}}\|^2 \qquad\text{(18)}
```

where $J_g$ is the predicted position error, $p_{i|k}$ is the predicted robot position at horizon index $i$, and $p_{\mathrm{goal}}$ is the goal position.

The motor-energy term is

```math
J_e = (\tau_{R,i|k}^2+\tau_{L,i|k}^2)\Delta t \qquad\text{(19)}
```

where $J_e$ is the torque-based energy proxy for one prediction step, $\tau_{R,i|k}$ and $\tau_{L,i|k}$ are the predicted wheel torques, and $\Delta t$ is the simulation time step.

The smoothness term is

```math
J_s = \|u_{i|k}-u_{i-1|k}\|^2 \qquad\text{(20)}
```

where $J_s$ penalizes changes in the planned input sequence, $u_{i|k}$ is the predicted control input, and $u_{i-1|k}$ is the previous predicted control input. For the first horizon step, the previous applied control is used.

The terminal cost is

```math
J_f(x_{N|k}) = w_f\|p_{N|k}-p_{\mathrm{goal}}\|^2 \qquad\text{(21)}
```

where $J_f$ is the terminal position cost, $p_{N|k}$ is the predicted terminal position, $p_{\mathrm{goal}}$ is the goal position, and $w_f$ is the terminal cost weight.

In each MPC step, the controller samples candidate torque sequences, rolls out the model, rejects candidates that violate the hard constraints, evaluates only feasible candidates, applies the first torque input, and stores the feasible sequence for warm starting the next step.

---

## 5. APF Baseline

The APF controller commands linear and angular velocity directly. It combines attraction to the goal with local repulsion from current obstacle positions. It does not predict future obstacle positions and does not optimize wheel torques over a horizon. This makes APF computationally light but more reactive in moving-obstacle scenarios.

The attractive potential is

```math
U_{\mathrm{att}}(p)=\frac{1}{2}k_{\mathrm{att}}\|p-p_{\mathrm{goal}}\|^2 \qquad\text{(22)}
```

where $U_{\mathrm{att}}$ is the attractive potential, $k_{\mathrm{att}}$ is the attraction gain, $p$ is the robot position, and $p_{\mathrm{goal}}$ is the goal position.

The repulsive potential is

```math
U_{\mathrm{rep},j}(p)=\frac{1}{2}k_{\mathrm{rep}}\left(\frac{1}{d_j(p)}-\frac{1}{d_0}\right)^2 \qquad\text{(23)}
```

where $U_{\mathrm{rep},j}$ is the repulsive potential from obstacle $j$, $k_{\mathrm{rep}}$ is the repulsion gain, $d_j(p)$ is the distance to obstacle $j$, and $d_0$ is the repulsion radius. This repulsive term is active only when $d_j(p)<d_0$.

---

## 6. Algorithm Pipeline

1. Initialize the scenario, robot state, goal, static obstacles, and moving obstacles.
2. Validate that obstacle disks do not overlap over the full simulation time.
3. Predict future robot states and moving-obstacle positions over the MPC horizon.
4. Reject infeasible MPC candidates that violate safety, input, velocity, or terminal constraints.
5. Select the feasible torque sequence with the lowest cost and apply only its first input.
6. Run APF on the same scenario using the same robot model limits.
7. Log goal distance, clearance, safety margin, path length, smoothness, effort, time, collisions, and boundary violations.
8. Repeat this procedure for five obstacle maps and generate GIFs and plots.

---

## 7. Simulation Setup and Metrics

| Quantity | Value |
|---|---:|
| Start state | $[0.0,0.0,0.08,0.0,0.0]^T$ |
| Goal | $[9.0,6.0]^T$ m |
| Time step | 0.15 s |
| Maximum steps | 180 |
| Goal tolerance | 0.35 m |
| Map bounds | $x\in[-0.25,9.75]$ m, $y\in[-0.25,6.55]$ m |
| Robot radius | 0.18 m |
| Safety buffer | 0.12 m |
| MPC horizon | 50 |
| Torque limit | 1.35 |

Path length is

```math
L_{\mathrm{path}} = \sum_{k=0}^{T-1}\|p_{k+1}-p_k\| \qquad\text{(24)}
```

where $L_{\mathrm{path}}$ is the total path length, $p_k$ is the robot position at step $k$, and $T$ is the final simulated step index.

Path efficiency is

```math
\eta_{\mathrm{path}}=\frac{\|p_{\mathrm{goal}}-p_0\|}{L_{\mathrm{path}}} \qquad\text{(25)}
```

where $\eta_{\mathrm{path}}$ is the path efficiency, $p_{\mathrm{goal}}$ is the goal position, $p_0$ is the initial position, and $L_{\mathrm{path}}$ is the total path length.

The position-based smoothness metric is

```math
S_{\mathrm{pos}}=\sum_{k=1}^{T-2}\|p_{k+1}-2p_k+p_{k-1}\| \qquad\text{(26)}
```

where $S_{\mathrm{pos}}$ is the position-based smoothness metric, $p_{k-1}$, $p_k$, and $p_{k+1}$ are consecutive robot positions, and lower values indicate smoother motion.

Control effort is

```math
E_{\mathrm{ctrl}}=\sum_{k=0}^{T-1}\|u_k\|^2\Delta t \qquad\text{(27)}
```

where $E_{\mathrm{ctrl}}$ is the cumulative control effort, $u_k$ is the applied input, and $\Delta t$ is the simulation time step. For MPC, $u_k$ is a torque input; for APF, $u_k$ is a velocity command.

Safety margin is

```math
m_{\mathrm{safe},k}=\min_j\left(\|p_k-c_{j,k}\|-r_{\mathrm{robot}}-r_j\right) \qquad\text{(28)}
```

where $m_{\mathrm{safe},k}$ is the minimum obstacle clearance at step $k$, $p_k$ is the robot position, $c_{j,k}$ is the center of obstacle $j$, $r_{\mathrm{robot}}$ is the robot radius, and $r_j$ is the obstacle radius.

Near-obstacle events are counted as

```math
N_{\mathrm{near}}=\sum_{k=0}^{T}\mathbf{1}\left[m_{\mathrm{safe},k}<m_{\mathrm{threshold}}\right] \qquad\text{(29)}
```

where $N_{\mathrm{near}}$ is the number of near-obstacle events, $\mathbf{1}[\cdot]$ is the indicator function, $m_{\mathrm{safe},k}$ is the minimum obstacle clearance, and $m_{\mathrm{threshold}}=0.45$ m is the implemented near-obstacle threshold.

The energy study uses the grid

```math
w_{\mathrm{energy}}\in\{0.00,0.03,0.06,\ldots,0.60\} \qquad\text{(30)}
```

where $w_{\mathrm{energy}}$ is the weight of the motor-energy term in the MPC cost. The implementation evaluates 21 values.

The motor energy proxy is

```math
E_{\mathrm{motor}}=\sum_{k=0}^{T-1}(\tau_{R,k}^2+\tau_{L,k}^2)\Delta t \qquad\text{(31)}
```

where $E_{\mathrm{motor}}$ is the torque-based motor-energy proxy, $\tau_{R,k}$ and $\tau_{L,k}$ are applied right and left wheel torques, and $\Delta t$ is the simulation time step.

---

## 8. Representative Map Results

![Trajectory comparison](results/comparison_trajectories.png)

**Figure 2.** Representative map 1 trajectory comparison. MPC moves through the corridor with a shorter, smoother path, while APF reacts later to the moving obstacles.

![Distance to goal comparison](results/comparison_distance_to_goal.png)

**Figure 3.** Distance to goal on map 1. Both methods reach the target, but MPC reaches it earlier.

![Safety margin comparison](results/comparison_safety_margin.png)

**Figure 4.** Safety margin on map 1. MPC keeps a larger safety margin for most of the run.

![Motion smoothness comparison](results/comparison_heading_change.png)

**Figure 5.** Heading and position-based smoothness on map 1. MPC has much lower accumulated heading change and second-difference smoothness.

![Control effort comparison](results/comparison_control_effort.png)

**Figure 6.** Cumulative control effort on map 1. MPC uses substantially less effort than APF.

The representative map is a dynamic timing problem: moving obstacles cross the natural lower-left to upper-right route. MPC reacts earlier because it evaluates future obstacle locations across the horizon. APF still reaches the goal, but its local repulsion produces sharper steering corrections, more near-obstacle events, and larger accumulated effort.

---

## 9. Robustness Validation on Five Maps

![Map 1 comparison](results/map_1_comparison.gif)

**Map 1.** Central crossing obstacle corridor.

![Map 2 comparison](results/map_2_comparison.gif)

**Map 2.** Staggered corridor with diagonal and vertical obstacle crossings. This map is intentionally different from map 1: the static obstacles are placed on opposite sides of the diagonal route and the moving obstacles use non-horizontal motion.

![Map 3 comparison](results/map_3_comparison.gif)

**Map 3.** Upper corridor with diagonal and vertical moving obstacle motion.

![Map 4 comparison](results/map_4_comparison.gif)

**Map 4.** Lower corridor with delayed crossing obstacles. The timing makes APF pass closer to the moving obstacle, while MPC preserves a larger clearance through prediction.

![Map 5 comparison](results/map_5_comparison.gif)

**Map 5.** Wider passage with two separated moving obstacle interactions.

| Map | Method | Reached goal | Final dist. | Time | Min clearance | Avg margin | Near events | Path length | Efficiency | Smoothness | Effort | Boundary violations | Collision |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | MPC | True | 0.307 | 8.70 | 0.338 | 1.066 | 6 | 10.671 | 1.014 | 0.595 | 7.076 | 0 | False |
| 1 | APF | True | 0.337 | 13.95 | 0.204 | 1.013 | 30 | 11.387 | 0.950 | 2.805 | 32.352 | 0 | False |
| 2 | MPC | True | 0.297 | 8.85 | 0.778 | 1.318 | 0 | 10.853 | 0.997 | 0.556 | 7.007 | 0 | False |
| 2 | APF | True | 0.346 | 11.25 | 0.590 | 1.124 | 0 | 11.112 | 0.973 | 1.046 | 18.997 | 0 | False |
| 3 | MPC | True | 0.339 | 8.55 | 0.546 | 0.992 | 0 | 10.610 | 1.019 | 0.495 | 7.258 | 0 | False |
| 3 | APF | True | 0.343 | 12.00 | 0.372 | 0.867 | 3 | 11.357 | 0.952 | 1.803 | 26.894 | 0 | False |
| 4 | MPC | True | 0.294 | 8.85 | 0.589 | 1.132 | 0 | 10.777 | 1.004 | 0.641 | 7.512 | 0 | False |
| 4 | APF | True | 0.346 | 11.40 | 0.445 | 1.118 | 1 | 11.110 | 0.974 | 1.310 | 21.844 | 0 | False |
| 5 | MPC | True | 0.340 | 8.55 | 0.289 | 1.013 | 6 | 10.627 | 1.018 | 0.554 | 7.417 | 0 | False |
| 5 | APF | True | 0.339 | 13.50 | 0.330 | 0.790 | 20 | 11.392 | 0.949 | 2.732 | 33.150 | 0 | False |

| Metric | MPC average | APF average | MPC advantage |
|---|---:|---:|---:|
| Time to goal | 8.700 s | 12.420 s | 30.0% lower |
| Average safety margin | 1.104 m | 0.982 m | 12.4% higher |
| Near-obstacle events | 2.400 | 10.800 | 77.8% lower |
| Path length | 10.708 m | 11.272 m | 5.0% lower |
| Motion smoothness | 0.568 m | 1.939 m | 70.7% lower |
| Control effort | 7.254 | 26.647 | 72.8% lower |
| Heading aggressiveness | 3.802 | 28.396 | 86.6% lower |

Across the five maps, both methods reach the goal with no collisions and no boundary violations. MPC is consistently smoother and lower effort. APF remains a valid baseline, but its local repulsive field causes sharper turns, more near-obstacle events, and higher cumulative effort. Map 2 confirms that the result is not tied to a single horizontal-crossing layout, while map 4 specifically highlights the value of predicting a delayed moving obstacle before it reaches the corridor.

---

## 10. Energy-Efficiency Analysis

![Energy weight study](results/energy_weight_study.png)

**Figure 7.** Energy penalty sweep on map 1 using 21 penalty weights from 0.00 to 0.60.

| Energy weight | Final dist. | Time | Path length | Motor energy | Effort | Smoothness | Avg margin |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.00 | 0.318 | 8.70 | 10.684 | 7.503 | 7.503 | 0.601 | 1.068 |
| 0.03 | 0.318 | 8.70 | 10.684 | 7.503 | 7.503 | 0.601 | 1.068 |
| 0.06 | 0.318 | 8.70 | 10.684 | 7.503 | 7.503 | 0.601 | 1.068 |
| 0.09 | 0.318 | 8.70 | 10.684 | 7.503 | 7.503 | 0.601 | 1.068 |
| 0.12 | 0.307 | 8.70 | 10.671 | 7.076 | 7.076 | 0.595 | 1.066 |
| 0.15 | 0.307 | 8.70 | 10.671 | 7.076 | 7.076 | 0.595 | 1.066 |
| 0.18 | 0.307 | 8.70 | 10.671 | 7.076 | 7.076 | 0.595 | 1.066 |
| 0.21 | 0.330 | 8.70 | 10.648 | 7.054 | 7.054 | 0.601 | 1.065 |
| 0.24 | 0.331 | 8.70 | 10.661 | 7.280 | 7.280 | 0.600 | 1.068 |
| 0.27 | 0.278 | 8.85 | 10.750 | 7.174 | 7.174 | 0.648 | 1.093 |
| 0.30 | 0.318 | 8.70 | 10.663 | 7.047 | 7.047 | 0.592 | 1.067 |
| 0.33 | 0.318 | 8.70 | 10.663 | 7.047 | 7.047 | 0.592 | 1.067 |
| 0.36 | 0.318 | 8.70 | 10.663 | 7.047 | 7.047 | 0.592 | 1.067 |
| 0.39 | 0.318 | 8.70 | 10.663 | 7.047 | 7.047 | 0.592 | 1.067 |
| 0.42 | 0.323 | 8.70 | 10.658 | 7.060 | 7.060 | 0.595 | 1.067 |
| 0.45 | 0.323 | 8.70 | 10.658 | 7.060 | 7.060 | 0.595 | 1.067 |
| 0.48 | 0.323 | 8.70 | 10.658 | 7.060 | 7.060 | 0.595 | 1.067 |
| 0.51 | 0.284 | 8.85 | 10.727 | 6.975 | 6.975 | 0.628 | 1.091 |
| 0.54 | 0.284 | 8.85 | 10.727 | 6.975 | 6.975 | 0.628 | 1.091 |
| 0.57 | 0.295 | 8.85 | 10.699 | 7.021 | 7.021 | 0.619 | 1.088 |
| 0.60 | 0.295 | 8.85 | 10.699 | 7.021 | 7.021 | 0.619 | 1.088 |

Increasing the energy penalty generally reduces aggressive torque usage. The trend is not perfectly monotonic because the optimizer is sampling-based and feasible candidates can change discretely, but the higher-weight region produces lower motor-energy values than the zero-weight case. Very large weights slightly increase travel time or path length because the controller becomes more conservative. The selected default value keeps the path fast and smooth while still reducing motor effort.

---

## 11. Project Structure

```text
project_4_mpc_obstacle_navigation/
├── README.md
├── main.py
├── robot_model.py
├── environment.py
├── mpc_controller.py
├── potential_field_controller.py
├── metrics.py
├── visualization.py
├── experiments.py
├── requirements.txt
└── results/
```

| File | Purpose |
|---|---|
| `environment.py` | five scenarios, robot parameters, obstacles, and validation |
| `robot_model.py` | differential-drive dynamics and rollout functions |
| `mpc_controller.py` | constrained sampling MPC with torque inputs |
| `potential_field_controller.py` | APF baseline |
| `experiments.py` | simulation loops and metrics |
| `visualization.py` | plots, robot schematic, and GIF generation |
| `main.py` | reproducible experiment pipeline |

---

## 12. How to Run

Install dependencies:

```bash
pip install -r requirements.txt
```

Run the project:

```bash
cd project_4_mpc_obstacle_navigation
python main.py
```

The command regenerates five comparison GIFs, representative plots, and the energy-weight study in `results/`.

---

## 13. Conclusion

This project implements a torque-based Model Predictive Controller for a differential-drive robot navigating among static and moving circular obstacles. The controller predicts future robot motion and future obstacle positions, rejects unsafe candidate trajectories, and optimizes wheel torques for goal tracking, smoothness, path efficiency, and motor-energy use.

The five-map validation shows that the MPC controller reaches the goal in all tested scenarios while satisfying map boundaries and avoiding collisions. APF also reaches the goal in all scenarios, which makes the comparison fair, but MPC provides a better safety-efficiency trade-off: fewer near-obstacle events, lower effort, smoother motion, and shorter travel time on average.

The main limitation is computational cost. The sampling-based MPC is much slower than APF, and the guarantees are practical and scenario-tested rather than global. Future work could use a deterministic constrained optimizer, add uncertainty in obstacle prediction, and test denser multi-agent environments.

---

## 14. References

[1] Huang, J., Liu, Z., Zeng, J., Chi, X., & Su, H. (2023). Obstacle Avoidance for Unicycle-Modelled Mobile Robots with Time-varying Control Barrier Functions. arXiv:2307.08227.

[2] Advanced Control Methods course lecture notes. Model Predictive Control, 2026.

---

## 15. AI Usage Declaration

This project used AI assistance to support code structuring, documentation drafting, experiment organization, and README editing. All mathematical formulations, simulation results, generated figures, and final project decisions were reviewed and validated by the authors.
