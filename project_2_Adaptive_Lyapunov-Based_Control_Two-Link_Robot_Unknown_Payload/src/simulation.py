"""
Simulation engine for the closed-loop manipulator.

Non-PID controllers are integrated as continuous closed-loop systems, including
any controller-owned extra states such as payload-mass estimates. The legacy
sampled PID path is kept for compatibility with the old project version.
"""

import numpy as np
from scipy.integrate import solve_ivp
from src.system import TwoLinkManipulator


def _time_grid(t_span: tuple, dt: float) -> np.ndarray:
    start, end = t_span
    n_steps = int(round((end - start) / dt)) + 1
    return np.linspace(start, end, n_steps)


def run_simulation(
    robot: TwoLinkManipulator,
    controller,
    initial_state: np.ndarray,
    theta_d: np.ndarray,
    t_span: tuple,
    dt: float,
    is_pid: bool = False,
    return_estimates: bool = False,
):
    """Simulate the closed-loop system.

    Parameters
    ----------
    robot         : TwoLinkManipulator plant
    controller    : controller object with compute_control(...)
    initial_state : [theta1, theta2, dtheta1, dtheta2]
    theta_d       : [theta1_d, theta2_d]
    t_span        : (t_start, t_end)
    dt            : fixed output time step
    is_pid        : if True, use the legacy sampled PID integration path
    return_estimates : if True, return controller extra states after torques

    Returns
    -------
    times   : (N,)    time stamps
    states  : (N, 4)  state trajectories
    torques : (N, 2)  applied control actions
    estimates : (N, k) optional controller extra states
    """
    t_eval = _time_grid(t_span, dt)
    n_steps = len(t_eval)

    if not is_pid:
        if hasattr(controller, "initial_extended_state"):
            y0 = controller.initial_extended_state(initial_state)
        else:
            y0 = np.asarray(initial_state, dtype=float)

        def rhs(t, y):
            torque = controller.compute_control(y, theta_d)
            plant_derivative = robot.dynamics(t, y[:4], torque)
            if hasattr(controller, "extra_state_derivative"):
                extra_derivative = controller.extra_state_derivative(y, theta_d)
                return np.concatenate([plant_derivative, extra_derivative])
            return plant_derivative

        sol = solve_ivp(
            fun=rhs,
            t_span=t_span,
            y0=y0,
            t_eval=t_eval,
            method="RK45",
            max_step=dt,
            rtol=1e-7,
            atol=1e-9,
        )
        if not sol.success:
            raise RuntimeError(f"Integration failed: {sol.message}")

        full_states = sol.y.T
        states = full_states[:, :4]
        torques = np.array([controller.compute_control(y, theta_d) for y in full_states])

        if return_estimates:
            return t_eval, states, torques, full_states[:, 4:]
        return t_eval, states, torques

    states = np.zeros((n_steps, 4))
    torques = np.zeros((n_steps, 2))

    current_state = np.array(initial_state, dtype=float)

    if is_pid and hasattr(controller, "reset"):
        controller.reset()

    for i in range(n_steps):
        states[i] = current_state

        # --- control action ---
        if is_pid:
            a = controller.compute_control(current_state, theta_d, dt)
        else:
            a = controller.compute_control(current_state, theta_d)

        torques[i] = a

        # --- integrate one step ---
        if i < n_steps - 1:
            sol = solve_ivp(
                fun=lambda t, y: robot.dynamics(t, y, a),
                t_span=[t_eval[i], t_eval[i] + dt],
                y0=current_state,
                method="RK45",
                rtol=1e-6,
                atol=1e-9,
            )
            if sol.success:
                current_state = sol.y[:, -1]
            else:
                print(f"[WARNING] Integration failed at t = {t_eval[i]:.3f}")
                break

    if return_estimates:
        return t_eval, states, torques, np.empty((n_steps, 0))
    return t_eval, states, torques
