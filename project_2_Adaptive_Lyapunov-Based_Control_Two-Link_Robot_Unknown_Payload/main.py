"""
Main entry point.

Loads parameters, runs adaptive and non-adaptive simulations on the same
loaded manipulator, then generates plots and animation.

Usage:
    python3 main.py
"""

import numpy as np
import yaml

from src.adaptive_controller import AdaptivePayloadMassController
from src.system import TwoLinkManipulator
from src.controller import LyapunovController
from src.simulation import run_simulation
from src.visualization import (
    create_animation,
    plot_comparison,
    plot_lyapunov,
    plot_parameter_estimation,
    plot_payload_compensation,
    plot_phase_portrait,
)


def load_config(path: str = "configs/params.yaml") -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def settling_time(times: np.ndarray, error_norm: np.ndarray, tolerance: float = 0.02) -> float:
    """First time after which the position error stays below tolerance."""
    for i, _ in enumerate(times):
        if np.all(error_norm[i:] < tolerance):
            return float(times[i])
    return float("nan")


def integral_error(times: np.ndarray, error_norm: np.ndarray) -> float:
    """Integral absolute position error for a scalar error norm."""
    if hasattr(np, "trapezoid"):
        return float(np.trapezoid(error_norm, times))
    return float(np.trapz(error_norm, times))


def main():
    cfg = load_config()

    # --- plant ---
    rp = cfg["robot"]
    true_payload_mass = float(rp["payload_mass"])
    robot = TwoLinkManipulator(
        m1=rp["m1"],
        m2=rp["m2"],
        l1=rp["l1"],
        l2=rp["l2"],
        g=rp["g"],
        payload_mass=true_payload_mass,
    )

    # --- simulation settings ---
    sim = cfg["simulation"]
    t_span = (sim["t_start"], sim["t_end"])
    dt = sim["dt"]
    x0 = np.array(sim["initial_state"], dtype=float)

    ctrl = cfg["control"]
    theta_d = np.array([ctrl["target_theta1"], ctrl["target_theta2"]])

    # --- controllers ---
    base = cfg["baseline"]
    ag = cfg["adaptive_control"]
    ap = cfg["adaptive"]
    adaptive_ctrl = AdaptivePayloadMassController(
        robot,
        lambda_gain=ag["lambda"],
        kd=ag["kd"],
        alpha=ap["alpha"],
        initial_estimate=ap["initial_payload_mass_hat"],
    )
    baseline_ctrl = LyapunovController(robot, k1=base["k1"], k2=base["k2"])

    # --- run simulations ---
    print("Running adaptive payload-mass-estimation simulation ...")
    t_adaptive, states_adaptive, torques_adaptive, estimates = run_simulation(
        robot,
        adaptive_ctrl,
        x0,
        theta_d,
        t_span,
        dt,
        return_estimates=True,
    )

    print("Running non-adaptive PD + known-link gravity baseline on loaded plant ...")
    t_baseline, states_baseline, torques_baseline = run_simulation(
        robot,
        baseline_ctrl,
        x0,
        theta_d,
        t_span,
        dt,
    )

    if not np.allclose(t_adaptive, t_baseline):
        raise RuntimeError("Adaptive and baseline simulations used different time grids")

    # --- compute composite Lyapunov function values ---
    n = len(t_adaptive)
    L_vals = np.zeros(n)
    dL_vals = np.zeros(n)
    for i in range(n):
        extended_state = np.concatenate([states_adaptive[i], estimates[i]])
        L_vals[i], dL_vals[i] = adaptive_ctrl.compute_lyapunov(extended_state, theta_d)

    # --- generate outputs ---
    print("Generating plots ...")
    plot_comparison(
        t_adaptive,
        states_adaptive,
        torques_adaptive,
        states_baseline,
        torques_baseline,
        theta_d,
    )
    plot_lyapunov(t_adaptive, L_vals, dL_vals)
    plot_parameter_estimation(t_adaptive, estimates, true_payload_mass)
    plot_payload_compensation(t_adaptive, states_adaptive, estimates, true_payload_mass, robot)
    plot_phase_portrait(t_adaptive, states_adaptive, theta_d, baseline_states=states_baseline)

    print("Creating animation ...")
    create_animation(
        t_adaptive,
        states_adaptive,
        rp["l1"],
        rp["l2"],
        theta_d,
        estimates=estimates,
        true_payload_mass=true_payload_mass,
        baseline_states=states_baseline,
    )


if __name__ == "__main__":
    main()
