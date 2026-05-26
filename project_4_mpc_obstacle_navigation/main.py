"""Run Project 4 simulations and generate all figures."""

from __future__ import annotations

from environment import default_robot_params, default_target, scenarios, validate_obstacle_layout
from experiments import run_all, run_mpc
from mpc_controller import LyapunovMPCController, MPCWeights
from visualization import (
    create_comparison_gif,
    ensure_dirs,
    plot_comparison_trajectories,
    plot_control_effort,
    plot_energy_weight_study,
    plot_heading_change,
    plot_robot_schematic,
    plot_time_comparison,
)


def clean_outputs(results_dir) -> None:
    for pattern in ("*.png", "*.gif"):
        for path in results_dir.glob(pattern):
            path.unlink()


def print_summary(results: list) -> None:
    for result in results:
        print(f"Experiment: {result.name}")
        print(f"  reached goal: {result.reached}")
        print(f"  final distance: {result.final_distance:.3f} m")
        print(f"  min clearance: {result.min_clearance:.3f} m")
        print(f"  min safety margin: {result.min_safety_margin:.3f} m")
        print(f"  avg safety margin: {result.avg_safety_margin:.3f} m")
        print(f"  boundary violations: {result.boundary_violations}")
        print(f"  path length: {result.path_length:.3f} m")
        print(f"  path smoothness: {result.path_smoothness:.3f} rad")
        print(f"  motion smoothness: {result.motion_smoothness:.3f} m")
        print(f"  heading aggressiveness: {result.heading_aggressiveness:.3f} rad/s")
        print(f"  near-obstacle events: {result.near_obstacle_events}")
        print(f"  progress efficiency: {result.progress_efficiency:.3f}")
        print(f"  average progress rate: {result.average_progress_rate:.3f} m/s")
        print(f"  control effort: {result.control_effort:.3f}")
        print(f"  torque energy: {result.torque_energy:.3f}")
        print(f"  time to goal: {result.time_to_goal:.3f} s")
        print(f"  collision: {result.collision}")
        print(f"  avg compute time: {1000.0 * result.avg_compute_time:.2f} ms")
        if result.name == "MPC":
            print(f"  shifted fallback uses: {result.mpc_shifted_fallbacks}")
            print(f"  emergency actions: {result.mpc_emergency_actions}")
            print(f"  infeasible sampling iterations: {result.mpc_infeasible_iterations}")


def print_terminal_decrease_check(scenario) -> None:
    controller = LyapunovMPCController(scenario.config, default_robot_params(), list(scenario.obstacles), default_target(scenario))
    check = controller.verify_terminal_decrease()
    percent = 100.0 * check["ratio"]
    print(
        "Terminal decrease check: "
        f"{check['satisfied']}/{check['tested']} sampled terminal states decrease "
        f"({percent:.1f}%), max delta {check['max_delta']:.4f}"
    )
    if check["ratio"] < 0.95:
        print("WARNING: terminal decrease check below 95%; terminal set/policy should be retuned.")


def run_energy_study(scenario) -> list[dict]:
    weights = [0.0, 0.01, 0.05, 0.1, 0.2, 0.5]
    records = []
    for weight in weights:
        result = run_mpc(scenario, weights=MPCWeights(energy=weight))
        records.append(
            {
                "weight": weight,
                "final_distance": result.final_distance,
                "time_to_goal": result.time_to_goal,
                "path_length": result.path_length,
                "torque_energy": result.torque_energy,
                "control_effort": result.control_effort,
                "motion_smoothness": result.motion_smoothness,
                "avg_safety_margin": result.avg_safety_margin,
            }
        )
    return records


def main() -> None:
    results_dir = ensure_dirs()
    clean_outputs(results_dir)
    all_scenarios = scenarios()
    all_results = {}
    for scenario in all_scenarios:
        margin, pair, time = validate_obstacle_layout(list(scenario.obstacles), scenario.config)
        print(f"{scenario.name}: obstacle layout valid, min margin {margin:.3f} m for {pair[0]}/{pair[1]} at t={time:.2f}s")
        result_map = run_all(scenario)
        results = [result_map["mpc"], result_map["apf"]]
        all_results[scenario.name] = results
        create_comparison_gif(results, results_dir / f"{scenario.name}_comparison.gif", scenario)
        print_summary(results)

    representative = all_scenarios[0]
    results = all_results[representative.name]
    print_terminal_decrease_check(representative)

    plot_robot_schematic(results_dir / "robot_model_schematic.png")
    plot_comparison_trajectories(results, results_dir / "comparison_trajectories.png", representative)
    plot_time_comparison(
        results,
        "goal_distances",
        "distance [m]",
        "Distance to goal",
        results_dir / "comparison_distance_to_goal.png",
        representative,
    )
    plot_time_comparison(
        results,
        "safety_margins",
        "safety margin [m]",
        "Safety margin to obstacles",
        results_dir / "comparison_safety_margin.png",
        representative,
    )
    plot_heading_change(results, results_dir / "comparison_heading_change.png")
    plot_control_effort(results, results_dir / "comparison_control_effort.png", representative)
    energy_records = run_energy_study(representative)
    plot_energy_weight_study(energy_records, results_dir / "energy_weight_study.png")
    print(f"Results saved to {results_dir.resolve()}")


if __name__ == "__main__":
    main()
