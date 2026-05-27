"""Run Project 4 simulations and generate all figures."""

from __future__ import annotations

import numpy as np

from environment import scenarios, validate_obstacle_layout
from experiments import run_all, run_mpc
from mpc_controller import MPCWeights
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


def clean_outputs(*output_dirs) -> None:
    for pattern in ("*.png", "*.gif"):
        for output_dir in output_dirs:
            for path in output_dir.glob(pattern):
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


def run_energy_study(scenario) -> list[dict]:
    weights = np.round(np.linspace(0.0, 1.50, 25), 2)
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


def validate_generated_artifacts(figures_dir, animations_dir, scenario_names) -> None:
    required_paths = [
        figures_dir / "robot_model_schematic.png",
        figures_dir / "comparison_trajectories.png",
        figures_dir / "comparison_distance_to_goal.png",
        figures_dir / "comparison_safety_margin.png",
        figures_dir / "comparison_heading_change.png",
        figures_dir / "comparison_control_effort.png",
        figures_dir / "energy_weight_study.png",
    ]
    required_paths.extend(animations_dir / f"{name}_comparison.gif" for name in scenario_names)

    print("Generated artifacts check:")
    for path in required_paths:
        if not path.exists():
            raise FileNotFoundError(f"Missing generated artifact: {path}")
        if path.stat().st_size == 0:
            raise RuntimeError(f"Generated artifact is empty: {path}")
        print(f"  [OK] {path.as_posix()}")


def main() -> None:
    figures_dir, animations_dir = ensure_dirs()
    clean_outputs(figures_dir, animations_dir)
    all_scenarios = scenarios()
    all_results = {}
    for scenario in all_scenarios:
        margin, pair, time = validate_obstacle_layout(list(scenario.obstacles), scenario.config)
        print(f"{scenario.name}: obstacle layout valid, min margin {margin:.3f} m for {pair[0]}/{pair[1]} at t={time:.2f}s")
        result_map = run_all(scenario)
        results = [result_map["mpc"], result_map["apf"]]
        all_results[scenario.name] = results
        create_comparison_gif(results, animations_dir / f"{scenario.name}_comparison.gif", scenario)
        print_summary(results)

    representative = all_scenarios[0]
    results = all_results[representative.name]
    plot_robot_schematic(figures_dir / "robot_model_schematic.png")
    plot_comparison_trajectories(results, figures_dir / "comparison_trajectories.png", representative)
    plot_time_comparison(
        results,
        "goal_distances",
        "distance [m]",
        "Distance to goal",
        figures_dir / "comparison_distance_to_goal.png",
        representative,
    )
    plot_time_comparison(
        results,
        "safety_margins",
        "safety margin [m]",
        "Safety margin to obstacles",
        figures_dir / "comparison_safety_margin.png",
        representative,
    )
    plot_heading_change(results, figures_dir / "comparison_heading_change.png")
    plot_control_effort(results, figures_dir / "comparison_control_effort.png", representative)
    energy_records = run_energy_study(representative)
    plot_energy_weight_study(energy_records, figures_dir / "energy_weight_study.png")
    validate_generated_artifacts(figures_dir, animations_dir, [scenario.name for scenario in all_scenarios])
    print(f"Figures saved to {figures_dir.resolve()}")
    print(f"Animations saved to {animations_dir.resolve()}")


if __name__ == "__main__":
    main()
