from __future__ import annotations

from pathlib import Path

from src.simulation import load_config, run_all
from src.visualization import create_all_outputs


def main() -> None:
    config = load_config(Path("configs") / "params.yaml")
    results = run_all(config)
    figures_dir, animations_dir = create_all_outputs(results, config)

    nominal = results["nominal"]
    adaptive = results["adaptive"]
    print("Simulation complete.")
    print(f"Figures saved to: {figures_dir.resolve()}")
    print(f"Animation saved to: {(animations_dir / 'rocket_flight.gif').resolve()}")
    print(
        "Final nominal error: "
        f"{float(nominal['e'][-1]):+.4f} m, "
        "final adaptive error: "
        f"{float(adaptive['e'][-1]):+.4f} m"
    )
    print(
        "Final adaptive rho_hat: "
        f"{float(adaptive['rho_hat'][-1]):.4f} "
        f"(true rho: {float(adaptive['rho_true'][-1]):.4f})"
    )


if __name__ == "__main__":
    main()
