import yaml
from src import RocketSystem, BacksteppingController, PDController, Simulation, Visualizer


def load_params(path="configs/params.yaml"):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    return {
        # physics
        "m0":    cfg["physics"]["m0"],
        "mf":    cfg["physics"]["mf"],
        "beta":  cfg["physics"]["beta"],
        "g":     cfg["physics"]["g"],
        # simulation
        "t_end": cfg["simulation"]["t_end"],
        "dt":    cfg["simulation"]["dt"],
        "z0":    cfg["simulation"]["z0"],
        "zdot0": cfg["simulation"]["zdot0"],
        # reference
        "z_step": cfg["reference"]["z_step"],
        "t_step": cfg["reference"]["t_step"],
        # backstepping gains
        "k1": cfg["backstepping"]["k1"],
        "K":  cfg["backstepping"]["K"],
        # PD gains
        "kp": cfg["pd"]["kp"],
        "kd": cfg["pd"]["kd"],
    }


def main():
    p = load_params()

    system = RocketSystem(m0=p["m0"], mf=p["mf"], beta=p["beta"], g=p["g"])
    bs_ctrl = BacksteppingController(k1=p["k1"], K=p["K"], g=p["g"])
    pd_ctrl = PDController(kp=p["kp"], kd=p["kd"], g=p["g"], m0=p["m0"])

    sim = Simulation(system, bs_ctrl, pd_ctrl, p)
    print("Running simulation...")
    results = sim.run()

    viz = Visualizer(results)
    viz.all()


if __name__ == "__main__":
    main()
