import numpy as np


class Simulation:
    """
    Fixed-step Euler integrator for the rocket system with two controllers.
    Stores full time histories for post-processing.
    """

    def __init__(self, system, bs_controller, pd_controller, params):
        self.system = system
        self.bs = bs_controller
        self.pd = pd_controller
        self.params = params

    def run(self):
        p = self.params
        t_arr = np.arange(0.0, p["t_end"] + p["dt"], p["dt"])
        N = len(t_arr)

        # Backstepping trajectory
        bs_state = np.zeros((N, 2))
        bs_state[0] = [p["z0"], p["zdot0"]]
        bs_T = np.zeros(N)
        bs_e = np.zeros(N)
        bs_v_e = np.zeros(N)
        bs_delta = np.zeros(N)
        bs_L1 = np.zeros(N)
        bs_dL1 = np.zeros(N)

        # PD trajectory
        pd_state = np.zeros((N, 2))
        pd_state[0] = [p["z0"], p["zdot0"]]
        pd_T = np.zeros(N)
        pd_e = np.zeros(N)
        pd_v_e = np.zeros(N)
        pd_L_PD = np.zeros(N)
        pd_dL_PD = np.zeros(N)

        for i, t in enumerate(t_arr):
            m = self.system.mass(t)
            mu = self.system.m0 / m
            zd, zd_dot, zd_ddot = self.system.reference(t, p["z_step"], p["t_step"])

            # Backstepping
            z_bs, zdot_bs = bs_state[i]
            T_bs, e_bs, ve_bs, delta_bs = self.bs.compute(z_bs, zdot_bs, zd, zd_dot, zd_ddot, m)
            bs_T[i] = T_bs
            bs_e[i] = e_bs
            bs_v_e[i] = ve_bs
            bs_delta[i] = delta_bs
            bs_L1[i] = self.bs.lyapunov(e_bs, delta_bs)
            bs_dL1[i] = self.bs.lyapunov_dot(e_bs, delta_bs)

            # PD
            z_pd, zdot_pd = pd_state[i]
            T_pd, e_pd, ve_pd = self.pd.compute(z_pd, zdot_pd, zd, zd_dot)
            pd_T[i] = T_pd
            pd_e[i] = e_pd
            pd_v_e[i] = ve_pd
            pd_L_PD[i] = self.pd.lyapunov_pd(e_pd, ve_pd)
            pd_dL_PD[i] = self.pd.lyapunov_pd_dot(e_pd, ve_pd, mu)

            if i < N - 1:
                dxdt_bs = self.system.dynamics(bs_state[i], T_bs, t)
                bs_state[i + 1] = bs_state[i] + p["dt"] * dxdt_bs

                dxdt_pd = self.system.dynamics(pd_state[i], T_pd, t)
                pd_state[i + 1] = pd_state[i] + p["dt"] * dxdt_pd

        zd_arr = np.array([self.system.reference(t, p["z_step"], p["t_step"])[0] for t in t_arr])

        return {
            "t": t_arr,
            "zd": zd_arr,
            "bs": {
                "z":     bs_state[:, 0],
                "zdot":  bs_state[:, 1],
                "T":     bs_T,
                "e":     bs_e,
                "v_e":   bs_v_e,
                "delta": bs_delta,
                "L1":    bs_L1,
                "dL1":   bs_dL1,
            },
            "pd": {
                "z":     pd_state[:, 0],
                "zdot":  pd_state[:, 1],
                "T":     pd_T,
                "e":     pd_e,
                "v_e":   pd_v_e,
                "L_PD":  pd_L_PD,
                "dL_PD": pd_dL_PD,
            },
        }
