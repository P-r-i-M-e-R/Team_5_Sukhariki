import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.animation import FuncAnimation, PillowWriter

# ---------------------------------------------------------------------------
# Rocket polygon: smooth ogive nose + slim body + swept fins
# ---------------------------------------------------------------------------
_N_NOSE = 14   # points per nose quarter-arc

def _build_rocket_template():
    r = 0.11   # body radius
    t = np.linspace(0, np.pi / 2, _N_NOSE)

    # Right nose: quarter ellipse from tip (0, 1) to shoulder (r, 0.52)
    nose_rx = r * np.sin(t)
    nose_ry = 1.0 - 0.48 * (1.0 - np.cos(t))

    # Right straight body + fin
    right = np.array([
        [r,     0.52],
        [r,    -0.38],   # fin root top
        [0.30, -0.60],   # fin tip (swept)
        [r,    -0.50],   # fin root bottom
        [0.08, -0.57],   # nozzle edge R
    ])

    bottom = np.array([[0.0, -0.61]])   # nozzle base centre

    left = np.array([
        [-0.08, -0.57],
        [-r,    -0.50],
        [-0.30, -0.60],
        [-r,    -0.38],
        [-r,     0.52],
    ])

    nose_lx = -nose_rx[::-1]
    nose_ly =  nose_ry[::-1]

    return np.vstack([
        np.column_stack([nose_rx, nose_ry]),
        right,
        bottom,
        left,
        np.column_stack([nose_lx, nose_ly]),
    ])

_ROCKET_TEMPLATE = _build_rocket_template()


def _rocket_verts(cx, cy, scale):
    r = _ROCKET_TEMPLATE * scale
    r[:, 0] += cx
    r[:, 1] += cy
    return r


def _flame_verts(cx, cy, scale, thrust, t_nom=52.0):
    """Teardrop flame; length proportional to thrust."""
    fl = float(np.clip(thrust / t_nom, 0.0, 1.8))
    n = 10
    th = np.linspace(0, np.pi, n)
    # half-width bell: 0 at edges, max 0.08 at centre
    fx = 0.08 * np.sin(th)
    # y: base at nozzle bottom (-0.61), tip at -(0.61 + fl*0.45)
    tip = -0.61 - fl * 0.45
    fy  = -0.61 + (tip - (-0.61)) * (1 - np.cos(th)) / 2
    v = np.column_stack([fx, fy]) * scale
    v[:, 0] += cx
    v[:, 1] += cy
    return v


# ---------------------------------------------------------------------------
class Visualizer:
    def __init__(self, results, fig_dir="figures", anim_dir="animations"):
        self.r = results
        self.fig_dir = fig_dir
        self.anim_dir = anim_dir
        os.makedirs(fig_dir, exist_ok=True)
        os.makedirs(anim_dir, exist_ok=True)

    def _save(self, fig, name):
        path = os.path.join(self.fig_dir, name)
        fig.savefig(path, dpi=150, bbox_inches="tight")
        plt.close(fig)
        print(f"  saved {path}")

    # ------------------------------------------------------------------
    # System diagram (no simulation data needed)
    # ------------------------------------------------------------------
    def system_diagram(self):
        fig, ax = plt.subplots(figsize=(3.8, 5.8))
        ax.set_xlim(-1.8, 1.8)
        ax.set_ylim(-0.6, 5.6)
        ax.set_aspect("equal")
        ax.axis("off")

        # Ground hatching
        ax.axhline(0, color="#555", lw=1.5, zorder=1)
        ax.fill_between([-1.8, 1.8], [-0.6, -0.6], [0, 0],
                        color="#d4c8a0", alpha=0.5, zorder=0)
        for xi in np.linspace(-1.5, 1.5, 9):
            ax.plot([xi, xi - 0.15], [0, -0.20], color="#888", lw=0.8, zorder=1)

        # z-axis (arrow only, no tick labels)
        ax.annotate("", xy=(0, 5.3), xytext=(0, 0.05),
                    arrowprops=dict(arrowstyle="-|>", color="#333", lw=1.8))
        ax.text(0.10, 5.35, r"$z$", fontsize=15, va="center", fontweight="bold")

        # Rocket at z = 2.6
        scale = 1.25
        cy = 2.6
        body = MplPolygon(_rocket_verts(0, cy, scale), closed=True,
                          facecolor="#ddeeff", edgecolor="#1a5fa8",
                          linewidth=2.0, zorder=4)
        ax.add_patch(body)

        # Thrust arrow — red, upward
        t_base = cy + scale * (1.0 + 0.06)
        ax.annotate("", xy=(0, t_base + 0.85), xytext=(0, t_base + 0.08),
                    arrowprops=dict(arrowstyle="-|>", color="#c0392b",
                                   lw=2.8, mutation_scale=20), zorder=5)
        ax.text(0.28, t_base + 0.50, r"$T$",
                fontsize=15, color="#c0392b", va="center", fontweight="bold")

        # Gravity arrow — blue, downward
        g_base = cy - scale * (0.61 + 0.06)
        ax.annotate("", xy=(0, g_base - 0.85), xytext=(0, g_base - 0.08),
                    arrowprops=dict(arrowstyle="-|>", color="#1a5fa8",
                                   lw=2.8, mutation_scale=20), zorder=5)
        ax.text(0.28, g_base - 0.50, r"$m(t)\,g$",
                fontsize=13, color="#1a5fa8", va="center", fontweight="bold")

        self._save(fig, "system_diagram.png")

    # ------------------------------------------------------------------
    # Altitude tracking
    # ------------------------------------------------------------------
    def altitude_tracking(self):
        r = self.r
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        for ax, key, label, color in zip(
            axes,
            ["bs", "pd"],
            ["Backstepping", "PD baseline"],
            ["tab:blue", "tab:orange"],
        ):
            ax.plot(r["t"], r["zd"], "k--", lw=1.5, label=r"Reference $z_d$")
            ax.plot(r["t"], r[key]["z"], color=color, lw=1.8, label=label)
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Altitude $z$ (m)")
            ax.set_title(f"Altitude Tracking — {label}")
            ax.legend()
            ax.grid(True, alpha=0.3)
        fig.suptitle(
            "Figure 2: Altitude $z(t)$ vs. reference $z_d(t)$.\n"
            "Backstepping maintains zero steady-state error; PD accumulates a growing offset as $m(t)$ decreases.",
            fontsize=11,
        )
        fig.tight_layout()
        self._save(fig, "altitude_tracking.png")

    # ------------------------------------------------------------------
    # Tracking error
    # ------------------------------------------------------------------
    def tracking_error(self):
        r = self.r
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(r["t"], r["bs"]["e"], color="tab:blue", lw=1.8,
                label="Backstepping $e(t)$")
        ax.plot(r["t"], r["pd"]["e"], color="tab:orange", lw=1.8,
                label="PD baseline $e(t)$")
        ax.axhline(0, color="k", lw=0.8, ls="--")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Altitude error $e$ (m)")
        ax.set_title("Tracking Error $e(t) = z(t) - z_d(t)$")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.suptitle(
            "Tracking error $e(t)$: backstepping converges to zero; "
            "PD converges to a nonzero steady-state offset that grows as fuel burns.",
            fontsize=10,
        )
        fig.tight_layout()
        self._save(fig, "tracking_error.png")

    # ------------------------------------------------------------------
    # Lyapunov functions and their derivatives (2 × 2)
    # ------------------------------------------------------------------
    def lyapunov(self):
        r = self.r
        t = r["t"]
        fig, axes = plt.subplots(2, 2, figsize=(14, 8))

        axes[0, 0].plot(t, r["bs"]["L1"], color="tab:blue", lw=1.8)
        axes[0, 0].set_title(
            r"Backstepping $L_1 = \frac{1}{2}e^2 + \frac{1}{2}\delta^2$")
        axes[0, 0].set_xlabel("Time (s)")
        axes[0, 0].set_ylabel(r"$L_1$ (m$^2$)")
        axes[0, 0].grid(True, alpha=0.3)

        axes[0, 1].plot(t, r["bs"]["dL1"], color="tab:blue", lw=1.8)
        axes[0, 1].axhline(0, color="k", lw=0.8, ls="--")
        axes[0, 1].set_title(
            r"Backstepping $\dot{L}_1 = -k_1 e^2 - K\delta^2 \leq 0$  [eq. 20]")
        axes[0, 1].set_xlabel("Time (s)")
        axes[0, 1].set_ylabel(r"$\dot{L}_1$ (m$^2$/s)")
        axes[0, 1].grid(True, alpha=0.3)

        axes[1, 0].plot(t, r["pd"]["L_PD"], color="tab:orange", lw=1.8)
        axes[1, 0].set_title(
            r"PD $L^{PD} = \frac{1}{2}v_e^2 + \frac{1}{2}k_p e^2$")
        axes[1, 0].set_xlabel("Time (s)")
        axes[1, 0].set_ylabel(r"$L^{PD}$ (m$^2$)")
        axes[1, 0].grid(True, alpha=0.3)

        axes[1, 1].plot(t, r["pd"]["dL_PD"], color="tab:orange", lw=1.8)
        axes[1, 1].axhline(0, color="k", lw=0.8, ls="--")
        axes[1, 1].set_title(
            r"PD $\dot{L}^{PD}$ under $m(t) \neq m_0$  [eq. 30]")
        axes[1, 1].set_xlabel("Time (s)")
        axes[1, 1].set_ylabel(r"$\dot{L}^{PD}$ (m$^2$/s)")
        axes[1, 1].grid(True, alpha=0.3)

        fig.suptitle(
            "Figure 3: Lyapunov functions and time derivatives.\n"
            r"$\dot{L}_1 \leq 0$ everywhere (backstepping asymptotically stable). "
            r"$\dot{L}^{PD}$ turns positive as $m(t)$ drops, explaining the PD steady-state drift.",
            fontsize=11,
        )
        fig.tight_layout()
        self._save(fig, "lyapunov.png")

    # ------------------------------------------------------------------
    # Thrust
    # ------------------------------------------------------------------
    def thrust(self):
        r = self.r
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.plot(r["t"], r["bs"]["T"], color="tab:blue", lw=1.8,
                label="Backstepping")
        ax.plot(r["t"], r["pd"]["T"], color="tab:orange", lw=1.8,
                label="PD baseline")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Thrust $T$ (N)")
        ax.set_title("Commanded Thrust $T(t)$")
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.suptitle(
            "Figure 4: Thrust $T(t)$. "
            "Backstepping reduces thrust as $m(t)$ falls (lighter rocket needs less force); "
            "PD keeps approximately constant thrust.",
            fontsize=10,
        )
        fig.tight_layout()
        self._save(fig, "thrust.png")

    # ------------------------------------------------------------------
    # Phase portrait — both trajectories on one axes, no colourbar
    # ------------------------------------------------------------------
    def phase_portrait(self):
        r = self.r
        fig, ax = plt.subplots(figsize=(8, 6))

        e_bs  = r["bs"]["e"]
        d_bs  = r["bs"]["delta"]
        e_pd  = r["pd"]["e"]
        ve_pd = r["pd"]["v_e"]

        ax.plot(e_bs,  d_bs,  color="tab:blue",   lw=2.0,
                label=r"Backstepping $(e,\,\delta)$")
        ax.plot(e_pd,  ve_pd, color="tab:orange",  lw=2.0,
                label=r"PD baseline $(e,\,v_e)$")

        # Start markers
        ax.plot(e_bs[0],  d_bs[0],  "o", ms=10, color="tab:blue",   zorder=6)
        ax.plot(e_pd[0],  ve_pd[0], "o", ms=10, color="tab:orange", zorder=6)

        # Finish markers
        ax.plot(e_bs[-1],  d_bs[-1],  "*", ms=14, color="tab:blue",   zorder=6)
        ax.plot(e_pd[-1],  ve_pd[-1], "*", ms=14, color="tab:orange", zorder=6)

        ax.axhline(0, color="k", lw=0.8, ls="--")
        ax.axvline(0, color="k", lw=0.8, ls="--")
        ax.set_xlabel(r"Altitude error $e$ (m)", fontsize=12)
        ax.set_ylabel(
            r"Backstepping error $\delta$ / velocity error $v_e$  (m/s)",
            fontsize=11,
        )
        ax.set_title("Phase Portrait", fontsize=13)

        extra = [
            Line2D([0], [0], marker="o", color="gray", ms=9,  ls="", label="Start"),
            Line2D([0], [0], marker="*", color="gray", ms=11, ls="", label="Finish"),
        ]
        handles, labels = ax.get_legend_handles_labels()
        ax.legend(handles=handles + extra, fontsize=10)
        ax.grid(True, alpha=0.3)

        fig.suptitle(
            "Figure 5: Phase portraits.\n"
            r"Backstepping ($\circ\to\star$) spirals into $(0,0)$; "
            "PD converges to off-origin equilibrium $e^* > 0$.",
            fontsize=11,
        )
        fig.tight_layout()
        self._save(fig, "phase_portrait.png")

    # ------------------------------------------------------------------
    # Animation — realistic rocket silhouette
    # ------------------------------------------------------------------
    def animation(self):
        r    = self.r
        t    = r["t"]
        z_bs = r["bs"]["z"]
        z_pd = r["pd"]["z"]
        bs_T = r["bs"]["T"]
        pd_T = r["pd"]["T"]
        zd   = r["zd"]
        e_bs = r["bs"]["e"]
        e_pd = r["pd"]["e"]

        step = max(1, int(len(t) / (25 * (t[-1] - t[0]))))
        idx  = np.arange(0, len(t), step)

        z_hi = max(float(z_bs.max()), float(z_pd.max()), float(zd.max())) + 1.5
        z_lo = min(float(z_bs.min()), float(z_pd.min()), -0.3)
        scene_h = z_hi - z_lo
        scale   = scene_h * 0.09

        fig, ax = plt.subplots(figsize=(9, 8))
        ax.set_xlim(-1.5, 5.5)
        ax.set_ylim(z_lo - 0.3, z_hi + 0.6)
        ax.set_xticks([])
        ax.set_ylabel("Altitude  $z$  (m)", fontsize=12)
        ax.set_title("Backstepping (blue)  vs  PD baseline (orange)", fontsize=12)

        # Ground
        ax.axhline(0, color="#555", lw=1.5, zorder=1)
        ax.fill_between([-1.5, 5.5], [z_lo - 0.3] * 2, [0, 0],
                        color="#d4c8a0", alpha=0.4, zorder=0)

        # Reference dashed line
        ref_line = ax.axhline(float(zd[0]), color="#333", ls="--",
                              lw=1.5, label="Reference $z_d$", zorder=2)

        # Column headings
        ax.text(1.0, z_hi + 0.35, "Backstepping", ha="center", fontsize=11,
                color="tab:blue", fontweight="bold")
        ax.text(3.5, z_hi + 0.35, "PD baseline",  ha="center", fontsize=11,
                color="#c0580a", fontweight="bold")

        # Rocket body patches
        patch_bs = MplPolygon(
            _rocket_verts(1.0, float(z_bs[0]), scale),
            closed=True, facecolor="#c8e4ff", edgecolor="#1a5fa8",
            lw=2.0, zorder=5,
        )
        patch_pd = MplPolygon(
            _rocket_verts(3.5, float(z_pd[0]), scale),
            closed=True, facecolor="#ffe5c8", edgecolor="#c0580a",
            lw=2.0, zorder=5,
        )
        ax.add_patch(patch_bs)
        ax.add_patch(patch_pd)

        # Flame patches
        flame_bs = MplPolygon(
            _flame_verts(1.0, float(z_bs[0]), scale, float(bs_T[0])),
            closed=True, facecolor="darkorange", edgecolor="gold",
            lw=0.5, alpha=0.9, zorder=4,
        )
        flame_pd = MplPolygon(
            _flame_verts(3.5, float(z_pd[0]), scale, float(pd_T[0])),
            closed=True, facecolor="darkorange", edgecolor="gold",
            lw=0.5, alpha=0.9, zorder=4,
        )
        ax.add_patch(flame_bs)
        ax.add_patch(flame_pd)

        # Altitude trails
        (trail_bs,) = ax.plot([], [], color="tab:blue",   lw=0.9,
                               alpha=0.45, zorder=3)
        (trail_pd,) = ax.plot([], [], color="tab:orange",  lw=0.9,
                               alpha=0.45, zorder=3)

        ax.legend(loc="lower right", fontsize=9)

        # Info text box
        info = ax.text(
            0.97, 0.97, "",
            transform=ax.transAxes, ha="right", va="top",
            fontsize=9.5, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                      alpha=0.88, edgecolor="#aaa"),
        )

        def _update(frame):
            i  = idx[frame]
            zb = float(z_bs[i])
            zp = float(z_pd[i])
            zr = float(zd[i])

            ref_line.set_ydata([zr, zr])
            patch_bs.set_xy(_rocket_verts(1.0, zb, scale))
            patch_pd.set_xy(_rocket_verts(3.5, zp, scale))
            flame_bs.set_xy(_flame_verts(1.0, zb, scale, float(bs_T[i])))
            flame_pd.set_xy(_flame_verts(3.5, zp, scale, float(pd_T[i])))
            trail_bs.set_data(np.full(i + 1, 1.0), z_bs[: i + 1])
            trail_pd.set_data(np.full(i + 1, 3.5), z_pd[: i + 1])

            info.set_text(
                f" t = {t[i]:.2f} s\n"
                f" Reference:    {zr:.1f} m\n"
                f" ─────────────────────\n"
                f" Backstepping  z = {zb:.2f} m\n"
                f"   error  e = {e_bs[i]:+.3f} m\n"
                f" ─────────────────────\n"
                f" PD baseline   z = {zp:.2f} m\n"
                f"   error  e = {e_pd[i]:+.3f} m"
            )
            return (patch_bs, patch_pd, flame_bs, flame_pd,
                    ref_line, trail_bs, trail_pd, info)

        anim = FuncAnimation(fig, _update, frames=len(idx),
                             interval=40, blit=True)
        path = os.path.join(self.anim_dir, "rocket_flight.gif")
        anim.save(path, writer=PillowWriter(fps=25))
        plt.close(fig)
        print(f"  saved {path}")

    # ------------------------------------------------------------------
    def all(self):
        print("Generating system diagram...")
        self.system_diagram()
        print("Generating figures...")
        self.altitude_tracking()
        self.tracking_error()
        self.lyapunov()
        self.thrust()
        self.phase_portrait()
        print("Generating animation...")
        self.animation()
        print("Done.")
