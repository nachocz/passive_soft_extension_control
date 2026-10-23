"""Plots of runs in the colours of the paper figures."""
from __future__ import annotations

from pathlib import Path

import numpy as np

TENDON = ("#0d9488", "#d4a017")       # tendon 1 teal, tendon 2 gold
PATH, TARGET, OBSTACLE, FULCRUM, INK, MUTED = "#1f4e8c", "#111111", "#d6402f", "#7b3fa0", "#1f1d1b", "#716b65"
JOINTS = ("#4c72b0", "#dd8452", "#55a868", "#c44e52", "#8172b3", "#937860")
OUTPUTS = Path(__file__).resolve().parent.parent / "outputs"


def style() -> None:
    import matplotlib as mpl
    mpl.rcParams.update({"font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8, "legend.fontsize": 7,
                         "xtick.labelsize": 7, "ytick.labelsize": 7, "axes.spines.top": False,
                         "axes.spines.right": False, "savefig.dpi": 300, "savefig.bbox": "tight"})


def save(fig, name: str) -> Path:
    folder = OUTPUTS / "figures"
    folder.mkdir(parents=True, exist_ok=True)
    for suffix in ("pdf", "png"):
        fig.savefig(folder / f"{name}.{suffix}")
    return folder / f"{name}.png"


def target_lines(ax, logs) -> None:
    t, target = logs["t"], logs["target"]
    for k in np.flatnonzero(np.diff(target)) + 1:
        ax.axvline(t[k], color=MUTED, lw=0.5, ls=(0, (1, 1.5)))


def draw_obstacle(ax, o, alpha: float = 0.18) -> None:
    c = np.asarray(o["center"], float)
    if o["kind"] == "sphere":
        u, v = np.mgrid[0:2 * np.pi:24j, 0:np.pi:12j]
        r = o["size"]
        ax.plot_surface(c[0] + r * np.cos(u) * np.sin(v), c[1] + r * np.sin(u) * np.sin(v), c[2] + r * np.cos(v),
                        color=OBSTACLE, alpha=alpha, linewidth=0)
    else:
        h = 0.5 * o["size"]
        for axis in range(3):
            for sign in (-1, 1):
                a, b = [i for i in range(3) if i != axis]
                g = np.mgrid[-1:1:2j, -1:1:2j]
                pts = np.zeros((3, 2, 2))
                pts[axis] = c[axis] + sign * h
                pts[a] = c[a] + h * g[0]
                pts[b] = c[b] + h * g[1]
                ax.plot_surface(*pts, color=OBSTACLE, alpha=alpha, linewidth=0)


def scene(ax, logs, scenario, ends=None) -> None:
    """3-D view: targets, obstacles, tip path, and the soft extension at the end of each target and of the run."""
    from .scenarios import as_dict
    s = as_dict(scenario)
    for o in s["obstacles"]:
        draw_obstacle(ax, o)
    tip = logs["tip"]
    ax.plot(*tip.T, color=PATH, lw=1.0)
    points = logs["body_points"]
    target = logs["target"]
    ends = np.r_[np.flatnonzero(np.diff(target)), len(target) - 1] if ends is None else ends
    for k, i in enumerate(ends):
        ax.plot(*points[i].T, color=INK, lw=2.2 if k == len(ends) - 1 else 0.7, alpha=1.0 if k == len(ends) - 1 else 0.5)
    for k, p in enumerate(s["targets"]):
        ax.scatter(*p, s=10, color=TARGET, depthshade=False)
        ax.text(p[0], p[1], p[2] + 0.008, f"T{k + 1}", fontsize=6, color=TARGET)
    if s["fulcrum"] and s["constraint"] == "fulcrum":
        ax.scatter(*s["fulcrum"], s=16, color=FULCRUM, marker="x", depthshade=False)
    every = np.vstack([tip, points.reshape(-1, 3)])
    lo, hi = every.min(axis=0), every.max(axis=0)
    centre, half = 0.5 * (lo + hi), 0.55 * float(np.max(hi - lo))
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(centre[2] - half, centre[2] + half)
    ax.set_xlabel("x [m]", labelpad=-8)
    ax.set_ylabel("y [m]", labelpad=-8)
    ax.set_zlabel("z [m]", labelpad=-8)
    ax.tick_params(pad=-3)
    ax.view_init(elev=22, azim=-60)


def time_plots(axes, logs, tolerance: float = 0.004) -> None:
    """Norm of the tip error, joint rates, and changes of the exterior spans."""
    from matplotlib.ticker import LogLocator, NullFormatter
    t = logs["t"]
    ax_e, ax_q, ax_s = axes
    ax_e.plot(t, 1e3 * logs["error"], color=PATH, lw=0.9)
    ax_e.axhline(1e3 * tolerance, color=MUTED, lw=0.6, ls="--")
    ax_e.set_yscale("log")
    ax_e.yaxis.set_major_locator(LogLocator(base=10.0))
    ax_e.yaxis.set_minor_formatter(NullFormatter())
    ax_e.set_ylabel("tip error [mm]")
    for j in range(6):
        ax_q.plot(t, np.rad2deg(logs["qdot"][:, j]), color=JOINTS[j], lw=0.7, label=f"$\\dot q_{j + 1}$")
    ax_q.set_ylabel("joint rates [deg/s]")
    spans = logs["spans"]
    for i in range(spans.shape[1]):
        ax_s.plot(t, 1e3 * (spans[:, i] - spans[0, i]), color=TENDON[i], lw=1.0, label=f"$\\Delta\\ell^o_{i + 1}$")
    ax_s.set_ylabel("span change [mm]")
    ax_s.set_xlabel("time [s]")
    ax_s.legend(loc="upper left", frameon=False, ncol=2)
    for ax in axes:
        target_lines(ax, logs)
        ax.set_xlim(t[0], t[-1])
