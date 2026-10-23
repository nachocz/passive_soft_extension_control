"""Fig. 7: convergence of the nominal controller (25) on the simulated plant (Sec. V-B).

From q0, the tip is sent to the five targets of S2 in turn (6 s each) under the
fulcrum, with W = I, no barriers, and no rate limits, once with the exact tip
Jacobian of the plant (Assumption 1) and once with the identified model (10).
Top: tip path and the soft extension at each target switch. Middle: V on a log
axis with the bound (31) restarted at each switch (smallest beta along the run).
Bottom: the descent rate -dV/dt / V and the band [2 k_t beta, 2 k_t].

    python figures/fig7_stability.py          # about 20 s
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from softext import Config, EEConstraint, Plant, paper_span_map, scenario  # noqa: E402
from softext.analysis import nominal_run  # noqa: E402
from softext.plotting import FULCRUM, INK, MUTED, PATH, TARGET, save, style  # noqa: E402

EXACT, IDENTIFIED = "#1f4e8c", "#0d9488"
FLOOR = 1e-9            # below this, V is at the numerical floor of the integration


def series(run):
    rows = run["rows"]
    t = np.array([r["t"] for r in rows])
    v = np.array([r["V"] for r in rows])
    beta = np.array([r["beta"] for r in rows])
    hold = int(round(run["hold"] / run["dt"]))
    rate = np.full(len(v), np.nan)
    rate[:-1] = -np.diff(np.log(np.maximum(v, 1e-30))) / np.diff(t)
    switch = np.array([(k + 1) % hold == 0 for k in range(len(v))])
    rate[switch | (v < FLOOR) | (np.r_[v[1:], v[-1]] < FLOOR)] = np.nan
    return t, v, beta, rate, hold


def main() -> None:
    style()
    config = Config.load()
    plant = Plant(config)
    s2 = scenario("S2")
    constraint = EEConstraint("fulcrum", c=s2.fulcrum)
    c = config.control
    kw = dict(k_t=c["k_t"], k_c=c["k_c"], xi=c["xi"])
    exact = nominal_run(plant, s2.targets, config.q0, constraint, None, **kw)
    identified = nominal_run(plant, s2.targets, config.q0, constraint, paper_span_map(config), **kw)
    t, v, beta, rate, hold = series(exact)
    _, v_id, _, rate_id, _ = series(identified)
    k_t, n = c["k_t"], len(s2.targets)

    fig = plt.figure(figsize=(3.5, 5.0))
    ax3 = fig.add_axes([0.0, 0.52, 1.0, 0.48], projection="3d")
    tips = np.array([r["tip"] for r in exact["rows"]])
    for k, shape in enumerate(exact["shapes"]):
        ax3.plot(*shape.T, color=INK, lw=1.6 if k in (0, n) else 0.7, alpha=1.0 if k in (0, n) else 0.45)
    ax3.plot(*tips.T, color=PATH, lw=1.2)
    for k, p in enumerate(s2.targets):
        ax3.scatter(*p, s=12, color=TARGET, depthshade=False)
        ax3.text(p[0], p[1], p[2] + 0.01, str(k + 1), fontsize=7)
    ax3.scatter(*s2.fulcrum, color=FULCRUM, marker="x", s=20, depthshade=False)
    ax3.view_init(elev=18, azim=-70)
    ax3.set_box_aspect((1, 1, 1.4))
    for setter in (ax3.set_xticklabels, ax3.set_yticklabels, ax3.set_zticklabels):
        setter([])

    ax_v = fig.add_axes([0.16, 0.27, 0.8, 0.2])
    beta_min = float(beta.min())
    for k in range(n):
        sl = slice(k * hold, (k + 1) * hold)
        bound = v[k * hold] * np.exp(-2 * k_t * beta_min * (t[sl] - t[k * hold]))
        ax_v.plot(t[sl], bound, color=INK, lw=0.8, ls=(0, (1, 1.2)), label="bound (31)" if k == 0 else None)
        ax_v.plot(t[sl], v[sl], color=EXACT, lw=1.1, label="exact $J_D$" if k == 0 else None)
        ax_v.plot(t[sl], v_id[sl], color=IDENTIFIED, lw=0.9, ls=(0, (3, 1.6)),
                  label=r"identified $\widehat J_D$" if k == 0 else None)
    ax_v.set_yscale("log")
    ax_v.set_ylim(1e-11, 1e-2)
    ax_v.set_ylabel(r"$V$ [m$^2$]")
    ax_v.tick_params(labelbottom=False)
    ax_v.legend(loc="upper right", frameon=False, ncol=3, fontsize=6, bbox_to_anchor=(1.0, 1.18))

    ax_r = fig.add_axes([0.16, 0.06, 0.8, 0.18])
    ax_r.fill_between(t, 2 * k_t * beta, 2 * k_t, color=EXACT, alpha=0.12, lw=0, step="mid")
    ax_r.plot(t, rate, color=EXACT, lw=1.1)
    ax_r.plot(t, rate_id, color=IDENTIFIED, lw=0.9, ls=(0, (3, 1.6)))
    ax_r.set_ylim(2.0, 3.1)
    ax_r.set_ylabel(r"$-\dot V/V$ [s$^{-1}$]")
    ax_r.set_xlabel("time [s]")
    for ax in (ax_v, ax_r):
        ax.set_xlim(0, t[-1])
        for k in range(1, n):
            ax.axvline(k * exact["hold"], color=MUTED, lw=0.5, ls=(0, (1, 1.5)))
    print(f"beta {beta.min():.3f}-{beta.max():.3f}; exact rate {np.nanmin(rate):.2f}-{np.nanmax(rate):.2f}; "
          f"identified rate {np.nanmin(rate_id):.2f}-{np.nanmax(rate_id):.2f}; "
          f"band [{2 * k_t * beta_min:.2f}, {2 * k_t:.2f}]; V increases in "
          f"{int(np.sum(np.diff(v)[np.isfinite(rate[:-1])] > 0))} steps (exact), "
          f"{int(np.sum(np.diff(v_id)[np.isfinite(rate_id[:-1])] > 0))} (identified)")
    print("written:", save(fig, "fig7_stability"))


if __name__ == "__main__":
    main()
