"""Proposition 1 at q0: the factors of the bound (22) on rank J_def for the three EE constraints,
for the anchors of the paper and for anchors moved onto the EE (Corollary 1), and with a slack tendon.

    python examples/rank_bound.py
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np  # noqa: E402

from softext import Config, EEConstraint, Plant, scenario  # noqa: E402
from softext.analysis import rank_report  # noqa: E402

DEG = np.pi / 180


def table(title: str, plant: Plant) -> None:
    state = plant.evaluate(plant.config.q0, warm=False)
    print(f"\n{title} (tensions {np.round(state.tensions, 2).tolist()} N)")
    print(f"  {'EE constraint':10s} rank J_Dtau  m_a  rank J_lq  n-r_c  rank(J_lq N_c)  bound  rank J_def  sigma(J_lq N_c) [mm/deg]")
    for c in (EEConstraint("free"), EEConstraint("position", p_star=state.p_B), EEConstraint("fulcrum", c=scenario("S2").fulcrum)):
        r = rank_report(plant, state, c)
        sv = ", ".join(f"{1e3 * DEG * s:.3f}" for s in r["sv_J_lq_N_c"])
        print(f"  {c.kind:10s} {r['rank_J_Dtau']:11d} {r['m_a']:4d} {r['rank_J_lq']:10d} {r['n_minus_r_c']:6d} "
              f"{r['rank_J_lq_N_c']:15d} {r['bound_first']:6d} {r['rank_J_def']:11d}  {sv}")


def main() -> None:
    config = Config.load()
    table("Anchors of the paper: tendon 1 on wrist 1 (link 4), tendon 2 on the forearm (link 3)", Plant(config))
    on_ee = [dict(t, anchor_link=6, anchor_fraction=0.5) for t in config.raw["tendons"]]
    table("Both anchors on the EE (Corollary 1: J_lq = 0)", Plant(config.replace(tendons=on_ee)))
    one = [dict(config.raw["tendons"][0], anchor_link=6, anchor_fraction=0.5), config.raw["tendons"][1]]
    table("Anchor 1 on the EE, anchor 2 on the forearm", Plant(config.replace(tendons=one)))
    slack = Plant(config)
    slack.rod.rest[1] += 0.01
    table("Tendon 2 slack (10 mm longer)", slack)


if __name__ == "__main__":
    main()
