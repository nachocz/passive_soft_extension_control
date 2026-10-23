"""Run a simulated scenario of Sec. VI and plot it.

    python examples/run_scenario.py S2                     # laparoscopic fulcrum
    python examples/run_scenario.py S1 --exact             # exact J_span instead of the identified one
    python examples/run_scenario.py S2 --no-obstacle-barrier
    python examples/run_scenario.py S2 --constraint aperture   # fulcrum at a finite aperture, barriers only

Writes outputs/<scenario>/summary.png and prints the outcome of each target, the
deviation from the EE constraint, and the smallest margin of each barrier.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from softext import Config, Simulation, identify, Plant, scenario  # noqa: E402
from softext.plotting import OUTPUTS, scene, style, time_plots  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario", nargs="?", default="S2", choices=["S1", "S2", "S3"])
    parser.add_argument("--config", default=None, help="parameter file (default: config.yaml)")
    parser.add_argument("--constraint", choices=["free", "position", "fulcrum", "aperture"], default=None,
                        help="another EE constraint than the scenario's")
    parser.add_argument("--exact", action="store_true", help="exact J_span of the plant instead of the identified model")
    parser.add_argument("--identify", action="store_true", help="identify a new span map instead of the paper's")
    parser.add_argument("--no-adapt", action="store_true", help="without the online (Broyden) correction")
    parser.add_argument("--no-barriers", action="store_true", help="nominal control law (15), (18), without the QP")
    parser.add_argument("--no-obstacle-barrier", action="store_true")
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    config = Config.load(args.config)
    span_map = identify(Plant(config))[0] if args.identify else None
    sim = Simulation(config, args.scenario, span_map, exact=args.exact, adapt=not args.no_adapt,
                     barriers=not args.no_barriers, obstacle_barrier=not args.no_obstacle_barrier,
                     constraint=args.constraint, seed=args.seed)
    logs = sim.run(600.0)
    s = scenario(args.scenario)
    print(f"{s.title}: {sim.time:.1f} s simulated")
    for e in logs["events"]:
        print(f"  T{e['target']}: {e['outcome']:8s} at {e['t']:6.1f} s, tip error {1e3 * e['error']:5.1f} mm")
    print(f"  EE deviation <= {1e3 * logs['ee_deviation'].max():.3f} mm")
    for name, h in logs["barriers"].items():
        print(f"  {name:16s} min h - eps = {1e3 * (np.nanmin(h) - logs['margins'][name]):7.2f} (mm or mrad)")
    for o in s.obstacle_objects():
        print(f"  clearance to {o.name}: {1e3 * min(o.clearance(p) for p in logs['body_points']):.1f} mm")

    style()
    fig = plt.figure(figsize=(7.0, 4.2))
    grid = fig.add_gridspec(3, 2, width_ratios=[1.1, 1], hspace=0.35, wspace=0.3)
    ax3 = fig.add_subplot(grid[:, 0], projection="3d")
    scene(ax3, logs, s)
    ax3.set_title(s.title)
    time_plots([fig.add_subplot(grid[k, 1]) for k in range(3)], logs, config.targets["tolerance"])
    folder = OUTPUTS / args.scenario
    folder.mkdir(parents=True, exist_ok=True)
    fig.savefig(folder / "summary.png")
    print("written:", folder / "summary.png")


if __name__ == "__main__":
    main()
