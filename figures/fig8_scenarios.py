"""Fig. 8: the three simulated scenarios (Sec. VI), rerun with the Python package.

Columns: (a) S1, position-constrained EE; (b) S2, laparoscopic fulcrum; (c) S3,
infeasible targets. Rows: 3-D view with the targets, the obstacles, the tip path,
and the soft extension at the end of each target (thin) and of the run (thick);
norm of the tip error with the 4 mm arrival tolerance; joint rates; changes of
the two exterior spans. The runs of the paper come from its ROS 2 simulator, with
the same plant, identified span map, controller, and noise level but another
noise realization and message timing, so the curves are close to Fig. 8, not equal.

    python figures/fig8_scenarios.py            # about 1.5 min; the runs are saved in outputs/runs/
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt  # noqa: E402

from softext import Config, scenario  # noqa: E402
from softext.plotting import save, scene, style, time_plots  # noqa: E402
from softext.runs import load_or_run  # noqa: E402


def main() -> None:
    style()
    config = Config.load()
    fig = plt.figure(figsize=(7.16, 7.4))
    grid = fig.add_gridspec(4, 3, height_ratios=[1.6, 1, 1, 1], hspace=0.35, wspace=0.42)
    for col, key in enumerate(("S1", "S2", "S3")):
        logs = load_or_run(config, key)
        ax3 = fig.add_subplot(grid[0, col], projection="3d")
        scene(ax3, logs, scenario(key))
        ax3.set_title(f"({'abc'[col]}) {scenario(key).title}", pad=0)
        axes = [fig.add_subplot(grid[row, col]) for row in (1, 2, 3)]
        time_plots(axes, logs)
        reached = sum(e["outcome"] == "reached" for e in logs["events"])
        print(f"{key}: {reached} of {len(scenario(key).targets)} targets reached; "
              f"EE deviation <= {1e3 * logs['ee_deviation'].max():.2f} mm; "
              + ", ".join(f"T{e['target']} {e['outcome']} ({1e3 * e['error']:.1f} mm)" for e in logs["events"]))
    print("written:", save(fig, "fig8_scenarios"))


if __name__ == "__main__":
    main()
