"""Write docs/data/model.json, the data of the interactive pages.

It holds the span map identified by the paper's simulator (assets/span_map.json),
the scenarios S1-S3, and two small previews for the overview page: the tip path
of S2 and the Lyapunov function of the nominal run of Fig. 7. Then run
tools/build_web.py, which writes the data into the pages.

    python examples/export_web_data.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from softext import SCENARIOS, Config, EEConstraint, Plant, Simulation  # noqa: E402
from softext.analysis import nominal_run  # noqa: E402
from softext.scenarios import as_dict  # noqa: E402


def rounded(x, digits=6):
    return np.round(np.asarray(x, float), digits).tolist()


def main() -> None:
    config = Config.load()
    span_map = json.loads((ROOT / "assets" / "span_map.json").read_text(encoding="utf-8"))
    keep = ("degree", "centre", "scale", "coefficients", "shape_points", "shape_coefficients", "shape_arc_m",
            "span_min_m", "span_max_m")
    s2 = Simulation(config, "S2")
    logs = s2.run(120)
    every = 10                                   # 5 Hz
    path = logs["tip"][::every]
    ends = np.r_[np.flatnonzero(np.diff(logs["target"])), len(logs["target"]) - 1]
    run = nominal_run(Plant(config), SCENARIOS["S2"].targets, config.q0, EEConstraint("fulcrum", c=SCENARIOS["S2"].fulcrum))
    rows = run["rows"][::5]
    data = {
        "span_map": {k: span_map[k] for k in keep},
        "scenarios": {k: as_dict(s) for k, s in SCENARIOS.items()},
        "preview": {
            "s2_path": rounded(path, 5),
            "s2_shapes": [rounded(logs["body_points"][i], 5) for i in ends],
            "nominal_t": rounded([r["t"] for r in rows], 3),
            "nominal_V": [float(f"{r['V']:.4e}") for r in rows],
            "nominal_beta": rounded([r["beta"] for r in rows], 4),
            "k_t": config.control["k_t"],
        },
    }
    out = ROOT / "docs" / "data" / "model.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(data, separators=(",", ":")), encoding="utf-8")
    print(f"written: {out.relative_to(ROOT)} ({out.stat().st_size / 1024:.0f} kB)")


if __name__ == "__main__":
    main()
