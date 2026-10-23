"""Identify the span map from probing motions and score it at validation spans (Sec. III-F, VI-A).

    python examples/identify.py                    # +/-10 mm and +/-22 mm, as in Sec. VI-A
    python examples/identify.py --range 0.014 --grid 9 --noise 0.001

The probing motions move the arm to configurations whose spans cover a grid around
the spans at q0, and the camera measures the tip and the body points with noise.
The validation error is the RMS tip error of the map at random spans in the
identified span domain that are reachable from q0 and were not used in the fit
(1.5 mm and 5.5 mm in the paper). It also scores the map identified by the
paper's simulator (assets/span_map.json).
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from softext import Config, Plant, identify, paper_span_map  # noqa: E402
from softext.identification import validation_error  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--range", type=float, nargs="*", default=[0.010, 0.022], help="probing range(s) [m]")
    parser.add_argument("--grid", type=int, default=11)
    parser.add_argument("--noise", type=float, default=None, help="camera noise [m] (default: config.yaml)")
    parser.add_argument("--degree", type=int, default=None)
    args = parser.parse_args()
    config = Config.load()
    if args.degree:
        config = config.replace(identification={"degree": args.degree})
    plant = Plant(config)
    print(f"map of the paper's simulator: validation RMS {1e3 * validation_error(paper_span_map(config), plant):.2f} mm")
    for span_range in args.range:
        span_map, data = identify(plant, grid=args.grid, span_range=span_range, noise=args.noise)
        print(f"+/-{1e3 * span_range:.0f} mm, {args.grid} x {args.grid} grid: {len(data['spans'])} probes reached, "
              f"{data['skipped']} skipped; fit RMS {1e3 * data['rms_tip']:.2f} mm, "
              f"validation RMS {1e3 * validation_error(span_map, plant):.2f} mm")


if __name__ == "__main__":
    main()
