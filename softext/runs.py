"""Closed-loop runs of the scenarios, saved in outputs/runs/<key>.npz so that the figure scripts reuse them."""
from __future__ import annotations

import json

import numpy as np

from .plotting import OUTPUTS
from .simulation import Simulation


def save_run(logs: dict, key: str) -> None:
    folder = OUTPUTS / "runs"
    folder.mkdir(parents=True, exist_ok=True)
    arrays = {k: v for k, v in logs.items() if isinstance(v, np.ndarray)}
    arrays.update({f"barrier/{k}": v for k, v in logs["barriers"].items()})
    np.savez_compressed(folder / f"{key}.npz", **arrays,
                        meta=json.dumps({"events": logs["events"], "margins": logs["margins"]}))


def load_run(key: str) -> dict | None:
    path = OUTPUTS / "runs" / f"{key}.npz"
    if not path.exists():
        return None
    data = np.load(path)
    logs = {k: data[k] for k in data.files if k != "meta" and not k.startswith("barrier/")}
    logs["barriers"] = {k.split("/", 1)[1]: data[k] for k in data.files if k.startswith("barrier/")}
    logs.update(json.loads(str(data["meta"])))
    return logs


def load_or_run(config, key: str, **kwargs) -> dict:
    """The saved run of a scenario, or a new one (saved when it uses the default settings)."""
    logs = None if kwargs else load_run(key)
    if logs is None:
        logs = Simulation(config, key, **kwargs).run(600.0)
        if not kwargs:
            save_run(logs, key)
    return logs
