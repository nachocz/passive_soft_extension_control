"""The simulated scenarios of Sec. VI-B (Table V), with the targets and obstacles of the paper's runs.

All start at q0, where the tool axis passes through the fulcrum point c.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .barriers import Obstacle

FULCRUM = [-0.8437, -0.1639, 0.2283]


@dataclass
class Scenario:
    key: str
    title: str
    constraint: str                       # free, position, fulcrum
    targets: list
    obstacles: list = field(default_factory=list)
    fulcrum: list | None = None
    infeasible: tuple = ()                # targets infeasible by design (1-based)

    def obstacle_objects(self) -> list[Obstacle]:
        return [Obstacle(o["name"], o["kind"], o["center"], o["size"]) for o in self.obstacles]


def sphere(name, x, y, z, radius):
    return {"name": name, "kind": "sphere", "center": [x, y, z], "size": radius}


def box(name, x, y, z, edge):
    return {"name": name, "kind": "box", "center": [x, y, z], "size": edge}


SCENARIOS = {
    "S1": Scenario(
        "S1", "S1: position-constrained EE", "position",
        [[-0.907362, -0.061136, 0.072054], [-0.695992, -0.113576, 0.120694], [-0.883322, -0.100471, 0.057675],
         [-0.753462, -0.277474, 0.10188], [-0.728124, -0.111937, 0.09271], [-0.825047, -0.070726, 0.063259],
         [-0.708042, -0.10993, 0.1074], [-0.786902, -0.083479, 0.066897], [-0.732699, -0.129059, 0.092891]],
        [sphere("upper sphere", -0.805, -0.035, 0.155, 0.055), box("lower box", -0.7, -0.25, 0.25, 0.12),
         sphere("outer sphere", -1.02, -0.25, 0.25, 0.065), box("high box", -0.65, -0.04, 0.27, 0.1)]),
    "S2": Scenario(
        "S2", "S2: laparoscopic fulcrum", "fulcrum",
        [[-0.753197, -0.099656, 0.044469], [-0.713412, -0.123692, 0.080384], [-0.771788, -0.092032, 0.070998],
         [-0.727814, -0.12106, 0.050137], [-0.695012, -0.095911, 0.088171]],
        [sphere("polyp", -0.7392, -0.0963, 0.0824, 0.016)], FULCRUM),
    "S3": Scenario(
        "S3", "S3: infeasible targets", "fulcrum",
        [[-0.713289, -0.123741, 0.080453], [-0.772029, -0.092165, 0.070856], [-0.745157, -0.070506, 0.005626],
         [-0.743763, -0.117458, 0.049452], [-0.736013, -0.06184, -0.015036], [-0.713289, -0.123741, 0.080453]],
        [sphere("blocked target", -0.745157, -0.070506, 0.005626, 0.03),
         sphere("rigid-tool gate", -0.89003, -0.20384, 0.31706, 0.03),
         sphere("slalom near", -0.74269, -0.03197, 0.00632, 0.02),
         sphere("slalom far", -0.70724, -0.06508, 0.01266, 0.02)], FULCRUM, infeasible=(3, 5)),
}


def scenario(key: str) -> Scenario:
    return SCENARIOS[key]


def as_dict(s: Scenario) -> dict:
    return {"key": s.key, "title": s.title, "constraint": s.constraint, "targets": np.round(s.targets, 6).tolist(),
            "obstacles": s.obstacles, "fulcrum": s.fulcrum, "infeasible": list(s.infeasible)}
