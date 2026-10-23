"""Parameters of the simulated plant and of the controller (config.yaml)."""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent

DEFAULTS = {
    "arm": {
        "a": [0.0, -0.612, -0.5723, 0.0, 0.0, 0.0],
        "alpha_deg": [90.0, 0.0, 0.0, 90.0, -90.0, 0.0],
        "d": [0.1273, 0.0, 0.0, 0.163941, 0.1157, 0.0922],
        "joint_limits_deg": [[-180, 180], [-180, 10], [-160, 160], [-180, 180], [-180, 180], [-180, 180]],
        "q0_deg": [0.0, -70.0, 95.0, -115.0, -90.0, 0.0],
    },
    "rod": {"length": 0.320, "radius": 0.009, "youngs_modulus": 1.1e6, "shear_modulus": 4.0e5,
            "sections": 5, "steps_per_section": 5},
    "tendons": [
        {"name": "tendon 1", "angle_deg": 0.0, "routing_radius": 0.0055, "stiffness": 260.0,
         "max_tension": 14.0, "anchor_link": 4, "anchor_fraction": 0.75},
        {"name": "tendon 2", "angle_deg": 90.0, "routing_radius": 0.0055, "stiffness": 260.0,
         "max_tension": 14.0, "anchor_link": 3, "anchor_fraction": 0.60},
    ],
    "home_tension": 1.5,
    "sensing": {"plant_rate": 100.0, "rate": 20.0, "noise": 0.0005, "body_points": 5, "seed": 0},
    "identification": {"grid": 11, "range": 0.010, "degree": 4, "broyden_gain": 0.4,
                       "broyden_decay": 0.995, "broyden_min_step": 0.0002},
    "control": {"rate": 50.0, "k_t": 1.4, "k_c": 3.0, "xi": 0.04, "mu": 0.04, "w_c": 200.0,
                "qdot_max": 0.30, "reference_speed": 0.10},
    "targets": {"tolerance": 0.004, "hold": 8.0, "stall": 10.0, "stall_progress": 0.0001,
                "hysteresis": 0.0015, "budget": 90.0},
    "barriers": {
        "obstacle": {"alpha": 3.0, "eps": 0.008, "offset": 0.012, "smoothing": 0.004},
        "span": {"alpha": 4.0, "eps": 0.0005},
        "cone": {"alpha": 3.0, "eps_deg": 2.0, "half_angle_deg": 35.0},
        "insertion": {"alpha": 3.0, "eps": 0.005, "min": 0.05, "max": 0.26},
        "aperture": {"alpha": 3.0, "eps": 0.002},
    },
}


@dataclass(frozen=True)
class Tendon:
    name: str
    angle: float            # phi_i [rad]
    routing_radius: float   # r_i [m]
    stiffness: float        # k_a,i [N/m]
    max_tension: float      # [N]
    anchor_link: int        # DH link (1..6) that carries the anchor a_i
    anchor_fraction: float  # position of a_i along that link

    @property
    def entry(self) -> np.ndarray:
        """{}^B b_i, the entry point in {B}."""
        return self.routing_radius * np.array([np.cos(self.angle), np.sin(self.angle), 0.0])


@dataclass
class Config:
    """All parameters; `Config()` holds the values of config.yaml."""
    raw: dict = field(default_factory=lambda: copy.deepcopy(DEFAULTS))

    @classmethod
    def load(cls, path: str | Path | None = None) -> "Config":
        import yaml
        path = ROOT / "config.yaml" if path is None else Path(path)
        with open(path, encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        raw = copy.deepcopy(DEFAULTS)
        for key, value in data.items():
            if isinstance(value, dict) and isinstance(raw.get(key), dict):
                for k, v in value.items():
                    raw[key][k] = {**raw[key][k], **v} if isinstance(v, dict) and isinstance(raw[key].get(k), dict) else v
            else:
                raw[key] = value
        return cls(raw)

    # -- arm ----------------------------------------------------------------
    @property
    def dh(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        arm = self.raw["arm"]
        return np.array(arm["a"], float), np.deg2rad(arm["alpha_deg"]), np.array(arm["d"], float)

    @property
    def joint_limits(self) -> np.ndarray:
        return np.deg2rad(np.array(self.raw["arm"]["joint_limits_deg"], float))

    @property
    def q0(self) -> np.ndarray:
        return np.deg2rad(np.array(self.raw["arm"]["q0_deg"], float))

    # -- soft extension -------------------------------------------------------
    @property
    def rod(self) -> dict:
        return self.raw["rod"]

    @property
    def tendons(self) -> list[Tendon]:
        return [Tendon(t["name"], np.deg2rad(t["angle_deg"]), t["routing_radius"], t["stiffness"],
                       t["max_tension"], int(t["anchor_link"]), t["anchor_fraction"]) for t in self.raw["tendons"]]

    def section(self, name: str) -> dict:
        return self.raw[name]

    @property
    def sensing(self) -> dict:
        return self.raw["sensing"]

    @property
    def identification(self) -> dict:
        return self.raw["identification"]

    @property
    def control(self) -> dict:
        return self.raw["control"]

    @property
    def targets(self) -> dict:
        return self.raw["targets"]

    @property
    def barriers(self) -> dict:
        return self.raw["barriers"]

    def replace(self, **sections) -> "Config":
        """A copy with some entries changed, e.g. replace(control={"k_t": 1.0})."""
        raw = copy.deepcopy(self.raw)
        for key, value in sections.items():
            if isinstance(value, dict) and isinstance(raw.get(key), dict):
                raw[key].update(value)
            else:
                raw[key] = value
        return Config(raw)
