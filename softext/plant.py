"""The simulated plant: joint angles q -> arm, tendons, and the equilibrium shape of the soft extension."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .arm import Arm
from .rod import Rod


@dataclass
class State:
    q: np.ndarray               # joint angles
    frames: list                # DH frames 0..6; frames[6] = T_B
    anchors: np.ndarray         # a_i, (m, 3)
    entries: np.ndarray         # b_i, (m, 3)
    spans: np.ndarray           # exterior spans l^o_i (2)
    tensions: np.ndarray        # tau_i
    active: np.ndarray          # taut and unsaturated tendons
    z: np.ndarray               # shape coordinates, (S, 3)
    local: np.ndarray           # centerline in {B}, (S * steps + 1, 3)
    centerline: np.ndarray      # centerline in the world
    R_D: np.ndarray             # orientation of {D}
    body_index: np.ndarray      # centerline samples of the body points p_j

    @property
    def R_B(self) -> np.ndarray:
        return self.frames[6][:3, :3]

    @property
    def p_B(self) -> np.ndarray:
        return self.frames[6][:3, 3]

    @property
    def p_D(self) -> np.ndarray:
        return self.centerline[-1]

    @property
    def body_points(self) -> np.ndarray:
        """p_j, the first at the base of the soft extension and the last at the tip."""
        return self.centerline[self.body_index]


class Plant:
    """UR10 + soft extension at quasi-static equilibrium. Warm-starts from the last shape."""

    def __init__(self, config, calibration_iterations: int = 8, initial_excess: float = 0.055):
        self.config = config
        self.arm = Arm(config)
        self.tendons = config.tendons
        self.rod = Rod(config)
        samples = self.rod.S * self.rod.steps + 1
        self.body_index = np.unique(np.linspace(0, samples - 1, config.sensing["body_points"]).astype(int))
        self.body_arc = self.body_index / (samples - 1) * self.rod.L
        # Rest lengths: each tendon carries the home tension at q0 (fixed-point iteration,
        # from an excess length of 55 mm beyond the rod, as in the simulator of the paper).
        tension = config.raw["home_tension"]
        self.rod.rest = np.full(self.rod.m, self.rod.L + initial_excess)
        for _ in range(calibration_iterations):
            spans = self.spans(config.q0)
            z, _ = self.rod.solve(spans)
            excess = spans + self.rod.interior_lengths(z) - self.rod.L - tension / self.rod.k
            self.rod.rest = self.rod.L + np.maximum(excess, 0.0)
        self._z = None

    # -- tendon geometry (known analytically, Sec. III-C) ---------------------
    def endpoints(self, frames: list) -> tuple[np.ndarray, np.ndarray]:
        """Anchors a_i and entry points b_i = p_B + R_B {}^B b_i, (2)."""
        T_B = frames[6]
        anchors = np.array([self.arm.point_on_link(frames, t.anchor_link, t.anchor_fraction) for t in self.tendons])
        entries = np.array([T_B[:3, 3] + T_B[:3, :3] @ t.entry for t in self.tendons])
        return anchors, entries

    def spans(self, q: np.ndarray) -> np.ndarray:
        anchors, entries = self.endpoints(self.arm.frames(q))
        return np.linalg.norm(anchors - entries, axis=1)

    # -- evaluation -----------------------------------------------------------
    def evaluate(self, q: np.ndarray, warm: bool = True) -> State:
        q = np.asarray(q, float).copy()
        frames = self.arm.frames(q)
        anchors, entries = self.endpoints(frames)
        spans = np.linalg.norm(anchors - entries, axis=1)
        z, _ = self.rod.solve(spans, self._z if warm else None)
        self._z = z
        _, tensions, active = self.rod.stretch(z, spans)
        local, rotations = self.rod.integrate(z)
        T_B = frames[6]
        centerline = T_B[:3, 3] + local @ T_B[:3, :3].T
        return State(q, frames, anchors, entries, spans, tensions, active, z, local, centerline,
                     T_B[:3, :3] @ rotations[-1], self.body_index)

    def reset(self) -> None:
        """Forget the warm start."""
        self._z = None
