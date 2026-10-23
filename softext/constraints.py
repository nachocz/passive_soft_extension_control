"""EE constraints (Sec. IV-A, Table II): the error e_c(q) and J_c = -d e_c / d q.

    free EE                  no rows                                 r_c = 0
    position-constrained EE  e_c = p_B* - p_B,  J_c = J_B^v           r_c = 3
    laparoscopic fulcrum     e_c = (I - u u^T)(c - p_B)              r_c = 2

For the fulcrum, the two components of (I - u u^T)(c - p_B) along the x and y axes
of {B} (both perpendicular to the tool axis u, the z axis of {B}) are used, which
gives a J_c of full row rank r_c = 2.
"""
from __future__ import annotations

import numpy as np


class EEConstraint:
    def __init__(self, kind: str = "free", p_star=None, c=None):
        if kind not in ("free", "position", "fulcrum"):
            raise ValueError(f"unknown EE constraint {kind!r}")
        self.kind = kind
        self.p_star = None if p_star is None else np.asarray(p_star, float)
        self.c = None if c is None else np.asarray(c, float)

    @property
    def rank(self) -> int:
        return {"free": 0, "position": 3, "fulcrum": 2}[self.kind]

    def error(self, frames: list) -> np.ndarray:
        T_B = frames[6]
        if self.kind == "position":
            return self.p_star - T_B[:3, 3]
        if self.kind == "fulcrum":
            w = self.c - T_B[:3, 3]
            return np.array([T_B[:3, 0] @ w, T_B[:3, 1] @ w])
        return np.zeros(0)

    def jacobian(self, frames: list, J_B: np.ndarray) -> np.ndarray:
        """J_c, with d e_c / dt = -J_c qdot, (12)."""
        T_B = frames[6]
        if self.kind == "position":
            return J_B[:3].copy()
        if self.kind == "fulcrum":
            w = self.c - T_B[:3, 3]
            return np.array([T_B[:3, i] @ J_B[:3] - np.cross(T_B[:3, i], w) @ J_B[3:] for i in (0, 1)])
        return np.zeros((0, 6))

    def deviation(self, frames: list) -> float:
        """Geometric distance from the constraint: |p_B* - p_B|, or the distance from c to the tool axis."""
        return float(np.linalg.norm(self.error(frames))) if self.kind != "free" else 0.0


def pinv(J: np.ndarray) -> np.ndarray:
    """Moore-Penrose pseudoinverse of a full-row-rank J (no rows: 6 x 0)."""
    if J.shape[0] == 0:
        return np.zeros((J.shape[1], 0))
    return J.T @ np.linalg.inv(J @ J.T)


def projector(J_c: np.ndarray) -> np.ndarray:
    """N_c = I - J_c^+ J_c, (13)."""
    return np.eye(J_c.shape[1]) - pinv(J_c) @ J_c
