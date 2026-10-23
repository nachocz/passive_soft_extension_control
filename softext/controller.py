"""The controller of Sec. IV: the null-space control law and the safety QP.

    qdot_c   = J_c^+ k_c e_c                                   (14)
    qdot_nom = qdot_c + N_c nu                                 (15)
    v*       = k_t e,   e = p_D* - p_D                         (16)
    A        = J_t N_c                                         (17)
    nu       = A^T f(A A^T) W v_res,  v_res = v* - J_t qdot_c  (18), (23)
               f(lambda) = 1 / (lambda + xi^2)  damped inverse (default)
               f(lambda) = 1                    Jacobian transpose

When barriers are defined, the command is the solution of the QP (19):

    min |J_t qdot - v*|^2 + w_c |J_c qdot - k_c e_c|^2 + mu^2 |qdot|^2
    s.t. grad h^T qdot >= -alpha_h (h - eps_h) for every barrier,  |qdot_j| <= qdot_max
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .arm import Arm
from .barriers import Fulcrum, Row, fulcrum_rows, obstacle_row, span_rows
from .constraints import EEConstraint, pinv, projector
from .model import Differential, differential
from .qp import solve_qp


def spectral_velocity(A: np.ndarray, residual: np.ndarray, xi: float, kind: str = "dls") -> np.ndarray:
    """A^T f(A A^T) residual, with f applied to the eigenvalues lambda_i of A A^T, (24)."""
    lam, U = np.linalg.eigh(A @ A.T)
    f = 1.0 / (lam + xi ** 2) if kind == "dls" else np.ones_like(lam)
    return A.T @ (U @ (f * (U.T @ residual)))


@dataclass
class Command:
    qdot: np.ndarray
    nominal: np.ndarray
    rows: list = field(default_factory=list)
    e_c: np.ndarray = None
    A: np.ndarray = None
    diff: Differential = None
    qp: bool = False


class Controller:
    """Tip regulation in the null space of an EE constraint.

    span_map: the identified model (10)-(11) and its span domain L.
    exact:    use the exact J_span of the plant for the tip instead (simulation only;
              the barriers keep the identified body points).
    """

    def __init__(self, plant, constraint: EEConstraint, span_map, obstacles=(), fulcrum: Fulcrum | None = None,
                 barriers: bool = True, obstacle_barrier: bool = True, span_barrier: bool = True, exact: bool = False,
                 spectral_filter: str = "dls", W: np.ndarray | None = None):
        self.plant, self.config = plant, plant.config
        self.constraint, self.span_map = constraint, span_map
        self.obstacles, self.fulcrum = list(obstacles), fulcrum
        self.barriers, self.obstacle_barrier, self.span_barrier, self.exact = barriers, obstacle_barrier, span_barrier, exact
        self.filter, self.W = spectral_filter, W
        c = self.config.control
        self.k_t, self.k_c, self.xi, self.mu, self.w_c, self.qdot_max = c["k_t"], c["k_c"], c["xi"], c["mu"], c["w_c"], c["qdot_max"]

    def rows(self, frames, J_B, spans, diff: Differential) -> list[Row]:
        """The barrier rows of the QP at this step."""
        cfg = self.config.barriers
        rows = span_rows(spans, diff.J_lq, self.span_map.lower, self.span_map.upper, cfg["span"]) if self.span_barrier else []
        if self.obstacle_barrier and self.obstacles:
            R_B, p_B = frames[6][:3, :3], frames[6][:3, 3]
            points = p_B + self.span_map.predict_shape(spans) @ R_B.T
            rows += [obstacle_row(o, points, diff.J_p, cfg["obstacle"]) for o in self.obstacles]
        if self.fulcrum is not None:
            rows += fulcrum_rows(self.fulcrum, frames, J_B, cfg)
        return rows

    def command(self, q: np.ndarray, tip: np.ndarray, reference: np.ndarray, z: np.ndarray | None = None) -> Command:
        frames = self.plant.arm.frames(q)
        anchors, entries = self.plant.endpoints(frames)
        spans = np.linalg.norm(anchors - entries, axis=1)
        diff = differential(self.plant, frames, tip, spans, None if self.exact else self.span_map, z)
        if self.exact and self.span_map.shape_coefficients is not None:
            diff.J_p = differential(self.plant, frames, tip, spans, self.span_map).J_p
        J_B, J_t = diff.J_B, diff.J_t
        e_c = self.constraint.error(frames)
        J_c = self.constraint.jacobian(frames, J_B)
        v_star = self.k_t * (reference - tip)

        qdot_c = pinv(J_c) @ (self.k_c * e_c)
        N_c = projector(J_c)
        A = J_t @ N_c
        v_res = v_star - J_t @ qdot_c
        W = np.eye(3) if self.W is None else self.W
        nominal = qdot_c + N_c @ spectral_velocity(A, W @ v_res, self.xi, self.filter)

        rows = self.rows(frames, J_B, spans, diff) if self.barriers else []
        if not rows:
            scale = max(1.0, float(np.max(np.abs(nominal))) / self.qdot_max)
            return Command(nominal / scale, nominal, rows, e_c, A, diff, False)

        H = J_t.T @ J_t + self.w_c * J_c.T @ J_c + self.mu ** 2 * np.eye(6)
        g = -(J_t.T @ v_star + self.w_c * self.k_c * J_c.T @ e_c)
        G = np.array([r.grad for r in rows])
        rhs = np.array([-r.alpha * (r.h - r.eps) for r in rows])
        A_in = np.vstack((G, np.eye(6), -np.eye(6)))
        bounds = np.full(12, -self.qdot_max)
        qdot = solve_qp(H, g, A_in, np.concatenate((rhs, bounds)))
        if qdot is None:            # a barrier already below its margin: do not decrease it further
            qdot = solve_qp(H, g, A_in, np.concatenate((np.minimum(rhs, 0.0), bounds)))
        return Command(np.clip(qdot, -self.qdot_max, self.qdot_max), nominal, rows, e_c, A, diff, True)
