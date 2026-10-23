"""Barrier functions h(q) >= eps_h (Sec. IV-C, Table III) and their gradients.

    obstacle clearance  h_obs  = soft-min over the segments of the polyline through the body points
                                 of dist(segment, O), minus a constant offset (Sec. VI-A)
    aperture crossing   h_ap   = (r_ap - r) cos(theta) - d_c
    tool-axis tilt      h_cone = theta_bar - theta
    insertion depth     h_ins  = delta - delta_min  and  delta_max - delta   (two rows)
    span domain         h_span = l^o_i - l^-_i  and  l^+_i - l^o_i           (two rows per tendon)

The body points and their Jacobians come from the identified model (11); the
fulcrum barriers and the span bounds use the arm kinematics alone.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Row:
    name: str
    kind: str          # obstacle, aperture, cone, insertion, span
    h: float
    grad: np.ndarray   # d h / d q
    alpha: float
    eps: float


class Obstacle:
    """A sphere (size: radius) or an axis-aligned box (size: edge lengths)."""

    def __init__(self, name: str, kind: str, center, size):
        self.name, self.kind = name, kind
        self.center = np.asarray(center, float)
        self.size = np.atleast_1d(np.asarray(size, float))
        if kind == "box" and self.size.size == 1:
            self.size = np.repeat(self.size, 3)

    def distance(self, p: np.ndarray) -> tuple[float, np.ndarray]:
        """Signed distance from p and its gradient (unit vector from the obstacle toward p)."""
        offset = p - self.center
        if self.kind == "sphere":
            d = float(np.linalg.norm(offset))
            if d < 1e-12:
                return -float(self.size[0]), np.array([1.0, 0.0, 0.0])
            return d - float(self.size[0]), offset / d
        sign = np.where(offset >= 0.0, 1.0, -1.0)
        delta = np.abs(offset) - 0.5 * self.size
        outside = np.maximum(delta, 0.0)
        length = float(np.linalg.norm(outside))
        if length > 1e-12:
            return length, sign * outside / length
        axis = int(np.argmax(delta))
        grad = np.zeros(3)
        grad[axis] = sign[axis]
        return float(delta[axis]), grad

    def closest_fraction(self, a: np.ndarray, b: np.ndarray) -> float:
        """Where the segment a-b is closest to the obstacle (the distance is convex along it)."""
        edge = b - a
        length = float(edge @ edge)
        if length < 1e-18:
            return 0.0
        if self.kind == "sphere":
            return float(np.clip((self.center - a) @ edge / length, 0.0, 1.0))
        lo, hi = 0.0, 1.0
        golden = 0.5 * (np.sqrt(5.0) - 1.0)
        x1, x2 = hi - golden * (hi - lo), lo + golden * (hi - lo)
        f1, f2 = self.distance(a + x1 * edge)[0], self.distance(a + x2 * edge)[0]
        for _ in range(60):
            if f1 <= f2:
                hi, x2, f2 = x2, x1, f1
                x1 = hi - golden * (hi - lo)
                f1 = self.distance(a + x1 * edge)[0]
            else:
                lo, x1, f1 = x1, x2, f2
                x2 = lo + golden * (hi - lo)
                f2 = self.distance(a + x2 * edge)[0]
        best = 0.5 * (lo + hi)
        for end in (0.0, 1.0):
            if self.distance(a + end * edge)[0] < self.distance(a + best * edge)[0]:
                best = end
        return best

    def clearance(self, points: np.ndarray) -> float:
        """Distance from the polyline through `points` (negative inside)."""
        return min(self.distance(a + self.closest_fraction(a, b) * (b - a))[0] for a, b in zip(points[:-1], points[1:]))


def obstacle_row(obstacle: Obstacle, points: np.ndarray, jacobians: np.ndarray, cfg: dict) -> Row:
    smoothing = cfg["smoothing"]
    distances, directions, fractions = [], [], []
    for a, b in zip(points[:-1], points[1:]):
        f = obstacle.closest_fraction(a, b)
        gap, n = obstacle.distance(a + f * (b - a))
        distances.append(gap - cfg["offset"])
        directions.append(n)
        fractions.append(f)
    distances = np.array(distances)
    floor = float(distances.min())
    weights = np.exp(-(distances - floor) / smoothing)
    h = floor - smoothing * float(np.log(weights.sum()))
    share = weights / weights.sum()
    grad = np.zeros(jacobians.shape[2])
    for k, (n, f) in enumerate(zip(directions, fractions)):
        grad += share[k] * (n @ ((1.0 - f) * jacobians[k] + f * jacobians[k + 1]))
    return Row(obstacle.name, "obstacle", h, grad, cfg["alpha"], cfg["eps"])


def span_rows(spans: np.ndarray, J_lq: np.ndarray, lower: np.ndarray, upper: np.ndarray, cfg: dict) -> list[Row]:
    rows = []
    for i in range(len(spans)):
        rows.append(Row(f"span {i + 1} lower", "span", float(spans[i] - lower[i]), J_lq[i].copy(), cfg["alpha"], cfg["eps"]))
        rows.append(Row(f"span {i + 1} upper", "span", float(upper[i] - spans[i]), -J_lq[i], cfg["alpha"], cfg["eps"]))
    return rows


@dataclass
class Fulcrum:
    """The fulcrum geometry: point c, aperture axis (unit, pointing into the aperture), and radii."""
    c: np.ndarray
    axis: np.ndarray = None          # direction of insertion; default -z (the tool points down)
    aperture_radius: float = 0.0     # r_ap; 0 for the ideal fulcrum (no aperture barrier)
    rod_radius: float = 0.009        # r

    def __post_init__(self):
        self.c = np.asarray(self.c, float)
        self.axis = np.array([0.0, 0.0, -1.0]) if self.axis is None else np.asarray(self.axis, float)


def fulcrum_rows(fulcrum: Fulcrum, frames: list, J_B: np.ndarray, cfg: dict) -> list[Row]:
    """Cone, insertion, and (for a finite aperture) aperture rows, with theta the tilt of the
    tool axis u from the aperture axis, delta = u^T (c - p_B), and d_c the distance from c to u."""
    T_B = frames[6]
    u, w = T_B[:3, 2], fulcrum.c - T_B[:3, 3]
    Jv, Jw = J_B[:3], J_B[3:]
    du = np.cross(Jw.T, u).T                                               # d u / d q_j = omega_j x u
    cos_t = float(np.clip(u @ fulcrum.axis, -1.0, 1.0))
    theta = float(np.arccos(cos_t))
    dcos = fulcrum.axis @ du                                               # d cos(theta) / d q
    sin_t = np.sqrt(max(1.0 - cos_t ** 2, 0.0))
    dtheta = -dcos / sin_t if sin_t > 1e-9 else np.zeros(6)
    delta = float(u @ w)
    ddelta = np.cross(u, w) @ Jw - u @ Jv
    rows = []
    cone, ins = cfg["cone"], cfg["insertion"]
    rows.append(Row("cone", "cone", np.deg2rad(cone["half_angle_deg"]) - theta, -dtheta, cone["alpha"], np.deg2rad(cone["eps_deg"])))
    rows.append(Row("insertion min", "insertion", delta - ins["min"], ddelta, ins["alpha"], ins["eps"]))
    rows.append(Row("insertion max", "insertion", ins["max"] - delta, -ddelta, ins["alpha"], ins["eps"]))
    if fulcrum.aperture_radius > 0.0:
        perp = w - delta * u                                               # from the tool axis to c
        d_c = float(np.linalg.norm(perp))
        dperp = -Jv - np.outer(u, ddelta) - delta * du
        dd_c = perp @ dperp / d_c if d_c > 1e-12 else np.zeros(6)
        h = (fulcrum.aperture_radius - fulcrum.rod_radius) * cos_t - d_c
        grad = (fulcrum.aperture_radius - fulcrum.rod_radius) * dcos - dd_c
        rows.append(Row("aperture", "aperture", h, grad, cfg["aperture"]["alpha"], cfg["aperture"]["eps"]))
    return rows
