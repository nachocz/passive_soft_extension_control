"""The identified model (Sec. III-F): probing motions and the span map.

With gravity neglected, the shape of the soft extension in {B} is a function of
the exterior spans alone. Probing motions move the arm to configurations whose
spans cover a grid around the spans at q0; at each one the camera measures the
tip and the body points, which are expressed in {B}. A polynomial in the spans
fitted to these samples predicts {}^B p_D and {}^B p_j, and its derivative is
the identified J_span used in (10) and (11). Online, a rank-one secant (Broyden)
correction adapts the tip rows of J_span to the measured motion. The identified
span domain L = [l^-, l^+] is the box covered by the probes.
"""
from __future__ import annotations

import json
from itertools import product
from pathlib import Path

import numpy as np

from .rng import Random


class SpanMap:
    def __init__(self, degree: int = 4):
        self.degree = degree
        self.centre = self.scale = None
        self.coefficients = None          # (terms, 3): {}^B p_D
        self.shape_coefficients = None    # (terms, 3k): {}^B p_j
        self.shape_points = 0
        self.lower = self.upper = None    # identified span domain
        self.residual = None              # (3, m) online correction of the tip rows
        self.gain, self.decay, self.min_step = 0.4, 0.995, 2e-4

    def exponents(self, m: int) -> list[tuple[int, ...]]:
        found = [p for p in product(range(self.degree + 1), repeat=m) if sum(p) <= self.degree]
        return sorted(found, key=lambda p: (sum(p), p))

    def features(self, spans: np.ndarray) -> np.ndarray:
        x = (np.asarray(spans, float) - self.centre) / self.scale
        return np.array([np.prod(x ** np.array(p)) for p in self.exponents(len(x))])

    def feature_gradient(self, spans: np.ndarray) -> np.ndarray:
        """d(features)/d(spans), (terms, m)."""
        x = (np.asarray(spans, float) - self.centre) / self.scale
        rows = []
        for p in self.exponents(len(x)):
            row = np.zeros(len(x))
            for i, power in enumerate(p):
                if power:
                    others = np.prod([x[j] ** p[j] for j in range(len(x)) if j != i])
                    row[i] = power * x[i] ** (power - 1) * others / self.scale[i]
            rows.append(row)
        return np.array(rows)

    # -- use --------------------------------------------------------------------
    def predict_tip(self, spans) -> np.ndarray:
        return self.features(spans) @ self.coefficients

    def tip_jacobian(self, spans) -> np.ndarray:
        """Identified J_span (tip rows), 3 x m, with the online correction."""
        return self.coefficients.T @ self.feature_gradient(spans) + self.residual

    def predict_shape(self, spans) -> np.ndarray:
        return (self.features(spans) @ self.shape_coefficients).reshape(self.shape_points, 3)

    def shape_jacobian(self, spans) -> np.ndarray:
        """d {}^B p_j / d l^o, (k, 3, m)."""
        return (self.shape_coefficients.T @ self.feature_gradient(spans)).reshape(self.shape_points, 3, -1)

    def update(self, spans, previous_spans, step_tip_local) -> None:
        """Broyden: correct the tip rows of J_span toward the measured motion in {B}."""
        step = np.asarray(spans, float) - np.asarray(previous_spans, float)
        if np.linalg.norm(step) < self.min_step:
            self.residual = self.decay * self.residual
            return
        predicted = self.tip_jacobian(spans) @ step
        self.residual = self.residual + self.gain * np.outer(step_tip_local - predicted, step) / float(step @ step)

    def reset(self) -> None:
        self.residual = np.zeros((3, len(self.centre)))

    # -- fit --------------------------------------------------------------------
    def fit(self, spans: np.ndarray, tips: np.ndarray, shapes: np.ndarray | None = None,
            regularization: float = 1e-6) -> dict:
        spans = np.asarray(spans, float)
        self.centre = spans.mean(axis=0)
        self.scale = np.maximum(spans.std(axis=0), 1e-6)
        self.lower, self.upper = spans.min(axis=0), spans.max(axis=0)
        F = np.array([self.features(s) for s in spans])
        gram = F.T @ F + regularization * np.eye(F.shape[1])
        self.coefficients = np.linalg.solve(gram, F.T @ tips)
        if shapes is not None:
            self.shape_points = shapes.shape[1]
            self.shape_coefficients = np.linalg.solve(gram, F.T @ shapes.reshape(len(spans), -1))
        self.reset()
        error = F @ self.coefficients - tips
        return {"samples": len(spans), "rms_tip": float(np.sqrt(np.mean(np.sum(error ** 2, axis=1))))}

    def to_dict(self) -> dict:
        return {"degree": self.degree, "centre": self.centre.tolist(), "scale": self.scale.tolist(),
                "coefficients": self.coefficients.tolist(),
                "shape_coefficients": None if self.shape_coefficients is None else self.shape_coefficients.tolist(),
                "shape_points": self.shape_points, "span_min_m": self.lower.tolist(), "span_max_m": self.upper.tolist()}

    @classmethod
    def from_dict(cls, data: dict, config=None) -> "SpanMap":
        """From to_dict(), or from the span maps of the paper's simulator (assets/span_map.json)."""
        out = cls(int(data["degree"]))
        out.centre, out.scale = np.array(data["centre"], float), np.array(data["scale"], float)
        out.coefficients = np.array(data["coefficients"], float)[:, :3]      # tip position (the file also has its rotation)
        if data.get("shape_coefficients") is not None:
            out.shape_coefficients = np.array(data["shape_coefficients"], float)
            out.shape_points = int(data["shape_points"])
        out.lower, out.upper = np.array(data["span_min_m"], float), np.array(data["span_max_m"], float)
        if config is not None:
            ident = config.identification
            out.gain, out.decay, out.min_step = ident["broyden_gain"], ident["broyden_decay"], ident["broyden_min_step"]
        out.reset()
        return out

    @classmethod
    def load(cls, path: str | Path, config=None) -> "SpanMap":
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")), config)


PAPER_MAP = Path(__file__).resolve().parent.parent / "assets" / "span_map.json"


def paper_span_map(config=None) -> SpanMap:
    """The span map identified by the paper's simulator, which its simulations used."""
    return SpanMap.load(PAPER_MAP, config)


# -- probing ----------------------------------------------------------------------
def joints_for_spans(plant, spans_wanted: np.ndarray, seed: np.ndarray, iterations: int = 60,
                     tolerance: float = 5e-5) -> np.ndarray | None:
    """Minimum-norm Gauss-Newton steps on J_lq from `seed`; None if the spans are not reached."""
    from .model import span_jacobian
    limits = plant.arm.limits
    q = np.array(seed, float)
    for _ in range(iterations):
        error = spans_wanted - plant.spans(q)
        if np.linalg.norm(error) < 1e-9:
            break
        J = span_jacobian(plant, plant.arm.frames(q))
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T, error), limits[:, 0], limits[:, 1])
    return q if np.linalg.norm(spans_wanted - plant.spans(q)) < tolerance else None


def probe(plant, grid: int | None = None, span_range: float | None = None, noise: float | None = None,
          seed: int | None = None) -> dict:
    """The probing motions: a grid x grid box of spans around q0, measured with camera noise."""
    ident, sensing = plant.config.identification, plant.config.sensing
    grid = ident["grid"] if grid is None else grid
    span_range = ident["range"] if span_range is None else span_range
    noise = sensing["noise"] if noise is None else noise
    rng = Random(sensing["seed"] if seed is None else seed)
    q0 = plant.config.q0
    centre = plant.spans(q0)
    axes = [np.linspace(c - span_range, c + span_range, grid) for c in centre]
    wanted = np.stack([a.ravel() for a in np.meshgrid(*axes)], axis=1)
    spans, tips, shapes, joints, skipped = [], [], [], [], 0
    for target in wanted:
        q = joints_for_spans(plant, target, q0)
        if q is None:
            skipped += 1
            continue
        state = plant.evaluate(q, warm=False)
        points = state.body_points + noise * rng.normal_array(state.body_points.shape)
        tip = state.p_D + noise * rng.normal_array(3)
        R_B, p_B = state.R_B, state.p_B
        spans.append(state.spans)
        tips.append(R_B.T @ (tip - p_B))
        shapes.append((points - p_B) @ R_B)
        joints.append(q)
    return {"spans": np.array(spans), "tips": np.array(tips), "shapes": np.array(shapes),
            "joints": np.array(joints), "skipped": skipped}


def identify(plant, **kwargs) -> tuple[SpanMap, dict]:
    """Probe and fit; returns the span map and the probing data."""
    data = probe(plant, **kwargs)
    span_map = SpanMap(plant.config.identification["degree"])
    report = span_map.fit(data["spans"], data["tips"], data["shapes"])
    ident = plant.config.identification
    span_map.gain, span_map.decay, span_map.min_step = ident["broyden_gain"], ident["broyden_decay"], ident["broyden_min_step"]
    return span_map, {**data, **report}


def validation_error(span_map: SpanMap, plant, samples: int = 160, seed: int = 99) -> float:
    """RMS tip error of the map at random spans in L, reachable from q0, that the fit did not use."""
    rng = Random(seed)
    errors = []
    for _ in range(samples):
        spans = span_map.lower + (span_map.upper - span_map.lower) * np.array([rng.uniform() for _ in span_map.lower])
        if joints_for_spans(plant, spans, plant.config.q0) is None:
            continue
        z, _ = plant.rod.solve(spans)
        local, _ = plant.rod.integrate(z)
        errors.append(np.sum((span_map.predict_tip(spans) - local[-1]) ** 2))
    return float(np.sqrt(np.mean(errors)))
