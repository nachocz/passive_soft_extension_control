"""The differential model of Sec. III: span kinematics, soft response, and tip velocity.

    J_lq          (3)-(4)   joint rates -> span rates, from the arm and the tendon geometry
    C_tau, J_span (5)-(7)   span changes -> tensions -> twist of {D} relative to {B}
    J_transport   (8)       motion of {D} at fixed shape
    J_D           (9)-(10)  J_transport + diag(R_B, R_B) J_span J_lq
    J_{p_j}       (11)      body points, from the span sensitivities of the identified model
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .arm import Arm, skew


def span_jacobian(plant, frames: list) -> np.ndarray:
    """J_lq, (4): row i is d_i^T (J_{a_i}^v - J_B^v + [R_B {}^B b_i]_x J_B^omega)."""
    J_B = Arm.geometric_jacobian(frames)
    R_B = frames[6][:3, :3]
    anchors, entries = plant.endpoints(frames)
    rows = []
    for t, a, b in zip(plant.tendons, anchors, entries):
        d = (a - b) / np.linalg.norm(a - b)
        J_a = Arm.point_jacobian(frames, t.anchor_link, t.anchor_fraction)
        rows.append(d @ (J_a - J_B[:3] + skew(R_B @ t.entry) @ J_B[3:]))
    return np.array(rows)


def transport_jacobian(J_B: np.ndarray, p_B: np.ndarray, p_D: np.ndarray) -> np.ndarray:
    """J_transport, (8): the Jacobian of {D} rigidly attached to {B} at its current position."""
    return np.vstack((J_B[:3] - skew(p_D - p_B) @ J_B[3:], J_B[3:]))


def distal_jacobian(J_transport: np.ndarray, R_B: np.ndarray, J_span: np.ndarray, J_lq: np.ndarray) -> np.ndarray:
    """J_D = J_transport + diag(R_B, R_B) J_span J_lq, (9); with an identified J_span, (10)."""
    R = np.zeros((6, 6))
    R[:3, :3] = R[3:, 3:] = R_B
    return J_transport + R @ J_span @ J_lq


@dataclass
class SoftResponse:
    """The factors of Sec. III-D at an equilibrium of the simulated rod."""
    J_lz: np.ndarray      # interior tendon lengths per shape change, m x 3S
    J_ztau: np.ndarray    # shape change per tension change (fixed active set), 3S x m
    J_Dtau: np.ndarray    # relative distal twist per tension change, 6 x m
    K_a: np.ndarray       # diag of the active stiffnesses
    C_tau: np.ndarray     # span -> tension, (6)
    J_span: np.ndarray    # J_Dtau C_tau, (7)


def soft_response(rod, z: np.ndarray, spans: np.ndarray) -> SoftResponse:
    """Exact soft factors of the plant (simulation only: the controller identifies J_span instead)."""
    _, tau, active = rod.stretch(z, spans)
    J_lz = rod.interior_gradient(z)
    J_ztau = -np.linalg.solve(rod.load_hessian(z, tau), J_lz.T)
    J_Dtau = rod.tip_shape_jacobian(z) @ J_ztau
    K_a = np.diag(np.where(active, rod.k, 0.0))
    C_tau = np.linalg.solve(np.eye(rod.m) - K_a @ J_lz @ J_ztau, K_a)
    return SoftResponse(J_lz, J_ztau, J_Dtau, K_a, C_tau, J_Dtau @ C_tau)


@dataclass
class Differential:
    """Everything the controller needs at one step."""
    J_B: np.ndarray
    J_lq: np.ndarray
    J_transport: np.ndarray
    J_span: np.ndarray        # 6 x m (only the translational rows are used for the tip)
    J_D: np.ndarray
    J_t: np.ndarray           # S J_D, the tip-position Jacobian
    J_p: np.ndarray | None    # body-point Jacobians (11), (k, 3, n)


def differential(plant, frames: list, tip: np.ndarray, spans: np.ndarray, span_model=None, z=None) -> Differential:
    """The model of the controller at the measured tip.

    With `span_model` (a SpanMap) this is the identified model (10) used by the
    controller; without it, the exact J_span of the plant at shape z, (9).
    """
    J_B = Arm.geometric_jacobian(frames)
    R_B, p_B = frames[6][:3, :3], frames[6][:3, 3]
    J_lq = span_jacobian(plant, frames)
    J_tr = transport_jacobian(J_B, p_B, tip)
    J_p = None
    if span_model is None:
        J_span = soft_response(plant.rod, z, spans).J_span
    else:
        J_span = np.vstack((span_model.tip_jacobian(spans), np.zeros((3, len(spans)))))
        if span_model.shape_coefficients is not None:
            local = span_model.predict_shape(spans)                       # {}^B p_j
            sens = span_model.shape_jacobian(spans)                       # d {}^B p_j / d l^o
            J_p = np.array([J_B[:3] - skew(R_B @ lp) @ J_B[3:] + R_B @ s @ J_lq for lp, s in zip(local, sens)])
    J_D = distal_jacobian(J_tr, R_B, J_span, J_lq)
    return Differential(J_B, J_lq, J_tr, J_span, J_D, J_D[:3], J_p)
