"""The analysis of Sec. V: deformation rank under an EE constraint, and descent of the nominal control law.

    J_def = J_Dtau C_tau J_lq N_c                                             (20)
    rank J_def <= min{rank J_Dtau, m_a, rank(J_lq N_c)}
               <= min{rank J_Dtau, m_a, rank J_lq, n - r_c}                    (22)
    on the constraint manifold:  qdot = N_c A^T f(A A^T) W (k_t e)            (25)
                                 V = e^T W e / 2,  Vdot <= 0                   (27)-(28)
                                 Vdot <= -2 k_t beta lambda_min(W) V           (31)
"""
from __future__ import annotations

import numpy as np

from .arm import Arm
from .constraints import EEConstraint, pinv, projector
from .model import differential, soft_response, span_jacobian


def rank(M: np.ndarray, tol: float = 1e-6) -> int:
    if M.size == 0:
        return 0
    s = np.linalg.svd(M, compute_uv=False)
    return int(np.sum(s > tol * max(1.0, s[0])))


def rank_report(plant, state, constraint: EEConstraint) -> dict:
    """The factors of the bound (22) at a configuration, with their singular values."""
    frames = state.frames
    J_B = Arm.geometric_jacobian(frames)
    J_lq = span_jacobian(plant, frames)
    N_c = projector(constraint.jacobian(frames, J_B))
    soft = soft_response(plant.rod, state.z, state.spans)
    J_def = soft.J_Dtau @ soft.C_tau @ J_lq @ N_c
    m_a = int(np.count_nonzero(np.diag(soft.K_a)))
    out = {
        "rank_J_Dtau": rank(soft.J_Dtau), "m_a": m_a, "rank_J_lq": rank(J_lq),
        "n_minus_r_c": 6 - constraint.rank, "rank_J_lq_N_c": rank(J_lq @ N_c), "rank_J_def": rank(J_def),
        "sv_J_lq": np.linalg.svd(J_lq, compute_uv=False), "sv_J_lq_N_c": np.linalg.svd(J_lq @ N_c, compute_uv=False),
        "sv_J_def": np.linalg.svd(J_def, compute_uv=False), "J_lq": J_lq, "J_def": J_def,
    }
    out["bound_first"] = min(out["rank_J_Dtau"], m_a, out["rank_J_lq_N_c"])
    out["bound_second"] = min(out["rank_J_Dtau"], m_a, out["rank_J_lq"], out["n_minus_r_c"])
    return out


def spectral_filter(lam: np.ndarray, xi: float, kind: str = "dls") -> np.ndarray:
    """f(lambda): damped inverse 1 / (lambda + xi^2), or 1 for the Jacobian transpose."""
    return 1.0 / (lam + xi ** 2) if kind == "dls" else np.ones_like(lam)


def descent(A: np.ndarray, e: np.ndarray, k_t: float, xi: float, kind: str = "dls", W: np.ndarray | None = None) -> dict:
    """V, Vdot (28), beta, and the rates rho_i = k_t lambda_i f(lambda_i) (32) for the error dynamics (26)."""
    W = np.eye(len(e)) if W is None else W
    lam, U = np.linalg.eigh(A @ A.T)
    lam = np.maximum(lam, 0.0)
    f = spectral_filter(lam, xi, kind)
    We = W @ e
    return {"V": 0.5 * float(e @ W @ e), "Vdot": -k_t * float((U.T @ We) ** 2 @ (lam * f)),
            "beta": float(np.min(lam * f)), "rates": k_t * lam * f, "lambda": lam, "grad": float(np.linalg.norm(A.T @ We))}


def nominal_run(plant, targets, q0, constraint: EEConstraint, span_map=None, k_t: float = 1.4, k_c: float = 3.0,
                xi: float = 0.04, dt: float = 0.02, hold: float = 6.0, kind: str = "dls") -> dict:
    """The nominal control law (15), (18)/(23) without barriers or rate limits, to fixed targets in turn (Fig. 7).

    With span_map=None the controller uses the exact Jacobian of the plant (Assumption 1);
    otherwise the identified model (10). The quantities of Proposition 2 are always computed
    from the exact A. W = I.
    """
    q = np.array(q0, float)
    plant.reset()
    state = plant.evaluate(q)
    rows, shapes = [], []
    t = 0.0
    for index, goal in enumerate(np.asarray(targets, float)):
        shapes.append(state.centerline.copy())
        for _ in range(int(round(hold / dt))):
            state = plant.evaluate(q)
            frames = state.frames
            tip = state.p_D
            exact = differential(plant, frames, tip, state.spans, None, state.z)
            model = exact if span_map is None else differential(plant, frames, tip, state.spans, span_map)
            e = goal - tip
            e_c, J_c = constraint.error(frames), constraint.jacobian(frames, exact.J_B)
            N_c = projector(J_c)
            qdot_c = pinv(J_c) @ (k_c * e_c)
            A = model.J_t @ N_c
            lam, U = np.linalg.eigh(A @ A.T)
            nu = A.T @ (U @ (spectral_filter(lam, xi, kind) * (U.T @ (k_t * e - model.J_t @ qdot_c))))
            qdot = qdot_c + N_c @ nu
            d = descent(exact.J_t @ N_c, e, k_t, xi, kind)
            rows.append({"t": t, "target": index, "V": d["V"], "grad": d["grad"], "beta": d["beta"],
                         "e_c": constraint.deviation(frames), "tip": tip.copy(), "qdot": float(np.max(np.abs(qdot)))})
            q = q + dt * qdot
            t += dt
    shapes.append(plant.evaluate(q).centerline.copy())
    return {"rows": rows, "shapes": shapes, "q_end": q, "k_t": k_t, "xi": xi, "dt": dt, "hold": hold}
