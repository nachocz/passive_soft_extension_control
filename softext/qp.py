"""Strictly convex QP with inequality constraints, solved exactly through NNLS.

    minimize 1/2 x^T H x + g^T x   subject to   A x >= b

With H = L L^T, the QP is the least-distance program min ||y|| s.t. (A L^-T) y >=
b + A H^-1 g, which the nonnegative least-squares algorithm of Lawson and Hanson
solves in a finite number of steps (Solving Least Squares Problems, 1974, Ch. 23).
The same algorithm is in docs/js/sim/linalg.js.
"""
from __future__ import annotations

import numpy as np


def householder_lstsq(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Least-squares solution of A x = b (A with full column rank) by Householder QR."""
    R, y = np.array(A, float), np.array(b, float)
    m, n = R.shape
    for k in range(n):
        v = R[k:, k].copy()
        alpha = -np.copysign(np.linalg.norm(v), v[0] if v[0] != 0 else 1.0)
        v[0] -= alpha
        vv = float(v @ v)
        if vv < 1e-300:
            continue
        R[k:, k:] -= np.outer(2.0 * v / vv, v @ R[k:, k:])
        y[k:] -= 2.0 * v * (v @ y[k:]) / vv
    x = np.zeros(n)
    for k in range(n - 1, -1, -1):
        x[k] = (y[k] - R[k, k + 1:n] @ x[k + 1:]) / R[k, k]
    return x


def nnls(E: np.ndarray, f: np.ndarray, tol: float = 1e-12) -> np.ndarray:
    """min ||E u - f|| subject to u >= 0 (Lawson-Hanson)."""
    n = E.shape[1]
    u = np.zeros(n)
    passive = np.zeros(n, bool)
    for _ in range(3 * n + 10):
        w = E.T @ (f - E @ u)
        candidates = np.where(~passive & (w > tol))[0]
        if candidates.size == 0:
            break
        passive[candidates[np.argmax(w[candidates])]] = True
        while True:
            idx = np.where(passive)[0]
            z = np.zeros(n)
            z[idx] = householder_lstsq(E[:, idx], f)
            if np.all(z[idx] > tol):
                u = z
                break
            neg = idx[z[idx] <= tol]
            alpha = np.min(u[neg] / (u[neg] - z[neg]))
            u = u + alpha * (z - u)
            passive &= u > tol
            u[~passive] = 0.0
    return u


def ldp(G: np.ndarray, h: np.ndarray) -> np.ndarray | None:
    """min ||y|| subject to G y >= h; None if infeasible."""
    n = G.shape[1]
    E = np.vstack((G.T, h[None, :]))
    f = np.zeros(n + 1)
    f[n] = 1.0
    u = nnls(E, f)
    r = E @ u - f
    if np.linalg.norm(r) < 1e-10 or abs(r[n]) < 1e-12:
        return None
    return -r[:n] / r[n]


def solve_qp(H: np.ndarray, g: np.ndarray, A: np.ndarray, b: np.ndarray) -> np.ndarray | None:
    L = np.linalg.cholesky(H)
    x_free = -np.linalg.solve(H, g)
    if len(b) == 0:
        return x_free
    G = np.linalg.solve(L, A.T).T                 # A L^-T
    y = ldp(G, b - A @ x_free)
    if y is None:
        return None
    return x_free + np.linalg.solve(L.T, y)
