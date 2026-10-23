"""The simulated soft extension: a quasi-static Cosserat rod loaded by elastic tendons.

The rod is inextensible and unshearable, with piecewise-constant curvature: the
shape coordinates z are the curvature vectors u_k (two bending components and
torsion) of S sections. Tendon i enters at {}^B b_i and runs parallel to the
centerline at the offset r_i = {}^B b_i, so its interior length is

    l_in,i(z) = sum_k l_s || e_z + u_k x r_i ||,                  l_s = L / S.

Its tension depends on the total stretch e_i = l^o_i + l_in,i - l_rest,i: it is
zero for a slack tendon, k_a,i e_i while taut, and saturates at tau_max,i. The
equilibrium shape minimizes the elastic and tendon energy (gravity is neglected,
as in the simulations of the paper), so in {B} it is a function of the exterior
spans l^o alone. The energy is convex in z, and Newton's method finds its unique
minimum. The controller never uses this module: it only simulates the plant.
"""
from __future__ import annotations

import numpy as np

from .arm import skew

E_Z = np.array([0.0, 0.0, 1.0])


def rotation_log(R: np.ndarray) -> np.ndarray:
    """Rotation vector of a rotation matrix."""
    w = 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(w)
    c = 0.5 * (np.trace(R) - 1.0)
    angle = np.arctan2(s, c)
    return w if s < 1e-12 else w * (angle / s)


def section_increment(u: np.ndarray, ds: float) -> tuple[np.ndarray, np.ndarray]:
    """Exact pose increment over a length ds of constant curvature u (unit tangent e_z)."""
    k = np.linalg.norm(u)
    U = skew(u)
    if k < 1e-10:
        return np.eye(3) + U * ds, np.array([0.0, 0.0, ds])
    U2 = U @ U
    angle = k * ds
    R = np.eye(3) + np.sin(angle) / k * U + (1.0 - np.cos(angle)) / k ** 2 * U2
    V = np.eye(3) * ds + (1.0 - np.cos(angle)) / k ** 2 * U + (angle - np.sin(angle)) / k ** 3 * U2
    return R, V @ E_Z


class Rod:
    def __init__(self, config, rest_lengths: np.ndarray | None = None):
        rod = config.rod
        self.L = float(rod["length"])
        self.S = int(rod["sections"])
        self.steps = int(rod["steps_per_section"])
        self.ls = self.L / self.S
        self.ds = self.ls / self.steps
        I = np.pi * rod["radius"] ** 4 / 4.0
        self.K = np.array([rod["youngs_modulus"] * I, rod["youngs_modulus"] * I, rod["shear_modulus"] * 2.0 * I])
        tendons = config.tendons
        self.m = len(tendons)
        self.r = np.array([t.entry for t in tendons])               # (m, 3)
        self.k = np.array([t.stiffness for t in tendons])
        self.tau_max = np.array([t.max_tension for t in tendons])
        self.rest = np.full(self.m, self.L) if rest_lengths is None else np.asarray(rest_lengths, float)

    # -- tendons --------------------------------------------------------------
    def interior_lengths(self, z: np.ndarray) -> np.ndarray:
        z = z.reshape(self.S, 3)
        return np.array([self.ls * np.linalg.norm(E_Z + np.cross(z, r), axis=1).sum() for r in self.r])

    def interior_gradient(self, z: np.ndarray) -> np.ndarray:
        """J_lz = d l_in / d z, m x 3S."""
        z = z.reshape(self.S, 3)
        rows = []
        for r in self.r:
            t = E_Z + np.cross(z, r)
            t_hat = t / np.linalg.norm(t, axis=1)[:, None]
            rows.append((self.ls * np.cross(r, t_hat)).ravel())
        return np.array(rows)

    def stretch(self, z: np.ndarray, spans: np.ndarray):
        """Extensions e_i, tensions tau_i, and the active set (taut, unsaturated)."""
        e = spans + self.interior_lengths(z) - self.rest
        tau = np.clip(self.k * e, 0.0, self.tau_max)
        active = (e > 0.0) & (self.k * e < self.tau_max)
        return e, tau, active

    # -- energy ---------------------------------------------------------------
    def energy(self, z: np.ndarray, spans: np.ndarray) -> float:
        z = z.reshape(self.S, 3)
        elastic = 0.5 * self.ls * float(np.sum(z ** 2 * self.K))
        e, _, _ = self.stretch(z, spans)
        cable = 0.0
        for ei, ki, ti in zip(e, self.k, self.tau_max):
            if ei <= 0.0:
                continue
            cable += 0.5 * ki * ei ** 2 if ki * ei <= ti else ti * ei - 0.5 * ti ** 2 / ki
        return elastic + cable

    def gradient(self, z: np.ndarray, spans: np.ndarray) -> np.ndarray:
        z = z.reshape(self.S, 3)
        _, tau, _ = self.stretch(z, spans)
        return (self.ls * z * self.K).ravel() + tau @ self.interior_gradient(z)

    def load_hessian(self, z: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """H_0: elastic stiffness plus sum_i tau_i d^2 l_in,i / dz^2, at fixed tensions."""
        z = z.reshape(self.S, 3)
        H = np.diag(np.tile(self.ls * self.K, self.S))
        for r, t_i in zip(self.r, tau):
            if t_i == 0.0:
                continue
            Rx = skew(r)
            for s in range(self.S):
                t = E_Z + np.cross(z[s], r)
                n = np.linalg.norm(t)
                t_hat = t / n
                H[3 * s:3 * s + 3, 3 * s:3 * s + 3] += t_i * self.ls * Rx.T @ (np.eye(3) - np.outer(t_hat, t_hat)) @ Rx / n
        return H

    def hessian(self, z: np.ndarray, spans: np.ndarray) -> np.ndarray:
        """Full stiffness: H_0 + J_lz^T K_a J_lz."""
        _, tau, active = self.stretch(z, spans)
        J = self.interior_gradient(z)
        Ka = np.where(active, self.k, 0.0)
        return self.load_hessian(z, tau) + J.T @ (Ka[:, None] * J)

    def solve(self, spans: np.ndarray, z0: np.ndarray | None = None, tol: float = 1e-13,
              max_iter: int = 60) -> tuple[np.ndarray, int]:
        """Equilibrium shape for the exterior spans, by Newton's method from z0."""
        z = np.zeros(3 * self.S) if z0 is None else np.array(z0, float).ravel()
        spans = np.asarray(spans, float)
        for it in range(max_iter):
            g = self.gradient(z, spans)
            if np.linalg.norm(g) < tol:
                return z.reshape(self.S, 3), it
            step = np.linalg.solve(self.hessian(z, spans), g)
            slope, t = float(g @ step), 1.0
            if slope > 1e-14:                   # backtracking, until the energy differences reach round-off
                u0 = self.energy(z, spans)
                while t > 1e-6 and self.energy(z - t * step, spans) > u0 - 1e-4 * t * slope:
                    t *= 0.5
            z = z - t * step
        return z.reshape(self.S, 3), max_iter

    # -- shape ----------------------------------------------------------------
    def integrate(self, z: np.ndarray, base: np.ndarray | None = None):
        """Centerline points and rotations, S * steps + 1 samples from the base to the tip."""
        base = np.eye(4) if base is None else base
        R, p = base[:3, :3].copy(), base[:3, 3].copy()
        points, rotations = [p.copy()], [R.copy()]
        for u in z.reshape(self.S, 3):
            dR, dp = section_increment(u, self.ds)
            for _ in range(self.steps):
                p = p + R @ dp
                R = R @ dR
                points.append(p.copy())
                rotations.append(R.copy())
        return np.array(points), np.array(rotations)

    def tip_shape_jacobian(self, z: np.ndarray, h: float = 1e-6) -> np.ndarray:
        """J_Dz: twist of {D} relative to {B}, in {B}, per unit change of z (6 x 3S)."""
        z = z.ravel()
        J = np.zeros((6, z.size))
        for j in range(z.size):
            dz = np.zeros_like(z)
            dz[j] = h
            p1, R1 = self.integrate(z + dz)
            p0, R0 = self.integrate(z - dz)
            J[:3, j] = (p1[-1] - p0[-1]) / (2 * h)
            J[3:, j] = rotation_log(R1[-1] @ R0[-1].T) / (2 * h)
        return J
