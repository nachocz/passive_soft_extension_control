"""The rigid manipulator: UR10 forward kinematics and Jacobians (Sec. III-B)."""
from __future__ import annotations

import numpy as np


def skew(x: np.ndarray) -> np.ndarray:
    """[x]_x, with [x]_x y = x cross y."""
    return np.array([[0.0, -x[2], x[1]], [x[2], 0.0, -x[0]], [-x[1], x[0], 0.0]])


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


class Arm:
    """6R arm with standard DH parameters; frame 6 is the EE frame {B}."""

    n = 6

    def __init__(self, config):
        self.a, self.alpha, self.d = config.dh
        self.limits = config.joint_limits

    def frames(self, q: np.ndarray) -> list[np.ndarray]:
        """World poses of the DH frames 0..6 (frames[6] = T_B)."""
        out = [np.eye(4)]
        for i in range(self.n):
            out.append(out[-1] @ dh_transform(self.a[i], self.alpha[i], self.d[i], q[i]))
        return out

    @staticmethod
    def origin_jacobian(frames: list[np.ndarray], k: int) -> np.ndarray:
        """3 x 6 velocity Jacobian of the origin of frame k (joint j turns about z_{j-1})."""
        jac = np.zeros((3, 6))
        p = frames[k][:3, 3]
        for j in range(1, k + 1):
            jac[:, j - 1] = np.cross(frames[j - 1][:3, 2], p - frames[j - 1][:3, 3])
        return jac

    @staticmethod
    def point_on_link(frames: list[np.ndarray], link: int, fraction: float) -> np.ndarray:
        """The point at `fraction` of DH link `link` (between the origins of frames link-1 and link)."""
        return (1.0 - fraction) * frames[link - 1][:3, 3] + fraction * frames[link][:3, 3]

    @classmethod
    def point_jacobian(cls, frames: list[np.ndarray], link: int, fraction: float) -> np.ndarray:
        """J_{a_i}^v of a point on a link."""
        return (1.0 - fraction) * cls.origin_jacobian(frames, link - 1) + fraction * cls.origin_jacobian(frames, link)

    @classmethod
    def geometric_jacobian(cls, frames: list[np.ndarray]) -> np.ndarray:
        """J_B = [J_B^v; J_B^omega], 6 x 6, in the world frame."""
        angular = np.stack([frames[j][:3, 2] for j in range(6)], axis=1)
        return np.vstack((cls.origin_jacobian(frames, 6), angular))
