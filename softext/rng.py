"""A small seeded generator (mulberry32 and Box-Muller), identical in docs/js/sim/linalg.js,
so that the noisy runs of the Python package and of the pages are the same."""
from __future__ import annotations

import math

import numpy as np

M32 = 0xFFFFFFFF


class Random:
    def __init__(self, seed: int = 0):
        self.state = int(seed) & M32

    def uniform(self) -> float:
        """Uniform in [0, 1)."""
        a = self.state = (self.state + 0x6D2B79F5) & M32
        t = ((a ^ (a >> 15)) * (1 | a)) & M32
        t = ((t + (((t ^ (t >> 7)) * (61 | t)) & M32)) & M32) ^ t
        return ((t ^ (t >> 14)) & M32) / 4294967296.0

    def normal(self) -> float:
        u1, u2 = 1.0 - self.uniform(), self.uniform()
        return math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)

    def normal_array(self, shape) -> np.ndarray:
        size = int(np.prod(shape))
        return np.array([self.normal() for _ in range(size)]).reshape(shape)
