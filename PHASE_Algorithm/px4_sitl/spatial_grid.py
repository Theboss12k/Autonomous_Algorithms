"""Uniform spatial hash grid for neighbour queries.

Checking every pair of N vehicles is O(N^2). Hashing each vehicle into a square
cell at least as wide as the largest interaction distance means any two
vehicles that can interact are in the same or adjacent cells. Building the
grid is O(N), and each query only looks at the 3x3 (or 3x3x3) block of cells
around a vehicle, so all-pairs-within-radius costs O(N * k), where k is the
number of vehicles actually nearby. At constant traffic density k stays
roughly constant as the fleet grows.
"""
import collections
import itertools
import math

import numpy as np


class SpatialGrid:
    def __init__(self, cell_size, use_z=False):
        if cell_size <= 0:
            raise ValueError("cell_size must be > 0")
        self.cell = float(cell_size)
        self.use_z = use_z
        self.cells = collections.defaultdict(list)
        self.pos = []

    def _key(self, p):
        k = (math.floor(p[0] / self.cell), math.floor(p[1] / self.cell))
        return k + (math.floor(p[2] / self.cell),) if self.use_z else k

    def build(self, positions):
        """positions: sequence of xyz; items are referred to by their index."""
        self.cells.clear()
        self.pos = [np.asarray(p, dtype=float) for p in positions]
        for i, p in enumerate(self.pos):
            self.cells[self._key(p)].append(i)
        return self

    def _offsets(self, radius):
        r = max(1, math.ceil(radius / self.cell))
        rng = range(-r, r + 1)
        return itertools.product(rng, rng, rng) if self.use_z else itertools.product(rng, rng)

    def _dist(self, a, b):
        d = self.pos[a] - self.pos[b]
        return float(math.sqrt(d[0] * d[0] + d[1] * d[1] + (d[2] * d[2] if self.use_z else 0.0)))

    def neighbors(self, i, radius):
        """Indices within ``radius`` of item i (excluding i), ascending."""
        k = self._key(self.pos[i])
        out = []
        for off in self._offsets(radius):
            for j in self.cells.get(tuple(a + b for a, b in zip(k, off)), ()):
                if j != i and self._dist(i, j) <= radius:
                    out.append(j)
        out.sort()
        return out

    def pairs(self, radius):
        """All (i, j), i < j, within ``radius``, in the same order a nested i<j loop gives."""
        out = []
        for i in range(len(self.pos)):
            out.extend((i, j) for j in self.neighbors(i, radius) if j > i)
        out.sort()
        return out


def brute_pairs(positions, radius=math.inf, use_z=False):
    """Reference O(N^2) version of SpatialGrid.pairs."""
    pos = [np.asarray(p, dtype=float) for p in positions]
    out = []
    for i in range(len(pos)):
        for j in range(i + 1, len(pos)):
            d = pos[i] - pos[j]
            dist = math.sqrt(d[0] ** 2 + d[1] ** 2 + (d[2] ** 2 if use_z else 0.0))
            if dist <= radius:
                out.append((i, j))
    return out
