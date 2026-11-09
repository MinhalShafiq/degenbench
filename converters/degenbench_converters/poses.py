"""Rigid transforms, TUM trajectories and pose interpolation.

Quaternions are (qx, qy, qz, qw), as in TUM files. Transforms are 4x4 numpy
arrays; T_a_b maps coordinates in frame b to frame a (the pose of b in a).
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(r: np.ndarray) -> np.ndarray:
    """Rotation matrix to (qx, qy, qz, qw) with qw >= 0 (Shepperd's method)."""
    r = np.asarray(r, dtype=float)
    tr = np.trace(r)
    if tr > 0:
        s = 2.0 * np.sqrt(tr + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * np.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    q /= np.linalg.norm(q)
    return -q if q[3] < 0 else q


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = rotation
    t[:3, 3] = translation
    return t


def invert(t: np.ndarray) -> np.ndarray:
    r, p = t[:3, :3], t[:3, 3]
    return make_transform(r.T, -r.T @ p)


def apply(t: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply one transform to N x 3 points."""
    return points @ t[:3, :3].T + t[:3, 3]


def slerp(q0: np.ndarray, q1: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Spherical interpolation between unit quaternions, vectorized over u (N,)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    u = np.atleast_1d(np.asarray(u, float))[:, None]
    dot = float(np.dot(q0, q1))
    if dot < 0.0:  # take the short way round
        q1, dot = -q1, -dot
    if dot > 0.9995:  # nearly parallel: normalized lerp is accurate and stable
        q = q0 + u * (q1 - q0)
    else:
        theta = np.arccos(dot)
        q = (np.sin((1 - u) * theta) * q0 + np.sin(u * theta) * q1) / np.sin(theta)
    return q / np.linalg.norm(q, axis=1, keepdims=True)


@dataclass
class Trajectory:
    """Time-stamped poses T_world_frame, sorted by time."""

    stamps: np.ndarray        # (N,) seconds
    positions: np.ndarray     # (N, 3)
    quaternions: np.ndarray   # (N, 4) qx qy qz qw

    def __post_init__(self) -> None:
        order = np.argsort(self.stamps)
        self.stamps = np.asarray(self.stamps, float)[order]
        self.positions = np.asarray(self.positions, float)[order]
        self.quaternions = np.asarray(self.quaternions, float)[order]
        if len(self.stamps) < 2:
            raise ValueError("trajectory needs at least two poses")

    @classmethod
    def from_tum(cls, path: str | Path) -> "Trajectory":
        rows = []
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                rows.append([float(v) for v in line.replace(",", " ").split()[:8]])
        data = np.array(rows)
        return cls(data[:, 0], data[:, 1:4], data[:, 4:8])

    def to_tum(self, path: str | Path) -> None:
        data = np.hstack([self.stamps[:, None], self.positions, self.quaternions])
        np.savetxt(path, data, fmt="%.9f", header="timestamp tx ty tz qx qy qz qw")

    def covers(self, t_start: float, t_end: float, max_gap: float) -> bool:
        """True if [t_start, t_end] lies inside the trajectory with no gap > max_gap."""
        if t_start < self.stamps[0] or t_end > self.stamps[-1]:
            return False
        lo = max(np.searchsorted(self.stamps, t_start, side="right") - 1, 0)
        hi = min(np.searchsorted(self.stamps, t_end, side="left"), len(self.stamps) - 1)
        return bool(np.all(np.diff(self.stamps[lo:hi + 1]) <= max_gap))

    def interpolate(self, t: np.ndarray) -> np.ndarray:
        """Poses at times t (M,) as (M, 4, 4); linear translation, slerp rotation."""
        t = np.atleast_1d(np.asarray(t, float))
        if np.any(t < self.stamps[0]) or np.any(t > self.stamps[-1]):
            raise ValueError("interpolation time outside trajectory")
        idx = np.clip(np.searchsorted(self.stamps, t, side="right") - 1, 0, len(self.stamps) - 2)
        out = np.tile(np.eye(4), (len(t), 1, 1))
        for i in np.unique(idx):
            sel = idx == i
            t0, t1 = self.stamps[i], self.stamps[i + 1]
            u = (t[sel] - t0) / (t1 - t0)
            q = slerp(self.quaternions[i], self.quaternions[i + 1], u)
            p = self.positions[i] + u[:, None] * (self.positions[i + 1] - self.positions[i])
            out[sel, :3, :3] = np.stack([quat_to_rot(qi) for qi in q])
            out[sel, :3, 3] = p
        return out
