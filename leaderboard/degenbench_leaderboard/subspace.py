"""Comparison of degenerate subspaces via principal angles.

Subspaces are given as 6 x k matrices whose columns span them, in the
threshold-normalized coordinates used by the labeler
(translation / 0.10 m, rotation / 2 deg, order tx ty tz rx ry rz).
"""

from __future__ import annotations

import numpy as np

DIM = 6
_RANK_TOL = 1e-9


def _orthonormal_basis(a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    if a.ndim != 2 or a.shape[0] != DIM:
        raise ValueError(f"subspace basis must be {DIM} x k, got shape {a.shape}")
    if a.shape[1] == 0:
        return a
    u, s, _ = np.linalg.svd(a, full_matrices=False)
    rank = int(np.sum(s > _RANK_TOL * max(1.0, s[0])))
    return u[:, :rank]


def principal_angles_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Principal angles between span(a) and span(b), ascending, in degrees.

    Returns min(dim a, dim b) angles; empty if either subspace is empty.
    """
    qa, qb = _orthonormal_basis(a), _orthonormal_basis(b)
    if qa.shape[1] == 0 or qb.shape[1] == 0:
        return np.zeros(0)
    # Singular values are descending, so the angles come out ascending.
    cosines = np.linalg.svd(qa.T @ qb, compute_uv=False)
    return np.degrees(np.arccos(np.clip(cosines, -1.0, 1.0)))


def subspace_error_deg(predicted: np.ndarray, labelled: np.ndarray) -> float:
    """Leaderboard subspace error: the largest principal angle.

    A dimension mismatch is maximally wrong (90 deg): a detector that misses or
    invents a degenerate direction has not found the labelled subspace. Two empty
    subspaces agree exactly (0 deg).
    """
    qp, ql = _orthonormal_basis(predicted), _orthonormal_basis(labelled)
    if qp.shape[1] != ql.shape[1]:
        return 90.0
    if qp.shape[1] == 0:
        return 0.0
    return float(np.max(principal_angles_deg(qp, ql)))
