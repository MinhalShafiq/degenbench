"""Decision rules of the scored degeneracy detectors.

Each detector maps one row of `detector_features` output (the point-to-plane
information H, the point-displacement metric M and X-ICP's contribution sums,
all at the label's reference pose) and one scalar threshold to a predicted
degenerate subspace. Subspaces are computed in physical units (m, rad) and
returned in the leaderboard's threshold-normalized coordinates.

| Name | Source | Rule | Published threshold |
| --- | --- | --- | --- |
| min_eigenvalue | Zhang, Kaess, Singh 2016 | eigenvectors of H with lambda < tau | none (set per system) |
| condition_number | common practice | eigenvectors of H with lambda_max / lambda > tau | none |
| x_icp | Tuna et al. 2023 (X-ICP) | block eigenvectors not "fully localizable" | kappa = 250 / 180 / 35 |
| dcreg | DCReg 2025 | Schur-complement block eigenvectors with lambda_max / lambda > tau | 10 to 50 |
| frame_independent | "Degenerate in Whose Frame?" 2026 | generalized eigenvectors of H v = lambda M v with lambda < tau | 1e-2 |

All five are reimplemented from the papers.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np

TRANS_SCALE_M = 0.10
ROT_SCALE_RAD = np.radians(2.0)
SCALE = np.array([TRANS_SCALE_M] * 3 + [ROT_SCALE_RAD] * 3)
XICP_KAPPA = (250.0, 180.0, 35.0)


@dataclass
class Features:
    index: int
    stamp: float
    refine_converged: bool
    refine_trans_m: float
    refine_rot_deg: float
    num_correspondences: int
    H: np.ndarray            # 6 x 6, [t; r], physical units
    M: np.ndarray
    V_t: np.ndarray          # 3 x 3, columns ascending
    V_r: np.ndarray
    L_c: np.ndarray          # (6,) translation then rotation
    L_s: np.ndarray
    features_ms: float
    xicp_ms: float


def read_features(path: str | Path) -> list[Features]:
    out = []
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            mat = lambda p, n: np.array([[float(r[f"{p}{a}{b}"]) for b in range(n)] for a in range(n)])
            cols = lambda p: np.array([[float(r[f"{p}{c}{row}"]) for c in range(3)] for row in range(3)])
            vec = lambda p: np.array([float(r[f"{p}{j}"]) for j in range(3)])
            out.append(Features(
                int(r["index"]), float(r["stamp"]), r["refine_converged"] == "1",
                float(r["refine_trans_m"]), float(r["refine_rot_deg"]), int(r["num_correspondences"]),
                mat("H", 6), mat("M", 6), cols("xicp_vt"), cols("xicp_vr"),
                np.concatenate([vec("xicp_lct"), vec("xicp_lcr")]),
                np.concatenate([vec("xicp_lst"), vec("xicp_lsr")]),
                float(r["features_ms"]), float(r["xicp_ms"])))
    return out


def to_normalized(basis_phys: np.ndarray) -> np.ndarray:
    """A direction x (m, rad) is x / SCALE in normalized coordinates."""
    return basis_phys / SCALE[:, None]


def _embed(v: np.ndarray, block: str) -> np.ndarray:
    out = np.zeros((6, v.shape[1]))
    out[(slice(0, 3) if block == "t" else slice(3, 6)), :] = v
    return out


def _sym(a: np.ndarray) -> np.ndarray:
    return 0.5 * (a + a.T)


# --- scores: per candidate direction, a scalar and its direction ------------
# Each detector is a "score" function returning (values, directions) and a rule
# "flag when value < tau" or "value > tau"; this keeps calibration generic.

def _min_eigenvalue(f: Features):
    w, v = np.linalg.eigh(_sym(f.H))
    return w, v


def _condition_number(f: Features):
    w, v = np.linalg.eigh(_sym(f.H))
    return w.max() / np.maximum(w, 1e-300), v


def _dcreg(f: Features):
    H = _sym(f.H)
    Htt, Htr, Hrr = H[:3, :3], H[:3, 3:], H[3:, 3:]
    # The paper assumes both blocks invertible; the pseudo-inverse (generalized
    # Schur complement) keeps exactly degenerate blocks defined.
    S_t = Htt - Htr @ np.linalg.pinv(Hrr, rcond=1e-12, hermitian=True) @ Htr.T
    S_r = Hrr - Htr.T @ np.linalg.pinv(Htt, rcond=1e-12, hermitian=True) @ Htr
    values, dirs = [], []
    for S, block in ((S_t, "t"), (S_r, "r")):
        w, v = np.linalg.eigh(_sym(S))
        values.append(w.max() / np.maximum(w, 1e-300))
        dirs.append(_embed(v, block))
    return np.concatenate(values), np.hstack(dirs)


def generalized_eigh(H: np.ndarray, M: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Solve H v = lambda M v for symmetric H and positive definite M, ascending."""
    L = np.linalg.cholesky(_sym(M))
    Li = np.linalg.inv(L)
    w, y = np.linalg.eigh(_sym(Li @ _sym(H) @ Li.T))
    return w, Li.T @ y


def _frame_independent(f: Features):
    return generalized_eigh(f.H, f.M)


def _xicp(f: Features):
    """Per block eigenvector: the smallest scale c at which it is 'fully localizable'.

    X-ICP: full if L_c >= c * kappa_1 or L_s >= c * kappa_2, so the direction is
    flagged (not full) when max(L_c / kappa_1, L_s / kappa_2) < c. The published
    rule is c = 1; calibration varies c.
    """
    k1, k2, _ = XICP_KAPPA
    return np.maximum(f.L_c / k1, f.L_s / k2), np.hstack([_embed(f.V_t, "t"), _embed(f.V_r, "r")])


@dataclass(frozen=True)
class Detector:
    name: str
    score: Callable[[Features], tuple[np.ndarray, np.ndarray]]
    flag_below: bool                  # flag when value < tau (else value > tau)
    published: tuple[float, ...]      # published threshold(s); empty if none
    runtime: Callable[[Features], float]


DETECTORS = {d.name: d for d in [
    Detector("min_eigenvalue", _min_eigenvalue, True, (), lambda f: f.features_ms),
    Detector("condition_number", _condition_number, False, (), lambda f: f.features_ms),
    Detector("x_icp", _xicp, True, (1.0,), lambda f: f.features_ms + f.xicp_ms),
    Detector("dcreg", _dcreg, False, (10.0, 50.0), lambda f: f.features_ms),
    Detector("frame_independent", _frame_independent, True, (1e-2,), lambda f: f.features_ms),
]}


def predict(detector: Detector, f: Features, tau: float) -> np.ndarray:
    """Predicted degenerate subspace (6 x k) in normalized coordinates."""
    values, dirs = detector.score(f)
    flagged = values < tau if detector.flag_below else values > tau
    return to_normalized(dirs[:, flagged])


def write_predictions(path: str | Path, rows: dict[int, np.ndarray]) -> None:
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh, lineterminator="\n")
        w.writerow(["index", "k"] + [f"v{c}{r}" for c in range(6) for r in range(6)])
        for idx in sorted(rows):
            b = rows[idx]
            q = np.linalg.qr(b)[0] if b.shape[1] else b
            full = [""] * 36
            for c in range(q.shape[1]):
                for r in range(6):
                    full[c * 6 + r] = f"{q[r, c]:.7g}"
            w.writerow([idx, q.shape[1]] + full)
