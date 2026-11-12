"""DegenBench label release format, built under the fallback for solver-dependent labels.

Per sequence, a release directory holds:

    labels.csv      one row per frame: status, consensus label, both solvers' raw results
    subspaces.csv   6-D bases: rows for icp and gicp on every frame, consensus where defined
    meta.yaml       inputs, calibration, thresholds version, checksums

Frame status (thresholds.yaml, labels.release):

    consensus             reference refinement converged and the two solvers agree
    disputed              refinement converged, solvers disagree
    unreliable_reference  refinement did not converge

The consensus subspace of dimension k is spanned by the top-k eigenvectors of
(P_icp + P_gicp) / 2, the mean of the two solvers' orthogonal projectors. It is
symmetric in the solvers and equals either one when they coincide.

Subspace columns are stored as v{c}{r}: column c, row r, in threshold-normalized
coordinates (t / 0.10 m, r / 2 deg; order tx ty tz rx ry rz).
"""

from __future__ import annotations

import csv
import hashlib
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml

from .subspace import DIM, subspace_error_deg

AXES = ("tx", "ty", "tz", "rx", "ry", "rz")
SOLVERS = ("icp", "gicp")
CONSENSUS, DISPUTED, UNRELIABLE = "consensus", "disputed", "unreliable_reference"
STATUSES = (CONSENSUS, DISPUTED, UNRELIABLE)
VEC_COLUMNS = [f"v{c}{r}" for c in range(DIM) for r in range(DIM)]


def _projector(basis: np.ndarray) -> np.ndarray:
    if basis.shape[1] == 0:
        return np.zeros((DIM, DIM))
    q, _ = np.linalg.qr(basis)
    return q @ q.T


def consensus_subspace(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Top-k eigenvectors of the mean projector of two k-dimensional subspaces."""
    if a.shape[1] != b.shape[1]:
        raise ValueError("consensus needs subspaces of equal dimension")
    k = a.shape[1]
    if k == 0:
        return np.zeros((DIM, 0))
    w, v = np.linalg.eigh(0.5 * (_projector(a) + _projector(b)))
    basis = v[:, np.argsort(w)[::-1][:k]]
    # Fix the sign so the release is deterministic: largest |component| positive.
    for c in range(k):
        if basis[np.argmax(np.abs(basis[:, c])), c] < 0:
            basis[:, c] *= -1
    return basis


def free_axes(basis: np.ndarray, min_projection: float) -> np.ndarray:
    """Axis i counts as free when |P e_i|^2 = P_ii exceeds `min_projection`."""
    return np.diag(_projector(basis)) > min_projection


def axes_string(flags: Iterable[bool]) -> str:
    return "".join("1" if f else "0" for f in flags)


def matrix_from_row(row: dict[str, str]) -> np.ndarray:
    """The 6 x 6 matrix stored in a row's v{c}{r} columns."""
    return np.array([[float(row[f"v{c}{r}"]) for c in range(DIM)] for r in range(DIM)])


@dataclass
class Frame:
    index: int
    stamp: float
    status: str
    k: int | None                      # consensus dimension, None unless consensus
    basis: np.ndarray | None           # consensus basis (6 x k), None unless consensus
    solver_k: dict[str, int] = field(default_factory=dict)
    solver_basis: dict[str, np.ndarray] = field(default_factory=dict)


# ---------------------------------------------------------------------------
# Building a release from labeler output
# ---------------------------------------------------------------------------

def _read_csv(path: Path) -> list[dict[str, str]]:
    with open(path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def frame_status(refine_converged: bool, agree: bool) -> str:
    if not refine_converged:
        return UNRELIABLE
    return CONSENSUS if agree else DISPUTED


def build_frames(labeler_csv: Path, subspaces_csv: Path, max_angle_deg: float) -> tuple[list[Frame], list[dict[str, str]]]:
    """Read label_sequence output and derive status and consensus per frame.

    Solver agreement is recomputed from the stored eigenvectors and must match
    the labeler's own `solvers_agree` column; a mismatch is an error.
    """
    rows = _read_csv(labeler_csv)
    eigvecs: dict[tuple[int, str], np.ndarray] = {}
    for r in _read_csv(subspaces_csv):
        eigvecs[(int(r["index"]), r["solver"])] = matrix_from_row(r)
    frames = []
    for r in rows:
        idx = int(r["index"])
        ks = {s: int(r[f"{s}_k"]) for s in SOLVERS}
        bases = {s: eigvecs[(idx, s)][:, DIM - ks[s]:] for s in SOLVERS}
        angle = subspace_error_deg(bases["icp"], bases["gicp"])
        agree = angle <= max_angle_deg
        if agree != (r["solvers_agree"] == "1"):
            raise ValueError(f"frame {idx}: recomputed agreement ({angle:.3f} deg) "
                             f"differs from the labeler's ({r['subspace_angle_deg']})")
        status = frame_status(r["refine_converged"] == "1", agree)
        basis = consensus_subspace(bases["icp"], bases["gicp"]) if status == CONSENSUS else None
        frames.append(Frame(idx, float(r["stamp"]), status,
                            ks["icp"] if basis is not None else None, basis, ks, bases))
    return frames, rows


_PASS_THROUGH = (["stamp", "map_points", "coverage", "refine_converged", "refine_trans_m",
                  "refine_rot_deg", "icp_k", "gicp_k", "subspace_angle_deg", "icp_axes", "gicp_axes"]
                 + [f"{s}_eig{i}" for s in SOLVERS for i in range(DIM)]
                 + [f"{s}_rms_{a}" for s in SOLVERS for a in AXES])
_ROBUST = ["robust_icp_k", "robust_gicp_k", "robust_subspace_angle_deg", "robust_solvers_agree"]


def write_release(frames: list[Frame], rows: list[dict[str, str]], out_dir: Path,
                  meta: dict[str, Any], min_projection: float) -> dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    robust = all(c in rows[0] for c in _ROBUST)
    cols = ["index", "status", "k", "consensus_axes"] + _PASS_THROUGH + (_ROBUST if robust else [])
    with open(out_dir / "labels.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(cols)
        for fr, r in zip(frames, rows):
            consensus = fr.status == CONSENSUS
            w.writerow([fr.index, fr.status,
                        fr.k if consensus else "",
                        axes_string(free_axes(fr.basis, min_projection)) if consensus else ""]
                       + [r[c] for c in cols[4:]])
    with open(out_dir / "subspaces.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f, lineterminator="\n")
        w.writerow(["index", "source", "k"] + VEC_COLUMNS)
        for fr in frames:
            entries = [(s, fr.solver_basis[s]) for s in SOLVERS]
            if fr.status == CONSENSUS:
                entries.append(("consensus", fr.basis))
            for source, b in entries:
                full = np.zeros((DIM, DIM))
                full[:, :b.shape[1]] = b
                w.writerow([fr.index, source, b.shape[1]] + [f"{full[r, c]:.7g}" for c in range(DIM) for r in range(DIM)])
    counts = {s: sum(fr.status == s for fr in frames) for s in STATUSES}
    resolved = counts[CONSENSUS] + counts[DISPUTED]
    summary = {
        "frames": len(frames),
        "status_counts": counts,
        "solver_agreement_rate": round(counts[CONSENSUS] / resolved, 4) if resolved else None,
        "consensus_k_counts": {int(k): sum(1 for fr in frames if fr.k == k)
                               for k in sorted({fr.k for fr in frames if fr.k is not None})},
    }
    meta = dict(meta)
    meta["summary"] = summary
    meta["files"] = {name: hashlib.sha256((out_dir / name).read_bytes()).hexdigest()
                     for name in ("labels.csv", "subspaces.csv")}
    with open(out_dir / "meta.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return meta


# ---------------------------------------------------------------------------
# Reading a release
# ---------------------------------------------------------------------------

def load_release(directory: str | Path) -> list[Frame]:
    directory = Path(directory)
    meta = yaml.safe_load((directory / "meta.yaml").read_text(encoding="utf-8"))
    for name, digest in meta.get("files", {}).items():
        if hashlib.sha256((directory / name).read_bytes()).hexdigest() != digest:
            raise ValueError(f"{directory / name}: checksum does not match meta.yaml")
    bases: dict[tuple[int, str], np.ndarray] = {}
    for r in _read_csv(directory / "subspaces.csv"):
        k = int(r["k"])
        bases[(int(r["index"]), r["source"])] = matrix_from_row(r)[:, :k]
    frames = []
    for r in _read_csv(directory / "labels.csv"):
        idx = int(r["index"])
        consensus = r["status"] == CONSENSUS
        frames.append(Frame(
            idx, float(r["stamp"]), r["status"],
            int(r["k"]) if consensus else None,
            bases[(idx, "consensus")] if consensus else None,
            {s: int(r[f"{s}_k"]) for s in SOLVERS},
            {s: bases[(idx, s)] for s in SOLVERS}))
    return frames
