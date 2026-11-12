"""Score a degeneracy detector against a DegenBench label release.

    python3 -m degenbench_leaderboard.score --labels labels/v0.1/subt_mrs/corridor01 \\
        --predictions my_detector.csv [--out score.yaml]

Predictions: one row per frame, `index,k,v00,...,v55`, where columns v{c}{r} for
c < k hold the predicted degenerate subspace (column c, row r) in
threshold-normalized coordinates (t / 0.10 m, r / 2 deg; tx ty tz rx ry rz).
Unused columns may be empty. A labelled frame without a prediction counts as
k = 0 and is reported in `missing_predictions`.

Headline metrics use consensus frames only. Disputed frames are scored
separately against each solver and never enter the headline
(thresholds.yaml, leaderboard.scoring).
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from . import thresholds
from .labels import AXES, CONSENSUS, DISPUTED, SOLVERS, STATUSES, Frame, free_axes, load_release
from .subspace import DIM, subspace_error_deg


def read_predictions(path: str | Path) -> dict[int, np.ndarray]:
    out: dict[int, np.ndarray] = {}
    with open(path, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            k = int(r["k"])
            if not 0 <= k <= DIM:
                raise ValueError(f"{path}: frame {r['index']}: k = {k} out of range")
            b = np.array([[float(r[f"v{c}{row}"]) for c in range(k)] for row in range(DIM)]).reshape(DIM, k)
            out[int(r["index"])] = b
    return out


def _prf(tp: int, fp: int, fn: int) -> dict[str, float | None]:
    p = tp / (tp + fp) if tp + fp else None
    r = tp / (tp + fn) if tp + fn else None
    f1 = 2 * p * r / (p + r) if p is not None and r is not None and p + r > 0 else (0.0 if tp + fp + fn else None)
    return {"precision": p, "recall": r, "f1": f1, "tp": tp, "fp": fp, "fn": fn}


def _err_summary(errors: list[float], match_deg: float) -> dict[str, float | None]:
    if not errors:
        return {"mean": None, "median": None, "match_rate": None}
    e = np.array(errors)
    return {"mean": float(e.mean()), "median": float(np.median(e)),
            "match_rate": float(np.mean(e <= match_deg))}


def score(frames: list[Frame], predictions: dict[int, np.ndarray], config: dict[str, Any]) -> dict[str, Any]:
    min_proj = thresholds.get(config, "labels.release.axis_free_min_projection")
    match_deg = thresholds.get(config, "labels.solver_agreement_max_principal_angle_deg")
    empty = np.zeros((DIM, 0))
    missing = [f.index for f in frames if f.index not in predictions]

    consensus = [f for f in frames if f.status == CONSENSUS]
    errors, axis_counts, frame_counts = [], np.zeros((DIM, 3), int), [0, 0, 0]
    for f in consensus:
        pred = predictions.get(f.index, empty)
        errors.append(subspace_error_deg(pred, f.basis))
        pa, la = free_axes(pred, min_proj), free_axes(f.basis, min_proj)
        axis_counts[:, 0] += pa & la
        axis_counts[:, 1] += pa & ~la
        axis_counts[:, 2] += ~pa & la
        pd, ld = pred.shape[1] > 0, f.k > 0
        frame_counts[0] += pd and ld
        frame_counts[1] += pd and not ld
        frame_counts[2] += ld and not pd

    disputed = [f for f in frames if f.status == DISPUTED]
    vs_solver = {}
    for s in SOLVERS:
        errs = [subspace_error_deg(predictions.get(f.index, empty), f.solver_basis[s]) for f in disputed]
        vs_solver[s] = _err_summary(errs, match_deg)

    return {
        "thresholds_version": config["version"],
        "frames": {s: sum(f.status == s for f in frames) for s in STATUSES},
        "missing_predictions": len(missing),
        "consensus": {
            "subspace_error_deg": _err_summary(errors, match_deg),
            "frame_detection": _prf(*frame_counts),
            "per_direction": {a: _prf(*map(int, axis_counts[i])) for i, a in enumerate(AXES)},
            "per_direction_micro": _prf(*map(int, axis_counts.sum(axis=0))),
        },
        "disputed": {"subspace_error_deg_vs": vs_solver},
    }


def _plain(x: Any) -> Any:
    if isinstance(x, dict):
        return {k: _plain(v) for k, v in x.items()}
    if isinstance(x, float):
        return round(x, 4)
    return x


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--labels", required=True, help="release directory of one sequence")
    p.add_argument("--predictions", required=True)
    p.add_argument("--thresholds", default=str(thresholds.DEFAULT_PATH))
    p.add_argument("--out")
    a = p.parse_args(argv)
    result = _plain(score(load_release(a.labels), read_predictions(a.predictions), thresholds.load(a.thresholds)))
    text = yaml.safe_dump(result, sort_keys=False)
    if a.out:
        Path(a.out).write_text(text, encoding="utf-8")
    sys.stdout.write(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
