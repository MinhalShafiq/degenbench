"""Run the detector leaderboard: calibrate on development sequences, score all.

    python3 -m degenbench_leaderboard.leaderboard \\
        --sequence ../labels/v0.1/subt_mrs/laurel_caverns_handheld3 laurel_features.csv \\
        --sequence ../labels/v0.1/subt_mrs/corridor01 corridor01_features.csv \\
        --out results.yaml [--predictions-dir DIR]

Each sequence's role (development or held_out) comes from the thresholds file
in force (labels.evaluation), never from the release: a sequence that has been
held out once becomes development in later rounds.
Features come from `detector_features`, run with the parameters in meta.yaml;
their reference poses are checked against the labels frame by frame.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from . import thresholds
from .detectors import DETECTORS, Detector, Features, predict, read_features, write_predictions
from .labels import CONSENSUS, Frame, load_release
from .score import _plain, score

REFINE_TOLERANCE_M = 1e-3
REFINE_TOLERANCE_DEG = 1e-2


def check_reference(frames: list[Frame], labels_csv: Path, feats: dict[int, Features]) -> dict[str, Any]:
    """The detector features must be computed at the labels' reference poses."""
    import csv
    rows = {int(r["index"]): r for r in csv.DictReader(open(labels_csv, encoding="utf-8"))}
    missing = [f.index for f in frames if f.index not in feats]
    if missing:
        raise ValueError(f"features missing for {len(missing)} labelled frames, e.g. {missing[:5]}")
    dt = [abs(feats[i].refine_trans_m - float(rows[i]["refine_trans_m"])) for i in rows]
    dr = [abs(feats[i].refine_rot_deg - float(rows[i]["refine_rot_deg"])) for i in rows]
    bad = [i for i, a, b in zip(rows, dt, dr) if a > REFINE_TOLERANCE_M or b > REFINE_TOLERANCE_DEG]
    return {"frames": len(rows), "max_refine_diff_m": max(dt), "max_refine_diff_deg": max(dr),
            "frames_outside_tolerance": len(bad)}


def sequence_role(config: dict[str, Any], name: str) -> str:
    ev = thresholds.get(config, "labels.evaluation")
    if name in ev["development_sequences"]:
        return "development"
    if name in ev["held_out_sequences"]:
        return "held_out"
    raise ValueError(f"{name} is in neither evaluation set of thresholds version {config['version']}")


def predictions(det: Detector, feats: dict[int, Features], tau: float) -> dict[int, np.ndarray]:
    return {i: predict(det, f, tau) for i, f in feats.items()}


def calibrate(det: Detector, sequences: list[tuple[list[Frame], dict[int, Features]]],
              config: dict[str, Any], grid_size: int = 200) -> dict[str, Any]:
    values = np.concatenate([det.score(feats[f.index])[0] for frames, feats in sequences
                             for f in frames if f.status == CONSENSUS])
    values = values[np.isfinite(values) & (values > 0)]
    grid = np.geomspace(values.min() / 2, values.max() * 2, grid_size)
    objective = []
    for tau in grid:
        tp = fp = fn = 0
        for frames, feats in sequences:
            pm = score(frames, predictions(det, feats, tau), config)["consensus"]["per_direction_micro"]
            tp, fp, fn = tp + pm["tp"], fp + pm["fp"], fn + pm["fn"]
        objective.append(2 * tp / (2 * tp + fp + fn) if tp + fp + fn else 0.0)
    objective = np.array(objective)
    best = objective.max()
    first = int(np.argmax(objective))
    last = first
    while last + 1 < len(grid) and objective[last + 1] == best:
        last += 1
    return {"tau": float(np.sqrt(grid[first] * grid[last])), "dev_per_direction_micro_f1": float(best),
            "plateau": [float(grid[first]), float(grid[last])]}


def runtime(det: Detector, feats: dict[int, Features]) -> dict[str, float]:
    t = np.array([det.runtime(f) for f in feats.values()])
    return {"median_ms": float(np.median(t)), "p95_ms": float(np.percentile(t, 95))}


def headline(s: dict[str, Any]) -> dict[str, Any]:
    c = s["consensus"]
    return {"subspace_match_rate": c["subspace_error_deg"]["match_rate"],
            "subspace_error_mean_deg": c["subspace_error_deg"]["mean"],
            "frame_detection_f1": c["frame_detection"]["f1"],
            "frame_detection_precision": c["frame_detection"]["precision"],
            "frame_detection_recall": c["frame_detection"]["recall"],
            "per_direction_micro_f1": c["per_direction_micro"]["f1"],
            "disputed_match_rate_vs_icp": s["disputed"]["subspace_error_deg_vs"]["icp"]["match_rate"],
            "disputed_match_rate_vs_gicp": s["disputed"]["subspace_error_deg_vs"]["gicp"]["match_rate"]}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sequence", nargs=2, action="append", required=True, metavar=("RELEASE_DIR", "FEATURES_CSV"))
    p.add_argument("--out", required=True)
    p.add_argument("--predictions-dir")
    p.add_argument("--thresholds", default=str(thresholds.DEFAULT_PATH))
    a = p.parse_args(argv)
    config = thresholds.load(a.thresholds)

    seqs = []
    for release_dir, features_csv in a.sequence:
        meta = yaml.safe_load((Path(release_dir) / "meta.yaml").read_text(encoding="utf-8"))
        frames = load_release(release_dir)
        all_feats = {f.index: f for f in read_features(features_csv)}
        feats = {f.index: all_feats[f.index] for f in frames if f.index in all_feats}
        check = check_reference(frames, Path(release_dir) / "labels.csv", feats)
        name = f"{meta['dataset']}/{meta['sequence']}"
        role = sequence_role(config, name)
        seqs.append({"name": name, "role": role, "frames": frames, "feats": feats, "check": check})
        print(f"{name} ({role}): reference check {check}", file=sys.stderr)

    dev = [(s["frames"], s["feats"]) for s in seqs if s["role"] == "development"]
    if not dev:
        raise SystemExit("no development sequence given: calibration needs one")
    result: dict[str, Any] = {
        "thresholds_version": config["version"],
        "sequences": {s["name"]: {"role": s["role"], "reference_check": s["check"]} for s in seqs},
        "detectors": {},
    }
    for name, det in DETECTORS.items():
        cal = calibrate(det, dev, config)
        print(f"{name}: calibrated tau {cal['tau']:.4g} (dev F1 {cal['dev_per_direction_micro_f1']:.3f})",
              file=sys.stderr)
        variants = {"calibrated": cal["tau"], **{f"published_{t:g}": t for t in det.published}}
        entry: dict[str, Any] = {"calibration": cal, "variants": {}}
        for vname, tau in variants.items():
            entry["variants"][vname] = {"tau": tau, "sequences": {}}
            for s in seqs:
                preds = predictions(det, s["feats"], tau)
                sc = score(s["frames"], preds, config)
                entry["variants"][vname]["sequences"][s["name"]] = {"headline": headline(sc), "full": sc}
                if a.predictions_dir:
                    d = Path(a.predictions_dir) / s["name"].replace("/", "_")
                    d.mkdir(parents=True, exist_ok=True)
                    write_predictions(d / f"{name}_{vname}.csv", preds)
        entry["runtime"] = {s["name"]: runtime(det, s["feats"]) for s in seqs}
        result["detectors"][name] = entry

    Path(a.out).write_text(yaml.safe_dump(_plain(result), sort_keys=False), encoding="utf-8")
    print(f"wrote {a.out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
