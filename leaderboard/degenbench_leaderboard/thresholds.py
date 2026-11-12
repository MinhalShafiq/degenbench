"""Load and validate the pre-registered thresholds file."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

DEFAULT_PATH = Path(__file__).resolve().parents[1] / "thresholds.yaml"

# Keys every scoring script relies on. Missing any of them is an error, so a
# silently edited file cannot change what "pass" means.
REQUIRED_KEYS = [
    "version",
    "labels.degeneracy_threshold.trans_m",
    "labels.degeneracy_threshold.rot_deg",
    "labels.solver_agreement_max_principal_angle_deg",
    "labels.pass.analytic_scenes",
    "labels.pass.real_frames.min_solver_agreement_rate",
    "labels.release.axis_free_min_projection",
    "leaderboard.scoring.headline_frames",
    "leaderboard.pass.min_detectors",
    "leaderboard.pass.min_datasets",
    "leaderboard.pass.min_lidar_types",
    "trajectory.alignment_window_s",
    "trajectory.drift_onset_position_error_m",
    "trajectory.divergence_position_error_m",
    "degen_monitor.pass.max_runtime_ms_per_scan",
    "degen_monitor.pass.min_sustained_flag_s",
    "degen_lio.pass.no_divergence.max_divergences",
    "degen_lio.pass.max_added_runtime_fraction",
]


def get(config: dict[str, Any], dotted: str) -> Any:
    node: Any = config
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            raise KeyError(dotted)
        node = node[part]
    return node


def load(path: str | Path = DEFAULT_PATH) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        config = yaml.safe_load(f)
    missing = []
    for key in REQUIRED_KEYS:
        try:
            get(config, key)
        except KeyError:
            missing.append(key)
    if missing:
        raise ValueError(f"{path}: missing required keys: {', '.join(missing)}")
    return config
