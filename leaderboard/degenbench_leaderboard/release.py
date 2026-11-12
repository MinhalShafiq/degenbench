"""Turn label_sequence output into a release directory (see labels.py).

    python3 -m degenbench_leaderboard.release --labeler-csv OUT.csv \\
        --meta sequence.yaml --out labels/v0.1/<dataset>/<sequence>

`--meta` is a hand-written YAML describing the inputs (dataset, sequence,
bags, topic, ground truth, map, time offset, extrinsic, world-to-map
transform, role). The thresholds version and the release rules in force are
added automatically, with a summary and file checksums.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import yaml

from . import thresholds
from .labels import build_frames, write_release


def _git_commit() -> str | None:
    try:
        return subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                              check=True, cwd=Path(__file__).parent).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--labeler-csv", required=True, help="label_sequence --out file; *_subspaces.csv beside it")
    p.add_argument("--meta", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--thresholds", default=str(thresholds.DEFAULT_PATH))
    a = p.parse_args(argv)

    config = thresholds.load(a.thresholds)
    labels_cfg = config["labels"]
    csv_path = Path(a.labeler_csv)
    frames, rows = build_frames(csv_path, csv_path.with_name(csv_path.stem + "_subspaces.csv"),
                                labels_cfg["solver_agreement_max_principal_angle_deg"])
    meta = yaml.safe_load(Path(a.meta).read_text(encoding="utf-8"))
    meta["thresholds_version"] = config["version"]
    meta["label_definition"] = {
        "monte_carlo": labels_cfg["monte_carlo"],
        "degeneracy_threshold": labels_cfg["degeneracy_threshold"],
        "solvers": labels_cfg["solvers"],
        "solver_agreement_max_principal_angle_deg": labels_cfg["solver_agreement_max_principal_angle_deg"],
        "release": labels_cfg["release"],
    }
    meta["release_tool_commit"] = _git_commit()
    out = write_release(frames, rows, Path(a.out), meta, labels_cfg["release"]["axis_free_min_projection"])
    print(yaml.safe_dump(out["summary"], sort_keys=False), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
