import csv
from pathlib import Path

import numpy as np
import pytest

from degenbench_leaderboard import thresholds
from degenbench_leaderboard.labels import (
    CONSENSUS, DISPUTED, UNRELIABLE, AXES, SOLVERS, build_frames, consensus_subspace, free_axes,
    load_release, write_release,
)
from degenbench_leaderboard.score import read_predictions, score
from degenbench_leaderboard.subspace import subspace_error_deg

E = np.eye(6)


def rot_in_plane(a, b, deg):
    """Unit vector rotated from axis a towards axis b by `deg`."""
    t = np.radians(deg)
    return (np.cos(t) * E[:, a] + np.sin(t) * E[:, b]).reshape(6, 1)


def test_consensus_of_identical_subspaces_is_that_subspace():
    b = np.stack([E[:, 0], E[:, 5]], axis=1)
    assert subspace_error_deg(consensus_subspace(b, b), b) < 1e-9


def test_consensus_bisects_two_lines():
    c = consensus_subspace(rot_in_plane(0, 1, 0), rot_in_plane(0, 1, 10))
    assert subspace_error_deg(c, rot_in_plane(0, 1, 5)) < 1e-6
    assert c[np.argmax(np.abs(c[:, 0])), 0] > 0  # deterministic sign


def test_consensus_rejects_dimension_mismatch():
    with pytest.raises(ValueError):
        consensus_subspace(E[:, :1], E[:, :2])


def test_free_axes_rule():
    assert list(free_axes(rot_in_plane(0, 1, 30), 0.5)) == [True, False, False, False, False, False]
    assert not free_axes(rot_in_plane(0, 1, 45), 0.5).any()  # split evenly: neither axis
    assert not free_axes(np.zeros((6, 0)), 0.5).any()


# ---------------------------------------------------------------------------
# A synthetic labeler output with one frame of each kind
# ---------------------------------------------------------------------------

def _eigvecs_with_top(direction):
    """Orthonormal 6x6 matrix whose last column is `direction` (eigenvalues ascending)."""
    q, _ = np.linalg.qr(np.hstack([direction, np.random.default_rng(0).normal(size=(6, 5))]))
    q = q[:, [1, 2, 3, 4, 5, 0]]
    q[:, 5] *= np.sign(q[:, 5] @ direction[:, 0])
    return q


def write_labeler_output(tmp: Path) -> Path:
    # frame: (refine_converged, icp_k, gicp_k, icp direction, gicp direction)
    frames = {
        0: (1, 1, 1, rot_in_plane(0, 1, 0), rot_in_plane(0, 1, 6)),    # consensus along tx
        1: (1, 0, 0, None, None),                                      # consensus, none free
        2: (1, 0, 1, None, rot_in_plane(3, 4, 0)),                     # disputed
        3: (0, 1, 1, rot_in_plane(0, 1, 0), rot_in_plane(0, 1, 0)),    # unreliable reference
    }
    cols = (["index", "stamp", "map_points", "coverage", "refine_converged", "refine_trans_m",
             "refine_rot_deg", "icp_k", "gicp_k", "subspace_angle_deg", "solvers_agree", "icp_axes",
             "gicp_axes"] + [f"{s}_eig{i}" for s in SOLVERS for i in range(6)]
            + [f"{s}_rms_{a}" for s in SOLVERS for a in AXES])
    out, sub = tmp / "seq.csv", tmp / "seq_subspaces.csv"
    with open(out, "w", newline="") as f, open(sub, "w", newline="") as g:
        w, ws = csv.writer(f), csv.writer(g)
        w.writerow(cols)
        ws.writerow(["index", "solver"] + [f"v{c}{r}" for c in range(6) for r in range(6)])
        for idx, (conv, ki, kg, di, dg) in frames.items():
            mats = {"icp": _eigvecs_with_top(di if di is not None else rot_in_plane(2, 3, 0)),
                    "gicp": _eigvecs_with_top(dg if dg is not None else rot_in_plane(2, 3, 0))}
            bases = {"icp": mats["icp"][:, 6 - ki:], "gicp": mats["gicp"][:, 6 - kg:]}
            angle = subspace_error_deg(bases["icp"], bases["gicp"])
            w.writerow([idx, 100.0 + idx, 1000, 0.99, conv, 0.05, 0.4, ki, kg, f"{angle:.6f}",
                        int(angle <= 15), "000000", "000000"] + [0.1] * 12 + [0.01] * 12)
            for s in SOLVERS:
                ws.writerow([idx, s] + [mats[s][r, c] for c in range(6) for r in range(6)])
    return out


@pytest.fixture
def release(tmp_path):
    csv_path = write_labeler_output(tmp_path)
    frames, rows = build_frames(csv_path, tmp_path / "seq_subspaces.csv", 15.0)
    meta = write_release(frames, rows, tmp_path / "rel", {"sequence": "synthetic"}, 0.5)
    return tmp_path / "rel", meta


def test_release_status_and_summary(release):
    directory, meta = release
    frames = load_release(directory)
    assert [f.status for f in frames] == [CONSENSUS, CONSENSUS, DISPUTED, UNRELIABLE]
    assert meta["summary"]["status_counts"] == {CONSENSUS: 2, DISPUTED: 1, UNRELIABLE: 1}
    assert meta["summary"]["solver_agreement_rate"] == pytest.approx(2 / 3, abs=1e-4)
    assert subspace_error_deg(frames[0].basis, rot_in_plane(0, 1, 3)) < 1e-4
    rows = list(csv.DictReader(open(directory / "labels.csv")))
    assert rows[0]["consensus_axes"] == "100000" and rows[2]["k"] == ""


def test_release_detects_tampering(release):
    directory, _ = release
    with open(directory / "labels.csv", "a") as f:
        f.write("\n")
    with pytest.raises(ValueError, match="checksum"):
        load_release(directory)


def test_agreement_mismatch_with_labeler_is_an_error(tmp_path):
    csv_path = write_labeler_output(tmp_path)
    text = csv_path.read_text().splitlines()
    parts = text[1].split(",")
    parts[10] = "0"  # frame 0 agrees; claim it does not
    text[1] = ",".join(parts)
    csv_path.write_text("\n".join(text) + "\n")
    with pytest.raises(ValueError, match="frame 0"):
        build_frames(csv_path, tmp_path / "seq_subspaces.csv", 15.0)


def _write_predictions(path: Path, preds: dict[int, np.ndarray]):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["index", "k"] + [f"v{c}{r}" for c in range(6) for r in range(6)])
        for idx, b in preds.items():
            full = [""] * 36
            for c in range(b.shape[1]):
                for r in range(6):
                    full[c * 6 + r] = b[r, c]
            w.writerow([idx, b.shape[1]] + full)


def test_perfect_detector_scores_perfectly(release, tmp_path):
    directory, _ = release
    frames = load_release(directory)
    preds = {f.index: (f.basis if f.basis is not None else np.zeros((6, 0))) for f in frames}
    _write_predictions(tmp_path / "p.csv", preds)
    s = score(frames, read_predictions(tmp_path / "p.csv"), thresholds.load())
    c = s["consensus"]
    assert c["subspace_error_deg"]["match_rate"] == 1.0 and c["subspace_error_deg"]["mean"] < 1e-4
    assert c["frame_detection"]["f1"] == 1.0
    assert c["per_direction"]["tx"]["f1"] == 1.0 and c["per_direction_micro"]["fp"] == 0
    assert s["missing_predictions"] == 0
    # Disputed frame 2: k = 0 matches ICP, not GICP.
    assert s["disputed"]["subspace_error_deg_vs"]["icp"]["match_rate"] == 1.0
    assert s["disputed"]["subspace_error_deg_vs"]["gicp"]["match_rate"] == 0.0


def test_silent_detector_and_missing_frames(release, tmp_path):
    directory, _ = release
    frames = load_release(directory)
    _write_predictions(tmp_path / "p.csv", {1: np.zeros((6, 0))})
    s = score(frames, read_predictions(tmp_path / "p.csv"), thresholds.load())
    assert s["missing_predictions"] == 3
    c = s["consensus"]
    assert c["frame_detection"]["recall"] == 0.0 and c["frame_detection"]["f1"] == 0.0
    assert c["subspace_error_deg"]["match_rate"] == 0.5  # right on frame 1, 90 deg on frame 0
    assert c["per_direction"]["tx"]["fn"] == 1


def test_scoring_rules_are_pre_registered():
    config = thresholds.load()
    assert config["version"] >= 3
    assert thresholds.get(config, "leaderboard.scoring.headline_frames") == "consensus"
    assert "frame_detection_f1" in config["leaderboard"]["metrics"]
