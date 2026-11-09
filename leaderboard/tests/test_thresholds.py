from pathlib import Path

import pytest
import yaml

from degenbench_leaderboard import thresholds


def test_committed_file_loads_and_is_complete():
    config = thresholds.load()
    assert config["version"] >= 1


def test_analytic_scene_expectations_match_geometry():
    scenes = thresholds.get(thresholds.load(), "labels.pass.analytic_scenes")
    assert scenes == {"plane": 3, "corridor": 1, "cylinder": 2, "corner": 0}


def test_offset_range_can_reveal_free_directions():
    # Mirrors the labeler's guard: uniform offsets on [-b, b] have RMS b / sqrt(3),
    # which must exceed the degeneracy threshold.
    labels = thresholds.load()["labels"]
    mc, thr = labels["monte_carlo"], labels["degeneracy_threshold"]
    assert mc["max_offset_trans_m"] / 3**0.5 > thr["trans_m"]
    assert mc["max_offset_rot_deg"] / 3**0.5 > thr["rot_deg"]


def test_drift_onset_precedes_divergence():
    traj = thresholds.load()["trajectory"]
    assert traj["drift_onset_position_error_m"] < traj["divergence_position_error_m"]


def test_missing_key_is_rejected(tmp_path: Path):
    config = thresholds.load()
    del config["trajectory"]["drift_onset_position_error_m"]
    path = tmp_path / "thresholds.yaml"
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError, match="drift_onset_position_error_m"):
        thresholds.load(path)
