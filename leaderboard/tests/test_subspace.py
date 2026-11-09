import numpy as np
import pytest

from degenbench_leaderboard.subspace import principal_angles_deg, subspace_error_deg


def axes(*indices):
    basis = np.zeros((6, len(indices)))
    for col, i in enumerate(indices):
        basis[i, col] = 1.0
    return basis


def test_identical_subspaces_have_zero_error():
    assert subspace_error_deg(axes(0, 1, 5), axes(0, 1, 5)) == pytest.approx(0.0, abs=1e-9)


def test_error_ignores_basis_choice():
    rotated = axes(0, 1) @ np.array([[np.cos(0.7), -np.sin(0.7)], [np.sin(0.7), np.cos(0.7)]])
    assert subspace_error_deg(rotated, axes(0, 1)) == pytest.approx(0.0, abs=1e-9)


def test_small_angles_are_resolved():
    tilted = np.zeros((6, 1))
    tilted[0, 0], tilted[3, 0] = np.cos(np.radians(1e-4)), np.sin(np.radians(1e-4))
    assert subspace_error_deg(tilted, axes(0)) == pytest.approx(1e-4, rel=1e-6)


def test_orthogonal_subspaces_are_90_degrees():
    assert subspace_error_deg(axes(0), axes(3)) == pytest.approx(90.0)


def test_known_angle():
    tilted = np.zeros((6, 1))
    tilted[0, 0], tilted[1, 0] = np.cos(np.radians(30)), np.sin(np.radians(30))
    assert subspace_error_deg(tilted, axes(0)) == pytest.approx(30.0)


def test_dimension_mismatch_is_maximal_error():
    assert subspace_error_deg(axes(0), axes(0, 3)) == 90.0
    assert subspace_error_deg(np.zeros((6, 0)), axes(0)) == 90.0


def test_both_empty_agree():
    assert subspace_error_deg(np.zeros((6, 0)), np.zeros((6, 0))) == 0.0


def test_rank_deficient_basis_counts_true_dimension():
    duplicate = np.hstack([axes(0), axes(0)])
    assert subspace_error_deg(duplicate, axes(0)) == pytest.approx(0.0, abs=1e-6)


def test_angles_ascending():
    a = axes(0, 1)
    b = np.zeros((6, 2))
    b[0, 0] = 1.0
    b[1, 1], b[3, 1] = np.cos(np.radians(40)), np.sin(np.radians(40))
    np.testing.assert_allclose(principal_angles_deg(a, b), [0.0, 40.0], atol=1e-6)


def test_rejects_wrong_ambient_dimension():
    with pytest.raises(ValueError):
        subspace_error_deg(np.eye(3), np.eye(3))
