import numpy as np
import pytest

from degenbench_converters.poses import (
    Trajectory, apply, invert, make_transform, quat_to_rot, rot_to_quat, slerp,
)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_quaternion_round_trip():
    rng = np.random.default_rng(0)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        q = q if q[3] >= 0 else -q
        np.testing.assert_allclose(rot_to_quat(quat_to_rot(q)), q, atol=1e-12)


def test_rot_to_quat_handles_180_degrees():
    r = np.diag([1.0, -1.0, -1.0])  # 180 deg about x: trace = -1
    np.testing.assert_allclose(quat_to_rot(rot_to_quat(r)), r, atol=1e-12)


def test_invert_and_apply():
    t = make_transform(rot_z(0.4), np.array([1.0, -2.0, 0.5]))
    p = np.array([[0.3, 0.2, 0.1], [5.0, -1.0, 2.0]])
    np.testing.assert_allclose(apply(invert(t), apply(t, p)), p, atol=1e-12)


def test_slerp_midpoint_and_short_path():
    q0 = rot_to_quat(rot_z(0.0))
    q1 = rot_to_quat(rot_z(1.0))
    mid = slerp(q0, q1, [0.5])[0]
    np.testing.assert_allclose(quat_to_rot(mid), rot_z(0.5), atol=1e-12)
    # Same rotation with the opposite sign must not take the long way round.
    np.testing.assert_allclose(quat_to_rot(slerp(q0, -q1, [0.5])[0]), rot_z(0.5), atol=1e-12)


def test_trajectory_interpolation():
    traj = Trajectory(
        stamps=np.array([0.0, 1.0]),
        positions=np.array([[0, 0, 0], [2, 0, 0]], float),
        quaternions=np.stack([rot_to_quat(rot_z(0)), rot_to_quat(rot_z(1.0))]),
    )
    T = traj.interpolate([0.25])[0]
    np.testing.assert_allclose(T[:3, 3], [0.5, 0, 0])
    np.testing.assert_allclose(T[:3, :3], rot_z(0.25), atol=1e-12)
    with pytest.raises(ValueError):
        traj.interpolate([1.5])


def test_covers_detects_gaps():
    stamps = np.array([0.0, 0.1, 0.2, 1.0, 1.1])
    traj = Trajectory(stamps, np.zeros((5, 3)), np.tile([0, 0, 0, 1.0], (5, 1)))
    assert traj.covers(0.05, 0.15, max_gap=0.2)
    assert not traj.covers(0.15, 1.05, max_gap=0.2)
    assert not traj.covers(-0.1, 0.1, max_gap=0.2)


def test_tum_round_trip(tmp_path):
    traj = Trajectory(np.array([1.0, 2.0]), np.array([[1, 2, 3], [4, 5, 6]], float),
                      np.array([[0, 0, 0, 1], [0, 0, np.sin(0.2), np.cos(0.2)]]))
    path = tmp_path / "gt.txt"
    traj.to_tum(path)
    back = Trajectory.from_tum(path)
    np.testing.assert_allclose(back.positions, traj.positions)
    np.testing.assert_allclose(back.quaternions, traj.quaternions, atol=1e-9)
