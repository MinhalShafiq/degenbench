import numpy as np

from degenbench_converters.poses import Trajectory, rot_to_quat
from degenbench_converters.time_offset import estimate_offset, gt_angular_speed


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_recovers_large_shift_with_subsample_precision():
    rng = np.random.default_rng(0)
    # Smoothly varying yaw rate, as when a person turns while walking.
    t_true = np.arange(0, 120, 0.002)
    rate = 0.6 * np.sin(t_true / 3.0) + 0.4 * np.sin(t_true / 1.1 + 1) + 0.2 * rng.normal(size=1)[0]
    yaw = np.cumsum(rate) * 0.002
    # Ground truth at 5 Hz, stamped in a clock shifted by -173096893.0371 s.
    shift = 173096893.0371
    idx = np.arange(0, len(t_true), 100)
    traj = Trajectory(t_true[idx] - shift, np.zeros((len(idx), 3)),
                      np.array([rot_to_quat(rot_z(y)) for y in yaw[idx]]))
    # IMU at 200 Hz in true time with noise, starting 0.4 s before the trajectory.
    t_imu = t_true[::2] - 0.4
    w_imu = np.abs(np.interp(t_imu, t_true, rate)) + rng.normal(0, 0.02, len(t_imu))
    t_gt, w_gt = gt_angular_speed(traj)
    coarse = t_imu[0] - traj.stamps[0]  # off by the 0.4 s head start
    offset, corr = estimate_offset(t_imu, w_imu, t_gt, w_gt, coarse)
    assert abs(offset - shift) < 0.01
    assert corr > 0.9


def test_angular_speed_of_constant_rotation():
    t = np.arange(0, 2, 0.2)
    traj = Trajectory(t, np.zeros((len(t), 3)), np.array([rot_to_quat(rot_z(0.5 * x)) for x in t]))
    _, w = gt_angular_speed(traj)
    np.testing.assert_allclose(w, 0.5, atol=1e-9)
