import numpy as np
import pytest

from degenbench_converters.handeye import estimate, rot, rotvec
from degenbench_converters.poses import invert, make_transform

# A rig like FusionPortableV2's UGV: ground-truth body yawed ~86 deg from the LiDAR.
X_TRUE = make_transform(rot(np.radians([1.0, -0.5, -85.8])), np.array([0.18, -0.10, 0.35]))


def planar_drive(n=80, seed=0, noise_t=0.0, noise_r=0.0):
    """Forward driving with turns: body moves along its -y axis, yaws about z."""
    rng = np.random.default_rng(seed)
    G, L = [], []
    for i in range(n):
        yaw = np.radians(8.0 * np.sin(i / 7.0)) * rng.uniform(0.5, 1.0)
        g = make_transform(rot(np.array([0, 0, yaw]) * 0.1), np.array([0.02, -0.4, 0.0]))
        l = invert(X_TRUE) @ g @ X_TRUE
        l = make_transform(rot(rng.normal(0, noise_r, 3)) @ l[:3, :3], l[:3, 3] + rng.normal(0, noise_t, 3))
        G.append(g); L.append(l)
    return G, L


def test_recovers_rotation_and_planar_lever_arm():
    G, L = planar_drive()
    r = estimate(G, L)
    R_err = np.degrees(np.linalg.norm(rotvec(r.T_gt_lidar[:3, :3].T @ X_TRUE[:3, :3])))
    assert R_err < 0.05
    np.testing.assert_allclose(r.T_gt_lidar[:2, 3], X_TRUE[:2, 3], atol=1e-3)


def test_vertical_lever_arm_is_reported_unobservable_not_fitted():
    G, L = planar_drive()
    r = estimate(G, L)
    # Yaw-only motion cannot see the vertical offset: it must be zeroed and flagged.
    assert r.T_gt_lidar[2, 3] == pytest.approx(0.0, abs=1e-6)
    assert len(r.unobservable_lever_directions) == 1
    assert abs(r.unobservable_lever_directions[0][2]) > 0.99


def test_rotation_robust_to_registration_noise():
    G, L = planar_drive(n=200, noise_t=0.02, noise_r=np.radians(0.2), seed=3)
    r = estimate(G, L)
    R_err = np.degrees(np.linalg.norm(rotvec(r.T_gt_lidar[:3, :3].T @ X_TRUE[:3, :3])))
    assert R_err < 0.5


def test_bootstrap_uncertainty_covers_the_truth():
    # Small per-pair yaw makes the lever arm noise-sensitive; the estimator must
    # say so rather than report a falsely precise value.
    G, L = planar_drive(n=200, noise_t=0.02, noise_r=np.radians(0.2), seed=3)
    r = estimate(G, L, bootstrap=100)
    err = np.abs(r.T_gt_lidar[:2, 3] - X_TRUE[:2, 3])
    assert np.all(err < 3 * r.lever_std_m[:2] + 1e-3)
    R_err = np.degrees(np.linalg.norm(rotvec(r.T_gt_lidar[:3, :3].T @ X_TRUE[:3, :3])))
    assert R_err < 3 * r.rotation_std_deg
    # Driving forward pins yaw down far better than tilt about the driving axis.
    assert r.yaw_std_deg < 0.3 < r.rotation_std_deg
    # Noise-free data: the spread collapses.
    G0, L0 = planar_drive()
    b0 = estimate(G0, L0, bootstrap=50)
    assert np.all(b0.lever_std_m[:2] < 1e-2) and b0.rotation_std_deg < 0.05


def test_rejects_too_little_motion():
    still = [np.eye(4)] * 5
    with pytest.raises(ValueError):
        estimate(still, still)
