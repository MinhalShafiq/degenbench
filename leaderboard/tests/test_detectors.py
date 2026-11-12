import numpy as np

from degenbench_leaderboard.detectors import (
    DETECTORS, SCALE, Features, generalized_eigh, predict, to_normalized, write_predictions,
)
from degenbench_leaderboard.score import read_predictions
from degenbench_leaderboard.subspace import subspace_error_deg


def corridor_features(n=4000, seed=0):
    """Points on the walls, floor and ceiling of a corridor along x, sensor inside."""
    rng = np.random.default_rng(seed)
    x = rng.uniform(-8, 8, n)
    side = rng.integers(0, 4, n)
    p = np.zeros((n, 3))
    nrm = np.zeros((n, 3))
    u = rng.uniform(-1, 1, n)
    for s, (axis, value, other) in enumerate([(1, -1.5, 2), (1, 1.5, 2), (2, -1.2, 1), (2, 1.3, 1)]):
        m = side == s
        p[m, 0] = x[m]
        p[m, axis] = value
        p[m, other] = u[m] * (1.2 if other == 2 else 1.5)
        nrm[m, axis] = -np.sign(value)
    J = np.hstack([nrm, np.cross(p, nrm)])
    H = J.T @ J
    M = np.zeros((6, 6))
    for q in p:
        P = np.array([[0, -q[2], q[1]], [q[2], 0, -q[0]], [-q[1], q[0], 0]])
        D = np.hstack([np.eye(3), -P])
        M += D.T @ D
    wt, vt = np.linalg.eigh(H[:3, :3])
    wr, vr = np.linalg.eigh(H[3:, 3:])
    tau = np.cross(p, nrm)
    tau_u = tau / np.maximum(1.0, np.linalg.norm(tau, axis=1))[:, None]
    def sums(contrib):
        c = np.where(contrib >= np.cos(np.radians(80)), contrib, 0)
        return c.sum(0), np.where(c >= np.cos(np.radians(45)), c, 0).sum(0)
    lct, lst = sums(np.abs(nrm @ vt))
    lcr, lsr = sums(np.abs(tau_u @ vr))
    return Features(0, 0.0, True, 0.0, 0.0, n, H, M, vt, vr,
                    np.concatenate([lct, lcr]), np.concatenate([lst, lsr]), 1.0, 0.1)


TX = np.eye(6)[:, :1]


def test_generalized_eigh_solves_the_pencil():
    rng = np.random.default_rng(1)
    A = rng.normal(size=(6, 6)); H = A @ A.T
    B = rng.normal(size=(6, 6)); M = B @ B.T + 6 * np.eye(6)
    w, v = generalized_eigh(H, M)
    np.testing.assert_allclose(H @ v, M @ v * w, atol=1e-8)
    assert np.all(np.diff(w) >= 0)


def test_normalization_divides_by_threshold_units():
    v = np.array([[0.1, 0, 0, np.radians(2.0), 0, 0]]).T
    np.testing.assert_allclose(to_normalized(v)[:, 0], [1, 0, 0, 1, 0, 0])
    assert SCALE[0] == 0.10


def test_every_detector_finds_the_corridor_axis():
    f = corridor_features()
    w = np.linalg.eigvalsh(f.H)
    taus = {"min_eigenvalue": np.sqrt(w[0] * w[1]) if w[0] > 0 else 1e-6,
            "condition_number": 1e6, "x_icp": 1.0, "dcreg": 50.0, "frame_independent": 1e-2}
    for name, det in DETECTORS.items():
        pred = predict(det, f, taus[name])
        assert pred.shape[1] == 1, name
        assert subspace_error_deg(pred, TX) < 1.0, name


def test_predictions_roundtrip(tmp_path):
    b = np.array([[1.0, 2.0, 0, 0, 0, 0], [0, 0, 0, 0, 3.0, 0]]).T
    write_predictions(tmp_path / "p.csv", {3: b, 4: np.zeros((6, 0))})
    back = read_predictions(tmp_path / "p.csv")
    assert back[4].shape == (6, 0)
    assert subspace_error_deg(back[3], b) < 1e-5


def test_roles_come_from_the_thresholds_in_force():
    import pytest
    from degenbench_leaderboard import thresholds
    from degenbench_leaderboard.leaderboard import sequence_role
    config = thresholds.load()
    dev = config["labels"]["evaluation"]["development_sequences"]
    held = config["labels"]["evaluation"]["held_out_sequences"]
    assert not set(dev) & set(held)
    assert sequence_role(config, dev[0]) == "development"
    assert sequence_role(config, held[0]) == "held_out"
    with pytest.raises(ValueError):
        sequence_role(config, "nowhere/none")
