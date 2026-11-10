import numpy as np

from degenbench_converters.map_align import rz, search


def synthetic_site(seed=0):
    """Ground plane plus asymmetric walls and poles: a map with a unique 2D layout."""
    rng = np.random.default_rng(seed)
    pts = [np.column_stack([rng.uniform(-60, 60, 40000), rng.uniform(-60, 60, 40000), np.zeros(40000)])]
    for _ in range(25):
        x0, y0, L, a = rng.uniform(-50, 50), rng.uniform(-50, 50), rng.uniform(3, 15), rng.uniform(0, np.pi)
        s = rng.uniform(0, L, 600)
        pts.append(np.column_stack([x0 + s * np.cos(a), y0 + s * np.sin(a), rng.uniform(0, 3, 600)]))
    for _ in range(40):
        x0, y0 = rng.uniform(-55, 55, 2)
        pts.append(np.column_stack([np.full(80, x0), np.full(80, y0), rng.uniform(0, 4, 80)]))
    return np.vstack(pts)


def test_recovers_known_yaw_and_translation():
    site = synthetic_site()
    # The "world" sees the part of the site within 30 m of (10, -5), in its own frame.
    seen = site[np.linalg.norm(site[:, :2] - [10, -5], axis=1) < 30]
    yaw_true, t_true = np.radians(137.0), np.array([-20.0, 33.0])
    R = rz(yaw_true)
    world = seen.copy()
    world[:, :2] = (seen[:, :2] - t_true) @ R  # inverse of p_map = R p_world + t
    best = search(world, site)[0]
    _, yaw_deg, x, y = best
    assert abs(((yaw_deg - 137.0) + 180) % 360 - 180) <= 1.0
    # Translation is consistent with the recovered yaw to within a cell or two.
    np.testing.assert_allclose([x, y], t_true, atol=1.5)


def test_candidates_are_sorted_by_score():
    site = synthetic_site(1)
    seen = site[np.linalg.norm(site[:, :2], axis=1) < 25]
    scores = [c[0] for c in search(seen, site, top=5)]
    assert scores == sorted(scores, reverse=True)
