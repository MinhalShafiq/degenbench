"""Estimate the clock offset between a ground-truth trajectory and a bag.

Some releases shift ground-truth timestamps (e.g. SubT-MRS challenge
trajectories carry 2018 stamps for 2023 recordings). The angular speed |w| is
independent of how the IMU is mounted, so the gyro magnitude from the bag and
the angular speed differentiated from the ground truth are cross-correlated;
the peak gives `offset` such that t_bag = t_gt + offset.

    python3 -m degenbench_converters.time_offset --bag BAG --imu-topic /imu/data \\
        --gt GT.txt [--out-gt shifted.txt]

The coarse offset is the difference of start times; the search covers +-window
seconds around it at `rate` Hz, refined to sub-sample precision by a parabola.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from .frames import open_bags
from .poses import Trajectory, quat_to_rot_batch


def gt_angular_speed(traj: Trajectory) -> tuple[np.ndarray, np.ndarray]:
    R = quat_to_rot_batch(traj.quaternions)
    rel = np.einsum("nji,njk->nik", R[:-1], R[1:])  # R_i^T R_{i+1}
    angle = np.arccos(np.clip((np.trace(rel, axis1=1, axis2=2) - 1) / 2, -1, 1))
    dt = np.diff(traj.stamps)
    return 0.5 * (traj.stamps[:-1] + traj.stamps[1:]), angle / dt


def estimate_offset(t_imu: np.ndarray, w_imu: np.ndarray, t_gt: np.ndarray, w_gt: np.ndarray,
                    coarse: float, window: float = 5.0, rate: float = 100.0) -> tuple[float, float]:
    """Return (offset, normalized correlation at the peak); t_bag = t_gt + offset."""
    # Smooth the IMU to the ground truth's bandwidth before comparing.
    gt_dt = float(np.median(np.diff(t_gt)))
    k = max(1, int(round(gt_dt * rate)))
    grid = np.arange(t_imu[0], t_imu[-1], 1.0 / rate)
    imu = np.convolve(np.interp(grid, t_imu, w_imu), np.ones(k) / k, mode="same")
    lags = np.arange(-window, window + 0.5 / rate, 1.0 / rate)
    scores = np.full(len(lags), -np.inf)
    for i, lag in enumerate(lags):
        t = t_gt + coarse + lag
        ok = (t > grid[0]) & (t < grid[-1])
        if ok.sum() < 20:
            continue
        a = np.interp(t[ok], grid, imu)
        b = w_gt[ok]
        a, b = a - a.mean(), b - b.mean()
        denom = np.linalg.norm(a) * np.linalg.norm(b)
        scores[i] = a @ b / denom if denom > 0 else -np.inf
    j = int(np.argmax(scores))
    lag = lags[j]
    if 0 < j < len(lags) - 1 and np.isfinite(scores[j - 1]) and np.isfinite(scores[j + 1]):
        y0, y1, y2 = scores[j - 1], scores[j], scores[j + 1]
        denom = y0 - 2 * y1 + y2
        if denom < 0:
            lag += 0.5 * (y0 - y2) / denom / rate
    return coarse + lag, float(scores[j])


def read_gyro(bag, topic: str) -> tuple[np.ndarray, np.ndarray]:
    t, w = [], []
    with open_bags(bag) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise ValueError(f"topic {topic} not found")
        for conn, _, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            t.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
            av = m.angular_velocity
            w.append(np.sqrt(av.x ** 2 + av.y ** 2 + av.z ** 2))
    t, w = np.array(t), np.array(w)
    order = np.argsort(t)
    return t[order], w[order]


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", required=True, nargs="+")
    p.add_argument("--imu-topic", required=True)
    p.add_argument("--gt", required=True)
    p.add_argument("--out-gt", help="write the ground truth with stamps shifted into bag time")
    p.add_argument("--window", type=float, default=5.0)
    a = p.parse_args(argv)
    bag = a.bag if len(a.bag) > 1 else a.bag[0]
    t_imu, w_imu = read_gyro(bag, a.imu_topic)
    traj = Trajectory.from_tum(a.gt)
    t_gt, w_gt = gt_angular_speed(traj)
    coarse = t_imu[0] - traj.stamps[0]
    offset, corr = estimate_offset(t_imu, w_imu, t_gt, w_gt, coarse, a.window)
    print(f"coarse offset {coarse:.6f} s, refined {offset:.6f} s "
          f"(refinement {offset - coarse:+.3f} s), peak correlation {corr:.3f}")
    if a.out_gt:
        Trajectory(traj.stamps + offset, traj.positions, traj.quaternions).to_tum(Path(a.out_gt))
        print(f"wrote {a.out_gt}")
    return 0 if corr > 0.5 else 1


if __name__ == "__main__":
    sys.exit(main())
