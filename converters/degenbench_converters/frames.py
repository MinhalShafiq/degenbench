"""Extract deskewed LiDAR frames with ground-truth poses for the labeler.

Output layout (no ROS needed to read it):

    <out>/frames/000000.bin   float32 x y z, N x 3, in the LiDAR frame at the
                              frame's reference time (the message stamp)
    <out>/frames.csv          index,stamp,tx,ty,tz,qx,qy,qz,qw,num_points
                              pose T_world_lidar at the reference time
    <out>/extraction.yaml     parameters and counts, for reproducibility
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from .pointcloud import decode_fields
from .poses import Trajectory, apply, invert, rot_to_quat


@dataclass
class ExtractionConfig:
    topic: str
    rate_hz: float = 1.0
    time_field: str = "t"          # per-point time offset field (Ouster: 't', ns)
    time_scale: float = 1e-9       # seconds per unit of time_field
    min_range: float = 0.5         # drop self-hits and no-return zeros [m]
    max_gt_gap: float = 0.2        # skip frames spanning a ground-truth gap [s]
    deskew: bool = True


@dataclass
class ExtractionStats:
    scans_seen: int = 0
    frames_written: int = 0
    skipped_no_ground_truth: int = 0
    skipped_empty: int = 0
    stamps: list[float] = field(default_factory=list)


def deskew(points: np.ndarray, times: np.ndarray, t_ref: float, trajectory: Trajectory,
           T_gt_lidar: np.ndarray) -> np.ndarray:
    """Express points measured at `times` in the LiDAR frame at `t_ref`.

    p_ref = T_world_lidar(t_ref)^-1 * T_world_lidar(t_i) * p_i, with
    T_world_lidar(t) = T_world_gt(t) * T_gt_lidar.
    """
    uniq, inverse = np.unique(times, return_inverse=True)
    poses = trajectory.interpolate(np.concatenate([[t_ref], uniq])) @ T_gt_lidar
    T_ref_inv = invert(poses[0])
    out = np.empty_like(points)
    for k, T in enumerate(poses[1:]):
        sel = inverse == k
        out[sel] = apply(T_ref_inv @ T, points[sel])
    return out


def extract_frames(bag: str | Path, trajectory: Trajectory, T_gt_lidar: np.ndarray,
                   out_dir: str | Path, config: ExtractionConfig) -> ExtractionStats:
    out_dir = Path(out_dir)
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    stats = ExtractionStats()
    rows = []
    period = 1.0 / config.rate_hz
    next_t = -np.inf

    typestore = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(bag)], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic == config.topic]
        if not conns:
            raise ValueError(f"{bag}: topic {config.topic} not found")
        for conn, _, raw in reader.messages(connections=conns):
            stats.scans_seen += 1
            msg = reader.deserialize(raw, conn.msgtype)
            t_ref = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
            if t_ref < next_t:
                continue

            names = ["x", "y", "z"] + ([config.time_field] if config.deskew else [])
            f = decode_fields(msg, names)
            pts = np.column_stack([f["x"], f["y"], f["z"]])
            times = t_ref + (f[config.time_field] * config.time_scale if config.deskew
                             else np.zeros(len(pts)))
            keep = np.all(np.isfinite(pts), axis=1) & (np.linalg.norm(pts, axis=1) >= config.min_range)
            pts, times = pts[keep], times[keep]
            if len(pts) == 0:
                stats.skipped_empty += 1
                continue

            t_lo, t_hi = min(t_ref, times.min()), max(t_ref, times.max())
            if not trajectory.covers(t_lo, t_hi, config.max_gt_gap):
                stats.skipped_no_ground_truth += 1
                continue

            if config.deskew:
                pts = deskew(pts, times, t_ref, trajectory, T_gt_lidar)
            T_world_lidar = trajectory.interpolate([t_ref])[0] @ T_gt_lidar

            idx = stats.frames_written
            pts.astype("<f4").tofile(out_dir / "frames" / f"{idx:06d}.bin")
            q = rot_to_quat(T_world_lidar[:3, :3])
            rows.append([idx, t_ref, *T_world_lidar[:3, 3], *q, len(pts)])
            stats.frames_written += 1
            stats.stamps.append(t_ref)
            next_t = t_ref + period - 1e-6

    with open(out_dir / "frames.csv", "w", encoding="utf-8") as f:
        f.write("index,stamp,tx,ty,tz,qx,qy,qz,qw,num_points\n")
        for r in rows:
            f.write(f"{r[0]},{r[1]:.9f}," + ",".join(f"{v:.9f}" for v in r[2:9]) + f",{r[9]}\n")
    with open(out_dir / "extraction.yaml", "w", encoding="utf-8") as f:
        meta = {"bag": str(bag), "config": asdict(config),
                "T_gt_lidar": T_gt_lidar.tolist(),
                "stats": {k: v for k, v in asdict(stats).items() if k != "stamps"}}
        yaml.safe_dump(meta, f, sort_keys=False)
    return stats
