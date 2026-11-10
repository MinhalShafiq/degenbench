"""Estimate the ground-truth-body-to-LiDAR extrinsic from motion (hand-eye).

Given relative motions of the ground-truth body, G_i = T_gt(t0)^-1 T_gt(t1), and
of the LiDAR between the same two scans, L_i (from scan-to-scan registration),
the extrinsic X = T_gt_lidar satisfies G_i X = X L_i.

Rotation is solved first (Wahba/Kabsch) from paired directions: translation
directions, which dominate for a driving vehicle, and rotation axes. The lever
arm then follows from (R_Gi - I) p = R t_Li - t_Gi. Planar motion leaves part of
the lever arm unobservable (vertical, for yaw-only driving); those components are
set to zero rather than fitted to noise, and reported.

Usage:
    python3 -m degenbench_converters.handeye --gt GT.txt --pairs PAIRS_DIR --out extrinsic.yaml
where PAIRS_DIR holds stamps.csv from `scan_pairs` and lidar_rel.csv from the
labeler's `register_pairs` tool.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass

import numpy as np
import yaml

from .poses import Trajectory, invert, make_transform


def rotvec(r: np.ndarray) -> np.ndarray:
    angle = np.arccos(np.clip((np.trace(r) - 1) / 2, -1, 1))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return angle / (2 * np.sin(angle)) * axis


def rot(rv: np.ndarray) -> np.ndarray:
    a = np.linalg.norm(rv)
    if a < 1e-12:
        return np.eye(3)
    k = rv / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


@dataclass
class HandEyeResult:
    T_gt_lidar: np.ndarray
    lever_singular_values: np.ndarray
    unobservable_lever_directions: np.ndarray  # rows: directions set to zero
    residual_rot_deg_median: float
    residual_trans_m_median: float
    pairs: int
    rotation_std_deg: float = float("nan")      # bootstrap, angle of deviation
    yaw_std_deg: float = float("nan")           # bootstrap, about the body z axis
    lever_std_m: np.ndarray | None = None       # bootstrap, per axis


def estimate(G: list[np.ndarray], L: list[np.ndarray], bootstrap: int = 0, seed: int = 0,
             **kwargs) -> HandEyeResult:
    """Point estimate, plus bootstrap uncertainty over resampled pairs if requested.

    The lever arm is weakly observable when per-pair rotations are small, so its
    bootstrap spread is the honest measure of how far to trust it.
    """
    result = _estimate(G, L, **kwargs)
    if bootstrap > 0:
        rng = np.random.default_rng(seed)
        rot_dev, yaw_dev, levers = [], [], []
        for _ in range(bootstrap):
            idx = rng.integers(0, len(G), len(G))
            try:
                b = _estimate([G[i] for i in idx], [L[i] for i in idx], **kwargs)
            except ValueError:
                continue
            d = b.T_gt_lidar[:3, :3] @ result.T_gt_lidar[:3, :3].T  # deviation in body frame
            rot_dev.append(np.degrees(np.linalg.norm(rotvec(d))))
            yaw_dev.append(np.degrees(np.arctan2(d[1, 0], d[0, 0])))
            levers.append(b.T_gt_lidar[:3, 3])
        result.rotation_std_deg = float(np.sqrt(np.mean(np.square(rot_dev))))
        result.yaw_std_deg = float(np.sqrt(np.mean(np.square(yaw_dev))))
        result.lever_std_m = np.std(np.array(levers), axis=0)
    return result


def _estimate(G: list[np.ndarray], L: list[np.ndarray], min_trans: float = 0.1,
              min_rot: float = np.radians(0.5), rot_weight: float = 20.0,
              observability_ratio: float = 0.2) -> HandEyeResult:
    if len(G) != len(L) or len(G) < 3:
        raise ValueError("need at least three paired motions")
    A, B = [], []
    for g, l in zip(G, L):
        if np.linalg.norm(l[:3, 3]) > min_trans:
            A.append(g[:3, 3]); B.append(l[:3, 3])
        rg, rl = rotvec(g[:3, :3]), rotvec(l[:3, :3])
        if np.linalg.norm(rl) > min_rot:
            A.append(rg * rot_weight); B.append(rl * rot_weight)
    if len(A) < 2:
        raise ValueError("motion too small to determine the rotation")
    A, B = np.array(A), np.array(B)
    U, _, Vt = np.linalg.svd(A.T @ B)
    R = U @ np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))]) @ Vt

    M = np.vstack([g[:3, :3] - np.eye(3) for g in G])
    y = np.concatenate([R @ l[:3, 3] - g[:3, 3] for g, l in zip(G, L)])
    Um, sv, Vtm = np.linalg.svd(M, full_matrices=False)
    keep = sv >= observability_ratio * sv[0]
    p = Vtm[keep].T @ ((Um[:, keep].T @ y) / sv[keep])  # minimum-norm on observable part

    T = make_transform(R, p)
    Ti = invert(T)
    res_r = [np.degrees(np.linalg.norm(rotvec((Ti @ g @ T)[:3, :3].T @ l[:3, :3]))) for g, l in zip(G, L)]
    res_t = [np.linalg.norm((Ti @ g @ T)[:3, 3] - l[:3, 3]) for g, l in zip(G, L)]
    return HandEyeResult(T, sv, Vtm[~keep], float(np.median(res_r)), float(np.median(res_t)), len(G))


def load_pairs(gt: Trajectory, stamps_csv: str, rel_csv: str):
    stamps = dict(np.loadtxt(stamps_csv, delimiter=",", ndmin=2).tolist())
    rel = np.loadtxt(rel_csv, delimiter=",", ndmin=2)
    G, L = [], []
    for row in rel:
        k = int(row[0])
        if not row[1] or k + 1 not in stamps:
            continue
        t0, t1 = stamps[k], stamps[k + 1]
        if t0 < gt.stamps[0] or t1 > gt.stamps[-1]:
            continue
        T0, T1 = gt.interpolate([t0, t1])
        G.append(invert(T0) @ T1)
        L.append(make_transform(rot(row[5:8]), row[2:5]))
    return G, L


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--gt", required=True)
    p.add_argument("--pairs", required=True)
    p.add_argument("--out", required=True)
    args = p.parse_args(argv)
    G, L = load_pairs(Trajectory.from_tum(args.gt), f"{args.pairs}/stamps.csv", f"{args.pairs}/lidar_rel.csv")
    r = estimate(G, L, bootstrap=200)
    R = r.T_gt_lidar[:3, :3]
    report = {
        "T_gt_lidar": r.T_gt_lidar.tolist(),
        "yaw_deg": float(np.degrees(np.arctan2(R[1, 0], R[0, 0]))),
        "yaw_std_deg": r.yaw_std_deg,
        "rotation_std_deg": r.rotation_std_deg,
        "lever_arm_m": r.T_gt_lidar[:3, 3].tolist(),
        "lever_arm_std_m": r.lever_std_m.tolist(),
        "lever_singular_values": r.lever_singular_values.tolist(),
        "unobservable_lever_directions_set_to_zero": r.unobservable_lever_directions.tolist(),
        "residual_per_pair": {"rotation_deg_median": r.residual_rot_deg_median,
                              "translation_m_median": r.residual_trans_m_median},
        "pairs": r.pairs,
    }
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(report, f, sort_keys=False)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
