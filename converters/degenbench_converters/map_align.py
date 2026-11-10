"""Coarse global alignment of a ground-truth world frame to a survey map frame.

When a dataset's ground truth and survey map are in unrelated frames, the
frames extracted along the ground truth are accumulated into one cloud and
matched to the map in 2D: both are reduced to occupancy images of vertical
structure (cells whose points span more than `span_m` in height), every yaw is
tried, and FFT cross-correlation gives the best translation per yaw. The top
candidates are refined in 3D by the labeler's `refine_alignment` tool.

    python3 -m degenbench_converters.map_align --frames FRAMES_DIR --map MAP.bin \\
        --out-cloud accum.bin --out-candidates candidates.txt

MAP.bin is float32 x y z (the labeler's `map_to_bin` writes it from a PCD).
Candidates are rows of: score, yaw_deg, x, y, meaning p_map = Rz(yaw) p_world + (x, y, 0).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from .poses import quat_to_rot_batch


def accumulate(frames_dir: Path, max_range: float = 40.0, stride: int = 2) -> np.ndarray:
    """All frames placed at their ground-truth poses, in the world frame."""
    rows = np.loadtxt(frames_dir / "frames.csv", delimiter=",", skiprows=1, ndmin=2)
    rots = quat_to_rot_batch(rows[:, 5:9])
    out = []
    for r, R in zip(rows, rots):
        p = np.fromfile(frames_dir / "frames" / f"{int(r[0]):06d}.bin", dtype="<f4").reshape(-1, 3)
        p = p[np.linalg.norm(p, axis=1) < max_range][::stride].astype(float)
        out.append(p @ R.T + r[2:5])
    return np.vstack(out)


def structure_image(xyz: np.ndarray, origin: np.ndarray, shape: tuple[int, int],
                    cell: float, span: float) -> np.ndarray:
    ij = np.floor((xyz[:, :2] - origin) / cell).astype(int)
    ok = (ij[:, 0] >= 0) & (ij[:, 1] >= 0) & (ij[:, 0] < shape[0]) & (ij[:, 1] < shape[1])
    ij, z = ij[ok], xyz[ok, 2]
    flat = ij[:, 0] * shape[1] + ij[:, 1]
    zmin = np.full(shape[0] * shape[1], np.inf)
    zmax = np.full_like(zmin, -np.inf)
    np.minimum.at(zmin, flat, z)
    np.maximum.at(zmax, flat, z)
    return ((zmax - zmin) > span).reshape(shape).astype(np.float32)


def rz(yaw_rad: float) -> np.ndarray:
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    return np.array([[c, -s], [s, c]])


def search(world: np.ndarray, map_xyz: np.ndarray, cell: float = 0.5, span: float = 1.0,
           yaw_step_deg: float = 1.0, top: int = 8) -> list[tuple[float, float, float, float]]:
    """Return top (score, yaw_deg, x, y) with p_map = Rz(yaw) p_world + (x, y)."""
    center = world[:, :2].mean(0)
    local = world.copy()
    local[:, :2] -= center
    m_origin = map_xyz[:, :2].min(0)
    m_shape = tuple((np.ceil((map_xyz[:, :2].max(0) - m_origin) / cell) + 1).astype(int))
    M = structure_image(map_xyz, m_origin, m_shape, cell, span)
    half = int(np.ceil(np.abs(local[:, :2]).max() / cell)) + 2
    H, W = M.shape[0] + 2 * half, M.shape[1] + 2 * half
    FM = np.fft.rfft2(M, s=(H, W))
    results = []
    for yaw_deg in np.arange(0.0, 360.0, yaw_step_deg):
        R = rz(np.radians(yaw_deg))
        xyz = local.copy()
        xyz[:, :2] = local[:, :2] @ R.T
        S = structure_image(xyz, np.array([-half * cell, -half * cell]), (2 * half, 2 * half), cell, span)
        n = S.sum()
        if n == 0:
            continue
        corr = np.fft.irfft2(FM * np.conj(np.fft.rfft2(S, s=(H, W))), s=(H, W))
        k = np.unravel_index(np.argmax(corr), corr.shape)
        shift = np.array([k[0] if k[0] < H // 2 else k[0] - H, k[1] if k[1] < W // 2 else k[1] - W])
        t = m_origin + shift * cell + half * cell - R @ center
        results.append((float(corr[k] / n), float(yaw_deg), float(t[0]), float(t[1])))
    results.sort(key=lambda r: -r[0])
    return results[:top]


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--frames", required=True)
    p.add_argument("--map", required=True, help="float32 x y z map (e.g. from map_to_bin)")
    p.add_argument("--out-cloud", required=True)
    p.add_argument("--out-candidates", required=True)
    p.add_argument("--cell", type=float, default=0.5)
    p.add_argument("--span", type=float, default=1.0)
    a = p.parse_args(argv)
    world = accumulate(Path(a.frames))
    world.astype("<f4").tofile(a.out_cloud)
    mp = np.fromfile(a.map, dtype="<f4").reshape(-1, 3).astype(float)
    cands = search(world, mp, a.cell, a.span)
    np.savetxt(a.out_candidates, np.array(cands), fmt="%.6f",
               header="score yaw_deg x y   (p_map = Rz(yaw) p_world + [x, y, 0])")
    for c in cands:
        print("score %.3f  yaw %6.1f  x %9.2f  y %9.2f" % c)
    return 0


if __name__ == "__main__":
    sys.exit(main())
