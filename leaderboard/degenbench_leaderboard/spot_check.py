"""Render frames for the pre-registered visual spot check of the labels.

    python3 -m degenbench_leaderboard.spot_check --labels RELEASE_DIR \\
        --features FEATURES.csv --frames FRAMES_DIR --map MAP.bin --out DIR \\
        [--consensus-free N] [--consensus-fixed N] [--disputed N] [--seed 0]

For each sampled frame, writes `<index>_<status>.png`, a top view (x forward,
y left) and a side view (x forward, z up) in the sensor frame, 30 m across:
the map in grey, the scan in blue, and each labelled free direction as a line
through the sensor. The line is red for the consensus label (or ICP on
disputed frames) and green for GICP on disputed frames. Free directions that
are mostly rotational are listed in `index.csv`, since they have no line.

Map: float32 x y z (labeler/tools/map_to_bin). Scans: the frames directory of
extract_frames. Reference poses: the ref_T columns of detector_features.
All inputs are local and derived from the dataset; outputs are for review only.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
import zlib
from pathlib import Path

import numpy as np

from .detectors import SCALE
from .labels import CONSENSUS, DISPUTED, load_release

HALF_M = 15.0
PX = 500
GREY, BLUE, RED, GREEN, BLACK = (170, 170, 170), (30, 90, 220), (220, 30, 30), (20, 160, 40), (0, 0, 0)


def write_png(path: Path, img: np.ndarray) -> None:
    h, w, _ = img.shape
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))
    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    path.write_bytes(png)


def _to_px(u: np.ndarray, v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    col = ((u + HALF_M) / (2 * HALF_M) * (PX - 1)).round().astype(int)
    row = ((HALF_M - v) / (2 * HALF_M) * (PX - 1)).round().astype(int)
    return row, col


def _plot(img: np.ndarray, u: np.ndarray, v: np.ndarray, color) -> None:
    r, c = _to_px(u, v)
    ok = (r >= 0) & (r < PX) & (c >= 0) & (c < PX)
    img[r[ok], c[ok]] = color


def _line(img: np.ndarray, d: np.ndarray, color, length: float = 12.0) -> None:
    t = np.linspace(-length, length, 2000)
    for off in (-0.06, 0.0, 0.06):  # 3 px wide
        n = np.array([-d[1], d[0]])
        _plot(img, t * d[0] + off * n[0], t * d[1] + off * n[1], color)


def render(map_s: np.ndarray, scan: np.ndarray, lines: list[tuple[np.ndarray, tuple]]) -> np.ndarray:
    panels = []
    for a, b in ((0, 1), (0, 2)):
        img = np.full((PX, PX, 3), 255, np.uint8)
        _plot(img, map_s[:, a], map_s[:, b], GREY)
        _plot(img, scan[:, a], scan[:, b], BLUE)
        for d3, color in lines:
            d = np.array([d3[a], d3[b]])
            if np.linalg.norm(d) > 0.3:
                _line(img, d / np.linalg.norm(d), color)
        r, c = _to_px(np.array([0.0]), np.array([0.0]))
        img[r[0] - 3:r[0] + 4, c[0] - 3:c[0] + 4] = BLACK
        panels.append(img)
    sep = np.zeros((PX, 6, 3), np.uint8)
    return np.concatenate([panels[0], sep, panels[1]], axis=1)


def describe(basis: np.ndarray) -> tuple[list[np.ndarray], list[str]]:
    """Translation line directions (sensor frame) and a text label per free direction."""
    lines, text = [], []
    for c in range(basis.shape[1]):
        phys = basis[:, c] * SCALE
        t, r = phys[:3], phys[3:]
        # Compare in normalized units: which part dominates the label direction?
        if np.linalg.norm(basis[:3, c]) >= np.linalg.norm(basis[3:, c]):
            lines.append(t / np.linalg.norm(t))
            text.append("t(" + ",".join(f"{x:+.2f}" for x in t / np.linalg.norm(t)) + ")")
        else:
            text.append("r(" + ",".join(f"{x:+.2f}" for x in r / np.linalg.norm(r)) + ")")
    return lines, text


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--labels", required=True)
    p.add_argument("--features", required=True)
    p.add_argument("--frames", required=True)
    p.add_argument("--map", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--consensus-free", type=int, default=0)
    p.add_argument("--consensus-fixed", type=int, default=0)
    p.add_argument("--disputed", type=int, default=0)
    p.add_argument("--seed", type=int, default=0)
    a = p.parse_args(argv)

    frames = load_release(a.labels)
    poses = {}
    with open(a.features, newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            T = np.eye(4)
            T[:3, :] = [[float(r[f"ref_T{i}{j}"]) for j in range(4)] for i in range(3)]
            poses[int(r["index"])] = T
    rng = np.random.default_rng(a.seed)
    strata = {
        "consensus_free": [f for f in frames if f.status == CONSENSUS and f.k > 0],
        "consensus_fixed": [f for f in frames if f.status == CONSENSUS and f.k == 0],
        "disputed": [f for f in frames if f.status == DISPUTED],
    }
    want = {"consensus_free": a.consensus_free, "consensus_fixed": a.consensus_fixed, "disputed": a.disputed}
    world = np.fromfile(a.map, dtype=np.float32).reshape(-1, 3).astype(float)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for stratum, pool in strata.items():
        n = min(want[stratum], len(pool))
        for i in sorted(rng.choice(len(pool), size=n, replace=False)) if n else []:
            fr = pool[i]
            T = poses[fr.index]
            local = world[np.linalg.norm(world - T[:3, 3], axis=1) < 1.5 * HALF_M]
            map_s = (local - T[:3, 3]) @ T[:3, :3]  # into the sensor frame
            scan = np.fromfile(Path(a.frames) / "frames" / f"{fr.index:06d}.bin",
                               dtype=np.float32).reshape(-1, 3).astype(float)
            lines, text = [], []
            if fr.status == CONSENSUS:
                l, t = describe(fr.basis)
                lines += [(d, RED) for d in l]
                text.append("consensus " + " ".join(t))
            else:
                for solver, color in (("icp", RED), ("gicp", GREEN)):
                    l, t = describe(fr.solver_basis[solver])
                    lines += [(d, color) for d in l]
                    text.append(f"{solver} " + (" ".join(t) or "none"))
            name = f"{fr.index:06d}_{stratum}.png"
            write_png(out / name, render(map_s, scan, lines))
            rows.append({"image": name, "index": fr.index, "stratum": stratum, "label": "; ".join(text)})
    with open(out / "index.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["image", "index", "stratum", "label"], lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {len(rows)} frames to {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
