"""Dump consecutive LiDAR scan pairs (k, k+1) every N scans for hand-eye calibration.

Writes <out>/<k>.bin (float32 x y z, raw, not deskewed) and <out>/stamps.csv
(scan index, header stamp). Register the pairs with the labeler's
`register_pairs` tool, then run `degenbench_converters.handeye`.

    python3 -m degenbench_converters.scan_pairs --bag BAG --out DIR [--every 10]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from .pointcloud import decode_fields


def dump_pairs(bag: Path, out: Path, topic: str, every: int, min_range: float, max_range: float) -> int:
    out.mkdir(parents=True, exist_ok=True)
    stamps = []
    with AnyReader([bag], default_typestore=get_typestore(Stores.ROS1_NOETIC)) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            raise ValueError(f"{bag}: topic {topic} not found")
        for k, (conn, _, raw) in enumerate(reader.messages(connections=conns)):
            if k % every not in (0, 1):
                continue
            msg = reader.deserialize(raw, conn.msgtype)
            f = decode_fields(msg, ["x", "y", "z"])
            p = np.column_stack([f["x"], f["y"], f["z"]])
            r = np.linalg.norm(p, axis=1)
            p = p[np.isfinite(r) & (r > min_range) & (r < max_range)]
            p.astype("<f4").tofile(out / f"{k:06d}.bin")
            stamps.append((k, msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec))
    np.savetxt(out / "stamps.csv", np.array(stamps), fmt=["%d", "%.9f"], delimiter=",")
    return len(stamps)


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--topic", default="/os_cloud_node/points")
    p.add_argument("--every", type=int, default=10)
    p.add_argument("--min-range", type=float, default=0.5)
    p.add_argument("--max-range", type=float, default=60.0)
    a = p.parse_args(argv)
    n = dump_pairs(Path(a.bag), Path(a.out), a.topic, a.every, a.min_range, a.max_range)
    print(f"wrote {n} scans")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
