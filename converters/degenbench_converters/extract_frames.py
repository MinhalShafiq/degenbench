"""CLI: extract deskewed 1 Hz LiDAR frames from a ROS 1 bag.

Example (FusionPortableV2):

    python3 -m degenbench_converters.extract_frames \\
        --bag data/fusionportable/handheld_underground00.bag \\
        --gt data/fusionportable/handheld_underground00_gt.txt \\
        --calib calibration_files/20230403_calib/calib/ouster00.yaml \\
        --gt-frame body_imu \\
        --out out/fusionportable/handheld_underground00
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import yaml

from .calibration import fusionportable_extrinsic
from .frames import ExtractionConfig, extract_frames
from .poses import Trajectory, invert


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", required=True, nargs="+", help="one bag, or all chunks of a sequence")
    p.add_argument("--gt", required=True, help="ground-truth trajectory, TUM format")
    p.add_argument("--out", required=True)
    p.add_argument("--topic", default="/os_cloud_node/points")
    p.add_argument("--calib", help="FusionPortable LiDAR calibration YAML (ouster00.yaml)")
    p.add_argument("--gt-frame", default="lidar",
                   help="frame the trajectory is expressed in; 'lidar' means no extrinsic")
    p.add_argument("--extrinsic", help="YAML with a 4x4 T_gt_lidar (e.g. from handeye); "
                                       "overrides --calib/--gt-frame")
    p.add_argument("--rate", type=float, default=1.0, help="frames per second")
    p.add_argument("--time-field", default="t")
    p.add_argument("--time-scale", type=float, default=1e-9)
    p.add_argument("--no-deskew", action="store_true")
    p.add_argument("--max-gt-gap", type=float, default=0.2,
                   help="skip scans spanning a ground-truth gap longer than this [s]; "
                        "raise it for low-rate ground truth (e.g. 0.3 for 5 Hz)")
    args = p.parse_args(argv)

    if args.extrinsic:
        with open(args.extrinsic, encoding="utf-8") as f:
            T_gt_lidar = np.array(yaml.safe_load(f)["T_gt_lidar"], dtype=float)
        if T_gt_lidar.shape != (4, 4):
            p.error("--extrinsic: T_gt_lidar must be 4x4")
    elif args.gt_frame == "lidar":
        T_gt_lidar = np.eye(4)
    else:
        if not args.calib:
            p.error("--calib is required unless --gt-frame is 'lidar'")
        # Calibration gives T_lidar_gt; the trajectory needs T_gt_lidar.
        T_gt_lidar = invert(fusionportable_extrinsic(args.calib, args.gt_frame))

    config = ExtractionConfig(topic=args.topic, rate_hz=args.rate, time_field=args.time_field,
                              time_scale=args.time_scale, deskew=not args.no_deskew,
                              max_gt_gap=args.max_gt_gap)
    stats = extract_frames(args.bag if len(args.bag) > 1 else args.bag[0], Trajectory.from_tum(args.gt), T_gt_lidar, args.out, config)
    print(f"scans seen {stats.scans_seen}, frames written {stats.frames_written}, "
          f"skipped (no ground truth) {stats.skipped_no_ground_truth}, "
          f"skipped (empty) {stats.skipped_empty}")
    return 0 if stats.frames_written else 1


if __name__ == "__main__":
    sys.exit(main())
