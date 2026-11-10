# First real-data labels: SubT-MRS Laurel Caverns Handheld3

Date: 2026-11-10. Thresholds: `leaderboard/thresholds.yaml` version 1, unchanged.

## Inputs

| Item | Value |
| --- | --- |
| Sequence | SubT-MRS `Laurel_Caverns_Handheld3` (11 ROS 1 bags, 955 s) |
| LiDAR | Velodyne VLP-16, raw packets, decoded by `degenbench_converters.velodyne` |
| Ground truth | `ground_truth_path.csv`, 5 Hz, 6-DoF |
| Map | `lauren_cavern.pcd`, 563,558 points (0.2 m voxels: 316,932) |
| Frames | 946 at 1 Hz (12 scans skipped at ground-truth gaps) |

## Frame alignment

The ground truth starts at the origin and is not in the map frame: as given,
only 34% of poses have map ground below them, and the median nearest map point
is 5.5 m away.

1. **GT body to LiDAR** (`scan_pairs` → `register_pairs` → `handeye`, 190 pairs):
   yaw −2.7° ± 1.3°, total rotation ± 1.9°, lever arm (−0.03, 0.15, 0.09) m
   ± (0.06, 0.06, 0.09) m. Residual per 0.1 s pair: 0.49°, 2.1 cm.
2. **World to map** (`map_align` → `refine_alignment`): yaw −73.97°,
   t = (33.87, −72.37, 21.12) m. 74.1% of accumulated points lie within 0.3 m
   of the map, against 20.6% with the result shifted 5 m and 3.1% at 20 m. All
   eight coarse candidates converge to the same transform.

## Per-frame reference poses

Each ground-truth pose is refined against the map before probing.

| Measure | Value |
| --- | --- |
| Refinement converged | 937 / 946 |
| Correction, median | 0.18 m, 2.79° |
| Correction, 90th percentile | 0.27 m, 3.42° |
| Correction, max | 0.79 m |
| Map coverage after refinement, median | 99.8% (no frame below 80%) |

The rotation correction is systematic (mean 2.83°, std 0.51°), which points
to the hand-eye extrinsic rotation rather than the ground truth. Hand-eye
reported ±1.9° on rotation. Labels are measured from the refined pose, so they
are unaffected. A map-based extrinsic estimate is the obvious improvement.

## Labels

| Free directions | Point-to-plane ICP | GICP |
| --- | --- | --- |
| 0 | 761 | 665 |
| 1 | 167 | 227 |
| 2 | 18 | 54 |

### Pre-registered check: solver agreement — FAIL

**72.1% of frames agree; the pre-registered pass condition is ≥ 90%.**

- 221 frames disagree on the number of free directions. These are not
  threshold cases: the solver reporting fewer free directions has a median
  normalized eigenvalue of 0.03 at the rank where the other crosses 1, and only
  18% are within [0.5, 1). One solver converges tightly from every offset while
  the other spreads. GICP reports more degeneracy (124 frames: ICP 0, GICP 1;
  23 frames: ICP 0, GICP 2), but the reverse also occurs (51 frames: ICP 1,
  GICP 0).
- 43 frames agree on the dimension but not the direction: subspace angles of
  16°, 23° and 48° at the 10th, 50th and 90th percentiles.

Per the plan's fallback for solver-dependent labels, both solvers' results and
the continuous per-axis errors are kept, and disagreeing frames are flagged
rather than dropped. The threshold is not changed after the fact, and solver
settings are not retuned against this sequence to raise the agreement rate.
Any change of method is a new, dated entry evaluated on a sequence it was not
tuned on.

## Open questions

- Why the solvers diverge here: VLP-16 scans are sparse (16 rings), and GICP's
  per-point covariances from 10 neighbours may be poor in such scans. This
  needs a controlled test on a held-out sequence, not tuning on this one.
- The visual spot check of 50 frames (pre-registered) is still to do.
