# Held-out evaluation of thresholds v2: SubT-MRS Corridor01

Date: 2026-11-11. Thresholds: `leaderboard/thresholds.yaml` version 2, committed
(3d8aa81) before any v2 result existed. Corridor01 is the held-out sequence: no
method decision was made on it. The data-preparation choices below (time
offset, extrinsic, map alignment) were made on it, as they must be for any
sequence, and are reported.

## Inputs

| Item | Value |
| --- | --- |
| Sequence | SubT-MRS `Hawkins_Long_Corridor_RC` (one 6.5 GB ROS 1 bag, 280 s) |
| LiDAR | Velodyne VLP-16, raw packets on `/velodyne_packets` |
| Ground truth | Challenge `Corridor01` trajectory, 5 Hz, 6-DoF |
| Map | `corridor01.pcd` |
| Frames | 277 at 1 Hz (8 scans skipped at ground-truth gaps) |

## Data preparation

- **Clock.** The challenge trajectory is stamped in 2018 and the bag was
  recorded in 2023, but the message headers use the same 2018 clock as the
  trajectory. `time_offset` (gyro magnitude vs ground-truth angular speed)
  finds +13.7 ms at correlation 0.958. The naive start-time difference was off
  by 0.65 s.
- **Extrinsic.** Hand-eye from 186 scan pairs is poorly determined, and says so:
  yaw 2.4° ± 7.1°, rotation ± 9.2°, lever arm ± 0.5 m. A robot driving down a
  straight corridor barely rotates, and scan-to-scan registration along the
  corridor is itself degenerate. Choosing by map fit instead:

  | Extrinsic | Inliers within 0.3 m | Shifted 5 m | Shifted 20 m |
  | --- | --- | --- | --- |
  | Hand-eye estimate | 30.7% | 4.9% | 0.7% |
  | Identity (ground truth already in the LiDAR frame) | **86.9%** | 6.8% | 1.2% |

  Identity is used. Rule recorded for future sequences: when the hand-eye
  bootstrap spread is large, compare candidate extrinsics by map fit.
- **World to map.** yaw 82.43°, t = (1.77, −7.10, −0.90) m.

## Reference poses

| Measure | Value |
| --- | --- |
| Refinement converged | 261 / 277 |
| Correction, median | 6.7 cm, 0.46° |
| Correction, 90th percentile | 0.58 m |
| Map coverage, median | 99.4% (no frame below 80%) |

The large 90th-percentile correction is expected here: along the corridor axis
nothing constrains the refinement, so it may slide. That direction is the one the
label marks free.

## Labels (v2, ±0.2 m, ±4°)

| Free directions | Point-to-plane ICP | GICP |
| --- | --- | --- |
| 0 | 204 | 220 |
| 1 | 73 | 57 |

In frames where both solvers agree on one free direction, it is tx, the
corridor axis, in 32 cases (ty in 5, mixed in the rest): the textbook corridor
degeneracy, recovered on real data.

### Pre-registered check: solver agreement — FAIL

| Offsets | Agreement |
| --- | --- |
| v2 label (±0.2 m, ±4°) | **85.9%** |
| v1 range, robustness output (±0.5 m, ±5°) | 78.7% |
| Pass condition | ≥ 90% |

For comparison, the development sequence (Laurel Caverns) went from 72.1% to
86.4% under the same change.

v2 improves agreement by 7.2 points on data it was not developed on, which
supports the basin diagnosis, but it does not reach the pass condition.
Disagreements: 22 frames ICP 1 / GICP 0, 6 frames ICP 0 / GICP 1, 11 frames
where both find one free direction but more than 15° apart.

Nothing is changed in response. Per the plan's fallback, both solvers' results
and continuous errors are kept and disagreeing frames are flagged.

## What this means

Reducing the offset range removes part of the basin effect, as predicted, but
not all of it: in a long corridor the along-axis constraint comes from sparse
features (doors, junctions), and whether a solver can use them still depends on
the solver. A label that is both detector-independent and solver-independent at
the 90% level has not been achieved with Monte Carlo registration on VLP-16 data.
That is a substantive result for the benchmark, not a reason to move the
threshold.
