# Held-out round 2: SubT-MRS Floor01 (labels v0.2, leaderboard v0.2)

Date: 2026-11-12. Thresholds: version 5, committed (2fbb1f3) before any
Floor01 data was processed. Floor01 is held out; Laurel Caverns and
Corridor01 are development.

## Inputs and preparation

| Item | Value |
| --- | --- |
| Sequence | SubT-MRS `Hawkins_Multi_Floor_LegRobot` (3 ROS 1 bags, 417 s, legged robot) |
| LiDAR | Velodyne VLP-16, raw packets on `/cmu_sp1/velodyne_packets` |
| Ground truth | Challenge `Floor01` trajectory, 5 Hz, 6-DoF |
| Map | `hawkins_multi_floor.pcd` |
| Frames | 413 at 1 Hz (16 scans skipped at ground-truth gaps) |
| Clock | `time_offset`: +12.6 ms, correlation 0.936 |
| Extrinsic | Identity, by map fit: 74.3% inliers within 0.3 m vs 50.8% for hand-eye (yaw -3.2 +- 3.1 deg) |
| World to map | yaw 28.22 deg, t = (39.97, -175.73, -5.13) m; 74.3% inliers (5 m shift 26.2%, 20 m shift 18.8%) |

## Labels: solver agreement — FAIL

**Agreement is 184 of 413 frames (44.6%; 46.9% excluding the 21
unreliable-reference frames). The pass condition is 90%.** Corridor01 reached
85.9%.

The dominant disagreement is ICP 1 / GICP 0 (157 frames), with ICP's free
axis mostly tz. A free vertical axis is implausible for a robot walking
hallways with floor and ceiling in view.

**Diagnosis: bias, not spread.** On five such frames, ICP converges from
every offset to the same pose 14–20 cm below the reference (spread 1 mm;
one frame also spreads). The reference pose is refined with GICP, so GICP
is unbiased by construction. The label's second moment is taken about zero
on purpose, so that a consistent bias counts as failure, and it therefore
marks ICP's disagreement with GICP's minimum as degeneracy. This asymmetry
between the solvers is a flaw in the label method. It stayed hidden until a
map where the two solvers' minima differ by more than the 0.10 m threshold.

Only one consensus frame has a free direction (frame 360, mostly tz). The
frame-independent spectrum calls that frame well constrained
(lambda_min 0.125), so it is probably the same artefact.

Nothing is changed in response. Floor01 is now spent as a held-out sequence
for the labels.

## Leaderboard v0.2

Calibration pools the development consensus frames (Laurel Caverns and
Corridor01), as pre-registered in v5. This time it finds working thresholds:

| Detector | Calibrated threshold | Development per-direction F1 |
| --- | --- | --- |
| Frame-independent | 0.0091 (published: 0.01) | 0.659 |
| X-ICP | 0.121 x kappa | 0.659 |
| Minimum eigenvalue | 15.4 | 0.651 |
| Condition number | 5831 | 0.629 |
| DCReg | 49.6 | 0.172 |

The frame-independent threshold calibrated from data lands on the paper's
published value. All four working detectors reach F1 0.90-0.97 on
Corridor01. Development F1 is capped near 0.66 because Laurel Caverns' free
labels are unreliable (docs/results/2026-11-12-label-spot-check.md).

**Held-out Floor01 cannot rank the detectors.** It has 183 consensus frames
without a free direction and only 1 with one (likely an artefact, see above):

| Detector (calibrated) | Subspace match | Frame F1 | Disputed: agrees with ICP | Disputed: agrees with GICP |
| --- | --- | --- | --- | --- |
| Frame-independent | 99.5% | 0 (1 positive) | 3.8% | 78.4% |
| X-ICP | 99.5% | 0 | 4.3% | 75.0% |
| Condition number | 99.5% | 0 | 3.8% | 78.4% |
| Minimum eigenvalue | 94.6% | 0 | 3.8% | 69.2% |
| DCReg | 78.3% | 0 | 4.8% | 48.6% |

The working detectors raise almost no false alarms on Floor01, but recall
cannot be measured. On disputed frames all of them side with GICP, as the
bias diagnosis predicts.

## What this means

- **Labels:** Monte Carlo labels from a GICP-refined reference are biased
  against the other solver wherever the two solvers' minima differ. A fix
  could take the moment about each solver's own converged mean, or refine
  the reference with each solver separately. That would be a new thresholds
  version, evaluated on a sequence nobody has seen.
- **Leaderboard:** the ordering on Corridor01 is the only held-out-style
  evidence of recall. Floor01 adds false-alarm evidence, not recall. A
  held-out sequence with real degeneracy is still needed. Corridor02 is the
  candidate; its download is blocked by Drive's quota.
