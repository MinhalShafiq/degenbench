# Detector leaderboard v0.1: five detectors on SubT-MRS

Date: 2026-11-12. Thresholds: `leaderboard/thresholds.yaml` version 4, whose
detector protocol was committed (2a56804) before any detector ran on real data.
Labels: v0.1. Full output: `leaderboard/results/2026-11-12-leaderboard-v0.1.yaml`.

## Setup

- **Detectors.** All five are reimplementations from the papers. Each is
  evaluated at the label's reference pose, with the labeler's own
  correspondences:
  - minimum eigenvalue (Zhang et al. 2016)
  - condition number
  - X-ICP (Tuna et al. 2023)
  - DCReg (2025)
  - the frame-independent generalized spectrum ("Degenerate in Whose Frame?", 2026)
- **Reference poses.** `detector_features` reproduces the labeler's refined
  reference pose exactly on all 1,223 frames (maximum difference 0).
- **Protocol.**
  - One threshold per detector, calibrated on the development sequence
    (Laurel Caverns) by per-direction micro F1, then frozen.
  - Headline: the calibrated variant on the held-out sequence (Corridor01).
  - Published thresholds are reported alongside.
- **Scored frames.** Consensus frames only: 223 on Corridor01 (30 with a free
  direction) and 810 on Laurel Caverns (35).

## Held-out results: Corridor01, consensus frames

| Detector | Variant | Threshold | Frame F1 | Precision | Recall | Per-direction F1 | Subspace match |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Frame-independent | published | 0.01 | **0.966** | 1.000 | 0.933 | **0.966** | **99.1%** |
| X-ICP | calibrated | 0.469 × kappa | 0.364 | 0.222 | 1.000 | 0.237 | 43.5% |
| DCReg | calibrated | 50.4 | 0.297 | 0.174 | 1.000 | 0.220 | 22.9% |
| DCReg | published | 50 | 0.297 | 0.174 | 1.000 | 0.228 | 22.9% |
| X-ICP | published | 1 × kappa | 0.282 | 0.164 | 1.000 | 0.164 | 17.9% |
| DCReg | published | 10 | 0.261 | 0.150 | 1.000 | 0.176 | 10.3% |
| Minimum eigenvalue | calibrated | 2014 | 0.237 | 0.135 | 1.000 | 0.075 | 0% |
| Condition number | calibrated | 43.9 | 0.237 | 0.135 | 1.000 | 0.069 | 0% |
| Frame-independent | calibrated | 0.332 | 0.237 | 0.135 | 1.000 | 0.076 | 0% |
| *Never degenerate* | reference | - | 0 | - | 0 | 0 | 86.6% |

- **Frame-independent metric at its published threshold.** This is the only
  detector that works here. It catches 28 of 30 labelled corridor
  degeneracies (tx 21/23, ty 7/7), with no false alarm on 193 constrained
  frames.
- **Its weakest direction matches the label.** It lies within 9 deg of the
  labelled free direction on all 30 frames.
- **Its threshold has margin.** Labelled-free frames have lambda_min < 0.008;
  90% of constrained frames are above 0.019.
- **Every other variant fails the same way.** None separates the classes
  better than flagging almost every frame.
- **Runtime.** Median 0.8 ms per scan for all detectors (X-ICP 0.9 ms) once
  correspondences exist. The 10 ms budget is not binding.

## The pre-registered calibration failed, and why

The calibrated headline variants are meaningless. Calibration reached
development F1 of only 0.014-0.024 for every detector, and three of them
converged to "flag everything".

The cause is the development labels, not the detectors. On Laurel Caverns,
the 35 consensus frames with a free direction are not geometrically weak:

- Their frame-independent lambda_min (median 0.149) is *higher* than on
  constrained frames (0.116).
- The detector's weakest direction is 52-90 deg away from the labelled one.

These are the convergence-basin failures diagnosed earlier
(docs/results/2026-11-10-subt-mrs-laurel-caverns.md). They survive the solver
consensus because both solvers can share a basin failure. Calibrating against
them fits nothing.

The protocol is reported as committed and is not changed after the fact.
Two conclusions follow:

1. **The consensus filter doesn't guarantee geometric labels.** The
   Monte Carlo label still mixes geometry with basin effects on Laurel
   Caverns. On a structured corridor it does match geometry.
2. **This calibration rule needs development positives that are geometric.**
   A revised rule is a new thresholds version, evaluated on a sequence that
   neither the labels nor the detectors have seen.

## Leaderboard pass condition

Requires at least 5 detectors, 3 datasets and 2 LiDAR types. Status: 5
detectors, 1 dataset (SubT-MRS), 1 LiDAR type (VLP-16). **Not met yet.**
