# DegenBench labels

Per-frame LiDAR registration degeneracy labels for public datasets. Labels only:
the raw data must be downloaded from each dataset's own site.

## v0.1 (2026-11-12): fallback release, preview

| Sequence | Role | Frames | Consensus | Disputed | Unreliable reference | Consensus frames with free directions |
| --- | --- | --- | --- | --- | --- | --- |
| [SubT-MRS Corridor01](v0.1/subt_mrs/corridor01) | held out | 277 | 223 | 38 | 16 | 30 (tx 23, ty 7) |
| [SubT-MRS Laurel Caverns Handheld3](v0.1/subt_mrs/laurel_caverns_handheld3) | development | 946 | 810 | 127 | 9 | 35 |

**Status: the labels do not meet their own pre-registered quality bar.** The
two registration solvers agree on 85.9% of held-out frames against a pass
condition of 90% ([results](../docs/results/2026-11-11-subt-mrs-corridor01-thresholds-v2.md)).
This release follows the fallback planned for that case: every frame keeps
both solvers' results and continuous errors, frames are classified by status,
and the leaderboard scores detectors on consensus frames only. The rules were
fixed in [`thresholds.yaml`](../leaderboard/thresholds.yaml) version 3 before
any detector was scored. The pre-registered visual spot check of 50 frames has
not been done yet, so this is a preview.

The agreement rate in each `meta.yaml` excludes unreliable-reference frames
(Corridor01 85.4%), so it differs slightly from the 85.9% computed over all
frames in the results document.

## What a label means

For each frame, the scan is registered to the survey map from 200 random
initial offsets (uniform, +-0.2 m and +-4 deg per axis) around a reference pose,
with point-to-plane ICP and with GICP. The second moment of the final 6-D
errors, in threshold-normalized coordinates (t / 0.10 m, r / 2 deg), is
eigendecomposed. A direction is **degenerate (free)** when its eigenvalue
exceeds 1, i.e. the registration error along it is RMS-larger than 0.10 m or
2 deg. The reference pose is the ground truth refined by GICP against the map.

Coordinates are the LiDAR sensor frame, order `tx ty tz rx ry rz`.

## Frame status

| Status | Meaning | Scored |
| --- | --- | --- |
| `consensus` | Reference refinement converged; both solvers find a free subspace of the same dimension, at most 15 deg apart | Headline |
| `disputed` | Refinement converged; solvers disagree | Separately, against each solver |
| `unreliable_reference` | Refinement did not converge | No |

The consensus subspace of dimension k is spanned by the top-k eigenvectors of
(P_icp + P_gicp) / 2, the mean of the solvers' orthogonal projectors.

## Files per sequence

- `labels.csv`: one row per frame.
  - `index`, `stamp` (bag time, seconds), `status`, `k` and `consensus_axes` (consensus frames only)
  - `{icp,gicp}_k`, `{icp,gicp}_axes`, `subspace_angle_deg`
  - `{icp,gicp}_eig0..5`: normalized eigenvalues, ascending
  - `{icp,gicp}_rms_{tx..rz}`: RMS final error per axis, in metres or radians
  - `coverage`, `map_points`, `refine_converged`, `refine_trans_m`, `refine_rot_deg`: reference-pose quality
  - Corridor01 only: `robust_*`, the same probe with the wider +-0.5 m / +-5 deg offsets (secondary, not a label)
- `subspaces.csv`: `index,source,k,v00..v55`. Here `source` is `icp`, `gicp` or
  `consensus`, and columns `v{c}{r}` for c < k hold the basis, as column c and row r.
- `meta.yaml`: source bags and topic, time offset, extrinsic, world-to-map
  transform, label definition, summary and SHA-256 checksums of both CSVs.
  [`inputs/`](inputs) holds the hand-written part.

An axis counts as free for the per-direction metrics when the subspace
projector's diagonal entry exceeds 0.5. `consensus_axes` uses this rule.

## Scoring a detector

Write one row per frame, `index,k,v00,...,v55`, with the predicted degenerate
subspace in the same normalized coordinates (columns c >= k may be empty).

```bash
cd leaderboard
python3 -m degenbench_leaderboard.score --labels ../labels/v0.1/subt_mrs/corridor01 \
    --predictions my_detector.csv --out score.yaml
```

Output metrics:
- consensus frames: subspace error (mean, median, share within 15 deg), frame
  detection, per-direction precision/recall/F1, micro-averaged F1
- disputed frames: subspace error against each solver

A missing prediction counts as "no degeneracy".

Most frames are well constrained. A detector that never reports degeneracy
already matches 86.6% of Corridor01 consensus frames, with frame-detection F1 of 0.
Read the subspace match rate together with the F1 scores.

## Reproducing

Reference poses are not published: they derive from the dataset's ground
truth. They are regenerated locally from the downloaded data, using the
converters (`extract_frames`, `time_offset`, `handeye`, `map_align`) with the
parameters in `meta.yaml`, followed by the labeler (`label_sequence`).
`leaderboard/degenbench_leaderboard/release.py` builds the release
directory and checks that its recomputed solver agreement matches the
labeler's frame by frame.
