# Visual spot check of labels v0.1

The labels' pre-registration (thresholds.yaml, `visual_spot_check_frames: 50`)
did not fix how frames would be sampled or judged. This protocol was committed
before any frame was rendered.

## Protocol (committed before viewing)

- **Sample:** 50 frames, seed 0, drawn by `degenbench_leaderboard.spot_check`.
  The same split is used for each of SubT-MRS Corridor01 and Laurel Caverns
  Handheld3:
  - 12 consensus frames with a free direction
  - 8 consensus frames with none
  - 5 disputed frames
- **View:** for each frame, a top view and a side view in the sensor frame,
  30 m across. Map in grey, scan in blue, labelled free translation
  directions as lines through the sensor.
- **Question:** is the label plausible given the visible geometry?
  - **Consensus, free:** the scene looks nearly invariant along the labelled
    direction, e.g. it runs along a corridor or tunnel axis.
  - **Consensus, none:** the scene has structure constraining every
    direction, e.g. walls at different orientations, ends, or clutter.
  - **Disputed:** which solver's label, if either, is plausible.
- **Verdict:** each frame is judged *plausible*, *implausible* or *unclear*
  (when the views cannot decide it).
- **Pass:** at least 90% of the judgeable consensus frames are plausible.
  Disputed frames are reported, not scored.
- **Who judges:** the judge is the author's assistant, from 2-D projections.
  This is weaker than a human inspecting the 3-D scene, and rotational free
  directions cannot be drawn. Images and verdicts are listed per frame so
  they can be re-judged.

## Results — FAIL

**26 of 37 judgeable consensus frames are plausible (70%); the bar is 90%.**

| Sequence | Stratum | Plausible | Implausible | Unclear |
| --- | --- | --- | --- | --- |
| Corridor01 | consensus, free | 12 | 0 | 0 |
| Corridor01 | consensus, none | 6 | 2 | 0 |
| Laurel Caverns | consensus, free | 0 | 9 | 3 |
| Laurel Caverns | consensus, none | 8 | 0 | 0 |
| **All consensus** | | **26** | **11** | **3** |

By sequence:
- **Corridor01: 18 of 20 plausible (90%).** Every free label lies along the
  corridor axis of a featureless stretch.
  - Frames 239 and 244 are labelled constrained but look as uniform as the
    free frames. Their frame-independent lambda_min (0.024, 0.027) sits
    between the free frames (< 0.008) and typical constrained ones, which is
    consistent with small features (door frames, fixtures) constraining the
    axis. The verdict follows the views, as the protocol says.
- **Laurel Caverns: 8 of 17 plausible.**
  - Every judgeable consensus-free label is implausible: the free line cuts
    across a passage, or runs steeply through floor and ceiling (frames 318,
    322, 355, 377, 379, 385, 391, 619, 635).
  - Unclear: frames 367, 371 and 372, a short connecting tube whose label runs
    along it but whose end faces are visible.
  - All constrained labels are plausible.

This agrees with the detector diagnosis
(docs/results/2026-11-12-detector-leaderboard-v0.1.md): the consensus-free
frames on Laurel Caverns are convergence-basin failures shared by both
solvers, not geometric degeneracy.

### Disputed frames (reported, not scored)

| Frame | Labels | Verdict |
| --- | --- | --- |
| Corridor01 0 | ICP rotation; GICP none | unclear (rotation not drawn) |
| Corridor01 1 | ICP along axis; GICP none | GICP plausible (junction) |
| Corridor01 112, 120 | ICP along axis; GICP none | ICP plausible (uniform corridor) |
| Corridor01 254 | ICP along axis; GICP none | unclear (side opening) |
| Laurel 0, 349, 361 | ICP none; GICP across the passage | ICP plausible |
| Laurel 5 | ICP none; GICP rotation | unclear |
| Laurel 347 | both across the passage, 15+ deg apart | neither |

ICP is the plausible one on 6 of 9 decidable disputed frames. In
Corridor01 the dispute is GICP missing the corridor degeneracy; in Laurel it
is GICP reporting basin failures.

### Consequences

- The labels' visual check fails, joining the solver-agreement check. Labels
  v0.1 stay a preview.
- **The consensus-free class is trustworthy on Corridor01 and not on
  Laurel Caverns.** Anyone using v0.1 should treat Laurel Caverns' free
  labels as unreliable.
- The label method needs a geometric check of its free directions, not only
  agreement between two solvers that can share a failure. Any such change is
  a new thresholds version, evaluated on sequences it was not developed on.
