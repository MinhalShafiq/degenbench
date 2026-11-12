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

## Results

Pending.
