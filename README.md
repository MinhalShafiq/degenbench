# DegenBench

Per-frame ground truth for LiDAR registration degeneracy on public real-world data,
a leaderboard of published degeneracy detectors scored against it, and a ROS 2 node
that reports live when LiDAR localization cannot be trusted.

> Status: early development. Labels v0.1 (preview, two SubT-MRS sequences) are in
> [`labels/`](labels). They do not meet the pre-registered solver-agreement bar
> (85.9% vs 90% on the held-out sequence), so they ship under the planned
> fallback, with consensus and disputed frames marked.

## Layout

| Directory | Contents |
| --- | --- |
| `labels/` | Released labels, per dataset and sequence |
| `converters/` | Dataset → unified ROS 2 bags, calibration, ground truth |
| `labeler/` | C++ label generator (point-to-plane ICP and GICP), map preprocessing |
| `detectors/` | C++ library of the scored degeneracy detectors |
| `leaderboard/` | Scoring scripts and pre-registered pass/fail thresholds |
| `degen_monitor/` | ROS 2 localizability monitor node and messages |

## Build

Everything builds in Docker (ROS 2 Jazzy, Ubuntu 24.04):

```bash
./docker/run.sh          # build the dev image and open a shell
./docker/run.sh test     # configure, build and run all unit tests
```

## Data

DegenBench publishes labels, scripts and converters only. Raw data must be
downloaded from each dataset's own site under its own license.

## License

Apache-2.0, see [LICENSE](LICENSE). Any FAST-LIO2-derived code will live in its own
directory under GPLv2.
