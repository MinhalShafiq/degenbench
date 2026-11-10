# Datasets

DegenBench uses five public datasets. None of their raw data is redistributed
here: download each from its own source under its own terms. DegenBench
publishes labels, scripts and converters only.

Access was checked on 2026-11-09.

## Summary

| Dataset | Access | License | Bag format | Survey map | Role |
| --- | --- | --- | --- | --- | --- |
| FusionPortableV2 | Google Drive / Baidu, no registration | Not stated | ROS 1 (`.bag`, `.7z`) | 2 maps: UGV Campus, Underground Parking (`.pcd`) | Labels; wheel fusion |
| SubT-MRS | Google Drive, no registration | Not stated | ROS 1 | Maps listed for Cave01–04, Corridor01–02, Floor01–02 | Labels |
| GEODE | Google Drive / Baidu (`sysu`), no registration | Not stated | ROS 1 (β, γ), ROS 2 (α) | Stairs only (Leica RTC360); download not yet confirmed | Trajectory scoring; labels on stairs |
| ENWIDE | ETH Research Collection, DOI [10.3929/ethz-c-000787551](https://doi.org/10.3929/ethz-c-000787551) | Not stated | ROS 1 | None | Trajectory scoring in the tunnel |
| M3DGR | OneDrive / Alipan, SHA256 checksums | MIT | ROS 1 | None | Wheel fusion in corridors |

Every dataset is reachable without a request form. Only M3DGR states a
license, so all DegenBench outputs that derive from the others are labels and
code, never data.

## FusionPortableV2

- Site: <https://fusionportable.github.io/dataset/fusionportable_v2_data/>
- Tools: <https://github.com/fusionportable/fusionportable_dataset_tools>
- Calibration: <https://github.com/fusionportable/calibration_files>, one
  `<frame_id>.yaml` per sensor per calibration date. Extrinsics are given as
  `T_sensor_<frame>`, the pose of `<frame>` in the Ouster sensor frame.

Label candidates are the sequences inside a mapped area:

| Sequence | Platform | Map | Notes |
| --- | --- | --- | --- |
| `handheld_underground00` | Handheld | Underground Parking | Structureless |
| `legged_underground00` | Legged | Underground Parking | Structureless |
| `ugv_parking00`–`03` | UGV | None found (`ugv_parking02` checked, see below) | Structureless; wheel encoders |
| `ugv_campus00`, `01` | UGV | UGV Campus | Large-scale; wheel encoders |

| Sensor | Topic | Type |
| --- | --- | --- |
| Ouster OS1-128 | `/os_cloud_node/points` | `sensor_msgs/PointCloud2`, 10 Hz |
| Ouster IMU | `/os_cloud_node/imu/data_raw` | `sensor_msgs/Imu` |
| STIM300 IMU | `/stim300/imu/data_raw` | `sensor_msgs/Imu` |
| UGV wheel encoders | `/mini_hercules/encoder` | `sensor_msgs/JointState` |
| Legged odometry | `/unitree/body_odom` | `nav_msgs/Odometry` |

Known issues:
- The maps contain unremoved dynamic objects. The authors recommend the
  original FusionPortable maps where a clean map is needed.
- Vehicle ground truth was post-processed (2024-05-08) to drop high-uncertainty
  poses, so vehicle trajectories have gaps.

### Ground truth, as downloaded (2026-11-10)

Checked from the trajectory files in `groundtruth/traj/` on the Drive folder:

| Sequence | Rate | Orientation | Max gap | Extent (x × y) |
| --- | --- | --- | --- | --- |
| `handheld_underground00` | 20 Hz | **None** (all identity) | 2.7 s | 400 m × 20 m |
| `legged_underground00` | 20 Hz | **None** (all identity) | 2.0 s | 160 m × 8 m |
| `ugv_parking00`–`03` | 30 Hz | Full 6-DoF | 0.07 s | up to 110 m × 70 m |
| `ugv_campus00` | 30 Hz | Full 6-DoF | 0.07 s | 250 m × 560 m |

- **The underground sequences are position-only** (consistent with total-station
  prism tracking). Their scene is a 400 m long, narrow tunnel: the most useful
  degenerate geometry in the dataset. Labels there need orientation from
  elsewhere: registering each scan to the survey map with the position fixed
  by ground truth. That is a lower-tier label and must be marked as such.
- **UGV sequences have full 6-DoF poses with no gaps**, all starting at the
  same point with different headings, so they share one world frame (the INS
  frame, from dual-antenna RTK-GNSS fused with an IMU).
- **Trajectories and maps are in different frames, and no transform is
  published.** Checked 2026-11-10: placed in the map files as given, no UGV or
  underground pose has ground below it, and the nearest map point is 25–290 m
  away. The paper confirms the three sources are independent (INS, Leica MS60
  total station, Leica RTC360/BLK360 scans merged in Cyclone) and aligns SLAM
  maps to the GT maps manually in CloudCompare. DegenBench therefore estimates
  one rigid `T_map_world` per sequence by registering scans accumulated along
  the ground truth to the survey map, and publishes that transform with its
  residual.
- **The UGV ground-truth body frame is none of the published frames.**
  Checked on `ugv_parking02` (2026-11-10). With the published `body_imu`
  extrinsic, scans placed by ground truth sit metres apart, and the GT
  velocity points along body −y while the LiDAR measures +x. Two
  diagnostics ruled out other causes: the pose convention (`T_world_body`
  moves consistently along one axis, its inverse does not) and time offset
  (the best fit is 0 s, ±0.2 s is worse). `3dm_imu` predicts the opposite
  turn direction. A hand-eye fit on 80 consecutive-scan pairs
  (`scan_pairs` → `register_pairs` → `handeye`) gives:

  | Quantity | Estimate | Bootstrap ± |
  | --- | --- | --- |
  | Yaw | −85.4° | 1.2° |
  | Total rotation | | 1.8° |
  | Lever arm x, y | 0.41 m, −0.16 m | 0.25 m, 0.15 m |
  | Lever arm z | Unobservable from planar driving, set to 0 | |

  Placing scans with this extrinsic raises scan-to-scan agreement within
  0.3 m from 68% to 81%, and GICP convergence against neighbouring scans
  from 9/39 to 33/39 frames. The extrinsic is only good enough to
  initialize: per-frame reference poses are refined against the survey map,
  and the size of each refinement is published as a ground-truth quality
  measure.
- **`ugv_parking02` is not covered by the campus map** (checked 2026-11-10).
  The method: accumulate the 77 deskewed 1 Hz frames along the ground
  truth, reduce both clouds to 0.5 m occupancy images of vertical structure,
  search all 360 yaws with FFT cross-correlation, then refine the top
  candidates with coarse-to-fine GICP. Two candidates 133° apart tie
  (36% of points within 0.3 m each). Scoring only above-ground points
  gives 21.8% at the best pose and 18–20% at poses shifted 5–20 m, i.e. no
  better than chance. The parking lot is presumably outside the surveyed
  area. Whether `ugv_campus00` lies inside the map is still open.
- Maps are shipped as `binary_compressed` (LZF) PCD at several resolutions;
  the file suffix is the voxel size (`_02` = 0.2 m, `_008` = 0.08 m).
- Timestamps carry sub-millisecond decimals. Tools that parse numbers using
  the system locale (e.g. `awk` under a decimal-comma locale) truncate them;
  use `LC_ALL=C`.

## SubT-MRS

- Site: <https://superodometry.com/datasets>
- Ground truth is distributed separately; trajectories are
  `timestamp x y z qx qy qz qw`.
- Label candidates with maps: `Cave01`–`04`, `Corridor01`, `Corridor02`,
  `Floor01`, `Floor02`.
- Ground truth in degraded corridors partly relies on visual odometry, so
  SubT-MRS frames are used for labels only, never for trajectory scoring in
  degenerate segments.
- **To verify:** whether the listed maps are the FARO survey or a
  SLAM reconstruction. Only survey-grade maps are used for top-tier labels.
- Downloads (checked 2026-11-10): `Cave03` is `SubT_MRS_Laurel_Caverns_Handheld3.zip`
  (610 MB, 11 ROS 1 bags), map `lauren_cavern.pcd` (17 MB, shared by Cave01–04);
  `Corridor01` is `SubT_MRS_Hawkins_Long_Corridor_RC.zip` (3.6 GB), map
  `corridor01.pcd` (7.7 MB).
- The LiDAR is stored as raw `velodyne_msgs/VelodyneScan` packets (VLP-16,
  single strongest return) on `/cmu_rc7/velodyne_packets`; DegenBench decodes
  them itself.
- The ground truth that matches the Laurel Caverns bag is `ground_truth_path.csv`
  in the ground-truth Drive folder (5 Hz, nanosecond stamps, CSV). The
  separately listed `Cave03` trajectory starts after the bag ends.
- Ground truth starts at the origin and is not in the map frame. The world-to-map
  transform and GT-to-LiDAR extrinsic are estimated; see
  `docs/results/2026-11-10-subt-mrs-laurel-caverns.md`.

## GEODE

- Repository: <https://github.com/PengYu-Team/GEODE_dataset>
- Metro tunnel ground truth is Leica MS60 prism tracking: position only.
- Shield-tunnel sequences are `Shield1`–`Shield10` in the paper; exact folder
  names to confirm on download.
- **Open risk:** the stairs map (`.las`, Leica RTC360) is described in the paper
  but not linked from the repository. If it is not downloadable, GEODE serves
  trajectory scoring only, and the second LiDAR type for labels comes from
  SubT-MRS (Velodyne) alongside FusionPortableV2 (Ouster).

## ENWIDE

- Site: <https://projects.asl.ethz.ch/datasets/enwide>
- One smooth (`_s`) and one dynamic (`_d`) sequence per environment, e.g.
  `2023-08-08-17-50-31-tunnel_d.bag`.
- Ground truth: Leica MS60 total station tracking a prism, position only, TUM
  CSV.
- Only the IMU-to-prism extrinsics are published (CAD-derived):
  <http://robotics.ethz.ch/~asl-datasets/2024_ICRA_ENWIDE/prism_imu_extrinsics.txt>.

## M3DGR

- Repository: <https://github.com/sjtuyinjie/M3DGR> (MIT)
- Corridor sequences: `Corridor01`, `Corridor02`, `Elevator01`, with ArUco
  ground truth evaluated by the dataset's `ArUco_evaluate.py`.

| Sensor | Topic |
| --- | --- |
| Livox Avia | `/livox/avia/lidar` |
| Livox Mid-360 | `/livox/mid360/lidar` |
| Wheel odometry | `/odom`, 20 Hz |

Livox topics need `livox_ros_driver` message definitions to read.

## Effect on the plan

- **Download risk is retired** for four of five datasets. SubT-MRS, whose
  access was the main worry, is public on Google Drive.
- **Label coverage** rests on the FusionPortableV2 Underground Parking
  sequences (Ouster, three platforms) and SubT-MRS caves and corridors
  (Velodyne). That still meets the two-LiDAR-type requirement without GEODE.
