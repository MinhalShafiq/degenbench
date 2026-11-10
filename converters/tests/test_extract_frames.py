"""End-to-end test on a synthetic ROS 1 bag with known motion and extrinsics.

A sensor moves fast (4 m/s, 1 rad/s yaw) past static world points while a
spinning LiDAR samples them over 0.1 s per scan. Extraction must undo the motion
distortion exactly and report the right pose.
"""

import numpy as np
import pytest
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore

from degenbench_converters.calibration import fusionportable_extrinsic
from degenbench_converters.extract_frames import main
from degenbench_converters.frames import ExtractionConfig, extract_frames
from degenbench_converters.poses import (
    Trajectory, apply, invert, make_transform, rot_to_quat,
)

TOPIC = "/os_cloud_node/points"
SCAN_PERIOD = 0.1


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T_world_gt(t):
    """Ground-truth (body) motion: constant velocity and yaw rate."""
    return make_transform(rot_z(1.0 * t), np.array([4.0 * t, 0.5 * t, 0.0]))


# Body-to-LiDAR extrinsic with rotation and offset, like a real rig.
T_GT_LIDAR = make_transform(rot_z(0.3), np.array([0.1, -0.05, 0.2]))


def gt_trajectory(t_end=3.0, dt=0.01):
    stamps = np.arange(0.0, t_end + dt / 2, dt)
    poses = [T_world_gt(t) for t in stamps]
    return Trajectory(stamps, np.array([p[:3, 3] for p in poses]),
                      np.array([rot_to_quat(p[:3, :3]) for p in poses]))


def world_points(n=400, seed=1):
    rng = np.random.default_rng(seed)
    return rng.uniform([-10, -10, -2], [20, 10, 3], size=(n, 3))


def make_cloud(typestore, stamp, pts, offsets_ns):
    PointCloud2 = typestore.types["sensor_msgs/msg/PointCloud2"]
    PointField = typestore.types["sensor_msgs/msg/PointField"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    rec = np.zeros(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                     ("intensity", "<f4"), ("t", "<u4")])
    rec["x"], rec["y"], rec["z"] = pts.T
    rec["t"] = offsets_ns
    fields = [PointField(name="x", offset=0, datatype=7, count=1),
              PointField(name="y", offset=4, datatype=7, count=1),
              PointField(name="z", offset=8, datatype=7, count=1),
              PointField(name="intensity", offset=12, datatype=7, count=1),
              PointField(name="t", offset=16, datatype=6, count=1)]
    sec = int(stamp)
    return PointCloud2(
        header=Header(seq=0, stamp=Time(sec=sec, nanosec=int(round((stamp - sec) * 1e9))),
                      frame_id="ouster00"),
        height=1, width=len(pts), fields=fields, is_bigendian=False,
        point_step=rec.dtype.itemsize, row_step=rec.dtype.itemsize * len(pts),
        data=np.frombuffer(rec.tobytes(), dtype=np.uint8), is_dense=True)


@pytest.fixture
def synthetic_bag(tmp_path):
    """10 Hz scans from t=0.2 s to 2.7 s, each point timed within its scan."""
    typestore = get_typestore(Stores.ROS1_NOETIC)
    world = world_points()
    rng = np.random.default_rng(2)
    path = tmp_path / "synthetic.bag"
    truth = {}
    with Writer(path) as writer:
        msgtype = "sensor_msgs/msg/PointCloud2"
        conn = writer.add_connection(TOPIC, msgtype, typestore=typestore)
        for k in range(26):
            stamp = 0.2 + k * SCAN_PERIOD
            offsets = np.sort(rng.uniform(0, SCAN_PERIOD, len(world)))
            offsets_ns = np.round(offsets * 1e9).astype(np.uint32)
            # Each point is measured from the LiDAR pose at its own time.
            local = np.stack([
                apply(invert(T_world_gt(stamp + o * 1e-9) @ T_GT_LIDAR), p[None])[0]
                for p, o in zip(world, offsets_ns)])
            msg = make_cloud(typestore, stamp, local, offsets_ns)
            writer.write(conn, int(stamp * 1e9), typestore.serialize_ros1(msg, msgtype))
            truth[round(stamp, 6)] = T_world_gt(stamp) @ T_GT_LIDAR
    return path, world, truth


def read_frames(out):
    rows = np.loadtxt(out / "frames.csv", delimiter=",", skiprows=1, ndmin=2)
    clouds = [np.fromfile(out / "frames" / f"{int(r[0]):06d}.bin", dtype="<f4").reshape(-1, 3)
              for r in rows]
    return rows, clouds


def test_deskew_recovers_static_world(synthetic_bag, tmp_path):
    bag, world, truth = synthetic_bag
    out = tmp_path / "out"
    stats = extract_frames(bag, gt_trajectory(), T_GT_LIDAR, out, ExtractionConfig(topic=TOPIC))

    assert stats.scans_seen == 26
    # 1 Hz from 0.2 s: frames at 0.2, 1.2, 2.2.
    np.testing.assert_allclose(stats.stamps, [0.2, 1.2, 2.2], atol=1e-6)

    rows, clouds = read_frames(out)
    for row, cloud in zip(rows, clouds):
        T_expected = truth[round(row[1], 6)]
        np.testing.assert_allclose(row[2:5], T_expected[:3, 3], atol=1e-6)
        # Deskewed points mapped through the reported pose land on the world points.
        recovered = apply(T_expected, cloud.astype(float))
        assert np.max(np.linalg.norm(recovered - world, axis=1)) < 2e-3  # float32 storage


def test_without_deskew_motion_distortion_is_visible(synthetic_bag, tmp_path):
    bag, world, truth = synthetic_bag
    out = tmp_path / "raw"
    extract_frames(bag, gt_trajectory(), T_GT_LIDAR, out,
                   ExtractionConfig(topic=TOPIC, deskew=False))
    rows, clouds = read_frames(out)
    err = np.linalg.norm(apply(truth[round(rows[0][1], 6)], clouds[0].astype(float)) - world, axis=1)
    assert err.max() > 0.2  # 4 m/s over up to 0.1 s


def test_frames_outside_ground_truth_are_skipped(synthetic_bag, tmp_path):
    bag, _, _ = synthetic_bag
    short = gt_trajectory(t_end=1.0)  # covers only the first frame
    stats = extract_frames(bag, short, T_GT_LIDAR, tmp_path / "o", ExtractionConfig(topic=TOPIC))
    assert stats.frames_written == 1
    assert stats.skipped_no_ground_truth > 0


def test_missing_topic_is_an_error(synthetic_bag, tmp_path):
    bag, _, _ = synthetic_bag
    with pytest.raises(ValueError, match="not found"):
        extract_frames(bag, gt_trajectory(), T_GT_LIDAR, tmp_path / "o",
                       ExtractionConfig(topic="/nope"))


def test_cli_with_extrinsic_yaml(synthetic_bag, tmp_path):
    import yaml
    bag, world, truth = synthetic_bag
    ext = tmp_path / "handeye.yaml"
    ext.write_text(yaml.safe_dump({"T_gt_lidar": T_GT_LIDAR.tolist()}))
    gt = tmp_path / "gt.txt"
    gt_trajectory().to_tum(gt)
    out = tmp_path / "cli_ext"
    assert main(["--bag", str(bag), "--gt", str(gt), "--out", str(out), "--extrinsic", str(ext)]) == 0
    rows, clouds = read_frames(out)
    recovered = apply(truth[round(rows[0][1], 6)], clouds[0].astype(float))
    assert np.max(np.linalg.norm(recovered - world, axis=1)) < 2e-3


def test_cli_with_fusionportable_calibration(synthetic_bag, tmp_path):
    bag, world, truth = synthetic_bag
    # Write a calibration file in FusionPortable's layout: T_sensor_body_imu.
    T_lidar_body = invert(T_GT_LIDAR)
    q = rot_to_quat(T_lidar_body[:3, :3])
    calib = tmp_path / "ouster00.yaml"
    calib.write_text(
        "quaternion_sensor_body_imu:\n  rows: 1\n  cols: 4\n  dt: f\n"
        f"  data: [{q[3]}, {q[0]}, {q[1]}, {q[2]}]\n"
        "translation_sensor_body_imu:\n  rows: 1\n  cols: 3\n  dt: f\n"
        f"  data: [{T_lidar_body[0, 3]}, {T_lidar_body[1, 3]}, {T_lidar_body[2, 3]}]\n")
    np.testing.assert_allclose(fusionportable_extrinsic(calib, "body_imu"), T_lidar_body, atol=1e-12)

    gt = tmp_path / "gt.txt"
    gt_trajectory().to_tum(gt)
    out = tmp_path / "cli"
    assert main(["--bag", str(bag), "--gt", str(gt), "--out", str(out),
                 "--calib", str(calib), "--gt-frame", "body_imu"]) == 0
    rows, clouds = read_frames(out)
    recovered = apply(truth[round(rows[0][1], 6)], clouds[0].astype(float))
    assert np.max(np.linalg.norm(recovered - world, axis=1)) < 2e-3
