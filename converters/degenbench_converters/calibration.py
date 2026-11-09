"""Read extrinsics from FusionPortable calibration YAML files.

FusionPortable stores, in `ouster00.yaml`, keys such as
`quaternion_sensor_body_imu` (qw, qx, qy, qz) and `translation_sensor_body_imu`:
the pose of `body_imu` in the Ouster sensor frame, i.e. T_sensor_body_imu.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from .poses import make_transform, quat_to_rot


def fusionportable_extrinsic(path: str | Path, target_frame: str) -> np.ndarray:
    """T_sensor_<target_frame> from a FusionPortable sensor calibration file."""
    with open(path, encoding="utf-8") as f:
        calib = yaml.safe_load(f)
    q_key = f"quaternion_sensor_{target_frame}"
    t_key = f"translation_sensor_{target_frame}"
    if q_key not in calib or t_key not in calib:
        raise KeyError(f"{path}: no extrinsic for frame '{target_frame}'")
    qw, qx, qy, qz = calib[q_key]["data"]
    return make_transform(quat_to_rot(np.array([qx, qy, qz, qw])), np.array(calib[t_key]["data"]))
