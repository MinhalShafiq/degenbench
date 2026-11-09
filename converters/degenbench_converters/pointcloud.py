"""Decode sensor_msgs/PointCloud2 into numpy arrays."""

from __future__ import annotations

import numpy as np

# sensor_msgs/PointField datatype codes.
_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4", 7: "f4", 8: "f8"}


def decode_fields(msg, names: list[str]) -> dict[str, np.ndarray]:
    """Return the requested fields of a PointCloud2 as flat arrays, one per name."""
    fields = {f.name: f for f in msg.fields}
    missing = [n for n in names if n not in fields]
    if missing:
        raise KeyError(f"PointCloud2 lacks fields {missing}; has {sorted(fields)}")
    order = ">" if msg.is_bigendian else "<"
    dtype = np.dtype({
        "names": names,
        "formats": [order + _DTYPES[fields[n].datatype] for n in names],
        "offsets": [fields[n].offset for n in names],
        "itemsize": msg.point_step,
    })
    count = msg.width * msg.height
    data = np.frombuffer(bytes(msg.data), dtype=dtype, count=count)
    return {n: data[n].astype(np.float64) for n in names}
