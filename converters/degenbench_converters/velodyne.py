"""Decode raw Velodyne VLP-16 packets (velodyne_msgs/VelodyneScan) into points.

Implements the VLP-16 single-return packet layout and firing timing from the
VLP-16 user manual: 12 data blocks per 1206-byte packet, each with a 0xFFEE
flag, a uint16 azimuth (0.01 deg), and two firing sequences of 16 channels
(uint16 distance in 2 mm units, uint8 reflectivity), then a uint32 timestamp
and two factory bytes (return mode, product id).

Axes follow ROS velodyne_pointcloud: x forward, y left, z up, i.e.
x = r cos(w) cos(a), y = -r cos(w) sin(a), z = r sin(w) for azimuth a
(clockwise from x seen from above) and elevation w. Nominal geometry is used
(no per-laser distance or offset corrections; the VLP-16 database has none).
"""

from __future__ import annotations

import numpy as np

PACKET_SIZE = 1206
BLOCKS = 12
CHANNELS = 16
BLOCK_BYTES = 100
DISTANCE_UNIT = 0.002                 # m
CHANNEL_SPACING = 2.304e-6            # s between channels within a sequence
SEQUENCE_PERIOD = 55.296e-6           # s between firing sequences
RETURN_MODES = {0x37: "strongest", 0x38: "last", 0x39: "dual"}
PRODUCT_VLP16 = 0x22

ELEVATION_DEG = np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15], float)

_seq = np.repeat([0, 1], CHANNELS)                   # (32,)
_chan = np.tile(np.arange(CHANNELS), 2)              # (32,)
# Firing time of each of the 32 slots within a block, and of each block.
_slot_time = _seq * SEQUENCE_PERIOD + _chan * CHANNEL_SPACING
_block_time = np.arange(BLOCKS) * 2 * SEQUENCE_PERIOD


def decode_packet(data: bytes | np.ndarray) -> dict[str, np.ndarray]:
    """Decode one packet. Returns x, y, z, time offset [s from packet], ring.

    Points with no return (distance 0) are dropped.
    """
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if buf.size != PACKET_SIZE:
        raise ValueError(f"packet has {buf.size} bytes, expected {PACKET_SIZE}")
    mode, product = int(buf[1204]), int(buf[1205])
    if product != PRODUCT_VLP16:
        raise ValueError(f"product id 0x{product:02x} is not a VLP-16")
    if RETURN_MODES.get(mode) not in ("strongest", "last"):
        raise ValueError(f"return mode 0x{mode:02x} not supported (single return only)")

    blocks = buf[:BLOCKS * BLOCK_BYTES].reshape(BLOCKS, BLOCK_BYTES)
    if not np.all((blocks[:, 0] == 0xFF) & (blocks[:, 1] == 0xEE)):
        raise ValueError("bad block flag")
    azimuth = (blocks[:, 2].astype(np.uint16) | (blocks[:, 3].astype(np.uint16) << 8)) / 100.0
    returns = blocks[:, 4:].reshape(BLOCKS, 32, 3)
    dist = (returns[:, :, 0].astype(np.uint16) | (returns[:, :, 1].astype(np.uint16) << 8)) * DISTANCE_UNIT

    # Azimuth advance per block, from neighbours (last block reuses the previous step).
    step = np.diff(azimuth)
    step = np.where(step < 0, step + 360.0, step)
    step = np.append(step, step[-1])
    az = azimuth[:, None] + step[:, None] * (_slot_time[None, :] / (2 * SEQUENCE_PERIOD))
    az = np.mod(az, 360.0)

    w = np.radians(ELEVATION_DEG[_chan])[None, :].repeat(BLOCKS, 0)
    a = np.radians(az)
    xy = dist * np.cos(w)
    t = _block_time[:, None] + _slot_time[None, :]
    ring = np.broadcast_to(_chan, (BLOCKS, 32))
    keep = dist > 0
    return {
        "x": (xy * np.cos(a))[keep], "y": (-xy * np.sin(a))[keep], "z": (dist * np.sin(w))[keep],
        "t": t[keep], "ring": ring[keep].astype(np.int32),
    }


def decode_scan(msg) -> tuple[np.ndarray, np.ndarray]:
    """Decode a velodyne_msgs/VelodyneScan into (N x 3 points, N absolute times [s])."""
    pts, times = [], []
    for pkt in msg.packets:
        d = decode_packet(pkt.data)
        stamp = pkt.stamp.sec + 1e-9 * pkt.stamp.nanosec
        pts.append(np.column_stack([d["x"], d["y"], d["z"]]))
        times.append(stamp + d["t"])
    if not pts:
        return np.zeros((0, 3)), np.zeros(0)
    return np.vstack(pts), np.concatenate(times)


def encode_packet(points_by_slot: dict[tuple[int, int], float], azimuths_deg: np.ndarray,
                  mode: int = 0x37) -> bytes:
    """Build a packet (for tests): {(block, slot): distance_m}, 12 block azimuths."""
    buf = bytearray(PACKET_SIZE)
    for b in range(BLOCKS):
        o = b * BLOCK_BYTES
        buf[o:o + 2] = b"\xff\xee"
        buf[o + 2:o + 4] = int(round(azimuths_deg[b] * 100)).to_bytes(2, "little")
        for s in range(32):
            d = points_by_slot.get((b, s), 0.0)
            buf[o + 4 + 3 * s:o + 6 + 3 * s] = int(round(d / DISTANCE_UNIT)).to_bytes(2, "little")
    buf[1204], buf[1205] = mode, PRODUCT_VLP16
    return bytes(buf)
