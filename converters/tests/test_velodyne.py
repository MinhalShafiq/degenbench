import numpy as np
import pytest

from degenbench_converters.velodyne import (
    CHANNEL_SPACING, ELEVATION_DEG, SEQUENCE_PERIOD, decode_packet, encode_packet,
)

AZ = np.arange(12) * 0.4 + 10.0  # 0.4 deg per block, ~ 10 Hz rotation


def test_geometry_of_known_returns():
    # Block 0, slot 1 = channel 1 (+1 deg), 10 m, at azimuth 10 deg.
    d = decode_packet(encode_packet({(0, 1): 10.0}, AZ))
    assert len(d["x"]) == 1
    a = np.radians(10.0 + 0.4 * CHANNEL_SPACING / (2 * SEQUENCE_PERIOD))
    w = np.radians(1.0)
    np.testing.assert_allclose([d["x"][0], d["y"][0], d["z"][0]],
                               [10 * np.cos(w) * np.cos(a), -10 * np.cos(w) * np.sin(a), 10 * np.sin(w)],
                               atol=1e-3)
    assert d["ring"][0] == 1


def test_azimuth_clockwise_means_negative_y():
    d = decode_packet(encode_packet({(0, 0): 5.0}, np.arange(12) * 0.4 + 90.0))
    assert d["y"][0] == pytest.approx(-5 * np.cos(np.radians(-15)), abs=1e-2)
    assert abs(d["x"][0]) < 0.05


def test_second_sequence_is_interpolated_and_timed():
    d = decode_packet(encode_packet({(2, 16 + 3): 8.0}, AZ))  # sequence 1, channel 3
    expected_t = 2 * 2 * SEQUENCE_PERIOD + SEQUENCE_PERIOD + 3 * CHANNEL_SPACING
    assert d["t"][0] == pytest.approx(expected_t)
    az = np.degrees(np.arctan2(-d["y"][0], d["x"][0]))
    assert az == pytest.approx(10.8 + 0.4 * (SEQUENCE_PERIOD + 3 * CHANNEL_SPACING) / (2 * SEQUENCE_PERIOD), abs=1e-3)
    assert d["z"][0] == pytest.approx(8 * np.sin(np.radians(ELEVATION_DEG[3])), abs=1e-3)


def test_azimuth_wraparound():
    az = np.mod(np.arange(12) * 0.4 + 359.0, 360.0)
    d = decode_packet(encode_packet({(b, 0): 5.0 for b in range(12)}, az))
    a = np.mod(np.degrees(np.arctan2(-d["y"], d["x"])), 360.0)
    steps = np.mod(np.diff(a), 360.0)
    np.testing.assert_allclose(steps, 0.4, atol=0.02)


def test_no_return_is_dropped_and_bad_packets_rejected():
    assert len(decode_packet(encode_packet({}, AZ))["x"]) == 0
    with pytest.raises(ValueError, match="return mode"):
        decode_packet(encode_packet({}, AZ, mode=0x39))
    with pytest.raises(ValueError, match="bytes"):
        decode_packet(b"\x00" * 100)
