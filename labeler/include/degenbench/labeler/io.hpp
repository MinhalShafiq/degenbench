#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace degenbench {

/// One frame written by `degenbench_converters.extract_frames`.
struct FrameRecord {
  int index = 0;
  double stamp = 0.0;
  Eigen::Isometry3d T_world_lidar = Eigen::Isometry3d::Identity();
  int num_points = 0;
};

/// Read `frames.csv` (index,stamp,tx,ty,tz,qx,qy,qz,qw,num_points).
std::vector<FrameRecord> read_frames_csv(const std::string& path);

/// Read a frame `.bin` file: little-endian float32 x y z triplets.
std::vector<Eigen::Vector3d> read_bin_points(const std::string& path);

/// Read x, y, z from a PCD file (ascii, binary or binary_compressed; float32 or
/// float64 fields). Other fields are skipped.
std::vector<Eigen::Vector3d> read_pcd(const std::string& path);

/// LZF decompression as used by PCD binary_compressed. Throws on corrupt input
/// or if the output is not exactly `expected_size` bytes.
std::vector<char> lzf_decompress(const char* in, size_t in_size, size_t expected_size);

/// Write x, y, z as a binary float32 PCD (for tests and debugging).
void write_pcd(const std::string& path, const std::vector<Eigen::Vector3d>& points);

}  // namespace degenbench
