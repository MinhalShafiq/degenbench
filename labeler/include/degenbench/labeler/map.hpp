#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace degenbench {

/// Voxel-grid downsampling (one centroid per occupied voxel).
std::vector<Eigen::Vector3d> voxel_downsample(const std::vector<Eigen::Vector3d>& points,
                                              double leaf_size);

/// Points within `radius` of `center`.
std::vector<Eigen::Vector3d> crop_radius(const std::vector<Eigen::Vector3d>& points,
                                         const Eigen::Vector3d& center, double radius);

/// Fraction of scan points (sensor frame) that, placed at `T_map_sensor`, have a
/// map point within `max_distance`. Low coverage means the map does not explain
/// the scan (unmapped area, dynamic objects, or a wrong ground-truth pose), and
/// such frames are flagged rather than labelled silently.
double map_coverage(const std::vector<Eigen::Vector3d>& map,
                    const std::vector<Eigen::Vector3d>& scan,
                    const Eigen::Isometry3d& T_map_sensor, double max_distance);

}  // namespace degenbench
