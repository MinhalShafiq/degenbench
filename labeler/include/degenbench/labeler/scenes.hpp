#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace degenbench {

/// Synthetic scene with an analytically known set of unobservable directions.
/// Used only to validate the labeler, never as benchmark data.
struct AnalyticScene {
  std::string name;
  std::vector<Eigen::Vector3d> map;   ///< Map frame.
  std::vector<Eigen::Vector3d> scan;  ///< Sensor frame.
  Eigen::Isometry3d T_map_sensor = Eigen::Isometry3d::Identity();

  int expected_num_degenerate = 0;
  /// Expected free axes in the sensor frame, ordered [tx, ty, tz, rx, ry, rz].
  std::array<bool, 6> expected_free_axes{};
};

struct SceneConfig {
  double map_spacing = 0.2;      ///< Approximate map point spacing [m].
  /// Spacing around curved surfaces [m]. Must be much finer than map_spacing: a
  /// tube sampled at 0.2 m is a 63-sided prism whose facets (every 5.7 deg)
  /// constrain rotation about its axis and make a free direction look partly
  /// constrained.
  double curved_spacing = 0.02;
  int num_scan_points = 6000;
  double scan_radius = 8.0;      ///< Scan points are kept within this range of the sensor [m].
  double scan_noise_std = 0.01;  ///< Isotropic Gaussian noise on scan points [m].
  std::uint64_t seed = 7;
};

/// Infinite ground plane: tx, ty and rz are free (3).
AnalyticScene make_plane_scene(const SceneConfig& config = {});

/// Straight rectangular corridor along x: tx is free (1).
AnalyticScene make_corridor_scene(const SceneConfig& config = {});

/// Circular tube along x: tx and rx are free (2).
AnalyticScene make_cylinder_scene(const SceneConfig& config = {});

/// Three mutually orthogonal planes meeting at a corner: fully constrained (0).
AnalyticScene make_corner_scene(const SceneConfig& config = {});

std::vector<AnalyticScene> make_all_analytic_scenes(const SceneConfig& config = {});

}  // namespace degenbench
