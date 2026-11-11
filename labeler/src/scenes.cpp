#include "degenbench/labeler/scenes.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <random>

namespace degenbench {
namespace {

/// Parallelogram patch o + a*u + b*v, a, b in [0, 1].
struct Patch {
  Eigen::Vector3d origin;
  Eigen::Vector3d u;
  Eigen::Vector3d v;
};

using SurfaceSampler = std::function<Eigen::Vector3d(std::mt19937_64&)>;

void add_patch_grid(const Patch& patch, double spacing, std::vector<Eigen::Vector3d>& out) {
  const int nu = std::max(1, static_cast<int>(std::ceil(patch.u.norm() / spacing)));
  const int nv = std::max(1, static_cast<int>(std::ceil(patch.v.norm() / spacing)));
  for (int i = 0; i <= nu; ++i) {
    for (int j = 0; j <= nv; ++j) {
      out.push_back(patch.origin + (static_cast<double>(i) / nu) * patch.u +
                    (static_cast<double>(j) / nv) * patch.v);
    }
  }
}

/// Area-weighted uniform sampler over a set of patches.
SurfaceSampler make_patch_sampler(std::vector<Patch> patches) {
  std::vector<double> areas;
  for (const auto& p : patches) areas.push_back(p.u.cross(p.v).norm());
  auto pick = std::make_shared<std::discrete_distribution<int>>(areas.begin(), areas.end());
  return [patches = std::move(patches), pick](std::mt19937_64& rng) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const Patch& p = patches[(*pick)(rng)];
    return Eigen::Vector3d(p.origin + unit(rng) * p.u + unit(rng) * p.v);
  };
}

/// Draw scan points from `sampler`, keep those within range of the sensor, express
/// them in the sensor frame and add measurement noise.
std::vector<Eigen::Vector3d> simulate_scan(const SurfaceSampler& sampler,
                                           const Eigen::Isometry3d& T_map_sensor,
                                           const SceneConfig& config) {
  std::mt19937_64 rng(config.seed);
  std::normal_distribution<double> noise(0.0, config.scan_noise_std);
  const Eigen::Isometry3d T_sensor_map = T_map_sensor.inverse();
  const Eigen::Vector3d sensor = T_map_sensor.translation();

  std::vector<Eigen::Vector3d> scan;
  scan.reserve(config.num_scan_points);
  const long max_draws = 1000L * config.num_scan_points;
  for (long draw = 0; draw < max_draws && static_cast<int>(scan.size()) < config.num_scan_points;
       ++draw) {
    const Eigen::Vector3d p = sampler(rng);
    if ((p - sensor).norm() > config.scan_radius) continue;
    scan.push_back(T_sensor_map * p + Eigen::Vector3d(noise(rng), noise(rng), noise(rng)));
  }
  return scan;
}

Eigen::Isometry3d translation_pose(double x, double y, double z) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.translation() << x, y, z;
  return T;
}

AnalyticScene make_patch_scene(std::string name, const std::vector<Patch>& map_patches,
                               const std::vector<Patch>& scan_patches,
                               const Eigen::Isometry3d& T_map_sensor, int expected,
                               std::array<bool, 6> free_axes, const SceneConfig& config) {
  AnalyticScene scene;
  scene.name = std::move(name);
  scene.T_map_sensor = T_map_sensor;
  scene.expected_num_degenerate = expected;
  scene.expected_free_axes = free_axes;
  for (const auto& p : map_patches) add_patch_grid(p, config.map_spacing, scene.map);
  scene.scan = simulate_scan(make_patch_sampler(scan_patches), T_map_sensor, config);
  return scene;
}

constexpr double kMapHalfLength = 40.0;  // Map extent along free directions [m].

}  // namespace

AnalyticScene make_plane_scene(const SceneConfig& config) {
  const double L = kMapHalfLength;
  const double R = config.scan_radius;
  const Patch map{{-L, -L, 0}, {2 * L, 0, 0}, {0, 2 * L, 0}};
  const Patch scan{{-R, -R, 0}, {2 * R, 0, 0}, {0, 2 * R, 0}};
  return make_patch_scene("plane", {map}, {scan}, translation_pose(0, 0, 1.5), 3,
                          {true, true, false, false, false, true}, config);
}

AnalyticScene make_corridor_scene(const SceneConfig& config) {
  constexpr double half_width = 1.5;
  constexpr double height = 2.5;
  auto faces = [&](double half_length) {
    const double L = half_length;
    const double W = half_width;
    return std::vector<Patch>{
        {{-L, -W, 0}, {2 * L, 0, 0}, {0, 2 * W, 0}},       // floor
        {{-L, -W, height}, {2 * L, 0, 0}, {0, 2 * W, 0}},  // ceiling
        {{-L, -W, 0}, {2 * L, 0, 0}, {0, 0, height}},      // right wall
        {{-L, W, 0}, {2 * L, 0, 0}, {0, 0, height}},       // left wall
    };
  };
  return make_patch_scene("corridor", faces(kMapHalfLength), faces(config.scan_radius),
                          translation_pose(0, 0, 1.2), 1,
                          {true, false, false, false, false, false}, config);
}

AnalyticScene make_cylinder_scene(const SceneConfig& config) {
  constexpr double radius = 2.0;
  AnalyticScene scene;
  scene.name = "cylinder";
  scene.T_map_sensor = Eigen::Isometry3d::Identity();  // Sensor on the tube axis.
  scene.expected_num_degenerate = 2;
  scene.expected_free_axes = {true, false, false, true, false, false};

  const int n_around = static_cast<int>(std::ceil(2 * M_PI * radius / config.curved_spacing));
  const int n_along = static_cast<int>(std::ceil(2 * kMapHalfLength / config.map_spacing));
  for (int i = 0; i <= n_along; ++i) {
    const double x = -kMapHalfLength + 2 * kMapHalfLength * i / n_along;
    for (int j = 0; j < n_around; ++j) {
      const double a = 2 * M_PI * j / n_around;
      scene.map.emplace_back(x, radius * std::cos(a), radius * std::sin(a));
    }
  }

  const double R = config.scan_radius;
  const SurfaceSampler sampler = [R](std::mt19937_64& rng) {
    std::uniform_real_distribution<double> along(-R, R);
    std::uniform_real_distribution<double> angle(0.0, 2 * M_PI);
    const double a = angle(rng);
    return Eigen::Vector3d(along(rng), radius * std::cos(a), radius * std::sin(a));
  };
  scene.scan = simulate_scan(sampler, scene.T_map_sensor, config);
  return scene;
}

AnalyticScene make_corner_scene(const SceneConfig& config) {
  constexpr double extent = 12.0;
  const std::vector<Patch> faces{
      {{0, 0, 0}, {extent, 0, 0}, {0, extent, 0}},  // z = 0
      {{0, 0, 0}, {extent, 0, 0}, {0, 0, extent}},  // y = 0
      {{0, 0, 0}, {0, extent, 0}, {0, 0, extent}},  // x = 0
  };
  return make_patch_scene("corner", faces, faces, translation_pose(3, 3, 3), 0, {}, config);
}

std::vector<AnalyticScene> make_all_analytic_scenes(const SceneConfig& config) {
  return {make_plane_scene(config), make_corridor_scene(config), make_cylinder_scene(config),
          make_corner_scene(config)};
}

}  // namespace degenbench
