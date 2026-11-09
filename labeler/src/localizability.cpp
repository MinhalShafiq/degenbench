#include "degenbench/labeler/localizability.hpp"

#include <cmath>
#include <random>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <small_gicp/registration/registration_helper.hpp>

namespace degenbench {

const char* to_string(Solver solver) {
  switch (solver) {
    case Solver::PointToPlaneICP: return "point_to_plane_icp";
    case Solver::GICP: return "gicp";
  }
  return "unknown";
}

Vector6d pose_error(const Eigen::Isometry3d& T_gt, const Eigen::Isometry3d& T_est) {
  const Eigen::Isometry3d T_err = T_gt.inverse() * T_est;
  const Eigen::AngleAxisd aa(T_err.linear());
  Vector6d e;
  e.head<3>() = T_err.translation();
  e.tail<3>() = aa.angle() * aa.axis();
  return e;
}

Eigen::Isometry3d offset_to_pose(const Vector6d& offset) {
  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d rv = offset.tail<3>();
  const double angle = rv.norm();
  if (angle > 1e-12) T.linear() = Eigen::AngleAxisd(angle, rv / angle).toRotationMatrix();
  T.translation() = offset.head<3>();
  return T;
}

FrameLabel label_frame(const std::vector<Eigen::Vector3d>& map,
                       const std::vector<Eigen::Vector3d>& scan,
                       const Eigen::Isometry3d& T_map_sensor, const LabelConfig& config) {
  if (map.empty() || scan.empty()) throw std::invalid_argument("label_frame: empty point cloud");
  if (config.num_samples < 2) throw std::invalid_argument("label_frame: need >= 2 samples");
  // Along a truly free direction the solver leaves the offset unchanged, so the
  // normalized eigenvalue equals the offset variance over threshold^2. A uniform
  // offset on [-b, b] has RMS b / sqrt(3); if that is below the threshold, free
  // directions could never be detected.
  if (config.max_offset_trans / std::sqrt(3.0) <= config.trans_threshold ||
      config.max_offset_rot / std::sqrt(3.0) <= config.rot_threshold) {
    throw std::invalid_argument("label_frame: offset range too small to reveal free directions");
  }

  using small_gicp::RegistrationSetting;
  // Single-threaded: small_gicp's parallel downsampling is not deterministic, and
  // labels must be bit-reproducible from the released generator.
  constexpr int prep_threads = 1;
  auto [target, target_tree] = small_gicp::preprocess_points(
      map, config.downsampling_resolution, config.num_neighbors, prep_threads);
  auto [source, source_tree] = small_gicp::preprocess_points(
      scan, config.downsampling_resolution, config.num_neighbors, prep_threads);
  (void)source_tree;

  RegistrationSetting setting;
  setting.type = config.solver == Solver::GICP ? RegistrationSetting::GICP
                                               : RegistrationSetting::PLANE_ICP;
  setting.max_correspondence_distance = config.max_correspondence_distance;
  setting.max_iterations = config.max_iterations;
  setting.num_threads = 1;  // Parallelism is across Monte Carlo samples.

  FrameLabel label;
  label.offsets.resize(config.num_samples);
  label.final_errors.resize(config.num_samples);

  // Draw all offsets up front so results do not depend on thread scheduling.
  std::mt19937_64 rng(config.seed);
  std::uniform_real_distribution<double> ut(-config.max_offset_trans, config.max_offset_trans);
  std::uniform_real_distribution<double> ur(-config.max_offset_rot, config.max_offset_rot);
  for (auto& o : label.offsets) o << ut(rng), ut(rng), ut(rng), ur(rng), ur(rng), ur(rng);

#pragma omp parallel for num_threads(config.num_threads) schedule(dynamic)
  for (int i = 0; i < config.num_samples; ++i) {
    const Eigen::Isometry3d init = T_map_sensor * offset_to_pose(label.offsets[i]);
    const auto result = small_gicp::align(*target, *source, *target_tree, init, setting);
    label.final_errors[i] = pose_error(T_map_sensor, result.T_target_source);
  }

  Vector6d scale;
  scale << Eigen::Vector3d::Constant(config.trans_threshold),
      Eigen::Vector3d::Constant(config.rot_threshold);

  // Second moment about zero: a consistent bias beyond the threshold is as much a
  // localization failure as spread is.
  Matrix6d moment = Matrix6d::Zero();
  Vector6d sq = Vector6d::Zero();
  for (const auto& e : label.final_errors) {
    const Vector6d en = e.cwiseQuotient(scale);
    moment += en * en.transpose();
    sq += e.cwiseProduct(e);
  }
  moment /= config.num_samples;
  label.normalized_second_moment = moment;
  label.rms_error = (sq / config.num_samples).cwiseSqrt();

  const Eigen::SelfAdjointEigenSolver<Matrix6d> eig(moment);
  label.eigenvalues = eig.eigenvalues();
  label.eigenvectors = eig.eigenvectors();

  for (int i = 0; i < 6; ++i) label.num_degenerate += label.eigenvalues[i] > 1.0;
  label.degenerate_basis = label.eigenvectors.rightCols(label.num_degenerate);

  for (int i = 0; i < 6; ++i) label.axis_degenerate[i] = moment(i, i) > 1.0;
  return label;
}

}  // namespace degenbench
