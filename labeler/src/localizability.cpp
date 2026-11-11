#include "degenbench/labeler/localizability.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
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
  label.iterations.resize(config.num_samples);
  label.converged.resize(config.num_samples);
  label.inliers.resize(config.num_samples);

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
    label.iterations[i] = static_cast<int>(result.iterations);
    label.converged[i] = result.converged ? 1 : 0;
    label.inliers[i] = static_cast<int>(result.num_inliers);
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

Information information_at(const std::vector<Eigen::Vector3d>& map,
                           const std::vector<Eigen::Vector3d>& scan,
                           const Eigen::Isometry3d& T_map_sensor, const LabelConfig& config) {
  auto [target, tree] = small_gicp::preprocess_points(map, config.downsampling_resolution,
                                                      config.num_neighbors, 1);
  auto [source, stree] = small_gicp::preprocess_points(scan, config.downsampling_resolution,
                                                       config.num_neighbors, 1);
  (void)stree;
  small_gicp::RegistrationSetting s;
  s.type = config.solver == Solver::GICP ? small_gicp::RegistrationSetting::GICP
                                         : small_gicp::RegistrationSetting::PLANE_ICP;
  s.max_correspondence_distance = config.max_correspondence_distance;
  s.max_iterations = 1;  // linearize at the given pose; H is evaluated before the step
  s.num_threads = std::max(1, config.num_threads);
  const auto r = small_gicp::align(*target, *source, *tree, T_map_sensor, s);
  // small_gicp orders the tangent space [r; t]; reorder to [t; r] and normalize.
  Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
  P.block<3, 3>(0, 3).setIdentity();
  P.block<3, 3>(3, 0).setIdentity();
  Vector6d scale;
  scale << Eigen::Vector3d::Constant(config.trans_threshold),
      Eigen::Vector3d::Constant(config.rot_threshold);
  Information info;
  const Matrix6d H = P.transpose() * r.H * P;
  info.H_normalized = scale.asDiagonal() * H * scale.asDiagonal();
  const Eigen::SelfAdjointEigenSolver<Matrix6d> eig(info.H_normalized);
  info.eigenvalues = eig.eigenvalues();
  info.eigenvectors = eig.eigenvectors();
  info.inliers = static_cast<int>(r.num_inliers);
  return info;
}

RefinedPose refine_pose(const std::vector<Eigen::Vector3d>& map,
                        const std::vector<Eigen::Vector3d>& scan,
                        const Eigen::Isometry3d& T_init, const LabelConfig& config) {
  auto [target, tree] = small_gicp::preprocess_points(map, config.downsampling_resolution,
                                                      config.num_neighbors, 1);
  auto [source, stree] = small_gicp::preprocess_points(scan, config.downsampling_resolution,
                                                       config.num_neighbors, 1);
  (void)stree;
  small_gicp::RegistrationSetting s;
  s.type = small_gicp::RegistrationSetting::GICP;
  s.max_iterations = config.max_iterations;
  s.num_threads = std::max(1, config.num_threads);
  RefinedPose out;
  Eigen::Isometry3d T = T_init;
  // Coarse to fine, ending at the probe's own correspondence distance.
  for (double d : {2.0 * config.max_correspondence_distance, config.max_correspondence_distance}) {
    s.max_correspondence_distance = d;
    const auto r = small_gicp::align(*target, *source, *tree, T, s);
    T = r.T_target_source;
    out.converged = r.converged;
  }
  out.T_map_sensor = T;
  out.correction = pose_error(T_init, T);
  return out;
}

namespace {

Eigen::MatrixXd orthonormal_basis(const Eigen::MatrixXd& a) {
  if (a.rows() != 6) throw std::invalid_argument("subspace basis must have 6 rows");
  if (a.cols() == 0) return Eigen::MatrixXd(6, 0);
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(a, Eigen::ComputeThinU);
  const auto& s = svd.singularValues();
  const double tol = 1e-9 * std::max(1.0, s[0]);
  int rank = 0;
  while (rank < s.size() && s[rank] > tol) ++rank;
  return svd.matrixU().leftCols(rank);
}

}  // namespace

double max_principal_angle_deg(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b) {
  const Eigen::MatrixXd qa = orthonormal_basis(a);
  const Eigen::MatrixXd qb = orthonormal_basis(b);
  if (qa.cols() != qb.cols()) return 90.0;
  if (qa.cols() == 0) return 0.0;
  // Largest angle: its cosine is the smallest singular value of qa^T qb and its
  // sine the largest singular value of qb's residual off span(qa). atan2 of the
  // two stays accurate near 0, where acos alone loses ~1e-6 deg to rounding.
  const Eigen::MatrixXd proj = qa.transpose() * qb;
  const Eigen::MatrixXd residual = qb - qa * proj;
  const double c = Eigen::JacobiSVD<Eigen::MatrixXd>(proj).singularValues().minCoeff();
  const double s = Eigen::JacobiSVD<Eigen::MatrixXd>(residual).singularValues().maxCoeff();
  return std::atan2(s, c) * 180.0 / M_PI;
}

DualSolverLabel label_frame_dual(const std::vector<Eigen::Vector3d>& map,
                                 const std::vector<Eigen::Vector3d>& scan,
                                 const Eigen::Isometry3d& T_map_sensor, LabelConfig config,
                                 double max_agreement_angle_deg) {
  DualSolverLabel out;
  config.solver = Solver::PointToPlaneICP;
  out.point_to_plane_icp = label_frame(map, scan, T_map_sensor, config);
  config.solver = Solver::GICP;  // Same seed, so both solvers see identical offsets.
  out.gicp = label_frame(map, scan, T_map_sensor, config);
  out.subspace_angle_deg = max_principal_angle_deg(out.point_to_plane_icp.degenerate_basis,
                                                   out.gicp.degenerate_basis);
  out.solvers_agree = out.subspace_angle_deg <= max_agreement_angle_deg;
  return out;
}

}  // namespace degenbench
