#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace degenbench {

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

/// Registration solver used to probe a frame.
enum class Solver { PointToPlaneICP, GICP };

const char* to_string(Solver solver);

/// Settings for the Monte Carlo localizability probe.
///
/// Error vectors are ordered [tx, ty, tz, rx, ry, rz] in the sensor frame,
/// translation in metres and rotation as a rotation vector in radians.
struct LabelConfig {
  Solver solver = Solver::PointToPlaneICP;

  int num_samples = 200;          ///< Random initial offsets per frame.
  double max_offset_trans = 0.5;  ///< Uniform per-axis translation offset bound [m].
  double max_offset_rot = 5.0 * M_PI / 180.0;  ///< Uniform per-axis rotation offset bound [rad].

  /// A direction is degenerate when the RMS final error along it exceeds these.
  /// Defaults follow the convergence thresholds of Nubert et al. (2022).
  double trans_threshold = 0.10;              ///< [m]
  double rot_threshold = 2.0 * M_PI / 180.0;  ///< [rad]

  double downsampling_resolution = 0.25;  ///< Voxel size for scan and map [m].
  double max_correspondence_distance = 1.0;  ///< [m]
  int num_neighbors = 10;   ///< Neighbours for normal / covariance estimation.
  int max_iterations = 50;
  int num_threads = 4;      ///< Threads across Monte Carlo samples.
  std::uint64_t seed = 42;
};

/// Per-frame localizability label.
///
/// All spectral quantities live in threshold-normalized coordinates: translation
/// is divided by `trans_threshold` and rotation by `rot_threshold`. This makes the
/// 6D eigen-decomposition independent of the choice of units, and a direction is
/// degenerate exactly when its normalized RMS spread exceeds 1.
struct FrameLabel {
  std::vector<Vector6d> offsets;       ///< Initial offsets applied (physical units).
  std::vector<Vector6d> final_errors;  ///< Final registration errors (physical units).

  Vector6d rms_error;  ///< Per-axis RMS final error (physical units).

  Matrix6d normalized_second_moment;  ///< E[e_n e_n^T] about zero.
  Vector6d eigenvalues;               ///< Ascending.
  Matrix6d eigenvectors;              ///< Columns match `eigenvalues`.

  int num_degenerate = 0;          ///< Eigenvalues above 1.
  Eigen::MatrixXd degenerate_basis;  ///< 6 x num_degenerate, orthonormal, normalized coords.

  /// Per-axis flags for compatibility with axis-aligned labels: axis i is flagged
  /// when its own RMS error exceeds the threshold.
  std::array<bool, 6> axis_degenerate{};
};

/// Probe how well `scan` (sensor frame) can be registered to `map` (map frame)
/// around the ground-truth pose `T_map_sensor`.
FrameLabel label_frame(const std::vector<Eigen::Vector3d>& map,
                       const std::vector<Eigen::Vector3d>& scan,
                       const Eigen::Isometry3d& T_map_sensor,
                       const LabelConfig& config);

/// Largest principal angle [deg] between the column spans of two 6 x k bases.
/// A dimension mismatch returns 90; two empty subspaces return 0. Matches
/// `degenbench_leaderboard.subspace.subspace_error_deg`.
double max_principal_angle_deg(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b);

/// Labels from both solvers on one frame, plus whether they agree.
struct DualSolverLabel {
  FrameLabel point_to_plane_icp;
  FrameLabel gicp;
  double subspace_angle_deg = 0.0;  ///< Between the two degenerate subspaces.
  bool solvers_agree = false;
};

/// Run the probe with point-to-plane ICP and GICP on the same offsets. They agree
/// when the subspace angle is at most `max_agreement_angle_deg` (pre-registered:
/// 15 deg); disagreeing frames are flagged, never dropped.
DualSolverLabel label_frame_dual(const std::vector<Eigen::Vector3d>& map,
                                 const std::vector<Eigen::Vector3d>& scan,
                                 const Eigen::Isometry3d& T_map_sensor, LabelConfig config,
                                 double max_agreement_angle_deg = 15.0);

/// Pose error T_gt^-1 * T_est as [t; rotation vector].
Vector6d pose_error(const Eigen::Isometry3d& T_gt, const Eigen::Isometry3d& T_est);

/// Inverse of pose_error's parameterization: [t; rotation vector] -> pose.
Eigen::Isometry3d offset_to_pose(const Vector6d& offset);

}  // namespace degenbench
