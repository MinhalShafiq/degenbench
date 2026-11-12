#pragma once
/// Inputs shared by the scored degeneracy detectors, computed once per frame at
/// the reference pose.
///
/// Parameters are a perturbation (t, theta) of the sensor pose,
/// T = T_ref * (Exp(theta), t), in the labeler's order [tx ty tz rx ry rz] and in
/// physical units (m, rad). This is the parametrization of the labels'
/// final errors (degenbench::pose_error), so detector subspaces and labels live
/// in the same space.
///
/// Each detector's decision (a threshold on these quantities) is applied in
/// leaderboard/degenbench_leaderboard/detectors.py, so thresholds can be
/// calibrated without recomputing features.

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "degenbench/labeler/localizability.hpp"

namespace degenbench::detectors {

using degenbench::Matrix6d;
using degenbench::Vector6d;

/// Point-to-plane correspondences, both expressed in the sensor frame.
struct Correspondences {
  std::vector<Eigen::Vector3d> points;   ///< scan point p_i
  std::vector<Eigen::Vector3d> normals;  ///< unit map normal n_i, rotated into the sensor frame
  size_t size() const { return points.size(); }
};

/// Nearest map point within `max_distance` of each prepared scan point, with its
/// normal. Points whose map neighbour has no valid normal are skipped.
Correspondences find_correspondences(const PreparedCloud& map, const PreparedCloud& scan,
                                     const Eigen::Isometry3d& T_map_sensor, double max_distance);

/// Point-to-plane Jacobian of correspondence i: J_i = [n_i ; p_i x n_i].
Vector6d point_to_plane_jacobian(const Eigen::Vector3d& p, const Eigen::Vector3d& n);

/// H = sum_i J_i J_i^T, the Gauss-Newton information matrix (Zhang et al. 2016,
/// X-ICP, DCReg all start from it).
Matrix6d point_to_plane_information(const Correspondences& c);

/// M = sum_i D_i^T D_i with D_i = [I_3, -[p_i]x], the point-displacement metric of
/// "Degenerate in Whose Frame?" (2026): x^T M x is the summed squared displacement
/// of the scan points under perturbation x.
Matrix6d point_displacement_metric(const Correspondences& c);

/// X-ICP (Tuna et al. 2023) localizability contributions.
///
/// Translation and rotation blocks of H are eigendecomposed separately. Each
/// correspondence contributes |n_i . v| to translation eigenvector v and
/// |tau_i / |tau_i| . v| to rotation eigenvector v, where tau_i = p_i x n_i; pairs
/// with |tau_i| < min_torque are left out of the rotation part. Contributions
/// below kappa_f are dropped (L_c); L_s keeps only those >= cos 45 deg.
struct XicpContributions {
  Eigen::Matrix3d V_t;     ///< columns: translation eigenvectors, ascending eigenvalue
  Eigen::Matrix3d V_r;     ///< columns: rotation eigenvectors, ascending eigenvalue
  Eigen::Vector3d L_c_t, L_c_r;  ///< combined contribution per eigenvector
  Eigen::Vector3d L_s_t, L_s_r;  ///< strong contribution per eigenvector
};

struct XicpConfig {
  double kappa_f = 0.17364817766693033;  ///< cos 80 deg, the paper's Velodyne value
  double strong = 0.70710678118654752;   ///< cos 45 deg
  double min_torque = 1e-6;
};

XicpContributions xicp_contributions(const Correspondences& c, const Matrix6d& H,
                                     const XicpConfig& config = {});

}  // namespace degenbench::detectors
