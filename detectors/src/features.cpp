#include "degenbench/detectors/features.hpp"

#include <cmath>

#include <Eigen/Eigenvalues>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>

namespace degenbench::detectors {

Correspondences find_correspondences(const PreparedCloud& map, const PreparedCloud& scan,
                                     const Eigen::Isometry3d& T_map_sensor, double max_distance) {
  const double max_sq = max_distance * max_distance;
  const Eigen::Matrix3d R_sensor_map = T_map_sensor.linear().transpose();
  Correspondences out;
  out.points.reserve(scan.size());
  out.normals.reserve(scan.size());
  for (size_t i = 0; i < scan.size(); ++i) {
    const Eigen::Vector3d p = scan.cloud->point(i).head<3>();
    Eigen::Vector4d q;
    q << T_map_sensor * p, 1.0;
    size_t idx;
    double sq;
    if (!map.tree->nearest_neighbor_search(q, &idx, &sq) || sq > max_sq) continue;
    const Eigen::Vector3d n = map.cloud->normal(idx).head<3>();
    const double norm = n.norm();
    if (!std::isfinite(norm) || norm < 0.5) continue;
    out.points.push_back(p);
    out.normals.push_back(R_sensor_map * (n / norm));
  }
  return out;
}

Vector6d point_to_plane_jacobian(const Eigen::Vector3d& p, const Eigen::Vector3d& n) {
  Vector6d j;
  j << n, p.cross(n);
  return j;
}

Matrix6d point_to_plane_information(const Correspondences& c) {
  Matrix6d H = Matrix6d::Zero();
  for (size_t i = 0; i < c.size(); ++i) {
    const Vector6d j = point_to_plane_jacobian(c.points[i], c.normals[i]);
    H.noalias() += j * j.transpose();
  }
  return H;
}

Matrix6d point_displacement_metric(const Correspondences& c) {
  // D = [I, -[p]x]:  D^T D = [[I, -[p]x], [[p]x, -[p]x [p]x]] = [[I, -P], [P, P^T P]].
  Matrix6d M = Matrix6d::Zero();
  for (const auto& p : c.points) {
    Eigen::Matrix3d P;
    P << 0, -p.z(), p.y(), p.z(), 0, -p.x(), -p.y(), p.x(), 0;
    M.topLeftCorner<3, 3>() += Eigen::Matrix3d::Identity();
    M.topRightCorner<3, 3>() += -P;
    M.bottomLeftCorner<3, 3>() += P;
    M.bottomRightCorner<3, 3>() += P.transpose() * P;
  }
  return M;
}

XicpContributions xicp_contributions(const Correspondences& c, const Matrix6d& H,
                                     const XicpConfig& config) {
  XicpContributions out;
  out.V_t = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(H.topLeftCorner<3, 3>()).eigenvectors();
  out.V_r = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(H.bottomRightCorner<3, 3>()).eigenvectors();
  out.L_c_t.setZero();
  out.L_c_r.setZero();
  out.L_s_t.setZero();
  out.L_s_r.setZero();
  auto accumulate = [&](const Eigen::Vector3d& contribution, Eigen::Vector3d& L_c, Eigen::Vector3d& L_s) {
    for (int j = 0; j < 3; ++j) {
      const double v = contribution[j];
      if (v < config.kappa_f) continue;
      L_c[j] += v;
      if (v >= config.strong) L_s[j] += v;
    }
  };
  for (size_t i = 0; i < c.size(); ++i) {
    const Eigen::Vector3d& n = c.normals[i];
    accumulate((out.V_t.transpose() * n).cwiseAbs(), out.L_c_t, out.L_s_t);
    // Torques are normalized only when |tau| >= 1, so short lever arms
    // contribute less; near-zero torques are left out.
    const Eigen::Vector3d tau = c.points[i].cross(n);
    const double m = tau.norm();
    if (m < config.min_torque) continue;
    accumulate((out.V_r.transpose() * (tau / std::max(1.0, m))).cwiseAbs(), out.L_c_r, out.L_s_r);
  }
  return out;
}

}  // namespace degenbench::detectors
