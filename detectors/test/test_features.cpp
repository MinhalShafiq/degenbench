#include <cmath>
#include <random>

#include <Eigen/Eigenvalues>
#include <gtest/gtest.h>

#include "degenbench/detectors/features.hpp"
#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/scenes.hpp"

using namespace degenbench;
using namespace degenbench::detectors;

namespace {

Eigen::Vector3d random_unit(std::mt19937_64& rng) {
  std::normal_distribution<double> g;
  return Eigen::Vector3d(g(rng), g(rng), g(rng)).normalized();
}

/// Generalized eigenvalues of H v = lambda M v, ascending.
Vector6d generalized_eigenvalues(const Matrix6d& H, const Matrix6d& M) {
  return Eigen::GeneralizedSelfAdjointEigenSolver<Matrix6d>(H, M).eigenvalues();
}

Correspondences scene_correspondences(const AnalyticScene& s) {
  LabelConfig config;
  return find_correspondences(prepare(s.map, config), prepare(s.scan, config), s.T_map_sensor, 1.0);
}

}  // namespace

TEST(FeaturesTest, JacobianMatchesLabelParametrization) {
  // Residual r(x) = n_map . (T_ref * offset_to_pose(x) * p - q): its gradient at 0
  // must be the analytic Jacobian, with n rotated into the sensor frame.
  std::mt19937_64 rng(3);
  Eigen::Isometry3d T_ref = Eigen::Isometry3d::Identity();
  T_ref.linear() = Eigen::AngleAxisd(0.7, random_unit(rng)).toRotationMatrix();
  T_ref.translation() << 4, -2, 1;
  for (int trial = 0; trial < 20; ++trial) {
    const Eigen::Vector3d p = 5.0 * random_unit(rng);
    const Eigen::Vector3d n_map = random_unit(rng);
    const Eigen::Vector3d q = T_ref * p;
    auto residual = [&](const Vector6d& x) { return n_map.dot(T_ref * offset_to_pose(x) * p - q); };
    const Vector6d J = point_to_plane_jacobian(p, T_ref.linear().transpose() * n_map);
    for (int k = 0; k < 6; ++k) {
      Vector6d dx = Vector6d::Zero();
      dx[k] = 1e-6;
      EXPECT_NEAR((residual(dx) - residual(-dx)) / 2e-6, J[k], 1e-6) << "axis " << k;
    }
  }
}

TEST(FeaturesTest, DisplacementMetricIsSumOfSquaredDisplacements) {
  std::mt19937_64 rng(5);
  Correspondences c;
  for (int i = 0; i < 50; ++i) {
    c.points.push_back(3.0 * random_unit(rng));
    c.normals.push_back(random_unit(rng));
  }
  const Matrix6d M = point_displacement_metric(c);
  Vector6d x;
  x << 0.1, -0.2, 0.05, 0.01, 0.03, -0.02;
  double sum = 0;
  for (const auto& p : c.points) {
    const Eigen::Vector3d d = x.head<3>() + x.tail<3>().cross(p);
    sum += d.squaredNorm();
  }
  EXPECT_NEAR(x.dot(M * x), sum, 1e-9 * sum);
}

TEST(FeaturesTest, FrameIndependentSpectrumFindsAnalyticFreeDirections) {
  // The generalized spectrum H v = lambda M v lies in [0, 1]. Free directions sit
  // at ~0 (only scan noise), constrained ones far above the paper's tau = 1e-2.
  for (const auto& scene : make_all_analytic_scenes()) {
    const auto c = scene_correspondences(scene);
    ASSERT_GT(c.size(), 1000u) << scene.name;
    const Vector6d lambda =
        generalized_eigenvalues(point_to_plane_information(c), point_displacement_metric(c));
    int free = 0;
    for (int i = 0; i < 6; ++i) free += lambda[i] < 1e-2;
    EXPECT_EQ(free, scene.expected_num_degenerate) << scene.name << ": " << lambda.transpose();
    EXPECT_GE(lambda.minCoeff(), -1e-9);
    EXPECT_LE(lambda.maxCoeff(), 1.0 + 1e-9);
    if (scene.expected_num_degenerate < 6) {
      EXPECT_GT(lambda[scene.expected_num_degenerate], 5e-2) << scene.name;
    }
  }
}

TEST(FeaturesTest, InformationNullSpaceIsTheExpectedAxes) {
  for (const auto& scene : make_all_analytic_scenes()) {
    const int k = scene.expected_num_degenerate;
    if (k == 0) continue;
    const auto c = scene_correspondences(scene);
    // Normalize units so eigenvectors are comparable across t and r.
    Vector6d s;
    s << Eigen::Vector3d::Constant(0.1), Eigen::Vector3d::Constant(2.0 * M_PI / 180.0);
    const Matrix6d Hn = s.asDiagonal() * point_to_plane_information(c) * s.asDiagonal();
    const Eigen::SelfAdjointEigenSolver<Matrix6d> eig(Hn);
    const Eigen::MatrixXd null = eig.eigenvectors().leftCols(k);
    for (int axis = 0; axis < 6; ++axis) {
      const double in_null = null.row(axis).squaredNorm();
      if (scene.expected_free_axes[axis]) {
        EXPECT_GT(in_null, 0.95) << scene.name << " axis " << axis;
      } else {
        EXPECT_LT(in_null, 0.05) << scene.name << " axis " << axis;
      }
    }
  }
}

TEST(FeaturesTest, XicpContributionsVanishAlongFreeTranslation) {
  // Corridor along x: no normal has an x component, so the translation
  // eigenvector along x collects no contribution; y and z collect many.
  const auto scene = make_corridor_scene();
  const auto c = scene_correspondences(scene);
  const XicpContributions x = xicp_contributions(c, point_to_plane_information(c));
  int along_x = 0;
  for (int j = 0; j < 3; ++j)
    if (std::abs(x.V_t(0, j)) > 0.99) along_x = j;
  EXPECT_EQ(along_x, 0);  // the weakest translation eigenvector
  EXPECT_LT(x.L_c_t[0], 0.01 * x.L_c_t[1]);
  EXPECT_GT(x.L_s_t[1], 100.0);
  EXPECT_GT(x.L_s_t[2], 100.0);
  for (int j = 0; j < 3; ++j) EXPECT_LE(x.L_s_t[j], x.L_c_t[j] + 1e-12);
}
