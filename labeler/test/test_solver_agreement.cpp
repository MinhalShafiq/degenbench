#include <cmath>

#include <gtest/gtest.h>

#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/scenes.hpp"

namespace degenbench {
namespace {

Eigen::MatrixXd axes(std::initializer_list<int> indices) {
  Eigen::MatrixXd b = Eigen::MatrixXd::Zero(6, static_cast<int>(indices.size()));
  int col = 0;
  for (int i : indices) b(i, col++) = 1.0;
  return b;
}

TEST(PrincipalAngle, IdenticalSubspacesAreZero) {
  EXPECT_NEAR(max_principal_angle_deg(axes({0, 1, 5}), axes({0, 1, 5})), 0.0, 1e-9);
}

TEST(PrincipalAngle, InvariantToBasisChoice) {
  Eigen::Matrix2d r;
  r << std::cos(0.7), -std::sin(0.7), std::sin(0.7), std::cos(0.7);
  EXPECT_NEAR(max_principal_angle_deg(axes({0, 1}) * r, axes({0, 1})), 0.0, 1e-9);
}

TEST(PrincipalAngle, KnownAngle) {
  Eigen::MatrixXd tilted = Eigen::MatrixXd::Zero(6, 1);
  tilted(0, 0) = std::cos(30.0 * M_PI / 180.0);
  tilted(1, 0) = std::sin(30.0 * M_PI / 180.0);
  EXPECT_NEAR(max_principal_angle_deg(tilted, axes({0})), 30.0, 1e-9);
}

TEST(PrincipalAngle, ResolvesSmallAngles) {
  Eigen::MatrixXd tilted = Eigen::MatrixXd::Zero(6, 1);
  tilted(0, 0) = std::cos(1e-4 * M_PI / 180.0);
  tilted(3, 0) = std::sin(1e-4 * M_PI / 180.0);
  EXPECT_NEAR(max_principal_angle_deg(tilted, axes({0})), 1e-4, 1e-10);
}

TEST(PrincipalAngle, DimensionMismatchIsNinety) {
  EXPECT_EQ(max_principal_angle_deg(axes({0}), axes({0, 3})), 90.0);
  EXPECT_EQ(max_principal_angle_deg(Eigen::MatrixXd(6, 0), axes({0})), 90.0);
}

TEST(PrincipalAngle, BothEmptyAgree) {
  EXPECT_EQ(max_principal_angle_deg(Eigen::MatrixXd(6, 0), Eigen::MatrixXd(6, 0)), 0.0);
}

TEST(PrincipalAngle, RankDeficientBasisUsesTrueDimension) {
  Eigen::MatrixXd dup(6, 2);
  dup << axes({0}), axes({0});
  EXPECT_NEAR(max_principal_angle_deg(dup, axes({0})), 0.0, 1e-9);
}

TEST(DualSolver, AgreesOnEveryAnalyticScene) {
  for (const auto& scene : make_all_analytic_scenes()) {
    const DualSolverLabel dual =
        label_frame_dual(scene.map, scene.scan, scene.T_map_sensor, LabelConfig{});
    EXPECT_TRUE(dual.solvers_agree)
        << scene.name << ": subspace angle " << dual.subspace_angle_deg << " deg";
    EXPECT_EQ(dual.point_to_plane_icp.offsets, dual.gicp.offsets)
        << scene.name << ": solvers must see identical offsets";
  }
}

}  // namespace
}  // namespace degenbench
