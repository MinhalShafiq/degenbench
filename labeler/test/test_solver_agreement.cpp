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

// The analytic information matrix must show the same free directions as the
// Monte Carlo probe on scenes with known answers, for both solvers.
class InformationTest : public ::testing::TestWithParam<Solver> {};

TEST_P(InformationTest, WeakDirectionsMatchAnalyticScenes) {
  LabelConfig config;
  config.solver = GetParam();
  for (const auto& scene : make_all_analytic_scenes()) {
    const Information info = information_at(scene.map, scene.scan, scene.T_map_sensor, config);
    const int k = scene.expected_num_degenerate;
    SCOPED_TRACE(scene.name);
    ASSERT_GT(info.inliers, 100);
    if (k == 0) {
      EXPECT_GT(info.eigenvalues[0] / info.eigenvalues[5], 1e-3) << "fully constrained";
      continue;
    }
    // A gap between the k free and 6 - k constrained directions. Not exact zero:
    // normals and covariances come from noisy, sampled points. Measured ratios
    // range from 0.004 (plane, GICP) to 0.24 (cylinder roll, ICP), so the gap is
    // a diagnostic, and the subspace check below is the strict one.
    EXPECT_LT(info.eigenvalues[k - 1], 0.3 * info.eigenvalues[k]);
    const Eigen::MatrixXd weak = info.eigenvectors.leftCols(k);
    for (int i = 0; i < 6; ++i)
      if (scene.expected_free_axes[i]) EXPECT_GT(weak.row(i).norm(), 0.95) << "axis " << i;
  }
}

INSTANTIATE_TEST_SUITE_P(BothSolvers, InformationTest,
                         ::testing::Values(Solver::PointToPlaneICP, Solver::GICP),
                         [](const auto& info) { return std::string(to_string(info.param)); });

}  // namespace
}  // namespace degenbench
