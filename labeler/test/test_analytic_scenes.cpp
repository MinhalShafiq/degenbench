// Labeler validation on scenes whose unobservable directions are known analytically.
// Pass condition (pre-registered): every scene returns its known number of free
// directions, for both solvers.

#include <gtest/gtest.h>

#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/scenes.hpp"

namespace degenbench {
namespace {

constexpr const char* kAxisNames[6] = {"tx", "ty", "tz", "rx", "ry", "rz"};

TEST(PoseError, RoundTripsOffsets) {
  Vector6d offset;
  offset << 0.3, -0.2, 0.1, 0.05, -0.04, 0.08;
  const Eigen::Isometry3d T_gt = offset_to_pose((Vector6d() << 1, 2, 3, 0.1, 0.2, 0.3).finished());
  const Vector6d e = pose_error(T_gt, T_gt * offset_to_pose(offset));
  EXPECT_LT((e - offset).norm(), 1e-9);
}

TEST(PoseError, ZeroForIdenticalPoses) {
  const Eigen::Isometry3d T = offset_to_pose((Vector6d() << 1, 2, 3, 0.1, 0.2, 0.3).finished());
  EXPECT_LT(pose_error(T, T).norm(), 1e-12);
}

TEST(LabelConfig, RejectsOffsetRangeTooSmallToRevealFreeDirections) {
  const AnalyticScene scene = make_plane_scene();
  LabelConfig config;
  config.max_offset_trans = 0.15;  // RMS 0.087 m < 0.10 m threshold
  EXPECT_THROW(label_frame(scene.map, scene.scan, scene.T_map_sensor, config),
               std::invalid_argument);
}

struct Case {
  int scene_index;
  Solver solver;
};

class AnalyticSceneTest : public ::testing::TestWithParam<Case> {
 protected:
  static const std::vector<AnalyticScene>& scenes() {
    static const std::vector<AnalyticScene> s = make_all_analytic_scenes();
    return s;
  }
};

TEST_P(AnalyticSceneTest, RecoversKnownFreeDirections) {
  const AnalyticScene& scene = scenes().at(GetParam().scene_index);
  LabelConfig config;
  config.solver = GetParam().solver;

  const FrameLabel label = label_frame(scene.map, scene.scan, scene.T_map_sensor, config);

  ::testing::Message eig;
  for (int i = 0; i < 6; ++i) eig << label.eigenvalues[i] << " ";
  SCOPED_TRACE(::testing::Message() << scene.name << " / " << to_string(config.solver)
                                    << " normalized eigenvalues: " << eig);

  EXPECT_EQ(label.num_degenerate, scene.expected_num_degenerate);

  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(label.axis_degenerate[i], scene.expected_free_axes[i])
        << "axis " << kAxisNames[i] << " rms " << label.rms_error[i];
  }

  // Each analytically free axis must lie inside the recovered degenerate subspace.
  for (int i = 0; i < 6; ++i) {
    if (!scene.expected_free_axes[i] || label.num_degenerate == 0) continue;
    const double in_subspace = label.degenerate_basis.row(i).norm();
    EXPECT_GT(in_subspace, 0.95) << "axis " << kAxisNames[i] << " outside degenerate subspace";
  }
}

std::string case_name(const ::testing::TestParamInfo<Case>& info) {
  static const char* names[] = {"plane", "corridor", "cylinder", "corner"};
  return std::string(names[info.param.scene_index]) + "_" + to_string(info.param.solver);
}

INSTANTIATE_TEST_SUITE_P(
    AllScenes, AnalyticSceneTest,
    ::testing::Values(Case{0, Solver::PointToPlaneICP}, Case{0, Solver::GICP},
                      Case{1, Solver::PointToPlaneICP}, Case{1, Solver::GICP},
                      Case{2, Solver::PointToPlaneICP}, Case{2, Solver::GICP},
                      Case{3, Solver::PointToPlaneICP}, Case{3, Solver::GICP}),
    case_name);

}  // namespace
}  // namespace degenbench
