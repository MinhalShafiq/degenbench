#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/map.hpp"
#include "degenbench/labeler/scenes.hpp"

namespace fs = std::filesystem;

namespace degenbench {
namespace {

fs::path temp_dir(const std::string& name) {
  const fs::path p = fs::temp_directory_path() / ("degenbench_test_" + name);
  fs::remove_all(p);
  fs::create_directories(p);
  return p;
}

std::vector<Eigen::Vector3d> sample_points() {
  return {{1.5, -2.25, 3.0}, {0.0, 0.0, 0.0}, {-100.125, 42.5, 7.75}};
}

void expect_points_eq(const std::vector<Eigen::Vector3d>& a, const std::vector<Eigen::Vector3d>& b) {
  ASSERT_EQ(a.size(), b.size());
  for (size_t i = 0; i < a.size(); ++i) EXPECT_LT((a[i] - b[i]).norm(), 1e-5) << "point " << i;
}

TEST(Pcd, BinaryRoundTrip) {
  const auto dir = temp_dir("pcd_bin");
  write_pcd((dir / "a.pcd").string(), sample_points());
  expect_points_eq(read_pcd((dir / "a.pcd").string()), sample_points());
}

TEST(Pcd, AsciiWithExtraFields) {
  const auto dir = temp_dir("pcd_ascii");
  std::ofstream f(dir / "a.pcd");
  f << "VERSION 0.7\nFIELDS intensity x y z normal\nSIZE 4 4 4 4 4\nTYPE F F F F F\n"
       "COUNT 1 1 1 1 3\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ascii\n";
  for (const auto& p : sample_points()) f << "9 " << p.x() << " " << p.y() << " " << p.z() << " 0 0 1\n";
  f.close();
  expect_points_eq(read_pcd((dir / "a.pcd").string()), sample_points());
}

TEST(Pcd, BinaryFloat64WithPadding) {
  const auto dir = temp_dir("pcd_f64");
  std::ofstream f(dir / "a.pcd", std::ios::binary);
  f << "VERSION 0.7\nFIELDS x y z rgb\nSIZE 8 8 8 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
       "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary\n";
  for (const auto& p : sample_points()) {
    f.write(reinterpret_cast<const char*>(p.data()), 24);
    const unsigned rgb = 0xffffff;
    f.write(reinterpret_cast<const char*>(&rgb), 4);
  }
  f.close();
  expect_points_eq(read_pcd((dir / "a.pcd").string()), sample_points());
}

TEST(Pcd, RejectsCompressedWithHint) {
  const auto dir = temp_dir("pcd_cmp");
  std::ofstream(dir / "a.pcd") << "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 1\nDATA binary_compressed\n";
  try {
    read_pcd((dir / "a.pcd").string());
    FAIL() << "expected an exception";
  } catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("pcl_convert_pcd_ascii_binary"), std::string::npos);
  }
}

TEST(Frames, CsvAndBin) {
  const auto dir = temp_dir("frames");
  std::ofstream(dir / "frames.csv")
      << "index,stamp,tx,ty,tz,qx,qy,qz,qw,num_points\n"
      << "0,1686284028.013323307,1.5,-2,0.25,0,0,0.7071067811865476,0.7071067811865476,3\n";
  const auto frames = read_frames_csv((dir / "frames.csv").string());
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_NEAR(frames[0].stamp, 1686284028.013323307, 1e-6);
  EXPECT_LT((frames[0].T_world_lidar.translation() - Eigen::Vector3d(1.5, -2, 0.25)).norm(), 1e-12);
  EXPECT_LT((frames[0].T_world_lidar * Eigen::Vector3d::UnitX() -
             Eigen::Vector3d(1.5, -1, 0.25)).norm(), 1e-9);  // 90 deg yaw

  std::ofstream bin(dir / "000000.bin", std::ios::binary);
  for (const auto& p : sample_points()) {
    const float xyz[3] = {float(p.x()), float(p.y()), float(p.z())};
    bin.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
  bin.close();
  expect_points_eq(read_bin_points((dir / "000000.bin").string()), sample_points());
}

TEST(Map, CropAndDownsample) {
  std::vector<Eigen::Vector3d> grid;
  for (int i = 0; i < 100; ++i)
    for (int j = 0; j < 100; ++j) grid.emplace_back(0.05 * i, 0.05 * j, 0.0);
  EXPECT_NEAR(voxel_downsample(grid, 0.5).size(), 100u, 4u);  // 5 m x 5 m in 0.5 m cells
  const auto crop = crop_radius(grid, Eigen::Vector3d(0, 0, 0), 1.0);
  for (const auto& p : crop) EXPECT_LE(p.norm(), 1.0);
  EXPECT_NEAR(static_cast<double>(crop.size()), M_PI / 4 / 0.0025, 40.0);  // quarter disc
}

TEST(Map, CoverageDetectsWrongPose) {
  const AnalyticScene s = make_corner_scene();
  EXPECT_GT(map_coverage(s.map, s.scan, s.T_map_sensor, 0.3), 0.99);
  Eigen::Isometry3d off = s.T_map_sensor;
  off.translation() += Eigen::Vector3d(1.0, 1.0, 1.0);  // walls no longer line up
  EXPECT_LT(map_coverage(s.map, s.scan, off, 0.3), 0.2);
}

// Run the CLI on a synthetic corridor "sequence" written in the converter's format.
TEST(LabelSequenceTool, CorridorEndToEnd) {
  const auto dir = temp_dir("tool");
  const AnalyticScene s = make_corridor_scene();
  write_pcd((dir / "map.pcd").string(), s.map);
  fs::create_directories(dir / "frames" / "frames");
  {
    std::ofstream csv(dir / "frames" / "frames.csv");
    csv << "index,stamp,tx,ty,tz,qx,qy,qz,qw,num_points\n";
    const Eigen::Vector3d t = s.T_map_sensor.translation();
    for (int i = 0; i < 2; ++i) csv << i << "," << 100 + i << "," << t.x() << "," << t.y() << ","
                                    << t.z() << ",0,0,0,1," << s.scan.size() << "\n";
    for (int i = 0; i < 2; ++i) {
      char name[32];
      std::snprintf(name, sizeof(name), "%06d.bin", i);
      std::ofstream bin(dir / "frames" / "frames" / name, std::ios::binary);
      for (const auto& p : s.scan) {
        const float xyz[3] = {float(p.x()), float(p.y()), float(p.z())};
        bin.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
      }
    }
  }
  const std::string cmd = std::string(LABEL_SEQUENCE_TOOL) + " --frames " + (dir / "frames").string() +
                          " --map " + (dir / "map.pcd").string() + " --out " +
                          (dir / "out" / "labels.csv").string() + " --map-voxel 0.1 2>/dev/null";
  ASSERT_EQ(std::system(cmd.c_str()), 0);

  std::ifstream labels(dir / "out" / "labels.csv");
  std::string header, row;
  std::getline(labels, header);
  int rows = 0;
  while (std::getline(labels, row)) {
    ++rows;
    // index,stamp,map_points,coverage,icp_k,gicp_k,angle,agree,icp_axes,gicp_axes,...
    std::vector<std::string> v;
    std::stringstream ss(row);
    for (std::string c; std::getline(ss, c, ',');) v.push_back(c);
    EXPECT_GT(std::stod(v[3]), 0.95) << "coverage";
    EXPECT_EQ(v[4], "1");
    EXPECT_EQ(v[5], "1");
    EXPECT_EQ(v[7], "1") << "solvers agree";
    EXPECT_EQ(v[8], "100000") << "only tx free";
  }
  EXPECT_EQ(rows, 2);
  EXPECT_TRUE(fs::exists(dir / "out" / "labels_subspaces.csv"));
}

}  // namespace
}  // namespace degenbench
