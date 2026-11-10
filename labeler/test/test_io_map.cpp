#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <gtest/gtest.h>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/localizability.hpp"
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

TEST(Lzf, LiteralsAndOverlappingBackReference) {
  // "abc" as a literal run, then a 6-byte back reference at distance 3.
  const unsigned char stream[] = {2, 'a', 'b', 'c', (4 << 5) | 0, 2};
  const auto out = lzf_decompress(reinterpret_cast<const char*>(stream), sizeof(stream), 9);
  EXPECT_EQ(std::string(out.begin(), out.end()), "abcabcabc");
}

TEST(Lzf, LongBackReference) {
  // Literal 'x', then length 7 + 3 + 2 = 12 at distance 1.
  const unsigned char stream[] = {0, 'x', (7 << 5) | 0, 3, 0};
  const auto out = lzf_decompress(reinterpret_cast<const char*>(stream), sizeof(stream), 13);
  EXPECT_EQ(std::string(out.begin(), out.end()), std::string(13, 'x'));
}

TEST(Lzf, RejectsCorruptStreams) {
  const unsigned char bad_ref[] = {0, 'x', (1 << 5) | 0, 5};  // distance beyond output
  EXPECT_THROW(lzf_decompress(reinterpret_cast<const char*>(bad_ref), sizeof(bad_ref), 4),
               std::runtime_error);
  const unsigned char short_out[] = {2, 'a', 'b', 'c'};
  EXPECT_THROW(lzf_decompress(reinterpret_cast<const char*>(short_out), sizeof(short_out), 5),
               std::runtime_error);
}

// binary_compressed with literal-only LZF (valid LZF; real files also use back
// references, covered above) and an extra rgb field, laid out field-major.
TEST(Pcd, BinaryCompressed) {
  const auto pts = sample_points();
  std::vector<char> raw;
  for (int a = 0; a < 3; ++a)
    for (const auto& p : pts) {
      const float v = static_cast<float>(p[a]);
      raw.insert(raw.end(), reinterpret_cast<const char*>(&v), reinterpret_cast<const char*>(&v) + 4);
    }
  for (size_t i = 0; i < pts.size(); ++i) raw.insert(raw.end(), {1, 2, 3, 4});  // rgb
  std::vector<char> comp;
  for (size_t i = 0; i < raw.size(); i += 32) {
    const size_t n = std::min<size_t>(32, raw.size() - i);
    comp.push_back(static_cast<char>(n - 1));
    comp.insert(comp.end(), raw.begin() + i, raw.begin() + i + n);
  }
  const auto dir = temp_dir("pcd_cmp");
  std::ofstream f(dir / "a.pcd", std::ios::binary);
  f << "VERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
       "WIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA binary_compressed\n";
  const uint32_t hdr[2] = {static_cast<uint32_t>(comp.size()), static_cast<uint32_t>(raw.size())};
  f.write(reinterpret_cast<const char*>(hdr), sizeof(hdr));
  f.write(comp.data(), static_cast<std::streamsize>(comp.size()));
  f.close();
  expect_points_eq(read_pcd((dir / "a.pcd").string()), pts);
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

// A ground-truth pose that is off in constrained directions is pulled back to the
// map; the degenerate direction (corridor axis) is left where it was.
TEST(RefinePose, FixesConstrainedDirectionsOnly) {
  const AnalyticScene s = make_corridor_scene();
  Eigen::Isometry3d off = s.T_map_sensor;
  off.translation() += Eigen::Vector3d(0.4, 0.15, -0.1);  // along-corridor, lateral, vertical
  off.linear() = Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const RefinedPose r = refine_pose(s.map, s.scan, off, LabelConfig{});
  const Vector6d e = pose_error(s.T_map_sensor, r.T_map_sensor);
  EXPECT_TRUE(r.converged);
  EXPECT_NEAR(e[1], 0.0, 0.01) << "lateral fixed";
  EXPECT_NEAR(e[2], 0.0, 0.01) << "vertical fixed";
  EXPECT_LT(e.tail<3>().norm(), 0.003) << "rotation fixed";
  // Along the corridor nothing constrains the pose: it may drift a little while the
  // rotation is corrected, but it is not pulled back to the true value.
  EXPECT_GT(e[0], 0.25) << "along-corridor offset is unobservable and is not 'fixed'";
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
    // index,stamp,map_points,coverage,refine_conv,refine_t,refine_r,icp_k,gicp_k,angle,agree,
    // icp_axes,gicp_axes,...
    std::vector<std::string> v;
    std::stringstream ss(row);
    for (std::string c; std::getline(ss, c, ',');) v.push_back(c);
    EXPECT_GT(std::stod(v[3]), 0.95) << "coverage";
    EXPECT_EQ(v[4], "1") << "refinement converged";
    EXPECT_LT(std::stod(v[5]), 0.05) << "exact pose needs almost no refinement";
    EXPECT_EQ(v[7], "1");
    EXPECT_EQ(v[8], "1");
    EXPECT_EQ(v[10], "1") << "solvers agree";
    EXPECT_EQ(v[11], "100000") << "only tx free";
  }
  EXPECT_EQ(rows, 2);
  EXPECT_TRUE(fs::exists(dir / "out" / "labels_subspaces.csv"));
}

}  // namespace
}  // namespace degenbench
