// Compute the inputs of every scored detector for each frame of a sequence.
//
//   detector_features --frames <dir with frames.csv> --map <map.pcd> --out <features.csv>
//                     [--crop-radius 60] [--map-voxel 0.2] [--threads 4] [--max-frames 0]
//                     [--T-map-world T.txt] [--max-correspondence 1.0]
//
// Frames, map preprocessing and the reference pose (ground truth refined
// against the map) follow label_sequence exactly, so the detectors see the same
// pose the labels were measured from; refine_* columns allow checking that.
//
// Columns: index, stamp, refine_converged, refine_trans_m, refine_rot_deg,
// num_correspondences, H{r}{c} (point-to-plane information, 36), M{r}{c}
// (point-displacement metric, 36), xicp_vt{c}{r}, xicp_vr{c}{r} (block
// eigenvectors, ascending), xicp_lct{j}, xicp_lcr{j}, xicp_lst{j}, xicp_lsr{j},
// features_ms, xicp_ms. Parameters: [tx ty tz rx ry rz], metres and radians.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "degenbench/detectors/features.hpp"
#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/map.hpp"

namespace fs = std::filesystem;
using namespace degenbench;
using namespace degenbench::detectors;

namespace {

struct Args {
  std::string frames, map, out, T_map_world;
  double crop_radius = 60.0;
  double map_voxel = 0.2;
  double max_correspondence = 1.0;
  int threads = 4;
  int max_frames = 0;
};

[[noreturn]] void usage(const char* msg) {
  std::cerr << "error: " << msg << "\n"
            << "usage: detector_features --frames DIR --map MAP.pcd --out FEATURES.csv\n"
            << "  [--crop-radius M] [--map-voxel M] [--threads N] [--max-frames N]\n"
            << "  [--T-map-world T.txt] [--max-correspondence M]\n";
  std::exit(2);
}

Args parse(int argc, char** argv) {
  std::map<std::string, std::string> kv;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (k.rfind("--", 0) != 0 || i + 1 >= argc) usage(("bad argument " + k).c_str());
    kv[k.substr(2)] = argv[++i];
  }
  Args a;
  auto take = [&](const char* k, auto& v) {
    auto it = kv.find(k);
    if (it == kv.end()) return;
    if constexpr (std::is_same_v<std::decay_t<decltype(v)>, std::string>) v = it->second;
    else if constexpr (std::is_integral_v<std::decay_t<decltype(v)>>) v = std::stoi(it->second);
    else v = std::stod(it->second);
    kv.erase(it);
  };
  take("frames", a.frames); take("map", a.map); take("out", a.out);
  take("T-map-world", a.T_map_world); take("crop-radius", a.crop_radius);
  take("map-voxel", a.map_voxel); take("max-correspondence", a.max_correspondence);
  take("threads", a.threads); take("max-frames", a.max_frames);
  if (!kv.empty()) usage(("unknown option --" + kv.begin()->first).c_str());
  if (a.frames.empty() || a.map.empty() || a.out.empty()) usage("--frames, --map and --out are required");
  return a;
}

double ms_since(std::chrono::steady_clock::time_point t) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count();
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse(argc, argv);
  const auto frames = read_frames_csv((fs::path(args.frames) / "frames.csv").string());
  const auto map = voxel_downsample(read_pcd(args.map), args.map_voxel);

  Eigen::Isometry3d T_map_world = Eigen::Isometry3d::Identity();
  if (!args.T_map_world.empty()) {
    std::ifstream tf(args.T_map_world);
    tf.imbue(std::locale::classic());
    Eigen::Matrix4d M;
    for (int i = 0; i < 16; ++i)
      if (!(tf >> M(i / 4, i % 4))) usage("--T-map-world: expected 16 numbers");
    T_map_world.matrix() = M;
  }

  LabelConfig config;
  config.num_threads = args.threads;
  const PreparedCloud prepared_map = prepare(map, config);

  const fs::path out_path(args.out);
  if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());
  std::ofstream out(out_path);
  out.imbue(std::locale::classic());
  out << "index,stamp,refine_converged,refine_trans_m,refine_rot_deg,num_correspondences";
  for (const char* m : {"H", "M"})
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) out << "," << m << r << c;
  for (const char* m : {"xicp_vt", "xicp_vr"})
    for (int c = 0; c < 3; ++c)
      for (int r = 0; r < 3; ++r) out << "," << m << c << r;
  for (const char* m : {"xicp_lct", "xicp_lcr", "xicp_lst", "xicp_lsr"})
    for (int j = 0; j < 3; ++j) out << "," << m << j;
  out << ",features_ms,xicp_ms\n";

  int done = 0;
  for (const FrameRecord& fr : frames) {
    if (args.max_frames > 0 && done >= args.max_frames) break;
    char name[32];
    std::snprintf(name, sizeof(name), "%06d.bin", fr.index);
    const auto scan = read_bin_points((fs::path(args.frames) / "frames" / name).string());
    const Eigen::Isometry3d T_gt = T_map_world * fr.T_world_lidar;
    if (crop_radius(map, T_gt.translation(), args.crop_radius).size() < 100) continue;  // as label_sequence
    const PreparedCloud prepared_scan = prepare(scan, config);
    const RefinedPose ref = refine_pose(prepared_map, prepared_scan, T_gt, config);

    const auto t0 = std::chrono::steady_clock::now();
    const Correspondences c =
        find_correspondences(prepared_map, prepared_scan, ref.T_map_sensor, args.max_correspondence);
    const Matrix6d H = point_to_plane_information(c);
    const Matrix6d M = point_displacement_metric(c);
    const double features_ms = ms_since(t0);
    const auto t1 = std::chrono::steady_clock::now();
    const XicpContributions x = xicp_contributions(c, H);
    const double xicp_ms = ms_since(t1);

    out << fr.index << std::fixed;
    out.precision(6);
    out << "," << fr.stamp << "," << (ref.converged ? 1 : 0) << ","
        << ref.correction.head<3>().norm() << "," << ref.correction.tail<3>().norm() * 180.0 / M_PI
        << "," << c.size();
    out << std::scientific;
    out.precision(10);
    for (const Matrix6d* m : {&H, &M})
      for (int r = 0; r < 6; ++r)
        for (int col = 0; col < 6; ++col) out << "," << (*m)(r, col);
    for (const Eigen::Matrix3d* v : {&x.V_t, &x.V_r})
      for (int col = 0; col < 3; ++col)
        for (int r = 0; r < 3; ++r) out << "," << (*v)(r, col);
    for (const Eigen::Vector3d* l : {&x.L_c_t, &x.L_c_r, &x.L_s_t, &x.L_s_r})
      for (int j = 0; j < 3; ++j) out << "," << (*l)[j];
    out << std::fixed;
    out.precision(4);
    out << "," << features_ms << "," << xicp_ms << "\n" << std::defaultfloat;
    if (++done % 50 == 0) std::cerr << done << " frames\n";
  }
  std::cerr << "wrote " << done << " frames to " << args.out << "\n";
  return 0;
}
