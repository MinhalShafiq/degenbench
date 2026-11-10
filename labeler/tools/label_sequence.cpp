// Label every extracted frame of a sequence against a survey map.
//
//   label_sequence --frames <dir with frames.csv> --map <map.pcd> --out <labels.csv>
//                  [--crop-radius 60] [--map-voxel 0.2] [--coverage-distance 0.5]
//                  [--every 1] [--max-frames 0] [--threads 4] [--samples 200]
//                  [--T-map-world T.txt] [--no-refine 1]
//
// --T-map-world: 4x4 matrix (row-major text) taking ground-truth world
// coordinates into the map frame, when the two differ.
// Each frame's reference pose is the ground-truth pose refined against the map
// (see refine_pose); the refinement is reported per frame.
//
// Writes <out> with one row per frame, and <out stem>_subspaces.csv with the
// eigenvectors of both solvers (column-major, ascending eigenvalue order; the
// degenerate basis is the last k columns).

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/map.hpp"

namespace fs = std::filesystem;
using namespace degenbench;

namespace {

struct Args {
  std::string frames, map, out;
  double crop_radius = 60.0;
  double map_voxel = 0.2;
  double coverage_distance = 0.5;
  int every = 1;
  int max_frames = 0;
  int threads = 4;
  int samples = 200;
  std::string T_map_world;
  int no_refine = 0;
};

[[noreturn]] void usage(const char* msg) {
  std::cerr << "error: " << msg << "\n"
            << "usage: label_sequence --frames DIR --map MAP.pcd --out LABELS.csv\n"
            << "  [--crop-radius M] [--map-voxel M] [--coverage-distance M]\n"
            << "  [--every N] [--max-frames N] [--threads N] [--samples N]\n"
            << "  [--T-map-world T.txt] [--no-refine 1]\n";
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
  take("crop-radius", a.crop_radius); take("map-voxel", a.map_voxel);
  take("coverage-distance", a.coverage_distance); take("every", a.every);
  take("max-frames", a.max_frames); take("threads", a.threads); take("samples", a.samples);
  take("T-map-world", a.T_map_world); take("no-refine", a.no_refine);
  if (!kv.empty()) usage(("unknown option --" + kv.begin()->first).c_str());
  if (a.frames.empty() || a.map.empty() || a.out.empty()) usage("--frames, --map and --out are required");
  if (a.every < 1) usage("--every must be >= 1");
  return a;
}

std::string axes_string(const std::array<bool, 6>& flags) {
  std::string s;
  for (bool f : flags) s += f ? '1' : '0';
  return s;
}

}  // namespace

int main(int argc, char** argv) {
  const Args args = parse(argc, argv);
  const auto t0 = std::chrono::steady_clock::now();

  const auto frames = read_frames_csv((fs::path(args.frames) / "frames.csv").string());
  std::cerr << "frames: " << frames.size() << "\n";
  const auto raw_map = read_pcd(args.map);
  const auto map = voxel_downsample(raw_map, args.map_voxel);
  std::cerr << "map: " << raw_map.size() << " points, " << map.size() << " after "
            << args.map_voxel << " m voxels\n";

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
  config.num_samples = args.samples;

  const fs::path out_path(args.out);
  if (out_path.has_parent_path()) fs::create_directories(out_path.parent_path());
  std::ofstream out(out_path);
  out.imbue(std::locale::classic());
  const fs::path sub_path =
      out_path.parent_path() / (out_path.stem().string() + "_subspaces.csv");
  std::ofstream sub(sub_path);
  sub.imbue(std::locale::classic());

  out << "index,stamp,map_points,coverage,refine_converged,refine_trans_m,refine_rot_deg,icp_k,gicp_k,subspace_angle_deg,solvers_agree,"
         "icp_axes,gicp_axes";
  for (const char* s : {"icp", "gicp"})
    for (int i = 0; i < 6; ++i) out << "," << s << "_eig" << i;
  for (const char* s : {"icp", "gicp"})
    for (const char* a : {"tx", "ty", "tz", "rx", "ry", "rz"}) out << "," << s << "_rms_" << a;
  out << "\n";
  sub << "index,solver";
  for (int c = 0; c < 6; ++c)
    for (int r = 0; r < 6; ++r) sub << ",v" << c << r;
  sub << "\n";

  int done = 0;
  for (size_t f = 0; f < frames.size(); f += args.every) {
    if (args.max_frames > 0 && done >= args.max_frames) break;
    const FrameRecord& fr = frames[f];
    char name[32];
    std::snprintf(name, sizeof(name), "%06d.bin", fr.index);
    const auto scan = read_bin_points((fs::path(args.frames) / "frames" / name).string());
    const Eigen::Isometry3d T_gt = T_map_world * fr.T_world_lidar;
    const auto local_map = crop_radius(map, T_gt.translation(), args.crop_radius);
    if (local_map.size() < 100) {
      std::cerr << "frame " << fr.index << ": only " << local_map.size()
                << " map points nearby, skipped\n";
      continue;
    }
    RefinedPose ref;
    ref.T_map_sensor = T_gt;
    ref.converged = true;
    if (!args.no_refine) ref = refine_pose(local_map, scan, T_gt, config);
    const double coverage = map_coverage(local_map, voxel_downsample(scan, config.downsampling_resolution),
                                         ref.T_map_sensor, args.coverage_distance);
    const DualSolverLabel d = label_frame_dual(local_map, scan, ref.T_map_sensor, config);

    out << fr.index << "," << std::fixed;
    out.precision(6);
    out << fr.stamp << "," << local_map.size() << "," << coverage << ","
        << (ref.converged ? 1 : 0) << "," << ref.correction.head<3>().norm() << ","
        << ref.correction.tail<3>().norm() * 180.0 / M_PI << ","
        << d.point_to_plane_icp.num_degenerate << "," << d.gicp.num_degenerate << ","
        << d.subspace_angle_deg << "," << (d.solvers_agree ? 1 : 0) << ","
        << axes_string(d.point_to_plane_icp.axis_degenerate) << ","
        << axes_string(d.gicp.axis_degenerate);
    out << std::scientific;
    for (const FrameLabel* l : {&d.point_to_plane_icp, &d.gicp})
      for (int i = 0; i < 6; ++i) out << "," << l->eigenvalues[i];
    for (const FrameLabel* l : {&d.point_to_plane_icp, &d.gicp})
      for (int i = 0; i < 6; ++i) out << "," << l->rms_error[i];
    out << "\n" << std::defaultfloat;
    for (const auto& [solver, l] : {std::pair{"icp", &d.point_to_plane_icp}, std::pair{"gicp", &d.gicp}}) {
      sub << fr.index << "," << solver;
      for (int c = 0; c < 6; ++c)
        for (int r = 0; r < 6; ++r) sub << "," << l->eigenvectors(r, c);
      sub << "\n";
    }
    out.flush();
    sub.flush();
    ++done;
    std::cerr << "frame " << fr.index << ": refine " << ref.correction.head<3>().norm() << " m, coverage "
              << coverage << ", k icp/gicp "
              << d.point_to_plane_icp.num_degenerate << "/" << d.gicp.num_degenerate
              << ", angle " << d.subspace_angle_deg << " deg\n";
  }

  const double secs =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::cerr << "labelled " << done << " frames in " << secs << " s\n";
  return done > 0 ? 0 : 1;
}
