// Diagnostic: rerun the probe on selected frames under one configuration
// variant and dump every Monte Carlo sample.
//
//   probe_frames --frames DIR --map MAP.pcd --indices 0,17,42 --out PREFIX
//                [--T-map-world T.txt] [--solver icp|gicp] [--max-iterations N]
//                [--neighbors N] [--max-corr M] [--offset-rot-deg D] [--offset-trans M]
//                [--crop-radius M] [--map-voxel M] [--threads N] [--samples N]
//
// Writes PREFIX_samples.csv (index, sample, offset[6], error[6], iterations,
// converged, inliers) and PREFIX_frames.csv (index, k, eigenvalues[6],
// converged fraction, median iterations, median inliers, information
// eigenvalues[6], and the weakest information eigenvector). The reference pose
// is refined against the map exactly as in label_sequence.

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <locale>
#include <map>
#include <sstream>
#include <string>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/map.hpp"

namespace fs = std::filesystem;
using namespace degenbench;

namespace {

template <typename T>
T median(std::vector<T> v) {
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

}  // namespace

int main(int argc, char** argv) {
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc; i += 2) kv[std::string(argv[i]).substr(2)] = argv[i + 1];
  for (const char* k : {"frames", "map", "indices", "out"}) {
    if (!kv.count(k)) {
      std::cerr << "usage: probe_frames --frames DIR --map MAP.pcd --indices i,j,k --out PREFIX [options]\n";
      return 2;
    }
  }
  auto num = [&](const char* k, double d) { return kv.count(k) ? std::stod(kv[k]) : d; };

  LabelConfig config;
  config.solver = kv.count("solver") && kv["solver"] == "gicp" ? Solver::GICP : Solver::PointToPlaneICP;
  config.max_iterations = static_cast<int>(num("max-iterations", config.max_iterations));
  config.num_neighbors = static_cast<int>(num("neighbors", config.num_neighbors));
  config.max_correspondence_distance = num("max-corr", config.max_correspondence_distance);
  config.max_offset_rot = num("offset-rot-deg", config.max_offset_rot * 180 / M_PI) * M_PI / 180;
  config.max_offset_trans = num("offset-trans", config.max_offset_trans);
  config.num_threads = static_cast<int>(num("threads", 12));
  config.num_samples = static_cast<int>(num("samples", config.num_samples));
  const double crop = num("crop-radius", 40.0);

  Eigen::Isometry3d T_map_world = Eigen::Isometry3d::Identity();
  if (kv.count("T-map-world")) {
    std::ifstream tf(kv["T-map-world"]);
    tf.imbue(std::locale::classic());
    Eigen::Matrix4d M;
    for (int i = 0; i < 16; ++i) tf >> M(i / 4, i % 4);
    T_map_world.matrix() = M;
  }

  const auto frames = read_frames_csv((fs::path(kv["frames"]) / "frames.csv").string());
  std::map<int, FrameRecord> by_index;
  for (const auto& f : frames) by_index[f.index] = f;
  const auto map = voxel_downsample(read_pcd(kv["map"]), num("map-voxel", 0.2));

  std::ofstream samples(kv["out"] + "_samples.csv"), summary(kv["out"] + "_frames.csv");
  samples.imbue(std::locale::classic());
  summary.imbue(std::locale::classic());
  samples << "index,sample,o_tx,o_ty,o_tz,o_rx,o_ry,o_rz,e_tx,e_ty,e_tz,e_rx,e_ry,e_rz,iterations,converged,inliers\n";
  summary << "index,k,eig0,eig1,eig2,eig3,eig4,eig5,converged_frac,median_iterations,median_inliers,"
             "info0,info1,info2,info3,info4,info5,weak_tx,weak_ty,weak_tz,weak_rx,weak_ry,weak_rz\n";

  std::stringstream ids(kv["indices"]);
  for (std::string tok; std::getline(ids, tok, ',');) {
    const int idx = std::stoi(tok);
    if (!by_index.count(idx)) { std::cerr << "no frame " << idx << "\n"; continue; }
    const FrameRecord& fr = by_index[idx];
    char name[32];
    std::snprintf(name, sizeof(name), "%06d.bin", idx);
    const auto scan = read_bin_points((fs::path(kv["frames"]) / "frames" / name).string());
    const Eigen::Isometry3d T_gt = T_map_world * fr.T_world_lidar;
    const auto local = crop_radius(map, T_gt.translation(), crop);
    LabelConfig ref_config = config;
    ref_config.max_correspondence_distance = 1.0;  // same reference as label_sequence
    ref_config.max_iterations = 50;
    ref_config.num_neighbors = 10;
    const RefinedPose ref = refine_pose(local, scan, T_gt, ref_config);

    const FrameLabel l = label_frame(local, scan, ref.T_map_sensor, config);
    const Information info = information_at(local, scan, ref.T_map_sensor, config);

    for (int i = 0; i < config.num_samples; ++i) {
      samples << idx << "," << i;
      for (int j = 0; j < 6; ++j) samples << "," << l.offsets[i][j];
      for (int j = 0; j < 6; ++j) samples << "," << l.final_errors[i][j];
      samples << "," << l.iterations[i] << "," << int(l.converged[i]) << "," << l.inliers[i] << "\n";
    }
    int conv = 0;
    for (char c : l.converged) conv += c;
    summary << idx << "," << l.num_degenerate;
    for (int j = 0; j < 6; ++j) summary << "," << l.eigenvalues[j];
    summary << "," << double(conv) / config.num_samples << "," << median(l.iterations) << ","
            << median(l.inliers);
    for (int j = 0; j < 6; ++j) summary << "," << info.eigenvalues[j];
    for (int j = 0; j < 6; ++j) summary << "," << info.eigenvectors(j, 0);
    summary << "\n";
    std::cerr << "frame " << idx << ": k " << l.num_degenerate << ", converged "
              << double(conv) / config.num_samples << ", median iterations " << median(l.iterations) << "\n";
  }
  return 0;
}
