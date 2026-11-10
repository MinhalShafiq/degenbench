// Register consecutive scan pairs dumped by degenbench_converters.scan_pairs.
//
//   register_pairs <pairs dir>  > <pairs dir>/lidar_rel.csv
//
// For each k with scans k and k+1 present, prints
//   k,converged,tx,ty,tz,rx,ry,rz
// where [t; rotation vector] is T_k_k+1 (scan k+1 expressed in scan k), from GICP
// started at identity.

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <locale>
#include <set>
#include <sstream>

#include <small_gicp/registration/registration_helper.hpp>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/localizability.hpp"

namespace fs = std::filesystem;
using namespace degenbench;

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "usage: register_pairs <pairs dir>\n");
    return 2;
  }
  const fs::path dir = argv[1];
  std::ifstream f(dir / "stamps.csv");
  if (!f) {
    std::fprintf(stderr, "cannot open %s\n", (dir / "stamps.csv").c_str());
    return 1;
  }
  f.imbue(std::locale::classic());
  std::set<int> ks;
  for (std::string line; std::getline(f, line);) {
    std::istringstream is(line);
    is.imbue(std::locale::classic());
    int k;
    if (is >> k) ks.insert(k);
  }

  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.max_correspondence_distance = 1.0;
  setting.max_iterations = 50;

  int written = 0;
  for (int k : ks) {
    if (!ks.count(k + 1)) continue;
    char n0[32], n1[32];
    std::snprintf(n0, sizeof(n0), "%06d.bin", k);
    std::snprintf(n1, sizeof(n1), "%06d.bin", k + 1);
    auto [target, tree] = small_gicp::preprocess_points(read_bin_points((dir / n0).string()), 0.2, 10, 1);
    auto [source, stree] = small_gicp::preprocess_points(read_bin_points((dir / n1).string()), 0.2, 10, 1);
    (void)stree;
    const auto r = small_gicp::align(*target, *source, *tree, Eigen::Isometry3d::Identity(), setting);
    const Vector6d e = pose_error(Eigen::Isometry3d::Identity(), r.T_target_source);
    std::printf("%d,%d,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", k, r.converged ? 1 : 0, e[0], e[1], e[2],
                e[3], e[4], e[5]);
    ++written;
  }
  std::fprintf(stderr, "registered %d pairs\n", written);
  return written > 0 ? 0 : 1;
}
