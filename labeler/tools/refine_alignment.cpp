// Refine coarse world-to-map candidates (from degenbench_converters.map_align)
// in 3D and choose the best.
//
//   refine_alignment --map MAP.pcd --cloud accum.bin --candidates cands.txt
//                    --out T_map_world.txt [--report report.yaml]
//
// For each candidate (score yaw_deg x y): height from the 5th percentile of map
// and cloud heights, then GICP at correspondence distances 3, 1.5, 0.7, 0.4 m.
// The winner maximizes the fraction of cloud points within 0.3 m of the map. The
// report also gives that fraction with the result shifted by 5 m and 20 m: a
// real alignment drops sharply; a spurious one (e.g. a map that does not cover
// the trajectory) does not.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <locale>
#include <map>
#include <memory>
#include <sstream>
#include <string>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/registration_helper.hpp>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/localizability.hpp"
#include "degenbench/labeler/map.hpp"

using namespace degenbench;

namespace {

double percentile(std::vector<double> v, double q) {
  const size_t k = static_cast<size_t>(q * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return v[k];
}

std::vector<Eigen::Vector3d> crop_xy(const std::vector<Eigen::Vector3d>& pts,
                                     const Eigen::Vector2d& c, double r) {
  std::vector<Eigen::Vector3d> out;
  for (const auto& p : pts)
    if ((p.head<2>() - c).norm() < r) out.push_back(p);
  return out;
}

double inlier_fraction(const std::vector<Eigen::Vector3d>& map,
                       const std::vector<Eigen::Vector3d>& cloud, const Eigen::Isometry3d& T,
                       double radius) {
  Eigen::Vector2d c(0, 0);
  for (const auto& p : cloud) c += (T * p).head<2>();
  c /= static_cast<double>(cloud.size());
  const auto crop = crop_xy(map, c, radius);
  if (crop.empty()) return 0.0;
  return map_coverage(crop, cloud, T, 0.3);
}

}  // namespace

int main(int argc, char** argv) {
  std::map<std::string, std::string> kv;
  for (int i = 1; i + 1 < argc; i += 2) kv[std::string(argv[i]).substr(2)] = argv[i + 1];
  for (const char* k : {"map", "cloud", "candidates", "out"}) {
    if (!kv.count(k)) {
      std::cerr << "usage: refine_alignment --map MAP.pcd --cloud accum.bin --candidates C.txt "
                   "--out T.txt [--report R.yaml]\n";
      return 2;
    }
  }
  const auto map = voxel_downsample(read_pcd(kv["map"]), 0.3);
  const auto cloud = voxel_downsample(read_bin_points(kv["cloud"]), 0.3);
  double extent = 0;
  {
    Eigen::Vector2d c(0, 0);
    for (const auto& p : cloud) c += p.head<2>();
    c /= static_cast<double>(cloud.size());
    for (const auto& p : cloud) extent = std::max(extent, (p.head<2>() - c).norm());
  }
  const double radius = extent + 20.0;

  std::ifstream cf(kv["candidates"]);
  cf.imbue(std::locale::classic());
  small_gicp::RegistrationSetting s;
  s.type = small_gicp::RegistrationSetting::GICP;
  s.num_threads = 8;
  s.max_iterations = 100;

  double best_fit = -1;
  Eigen::Isometry3d best = Eigen::Isometry3d::Identity();
  int n = 0;
  for (std::string line; std::getline(cf, line);) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    is.imbue(std::locale::classic());
    double score, yaw, x, y;
    if (!(is >> score >> yaw >> x >> y)) continue;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.linear() = Eigen::AngleAxisd(yaw * M_PI / 180.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    T.translation() << x, y, 0.0;
    Eigen::Vector2d c(0, 0);
    for (const auto& p : cloud) c += (T * p).head<2>();
    c /= static_cast<double>(cloud.size());
    const auto crop = crop_xy(map, c, radius);
    if (crop.size() < 100) continue;
    std::vector<double> mz, az;
    for (const auto& p : crop) mz.push_back(p.z());
    for (const auto& p : cloud) az.push_back((T * p).z());
    T.translation().z() = percentile(mz, 0.05) - percentile(az, 0.05);

    auto [tgt, tree] = small_gicp::preprocess_points(crop, 0.3, 10, 8);
    auto [src, stree] = small_gicp::preprocess_points(cloud, 0.3, 10, 8);
    (void)stree;
    for (double d : {3.0, 1.5, 0.7, 0.4}) {
      s.max_correspondence_distance = d;
      T = small_gicp::align(*tgt, *src, *tree, T, s).T_target_source;
    }
    const double fit = map_coverage(crop, cloud, T, 0.3);
    std::cerr << "candidate yaw " << yaw << ": inliers " << 100 * fit << "%\n";
    ++n;
    if (fit > best_fit) { best_fit = fit; best = T; }
  }
  if (n == 0) { std::cerr << "no usable candidates\n"; return 1; }

  std::ofstream out(kv["out"]);
  out.imbue(std::locale::classic());
  out.precision(12);
  out << best.matrix() << "\n";

  auto shifted = [&](double dx) { Eigen::Isometry3d T = best; T.translation().x() += dx; return T; };
  const double f5 = inlier_fraction(map, cloud, shifted(5.0), radius);
  const double f20 = inlier_fraction(map, cloud, shifted(20.0), radius);
  const double yaw = std::atan2(best.linear()(1, 0), best.linear()(0, 0)) * 180.0 / M_PI;
  std::ostringstream rep;
  rep.imbue(std::locale::classic());
  rep << "yaw_deg: " << yaw << "\ntranslation_m: [" << best.translation().x() << ", "
      << best.translation().y() << ", " << best.translation().z() << "]\n"
      << "inliers_within_0.3m: " << best_fit << "\nshifted_5m_inliers: " << f5
      << "\nshifted_20m_inliers: " << f20 << "\ncandidates_refined: " << n << "\n";
  std::cout << rep.str();
  if (kv.count("report")) std::ofstream(kv["report"]) << rep.str();
  return 0;
}
