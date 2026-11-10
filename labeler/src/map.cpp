#include "degenbench/labeler/map.hpp"

#include <memory>

#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/util/downsampling.hpp>

namespace degenbench {

std::vector<Eigen::Vector3d> voxel_downsample(const std::vector<Eigen::Vector3d>& points,
                                              double leaf_size) {
  const small_gicp::PointCloud cloud(points);
  const auto down = small_gicp::voxelgrid_sampling(cloud, leaf_size);
  std::vector<Eigen::Vector3d> out(down->size());
  for (size_t i = 0; i < down->size(); ++i) out[i] = down->point(i).head<3>();
  return out;
}

std::vector<Eigen::Vector3d> crop_radius(const std::vector<Eigen::Vector3d>& points,
                                         const Eigen::Vector3d& center, double radius) {
  const double r2 = radius * radius;
  std::vector<Eigen::Vector3d> out;
  for (const auto& p : points)
    if ((p - center).squaredNorm() <= r2) out.push_back(p);
  return out;
}

double map_coverage(const std::vector<Eigen::Vector3d>& map,
                    const std::vector<Eigen::Vector3d>& scan,
                    const Eigen::Isometry3d& T_map_sensor, double max_distance) {
  if (map.empty() || scan.empty()) return 0.0;
  const auto cloud = std::make_shared<small_gicp::PointCloud>(map);
  const small_gicp::KdTree<small_gicp::PointCloud> tree(cloud);
  const double max_sq = max_distance * max_distance;
  size_t covered = 0;
  for (const auto& p : scan) {
    Eigen::Vector4d q;
    q << T_map_sensor * p, 1.0;
    size_t idx;
    double sq;
    if (tree.nearest_neighbor_search(q, &idx, &sq) && sq <= max_sq) ++covered;
  }
  return static_cast<double>(covered) / static_cast<double>(scan.size());
}

}  // namespace degenbench
