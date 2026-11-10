// Voxelize a PCD map (any supported encoding) and write float32 x y z.
//
//   map_to_bin MAP.pcd OUT.bin [voxel_m=0.3]
//
// Lets Python tools read maps without an LZF decoder.

#include <cstdio>
#include <fstream>
#include <string>

#include "degenbench/labeler/io.hpp"
#include "degenbench/labeler/map.hpp"

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: map_to_bin MAP.pcd OUT.bin [voxel_m]\n");
    return 2;
  }
  const double voxel = argc > 3 ? std::stod(argv[3]) : 0.3;
  const auto pts = degenbench::voxel_downsample(degenbench::read_pcd(argv[1]), voxel);
  std::ofstream f(argv[2], std::ios::binary);
  for (const auto& p : pts) {
    const float v[3] = {float(p.x()), float(p.y()), float(p.z())};
    f.write(reinterpret_cast<const char*>(v), sizeof(v));
  }
  std::printf("%zu points\n", pts.size());
  return 0;
}
