#include "degenbench/labeler/io.hpp"

#include <cstring>
#include <fstream>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace degenbench {
namespace {

std::ifstream open_or_throw(const std::string& path, std::ios::openmode mode = std::ios::in) {
  std::ifstream f(path, mode);
  if (!f) throw std::runtime_error("cannot open " + path);
  f.imbue(std::locale::classic());  // '.' decimals regardless of the host locale
  return f;
}

std::vector<std::string> split(const std::string& line, char sep) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, sep)) out.push_back(item);
  return out;
}

double to_double(const std::string& s) {
  std::istringstream is(s);
  is.imbue(std::locale::classic());
  double v;
  if (!(is >> v)) throw std::runtime_error("not a number: '" + s + "'");
  return v;
}

}  // namespace

std::vector<FrameRecord> read_frames_csv(const std::string& path) {
  auto f = open_or_throw(path);
  std::string line;
  std::getline(f, line);  // header
  std::vector<FrameRecord> frames;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    const auto v = split(line, ',');
    if (v.size() < 10) throw std::runtime_error(path + ": malformed row: " + line);
    FrameRecord r;
    r.index = static_cast<int>(to_double(v[0]));
    r.stamp = to_double(v[1]);
    r.T_world_lidar.translation() << to_double(v[2]), to_double(v[3]), to_double(v[4]);
    const Eigen::Quaterniond q(to_double(v[8]), to_double(v[5]), to_double(v[6]), to_double(v[7]));
    r.T_world_lidar.linear() = q.normalized().toRotationMatrix();
    r.num_points = static_cast<int>(to_double(v[9]));
    frames.push_back(r);
  }
  return frames;
}

std::vector<Eigen::Vector3d> read_bin_points(const std::string& path) {
  auto f = open_or_throw(path, std::ios::binary);
  std::vector<char> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (buf.size() % (3 * sizeof(float)) != 0) throw std::runtime_error(path + ": size not a multiple of 12");
  const size_t n = buf.size() / (3 * sizeof(float));
  std::vector<Eigen::Vector3d> pts(n);
  for (size_t i = 0; i < n; ++i) {
    float xyz[3];
    std::memcpy(xyz, buf.data() + i * sizeof(xyz), sizeof(xyz));
    pts[i] = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  }
  return pts;
}

std::vector<Eigen::Vector3d> read_pcd(const std::string& path) {
  auto f = open_or_throw(path, std::ios::binary);
  std::vector<std::string> fields;
  std::vector<int> sizes, counts;
  std::vector<char> types;
  size_t points = 0;
  std::string data, line;
  while (std::getline(f, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    std::istringstream is(line);
    is.imbue(std::locale::classic());
    std::string key;
    is >> key;
    std::string tok;
    if (key == "FIELDS") { while (is >> tok) fields.push_back(tok); }
    else if (key == "SIZE") { while (is >> tok) sizes.push_back(std::stoi(tok)); }
    else if (key == "TYPE") { while (is >> tok) types.push_back(tok[0]); }
    else if (key == "COUNT") { while (is >> tok) counts.push_back(std::stoi(tok)); }
    else if (key == "POINTS") { is >> points; }
    else if (key == "DATA") { is >> data; break; }
  }
  if (counts.empty()) counts.assign(fields.size(), 1);
  if (fields.empty() || sizes.size() != fields.size() || types.size() != fields.size() ||
      counts.size() != fields.size()) {
    throw std::runtime_error(path + ": malformed PCD header");
  }

  // Byte offset and index of each component within a point.
  int xyz_field[3] = {-1, -1, -1};
  std::vector<int> offsets(fields.size());
  int stride = 0, columns = 0;
  std::vector<int> column_of(fields.size());
  for (size_t i = 0; i < fields.size(); ++i) {
    offsets[i] = stride;
    column_of[i] = columns;
    stride += sizes[i] * counts[i];
    columns += counts[i];
    for (int a = 0; a < 3; ++a)
      if (fields[i] == std::string(1, "xyz"[a])) xyz_field[a] = static_cast<int>(i);
  }
  for (int a = 0; a < 3; ++a) {
    if (xyz_field[a] < 0) throw std::runtime_error(path + ": PCD lacks x, y or z");
    const int i = xyz_field[a];
    if (types[i] != 'F' || (sizes[i] != 4 && sizes[i] != 8))
      throw std::runtime_error(path + ": x/y/z must be float32 or float64");
  }

  std::vector<Eigen::Vector3d> pts;
  pts.reserve(points);
  if (data == "ascii") {
    while (pts.size() < points && std::getline(f, line)) {
      if (line.empty()) continue;
      std::istringstream is(line);
      is.imbue(std::locale::classic());
      std::vector<double> vals(columns);
      for (auto& v : vals) is >> v;
      if (!is) throw std::runtime_error(path + ": short ASCII row");
      pts.emplace_back(vals[column_of[xyz_field[0]]], vals[column_of[xyz_field[1]]],
                       vals[column_of[xyz_field[2]]]);
    }
  } else if (data == "binary") {
    std::vector<char> buf(static_cast<size_t>(stride) * points);
    f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    if (static_cast<size_t>(f.gcount()) != buf.size()) throw std::runtime_error(path + ": truncated");
    for (size_t p = 0; p < points; ++p) {
      Eigen::Vector3d v;
      for (int a = 0; a < 3; ++a) {
        const int i = xyz_field[a];
        const char* src = buf.data() + p * stride + offsets[i];
        if (sizes[i] == 4) { float x; std::memcpy(&x, src, 4); v[a] = x; }
        else { double x; std::memcpy(&x, src, 8); v[a] = x; }
      }
      pts.push_back(v);
    }
  } else if (data == "binary_compressed") {
    // uint32 compressed size, uint32 uncompressed size, LZF payload. The payload
    // is field-major: all values of field 0, then all of field 1, and so on.
    uint32_t sizes_hdr[2];
    f.read(reinterpret_cast<char*>(sizes_hdr), sizeof(sizes_hdr));
    if (!f) throw std::runtime_error(path + ": truncated compressed header");
    std::vector<char> comp(sizes_hdr[0]);
    f.read(comp.data(), static_cast<std::streamsize>(comp.size()));
    if (static_cast<size_t>(f.gcount()) != comp.size()) throw std::runtime_error(path + ": truncated");
    if (sizes_hdr[1] != static_cast<size_t>(stride) * points)
      throw std::runtime_error(path + ": compressed size does not match header");
    const std::vector<char> raw = lzf_decompress(comp.data(), comp.size(), sizes_hdr[1]);
    std::vector<size_t> field_start(fields.size());
    size_t acc = 0;
    for (size_t i = 0; i < fields.size(); ++i) {
      field_start[i] = acc;
      acc += static_cast<size_t>(sizes[i]) * counts[i] * points;
    }
    pts.resize(points);
    for (int a = 0; a < 3; ++a) {
      const int i = xyz_field[a];
      const size_t step = static_cast<size_t>(sizes[i]) * counts[i];
      for (size_t p = 0; p < points; ++p) {
        const char* src = raw.data() + field_start[i] + p * step;
        if (sizes[i] == 4) { float x; std::memcpy(&x, src, 4); pts[p][a] = x; }
        else { double x; std::memcpy(&x, src, 8); pts[p][a] = x; }
      }
    }
  } else {
    throw std::runtime_error(path + ": unsupported PCD DATA '" + data + "'");
  }
  if (pts.size() != points) throw std::runtime_error(path + ": fewer points than header says");
  return pts;
}

std::vector<char> lzf_decompress(const char* in, size_t in_size, size_t expected_size) {
  std::vector<char> out(expected_size);
  const auto* ip = reinterpret_cast<const unsigned char*>(in);
  const auto* const ip_end = ip + in_size;
  size_t op = 0;
  auto corrupt = [] { throw std::runtime_error("corrupt LZF stream"); };
  while (ip < ip_end) {
    unsigned ctrl = *ip++;
    if (ctrl < 32) {  // literal run of ctrl + 1 bytes
      const size_t len = ctrl + 1;
      if (ip + len > ip_end || op + len > expected_size) corrupt();
      std::memcpy(out.data() + op, ip, len);
      ip += len;
      op += len;
    } else {  // back reference
      size_t len = ctrl >> 5;
      if (len == 7) {
        if (ip >= ip_end) corrupt();
        len += *ip++;
      }
      if (ip >= ip_end) corrupt();
      const size_t back = ((static_cast<size_t>(ctrl & 0x1f) << 8) | *ip++) + 1;
      len += 2;
      if (back > op || op + len > expected_size) corrupt();
      for (size_t k = 0; k < len; ++k, ++op) out[op] = out[op - back];  // may overlap
    }
  }
  if (op != expected_size) corrupt();
  return out;
}

void write_pcd(const std::string& path, const std::vector<Eigen::Vector3d>& points) {
  std::ofstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("cannot write " + path);
  f << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
    << "WIDTH " << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
    << "POINTS " << points.size() << "\nDATA binary\n";
  for (const auto& p : points) {
    const float xyz[3] = {static_cast<float>(p.x()), static_cast<float>(p.y()),
                          static_cast<float>(p.z())};
    f.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
}

}  // namespace degenbench
