#include "mesh2solid/pipeline.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <ios>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <zlib.h>

namespace mesh2solid {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
  double x {};
  double y {};
};

struct EdgeKey {
  int a {};
  int b {};

  bool operator==(const EdgeKey& other) const { return a == other.a && b == other.b; }
};

struct EdgeKeyHash {
  std::size_t operator()(const EdgeKey& key) const {
    return (static_cast<std::size_t>(key.a) << 32U) ^ static_cast<std::size_t>(key.b);
  }
};

struct DirectedEdgeKey {
  int from {};
  int to {};

  bool operator==(const DirectedEdgeKey& other) const {
    return from == other.from && to == other.to;
  }
};

struct DirectedEdgeKeyHash {
  std::size_t operator()(const DirectedEdgeKey& key) const {
    return (static_cast<std::size_t>(key.from) << 32U) ^ static_cast<std::size_t>(key.to);
  }
};

struct FaceKey {
  int a {};
  int b {};
  int c {};

  bool operator==(const FaceKey& other) const {
    return a == other.a && b == other.b && c == other.c;
  }
};

struct FaceKeyHash {
  std::size_t operator()(const FaceKey& key) const {
    return (static_cast<std::size_t>(key.a) << 42U) ^
           (static_cast<std::size_t>(key.b) << 21U) ^
           static_cast<std::size_t>(key.c);
  }
};

struct GridKey {
  long long x {};
  long long y {};
  long long z {};

  bool operator==(const GridKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridKeyHash {
  std::size_t operator()(const GridKey& key) const {
    std::size_t seed = static_cast<std::size_t>(key.x);
    seed ^= static_cast<std::size_t>(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    seed ^= static_cast<std::size_t>(key.z) + 0x9e3779b97f4a7c15ULL + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

GridKey offset_grid_key(const GridKey& key, long long dx, long long dy, long long dz) {
  return {key.x + dx, key.y + dy, key.z + dz};
}

struct TriangleEdgeOccurrence {
  int triangle {};
  int edge_index {};
};

struct AdjacencyData {
  std::vector<std::array<int, 3>> neighbors;
  std::vector<std::vector<int>> triangle_graph;
  std::vector<std::vector<int>> vertex_to_triangles;
  std::unordered_map<EdgeKey, std::vector<TriangleEdgeOccurrence>, EdgeKeyHash> edge_occurrences;
};

struct ComponentRemoval {
  std::size_t components_removed {};
  std::size_t triangles_removed {};
};

struct PlaneBasis {
  Vec3 u {};
  Vec3 v {};
};

struct ZipEntry {
  std::string name;
  std::uint16_t compression_method {};
  std::uint32_t compressed_size {};
  std::uint32_t uncompressed_size {};
  std::uint32_t local_header_offset {};
};

struct ZipArchive {
  std::vector<std::uint8_t> bytes;
  std::unordered_map<std::string, ZipEntry> entries;
};

struct XmlNode {
  std::string name;
  std::unordered_map<std::string, std::string> attributes;
  std::vector<XmlNode> children;
};

struct AffineTransform {
  std::array<std::array<double, 4>, 4> values {{
      {1.0, 0.0, 0.0, 0.0},
      {0.0, 1.0, 0.0, 0.0},
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0},
  }};
};

struct ThreeMFComponent {
  int object_id {};
  AffineTransform transform;
};

struct ThreeMFObject {
  int id {};
  std::vector<Vec3> vertices;
  std::vector<std::array<int, 3>> triangles;
  std::vector<ThreeMFComponent> components;
};

struct UnionFind {
  explicit UnionFind(std::size_t count) : parent(count), rank(count, 0) {
    std::iota(parent.begin(), parent.end(), 0);
  }

  int find(int value) {
    if (parent[value] != value) {
      parent[value] = find(parent[value]);
    }
    return parent[value];
  }

  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b) {
      return;
    }
    if (rank[a] < rank[b]) {
      std::swap(a, b);
    }
    parent[b] = a;
    if (rank[a] == rank[b]) {
      ++rank[a];
    }
  }

  std::vector<int> parent;
  std::vector<int> rank;
};

double clamp(double value, double low, double high) {
  return std::max(low, std::min(high, value));
}

Vec3 operator+(const Vec3& lhs, const Vec3& rhs) {
  return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 operator-(const Vec3& lhs, const Vec3& rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 operator*(const Vec3& value, double scale) {
  return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 operator/(const Vec3& value, double scale) {
  return {value.x / scale, value.y / scale, value.z / scale};
}

Vec3& operator+=(Vec3& lhs, const Vec3& rhs) {
  lhs = lhs + rhs;
  return lhs;
}

double dot(const Vec3& lhs, const Vec3& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

Vec3 cross(const Vec3& lhs, const Vec3& rhs) {
  return {
      lhs.y * rhs.z - lhs.z * rhs.y,
      lhs.z * rhs.x - lhs.x * rhs.z,
      lhs.x * rhs.y - lhs.y * rhs.x,
  };
}

double length_squared(const Vec3& value) {
  return dot(value, value);
}

double length(const Vec3& value) {
  return std::sqrt(length_squared(value));
}

Vec3 normalized(const Vec3& value) {
  const double magnitude = length(value);
  if (magnitude <= 1e-12) {
    return {0.0, 0.0, 0.0};
  }
  return value / magnitude;
}

double angle_degrees_unsigned(const Vec3& lhs, const Vec3& rhs) {
  const Vec3 a = normalized(lhs);
  const Vec3 b = normalized(rhs);
  const double cosine = clamp(std::abs(dot(a, b)), -1.0, 1.0);
  return std::acos(cosine) * 180.0 / kPi;
}

void include_point(Bounds& bounds, const Vec3& point) {
  if (!bounds.valid) {
    bounds.min = point;
    bounds.max = point;
    bounds.valid = true;
    return;
  }
  bounds.min.x = std::min(bounds.min.x, point.x);
  bounds.min.y = std::min(bounds.min.y, point.y);
  bounds.min.z = std::min(bounds.min.z, point.z);
  bounds.max.x = std::max(bounds.max.x, point.x);
  bounds.max.y = std::max(bounds.max.y, point.y);
  bounds.max.z = std::max(bounds.max.z, point.z);
}

Vec3 bounds_extents(const Bounds& bounds) {
  return {bounds.max.x - bounds.min.x, bounds.max.y - bounds.min.y, bounds.max.z - bounds.min.z};
}

double bounds_diagonal(const Bounds& bounds) {
  return bounds.valid ? length(bounds_extents(bounds)) : 0.0;
}

EdgeKey make_edge_key(int a, int b) {
  if (a < b) {
    return {a, b};
  }
  return {b, a};
}

FaceKey make_face_key(std::array<int, 3> vertices) {
  std::sort(vertices.begin(), vertices.end());
  return {vertices[0], vertices[1], vertices[2]};
}

std::array<std::pair<int, int>, 3> oriented_edges(const Triangle& triangle) {
  return {{
      {triangle.vertices[0], triangle.vertices[1]},
      {triangle.vertices[1], triangle.vertices[2]},
      {triangle.vertices[2], triangle.vertices[0]},
  }};
}

bool edge_direction_matches(const Triangle& triangle, const EdgeKey& key) {
  for (const auto& edge : oriented_edges(triangle)) {
    if (make_edge_key(edge.first, edge.second) == key) {
      return edge.first == key.a && edge.second == key.b;
    }
  }
  return false;
}

std::optional<EdgeKey> shared_edge_key(const Triangle& lhs, const Triangle& rhs) {
  for (const auto& lhs_edge : oriented_edges(lhs)) {
    const EdgeKey key = make_edge_key(lhs_edge.first, lhs_edge.second);
    for (const auto& rhs_edge : oriented_edges(rhs)) {
      if (make_edge_key(rhs_edge.first, rhs_edge.second) == key) {
        return key;
      }
    }
  }
  return std::nullopt;
}

void flip_triangle(Triangle& triangle) {
  std::swap(triangle.vertices[1], triangle.vertices[2]);
  triangle.normal = triangle.normal * -1.0;
}

double triangle_area(const Vec3& a, const Vec3& b, const Vec3& c) {
  return length(cross(b - a, c - a)) * 0.5;
}

Vec3 triangle_normal(const Vec3& a, const Vec3& b, const Vec3& c) {
  return normalized(cross(b - a, c - a));
}

std::string format_double(double value, int precision = 8) {
  const double epsilon = 0.5 / std::pow(10.0, precision);
  if (std::abs(value) < epsilon) {
    value = 0.0;
  }
  std::ostringstream output;
  output << std::fixed << std::setprecision(precision) << value;
  return output.str();
}

std::string escape_json(const std::string& value) {
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\':
        output << "\\\\";
        break;
      case '"':
        output << "\\\"";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        output << character;
        break;
    }
  }
  return output.str();
}

std::string bool_json(bool value) {
  return value ? "true" : "false";
}

std::string vec3_json(const Vec3& value) {
  std::ostringstream output;
  output << "[" << format_double(value.x) << "," << format_double(value.y) << ","
         << format_double(value.z) << "]";
  return output.str();
}

Vec2 project_to_basis(const Vec3& point, const PlaneBasis& basis) {
  return {dot(point, basis.u), dot(point, basis.v)};
}

double signed_area_2d(const std::vector<Vec2>& loop) {
  if (loop.size() < 3) {
    return 0.0;
  }
  double area = 0.0;
  for (std::size_t i = 0; i < loop.size(); ++i) {
    const Vec2& a = loop[i];
    const Vec2& b = loop[(i + 1) % loop.size()];
    area += a.x * b.y - b.x * a.y;
  }
  return area * 0.5;
}

double cross_2d(const Vec2& a, const Vec2& b, const Vec2& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double point_line_distance_2d(const Vec2& point, const Vec2& a, const Vec2& b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_sq = dx * dx + dy * dy;
  if (length_sq <= 1e-12) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  const double cross_value = std::abs((point.x - a.x) * dy - (point.y - a.y) * dx);
  return cross_value / std::sqrt(length_sq);
}

double point_segment_distance_3d(const Vec3& point, const Vec3& a, const Vec3& b, double& t) {
  const Vec3 segment = b - a;
  const double length_sq = length_squared(segment);
  if (length_sq <= 1e-12) {
    t = 0.0;
    return length(point - a);
  }
  t = dot(point - a, segment) / length_sq;
  const double clamped_t = clamp(t, 0.0, 1.0);
  const Vec3 closest = a + segment * clamped_t;
  return length(point - closest);
}

std::optional<int> find_nearby_grid_vertex(
    const std::unordered_map<GridKey, std::vector<int>, GridKeyHash>& buckets,
    const std::vector<Vec3>& vertices,
    const GridKey& key,
    const Vec3& point,
    double epsilon) {
  std::optional<int> best_index;
  double best_distance = epsilon;

  for (long long dx = -1; dx <= 1; ++dx) {
    for (long long dy = -1; dy <= 1; ++dy) {
      for (long long dz = -1; dz <= 1; ++dz) {
        const auto bucket_it = buckets.find(offset_grid_key(key, dx, dy, dz));
        if (bucket_it == buckets.end()) {
          continue;
        }
        for (int candidate : bucket_it->second) {
          const double distance = length(vertices[candidate] - point);
          if (distance <= best_distance) {
            best_distance = distance;
            best_index = candidate;
          }
        }
      }
    }
  }

  return best_index;
}

PlaneBasis basis_from_normal(const Vec3& normal) {
  Vec3 reference = std::abs(normal.z) < 0.9 ? Vec3 {0.0, 0.0, 1.0} : Vec3 {0.0, 1.0, 0.0};
  Vec3 u = normalized(cross(reference, normal));
  if (length_squared(u) <= 1e-12) {
    reference = {1.0, 0.0, 0.0};
    u = normalized(cross(reference, normal));
  }
  const Vec3 v = normalized(cross(normal, u));
  return {u, v};
}

double plane_distance(const PlaneFit& fit, const Vec3& point) {
  return dot(fit.normal, point) + fit.d;
}

std::string lowercase_copy(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string local_xml_name(std::string name) {
  const std::size_t separator = name.find(':');
  if (separator != std::string::npos) {
    name = name.substr(separator + 1);
  }
  return lowercase_copy(std::move(name));
}

class SimpleXmlParser {
 public:
  explicit SimpleXmlParser(std::string text) : text_(std::move(text)) {}

  XmlNode parse() {
    XmlNode document;
    document.name = "#document";
    skip_misc();
    while (position_ < text_.size()) {
      if (text_[position_] != '<') {
        ++position_;
        continue;
      }
      document.children.push_back(parse_element());
      skip_misc();
    }
    return document;
  }

 private:
  void skip_whitespace() {
    while (position_ < text_.size() &&
           std::isspace(static_cast<unsigned char>(text_[position_]))) {
      ++position_;
    }
  }

  bool starts_with(const std::string& token) const {
    return text_.compare(position_, token.size(), token) == 0;
  }

  void skip_until(const std::string& token) {
    const std::size_t found = text_.find(token, position_);
    if (found == std::string::npos) {
      throw std::runtime_error("Malformed XML: expected token " + token);
    }
    position_ = found + token.size();
  }

  void skip_misc() {
    while (position_ < text_.size()) {
      skip_whitespace();
      if (starts_with("<?")) {
        skip_until("?>");
        continue;
      }
      if (starts_with("<!--")) {
        skip_until("-->");
        continue;
      }
      if (starts_with("<![CDATA[")) {
        skip_until("]]>");
        continue;
      }
      break;
    }
  }

  std::string parse_name() {
    const std::size_t start = position_;
    while (position_ < text_.size()) {
      const char character = text_[position_];
      if (std::isalnum(static_cast<unsigned char>(character)) || character == '_' ||
          character == ':' || character == '-' || character == '.') {
        ++position_;
      } else {
        break;
      }
    }
    if (start == position_) {
      throw std::runtime_error("Malformed XML: expected tag or attribute name");
    }
    return text_.substr(start, position_ - start);
  }

  std::string parse_attribute_value() {
    skip_whitespace();
    if (position_ >= text_.size() || (text_[position_] != '"' && text_[position_] != '\'')) {
      throw std::runtime_error("Malformed XML: expected quoted attribute value");
    }
    const char quote = text_[position_++];
    std::string value;
    while (position_ < text_.size() && text_[position_] != quote) {
      value.push_back(text_[position_++]);
    }
    if (position_ >= text_.size()) {
      throw std::runtime_error("Malformed XML: unterminated attribute value");
    }
    ++position_;
    return value;
  }

  XmlNode parse_element() {
    if (position_ >= text_.size() || text_[position_] != '<') {
      throw std::runtime_error("Malformed XML: expected start tag");
    }
    ++position_;
    if (position_ < text_.size() && text_[position_] == '/') {
      throw std::runtime_error("Malformed XML: unexpected closing tag");
    }

    XmlNode node;
    node.name = local_xml_name(parse_name());

    while (true) {
      skip_whitespace();
      if (position_ >= text_.size()) {
        throw std::runtime_error("Malformed XML: unexpected end of input");
      }
      if (starts_with("/>")) {
        position_ += 2;
        return node;
      }
      if (text_[position_] == '>') {
        ++position_;
        break;
      }

      std::string attribute_name = local_xml_name(parse_name());
      skip_whitespace();
      if (position_ >= text_.size() || text_[position_] != '=') {
        throw std::runtime_error("Malformed XML: expected '=' after attribute");
      }
      ++position_;
      node.attributes[attribute_name] = parse_attribute_value();
    }

    while (true) {
      skip_misc();
      if (position_ >= text_.size()) {
        throw std::runtime_error("Malformed XML: missing closing tag for " + node.name);
      }
      if (starts_with("</")) {
        position_ += 2;
        const std::string closing_name = local_xml_name(parse_name());
        if (closing_name != node.name) {
          throw std::runtime_error("Malformed XML: mismatched closing tag");
        }
        skip_whitespace();
        if (position_ >= text_.size() || text_[position_] != '>') {
          throw std::runtime_error("Malformed XML: malformed closing tag");
        }
        ++position_;
        return node;
      }
      if (text_[position_] == '<') {
        node.children.push_back(parse_element());
      } else {
        ++position_;
      }
    }
  }

  std::string text_;
  std::size_t position_ {};
};

const XmlNode* find_first_child(const XmlNode& parent, const std::string& name) {
  const std::string lowered_name = lowercase_copy(name);
  for (const XmlNode& child : parent.children) {
    if (child.name == lowered_name) {
      return &child;
    }
  }
  return nullptr;
}

std::vector<const XmlNode*> find_children(const XmlNode& parent, const std::string& name) {
  const std::string lowered_name = lowercase_copy(name);
  std::vector<const XmlNode*> matches;
  for (const XmlNode& child : parent.children) {
    if (child.name == lowered_name) {
      matches.push_back(&child);
    }
  }
  return matches;
}

std::string attribute_or_empty(const XmlNode& node, const std::string& name) {
  const auto found = node.attributes.find(lowercase_copy(name));
  return found == node.attributes.end() ? std::string() : found->second;
}

std::vector<std::uint8_t> read_binary_file(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }
  input.seekg(0, std::ios::end);
  const std::streamoff size = input.tellg();
  input.seekg(0, std::ios::beg);
  if (size < 0) {
    throw std::runtime_error("Unable to determine file size: " + path.string());
  }
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!input && !bytes.empty()) {
    throw std::runtime_error("Unable to read input file: " + path.string());
  }
  return bytes;
}

std::uint16_t read_le16(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return static_cast<std::uint16_t>(bytes[offset]) |
         (static_cast<std::uint16_t>(bytes[offset + 1]) << 8U);
}

std::uint32_t read_le32(const std::vector<std::uint8_t>& bytes, std::size_t offset) {
  return static_cast<std::uint32_t>(bytes[offset]) |
         (static_cast<std::uint32_t>(bytes[offset + 1]) << 8U) |
         (static_cast<std::uint32_t>(bytes[offset + 2]) << 16U) |
         (static_cast<std::uint32_t>(bytes[offset + 3]) << 24U);
}

std::string normalize_zip_path(std::string path) {
  std::replace(path.begin(), path.end(), '\\', '/');
  while (!path.empty() && (path.front() == '/' || path.front() == '.')) {
    if (path.front() == '.') {
      if (path.size() >= 2 && path[1] == '/') {
        path.erase(0, 2);
      } else {
        path.erase(path.begin());
      }
    } else {
      path.erase(path.begin());
    }
  }

  std::filesystem::path normalized = std::filesystem::path(path).lexically_normal();
  path = normalized.generic_string();
  while (!path.empty() && path.front() == '/') {
    path.erase(path.begin());
  }
  return path;
}

ZipArchive read_zip_archive(const std::filesystem::path& path) {
  ZipArchive archive;
  archive.bytes = read_binary_file(path);
  if (archive.bytes.size() < 22) {
    throw std::runtime_error("3MF package is too small to be a valid ZIP archive: " + path.string());
  }

  const std::uint32_t eocd_signature = 0x06054b50U;
  std::size_t eocd_offset = std::string::npos;
  const std::size_t search_start =
      archive.bytes.size() > 65557 ? archive.bytes.size() - 65557 : 0;
  for (std::size_t offset = archive.bytes.size() - 22; offset + 1 > search_start; --offset) {
    if (read_le32(archive.bytes, offset) == eocd_signature) {
      eocd_offset = offset;
      break;
    }
    if (offset == 0) {
      break;
    }
  }
  if (eocd_offset == std::string::npos) {
    throw std::runtime_error("Unable to locate ZIP central directory in 3MF package: " +
                             path.string());
  }

  const std::uint16_t entry_count = read_le16(archive.bytes, eocd_offset + 10);
  const std::uint32_t central_directory_size = read_le32(archive.bytes, eocd_offset + 12);
  const std::uint32_t central_directory_offset = read_le32(archive.bytes, eocd_offset + 16);

  if (central_directory_offset + central_directory_size > archive.bytes.size()) {
    throw std::runtime_error("Malformed ZIP central directory in 3MF package: " + path.string());
  }

  std::size_t cursor = central_directory_offset;
  for (std::uint16_t index = 0; index < entry_count; ++index) {
    if (cursor + 46 > archive.bytes.size() || read_le32(archive.bytes, cursor) != 0x02014b50U) {
      throw std::runtime_error("Malformed ZIP directory entry in 3MF package: " + path.string());
    }

    const std::uint16_t compression_method = read_le16(archive.bytes, cursor + 10);
    const std::uint32_t compressed_size = read_le32(archive.bytes, cursor + 20);
    const std::uint32_t uncompressed_size = read_le32(archive.bytes, cursor + 24);
    const std::uint16_t file_name_length = read_le16(archive.bytes, cursor + 28);
    const std::uint16_t extra_length = read_le16(archive.bytes, cursor + 30);
    const std::uint16_t comment_length = read_le16(archive.bytes, cursor + 32);
    const std::uint32_t local_header_offset = read_le32(archive.bytes, cursor + 42);

    if (cursor + 46 + file_name_length + extra_length + comment_length > archive.bytes.size()) {
      throw std::runtime_error("Malformed ZIP filename in 3MF package: " + path.string());
    }

    std::string name(reinterpret_cast<const char*>(archive.bytes.data() + cursor + 46),
                     file_name_length);
    name = normalize_zip_path(std::move(name));
    archive.entries[name] = {name, compression_method, compressed_size, uncompressed_size,
                             local_header_offset};

    cursor += 46 + file_name_length + extra_length + comment_length;
  }

  return archive;
}

std::vector<std::uint8_t> unzip_entry(const ZipArchive& archive, const ZipEntry& entry) {
  if (entry.local_header_offset + 30 > archive.bytes.size() ||
      read_le32(archive.bytes, entry.local_header_offset) != 0x04034b50U) {
    throw std::runtime_error("Malformed ZIP local file header in 3MF package");
  }

  const std::uint16_t file_name_length = read_le16(archive.bytes, entry.local_header_offset + 26);
  const std::uint16_t extra_length = read_le16(archive.bytes, entry.local_header_offset + 28);
  const std::size_t data_offset =
      entry.local_header_offset + 30 + file_name_length + extra_length;
  if (data_offset + entry.compressed_size > archive.bytes.size()) {
    throw std::runtime_error("Malformed ZIP file data in 3MF package");
  }

  const std::uint8_t* compressed = archive.bytes.data() + data_offset;
  if (entry.compression_method == 0) {
    return std::vector<std::uint8_t>(compressed, compressed + entry.compressed_size);
  }
  if (entry.compression_method != 8) {
    throw std::runtime_error("Unsupported ZIP compression method in 3MF package");
  }

  std::vector<std::uint8_t> uncompressed(entry.uncompressed_size);
  z_stream stream {};
  stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(compressed));
  stream.avail_in = entry.compressed_size;
  stream.next_out = reinterpret_cast<Bytef*>(uncompressed.data());
  stream.avail_out = entry.uncompressed_size;

  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
    throw std::runtime_error("Unable to initialize ZIP decompression for 3MF package");
  }
  const int status = inflate(&stream, Z_FINISH);
  inflateEnd(&stream);
  if (status != Z_STREAM_END) {
    throw std::runtime_error("Unable to decompress ZIP entry from 3MF package");
  }

  uncompressed.resize(stream.total_out);
  return uncompressed;
}

std::string unzip_text_entry(const ZipArchive& archive, const std::string& name) {
  const auto found = archive.entries.find(normalize_zip_path(name));
  if (found == archive.entries.end()) {
    throw std::runtime_error("Missing entry in 3MF package: " + name);
  }
  const std::vector<std::uint8_t> bytes = unzip_entry(archive, found->second);
  return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}

std::string resolve_zip_target(const std::string& source_part, const std::string& target) {
  if (!target.empty() && target.front() == '/') {
    return normalize_zip_path(target.substr(1));
  }

  const std::filesystem::path source_path = std::filesystem::path(source_part).parent_path();
  return normalize_zip_path((source_path / target).generic_string());
}

std::string find_3mf_model_part(const ZipArchive& archive) {
  auto direct = archive.entries.find("3D/3dmodel.model");
  if (direct != archive.entries.end()) {
    return direct->first;
  }

  if (archive.entries.count("_rels/.rels") > 0) {
    const XmlNode relationships_document = SimpleXmlParser(unzip_text_entry(archive, "_rels/.rels")).parse();
    const XmlNode* relationships = find_first_child(relationships_document, "relationships");
    if (relationships != nullptr) {
      for (const XmlNode* relationship : find_children(*relationships, "relationship")) {
        const std::string type = attribute_or_empty(*relationship, "type");
        if (type.find("3dmodel") != std::string::npos) {
          const std::string target = attribute_or_empty(*relationship, "target");
          if (!target.empty()) {
            return resolve_zip_target("", target);
          }
        }
      }
    }
  }

  for (const auto& [name, entry] : archive.entries) {
    (void)entry;
    if (name.size() >= 6 && lowercase_copy(name.substr(name.size() - 6)) == ".model") {
      return name;
    }
  }

  throw std::runtime_error("Unable to find model part in 3MF package");
}

double parse_required_double(const std::string& value, const std::string& context) {
  try {
    return std::stod(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid floating-point value in " + context + ": " + value);
  }
}

int parse_required_int(const std::string& value, const std::string& context) {
  try {
    return std::stoi(value);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid integer value in " + context + ": " + value);
  }
}

AffineTransform identity_transform() {
  return {};
}

AffineTransform combine_transforms(const AffineTransform& lhs, const AffineTransform& rhs) {
  AffineTransform combined;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      combined.values[row][column] = 0.0;
      for (int inner = 0; inner < 4; ++inner) {
        combined.values[row][column] += lhs.values[row][inner] * rhs.values[inner][column];
      }
    }
  }
  return combined;
}

Vec3 apply_transform(const AffineTransform& transform, const Vec3& point) {
  return {
      point.x * transform.values[0][0] + point.y * transform.values[1][0] +
          point.z * transform.values[2][0] + transform.values[3][0],
      point.x * transform.values[0][1] + point.y * transform.values[1][1] +
          point.z * transform.values[2][1] + transform.values[3][1],
      point.x * transform.values[0][2] + point.y * transform.values[1][2] +
          point.z * transform.values[2][2] + transform.values[3][2],
  };
}

AffineTransform parse_3mf_transform(const std::string& value) {
  if (value.empty()) {
    return identity_transform();
  }

  std::istringstream parser(value);
  std::array<double, 12> terms {};
  for (double& term : terms) {
    if (!(parser >> term)) {
      throw std::runtime_error("Malformed 3MF transform attribute");
    }
  }

  AffineTransform transform;
  transform.values = {{
      {terms[0], terms[1], terms[2], 0.0},
      {terms[3], terms[4], terms[5], 0.0},
      {terms[6], terms[7], terms[8], 0.0},
      {terms[9], terms[10], terms[11], 1.0},
  }};
  return transform;
}

double unit_scale_to_millimeters(const std::string& unit_name) {
  const std::string unit = lowercase_copy(unit_name);
  if (unit.empty() || unit == "millimeter") {
    return 1.0;
  }
  if (unit == "micron") {
    return 0.001;
  }
  if (unit == "centimeter") {
    return 10.0;
  }
  if (unit == "meter") {
    return 1000.0;
  }
  if (unit == "inch") {
    return 25.4;
  }
  if (unit == "foot") {
    return 304.8;
  }
  throw std::runtime_error("Unsupported 3MF unit: " + unit_name);
}

bool file_looks_binary_stl(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }

  input.seekg(0, std::ios::end);
  const std::streamoff size = input.tellg();
  if (size < 84) {
    return false;
  }

  input.seekg(80, std::ios::beg);
  std::uint32_t triangle_count = 0;
  input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
  if (!input) {
    return false;
  }

  const std::uint64_t expected_size = 84ULL + static_cast<std::uint64_t>(triangle_count) * 50ULL;
  return expected_size == static_cast<std::uint64_t>(size);
}

MeshModel load_ascii_stl(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }

  MeshModel mesh;
  mesh.source_format = "ascii_stl";

  std::vector<int> current_triangle_vertices;
  current_triangle_vertices.reserve(3);
  std::string line;

  while (std::getline(input, line)) {
    std::istringstream parser(line);
    std::string keyword;
    parser >> keyword;
    if (keyword != "vertex") {
      continue;
    }

    Vec3 vertex {};
    parser >> vertex.x >> vertex.y >> vertex.z;
    if (!parser) {
      throw std::runtime_error("Malformed vertex in ASCII STL: " + path.string());
    }

    mesh.vertices.push_back(vertex);
    current_triangle_vertices.push_back(static_cast<int>(mesh.vertices.size() - 1));
    if (current_triangle_vertices.size() == 3) {
      Triangle triangle;
      triangle.vertices = {current_triangle_vertices[0], current_triangle_vertices[1],
                           current_triangle_vertices[2]};
      mesh.triangles.push_back(triangle);
      current_triangle_vertices.clear();
    }
  }

  if (!current_triangle_vertices.empty()) {
    throw std::runtime_error("ASCII STL ended mid-triangle: " + path.string());
  }

  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles found in ASCII STL: " + path.string());
  }

  return mesh;
}

MeshModel load_binary_stl(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("Unable to open input file: " + path.string());
  }

  MeshModel mesh;
  mesh.source_format = "binary_stl";

  char header[80] {};
  input.read(header, sizeof(header));
  std::uint32_t triangle_count = 0;
  input.read(reinterpret_cast<char*>(&triangle_count), sizeof(triangle_count));
  if (!input) {
    throw std::runtime_error("Malformed binary STL header: " + path.string());
  }

  mesh.vertices.reserve(static_cast<std::size_t>(triangle_count) * 3);
  mesh.triangles.reserve(triangle_count);

  for (std::uint32_t i = 0; i < triangle_count; ++i) {
    float facet_normal[3] {};
    float facet_vertices[9] {};
    std::uint16_t attribute_byte_count = 0;
    input.read(reinterpret_cast<char*>(facet_normal), sizeof(facet_normal));
    input.read(reinterpret_cast<char*>(facet_vertices), sizeof(facet_vertices));
    input.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(attribute_byte_count));
    if (!input) {
      throw std::runtime_error("Malformed binary STL facet: " + path.string());
    }

    Triangle triangle;
    for (int vertex_index = 0; vertex_index < 3; ++vertex_index) {
      const Vec3 vertex {facet_vertices[vertex_index * 3], facet_vertices[vertex_index * 3 + 1],
                         facet_vertices[vertex_index * 3 + 2]};
      mesh.vertices.push_back(vertex);
      triangle.vertices[vertex_index] = static_cast<int>(mesh.vertices.size() - 1);
    }
    mesh.triangles.push_back(triangle);
  }

  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles found in binary STL: " + path.string());
  }

  return mesh;
}

void recompute_mesh_geometry(MeshModel& mesh) {
  mesh.bounds = {};
  for (const Vec3& vertex : mesh.vertices) {
    include_point(mesh.bounds, vertex);
  }

  for (Triangle& triangle : mesh.triangles) {
    const Vec3& a = mesh.vertices[triangle.vertices[0]];
    const Vec3& b = mesh.vertices[triangle.vertices[1]];
    const Vec3& c = mesh.vertices[triangle.vertices[2]];
    triangle.area = triangle_area(a, b, c);
    triangle.normal = triangle.area > 1e-12 ? triangle_normal(a, b, c) : Vec3 {0.0, 0.0, 0.0};
  }
}

MeshModel load_stl(const std::filesystem::path& path) {
  MeshModel mesh = file_looks_binary_stl(path) ? load_binary_stl(path) : load_ascii_stl(path);
  recompute_mesh_geometry(mesh);
  return mesh;
}

MeshModel load_3mf(const std::filesystem::path& path) {
  const ZipArchive archive = read_zip_archive(path);
  const std::string model_part = find_3mf_model_part(archive);
  const XmlNode document = SimpleXmlParser(unzip_text_entry(archive, model_part)).parse();
  const XmlNode* model = find_first_child(document, "model");
  if (model == nullptr) {
    throw std::runtime_error("Malformed 3MF document: missing <model>");
  }

  const double unit_scale = unit_scale_to_millimeters(attribute_or_empty(*model, "unit"));
  const XmlNode* resources = find_first_child(*model, "resources");
  if (resources == nullptr) {
    throw std::runtime_error("Malformed 3MF document: missing <resources>");
  }

  std::unordered_map<int, ThreeMFObject> objects;
  for (const XmlNode* object_node : find_children(*resources, "object")) {
    const int object_id = parse_required_int(attribute_or_empty(*object_node, "id"), "3MF object id");
    ThreeMFObject object;
    object.id = object_id;

    if (const XmlNode* mesh_node = find_first_child(*object_node, "mesh")) {
      const XmlNode* vertices_node = find_first_child(*mesh_node, "vertices");
      const XmlNode* triangles_node = find_first_child(*mesh_node, "triangles");
      if (vertices_node == nullptr || triangles_node == nullptr) {
        throw std::runtime_error("Malformed 3MF mesh object: missing <vertices> or <triangles>");
      }

      for (const XmlNode* vertex_node : find_children(*vertices_node, "vertex")) {
        object.vertices.push_back({
            parse_required_double(attribute_or_empty(*vertex_node, "x"), "3MF vertex x") * unit_scale,
            parse_required_double(attribute_or_empty(*vertex_node, "y"), "3MF vertex y") * unit_scale,
            parse_required_double(attribute_or_empty(*vertex_node, "z"), "3MF vertex z") * unit_scale,
        });
      }

      for (const XmlNode* triangle_node : find_children(*triangles_node, "triangle")) {
        object.triangles.push_back({
            parse_required_int(attribute_or_empty(*triangle_node, "v1"), "3MF triangle v1"),
            parse_required_int(attribute_or_empty(*triangle_node, "v2"), "3MF triangle v2"),
            parse_required_int(attribute_or_empty(*triangle_node, "v3"), "3MF triangle v3"),
        });
      }
    }

    if (const XmlNode* components_node = find_first_child(*object_node, "components")) {
      for (const XmlNode* component_node : find_children(*components_node, "component")) {
        object.components.push_back({
            parse_required_int(attribute_or_empty(*component_node, "objectid"), "3MF component objectid"),
            parse_3mf_transform(attribute_or_empty(*component_node, "transform")),
        });
      }
    }

    objects[object.id] = std::move(object);
  }

  std::vector<std::pair<int, AffineTransform>> build_items;
  if (const XmlNode* build_node = find_first_child(*model, "build")) {
    for (const XmlNode* item_node : find_children(*build_node, "item")) {
      build_items.push_back({
          parse_required_int(attribute_or_empty(*item_node, "objectid"), "3MF build item objectid"),
          parse_3mf_transform(attribute_or_empty(*item_node, "transform")),
      });
    }
  }

  if (build_items.empty()) {
    for (const auto& [object_id, object] : objects) {
      if (!object.vertices.empty() && !object.triangles.empty()) {
        build_items.push_back({object_id, identity_transform()});
      }
    }
  }

  if (build_items.empty()) {
    throw std::runtime_error("No build items or mesh objects found in 3MF document");
  }

  MeshModel mesh;
  mesh.source_format = "3mf";

  std::vector<int> recursion_stack;
  std::function<void(int, const AffineTransform&)> emit_object =
      [&](int object_id, const AffineTransform& transform) {
        const auto object_it = objects.find(object_id);
        if (object_it == objects.end()) {
          throw std::runtime_error("3MF build references unknown object id: " + std::to_string(object_id));
        }

        if (std::find(recursion_stack.begin(), recursion_stack.end(), object_id) != recursion_stack.end()) {
          throw std::runtime_error("3MF object graph contains a component cycle");
        }

        const ThreeMFObject& object = object_it->second;
        recursion_stack.push_back(object_id);

        if (!object.vertices.empty() && !object.triangles.empty()) {
          const int base_index = static_cast<int>(mesh.vertices.size());
          for (const Vec3& vertex : object.vertices) {
            mesh.vertices.push_back(apply_transform(transform, vertex));
          }
          for (const auto& triangle : object.triangles) {
            if (triangle[0] < 0 || triangle[1] < 0 || triangle[2] < 0 ||
                triangle[0] >= static_cast<int>(object.vertices.size()) ||
                triangle[1] >= static_cast<int>(object.vertices.size()) ||
                triangle[2] >= static_cast<int>(object.vertices.size())) {
              throw std::runtime_error("3MF triangle references an invalid vertex index");
            }
            Triangle emitted;
            emitted.vertices = {base_index + triangle[0], base_index + triangle[1], base_index + triangle[2]};
            mesh.triangles.push_back(emitted);
          }
        }

        for (const ThreeMFComponent& component : object.components) {
          emit_object(component.object_id, combine_transforms(component.transform, transform));
        }

        recursion_stack.pop_back();
      };

  for (const auto& [object_id, transform] : build_items) {
    emit_object(object_id, transform);
  }

  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles found in 3MF document");
  }

  recompute_mesh_geometry(mesh);
  return mesh;
}

MeshModel load_mesh(const std::filesystem::path& path) {
  const std::string extension = lowercase_copy(path.extension().string());
  if (extension == ".3mf") {
    return load_3mf(path);
  }
  return load_stl(path);
}

AdjacencyData build_adjacency(const MeshModel& mesh) {
  AdjacencyData adjacency;
  adjacency.neighbors.assign(mesh.triangles.size(), {-1, -1, -1});
  adjacency.triangle_graph.assign(mesh.triangles.size(), {});
  adjacency.vertex_to_triangles.assign(mesh.vertices.size(), {});

  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    for (int vertex : triangle.vertices) {
      adjacency.vertex_to_triangles[vertex].push_back(static_cast<int>(triangle_index));
    }
    const auto edges = oriented_edges(triangle);
    for (int edge_index = 0; edge_index < 3; ++edge_index) {
      adjacency.edge_occurrences[make_edge_key(edges[edge_index].first, edges[edge_index].second)]
          .push_back({static_cast<int>(triangle_index), edge_index});
    }
  }

  for (const auto& [key, occurrences] : adjacency.edge_occurrences) {
    (void)key;
    for (std::size_t i = 0; i < occurrences.size(); ++i) {
      for (std::size_t j = i + 1; j < occurrences.size(); ++j) {
        adjacency.triangle_graph[occurrences[i].triangle].push_back(occurrences[j].triangle);
        adjacency.triangle_graph[occurrences[j].triangle].push_back(occurrences[i].triangle);
      }
    }
    if (occurrences.size() == 2) {
      adjacency.neighbors[occurrences[0].triangle][occurrences[0].edge_index] =
          occurrences[1].triangle;
      adjacency.neighbors[occurrences[1].triangle][occurrences[1].edge_index] =
          occurrences[0].triangle;
    }
  }

  for (auto& neighbors : adjacency.triangle_graph) {
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
  }

  return adjacency;
}

std::size_t count_components(const AdjacencyData& adjacency) {
  if (adjacency.triangle_graph.empty()) {
    return 0;
  }

  std::vector<bool> visited(adjacency.triangle_graph.size(), false);
  std::size_t count = 0;

  for (std::size_t triangle_index = 0; triangle_index < adjacency.triangle_graph.size();
       ++triangle_index) {
    if (visited[triangle_index]) {
      continue;
    }
    ++count;
    std::queue<int> frontier;
    frontier.push(static_cast<int>(triangle_index));
    visited[triangle_index] = true;
    while (!frontier.empty()) {
      const int current = frontier.front();
      frontier.pop();
      for (int neighbor : adjacency.triangle_graph[current]) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          frontier.push(neighbor);
        }
      }
    }
  }
  return count;
}

MeshStats compute_mesh_stats(const MeshModel& mesh) {
  MeshStats stats;
  stats.vertex_count = mesh.vertices.size();
  stats.triangle_count = mesh.triangles.size();
  stats.surface_area = 0.0;
  for (const Triangle& triangle : mesh.triangles) {
    stats.surface_area += triangle.area;
  }
  stats.bbox_diagonal = bounds_diagonal(mesh.bounds);

  const AdjacencyData adjacency = build_adjacency(mesh);
  stats.connected_components = count_components(adjacency);
  for (const auto& [edge_key, occurrences] : adjacency.edge_occurrences) {
    (void)edge_key;
    if (occurrences.size() == 1) {
      ++stats.open_edge_count;
    } else if (occurrences.size() > 2) {
      ++stats.non_manifold_edge_count;
    }
  }
  return stats;
}

double compute_median_edge_length(const MeshModel& mesh) {
  std::vector<double> lengths;
  lengths.reserve(mesh.triangles.size() * 3);
  for (const Triangle& triangle : mesh.triangles) {
    for (const auto& edge : oriented_edges(triangle)) {
      lengths.push_back(length(mesh.vertices[edge.first] - mesh.vertices[edge.second]));
    }
  }
  if (lengths.empty()) {
    return 0.0;
  }
  const std::size_t middle = lengths.size() / 2;
  std::nth_element(lengths.begin(), lengths.begin() + middle, lengths.end());
  return lengths[middle];
}

Tolerances build_tolerances(const MeshModel& mesh, const std::string& preset) {
  (void)preset;
  const double diagonal = std::max(bounds_diagonal(mesh.bounds), 1e-6);
  const double median_edge = std::max(compute_median_edge_length(mesh), diagonal * 1e-4);

  Tolerances tolerances;
  tolerances.vertex_weld = std::max(diagonal * 1e-6, median_edge * 0.02);
  tolerances.plane_distance = std::max(diagonal * 2e-4, median_edge * 0.20);
  tolerances.plane_merge_distance = std::max(diagonal * 1e-4, median_edge * 0.12);
  tolerances.collinear_distance = std::max(diagonal * 5e-5, median_edge * 0.05);
  tolerances.normal_angle_degrees = 8.0;
  tolerances.merge_angle_degrees = 4.0;
  tolerances.parallel_angle_degrees = 3.0;
  tolerances.perpendicular_angle_degrees = 3.0;
  tolerances.min_region_confidence = 0.45;
  tolerances.min_component_area_ratio = 0.01;
  tolerances.min_component_triangles = 4;
  return tolerances;
}

void compact_vertices(MeshModel& mesh) {
  std::vector<int> remap(mesh.vertices.size(), -1);
  std::vector<Vec3> compacted;
  compacted.reserve(mesh.vertices.size());

  for (Triangle& triangle : mesh.triangles) {
    for (int& vertex_index : triangle.vertices) {
      if (remap[vertex_index] == -1) {
        remap[vertex_index] = static_cast<int>(compacted.size());
        compacted.push_back(mesh.vertices[vertex_index]);
      }
      vertex_index = remap[vertex_index];
    }
  }

  mesh.vertices = std::move(compacted);
  recompute_mesh_geometry(mesh);
}

std::size_t weld_vertices(MeshModel& mesh, double epsilon) {
  if (mesh.vertices.empty()) {
    return 0;
  }

  std::unordered_map<GridKey, int, GridKeyHash> canonical;
  std::vector<int> remap(mesh.vertices.size(), -1);
  std::vector<Vec3> welded;
  welded.reserve(mesh.vertices.size());

  auto quantize = [epsilon](double value) -> long long {
    return static_cast<long long>(std::llround(value / epsilon));
  };

  for (std::size_t index = 0; index < mesh.vertices.size(); ++index) {
    const Vec3& vertex = mesh.vertices[index];
    const GridKey key {quantize(vertex.x), quantize(vertex.y), quantize(vertex.z)};
    const auto found = canonical.find(key);
    if (found != canonical.end()) {
      remap[index] = found->second;
    } else {
      const int canonical_index = static_cast<int>(welded.size());
      welded.push_back(vertex);
      canonical.emplace(key, canonical_index);
      remap[index] = canonical_index;
    }
  }

  for (Triangle& triangle : mesh.triangles) {
    for (int& vertex_index : triangle.vertices) {
      vertex_index = remap[vertex_index];
    }
  }

  const std::size_t removed = mesh.vertices.size() - welded.size();
  mesh.vertices = std::move(welded);
  recompute_mesh_geometry(mesh);
  return removed;
}

std::size_t remove_duplicate_triangles(MeshModel& mesh) {
  std::unordered_set<FaceKey, FaceKeyHash> seen;
  std::vector<Triangle> deduplicated;
  deduplicated.reserve(mesh.triangles.size());

  for (const Triangle& triangle : mesh.triangles) {
    const FaceKey key = make_face_key(triangle.vertices);
    if (seen.insert(key).second) {
      deduplicated.push_back(triangle);
    }
  }

  const std::size_t removed = mesh.triangles.size() - deduplicated.size();
  mesh.triangles = std::move(deduplicated);
  recompute_mesh_geometry(mesh);
  return removed;
}

std::size_t remove_degenerate_triangles(MeshModel& mesh) {
  const double area_epsilon = std::max(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) *
                                           1e-12,
                                       1e-16);
  std::vector<Triangle> filtered;
  filtered.reserve(mesh.triangles.size());
  for (const Triangle& triangle : mesh.triangles) {
    if (triangle.vertices[0] == triangle.vertices[1] || triangle.vertices[1] == triangle.vertices[2] ||
        triangle.vertices[0] == triangle.vertices[2] || triangle.area <= area_epsilon) {
      continue;
    }
    filtered.push_back(triangle);
  }

  const std::size_t removed = mesh.triangles.size() - filtered.size();
  mesh.triangles = std::move(filtered);
  recompute_mesh_geometry(mesh);
  return removed;
}

std::size_t remove_non_manifold_sliver_triangles(MeshModel& mesh) {
  std::size_t removed_total = 0;

  for (int iteration = 0; iteration < 8; ++iteration) {
    const AdjacencyData adjacency = build_adjacency(mesh);
    std::unordered_set<int> triangles_to_remove;

    for (const auto& [edge_key, occurrences] : adjacency.edge_occurrences) {
      (void)edge_key;
      if (occurrences.size() <= 2) {
        continue;
      }

      std::vector<int> candidates;
      candidates.reserve(occurrences.size());
      for (const TriangleEdgeOccurrence& occurrence : occurrences) {
        candidates.push_back(occurrence.triangle);
      }

      auto should_remove_smallest = [&](int triangle_index,
                                        const std::vector<int>& active_candidates) -> bool {
        const Triangle& smallest = mesh.triangles[triangle_index];
        double largest_area = 0.0;
        bool has_parallel_larger = false;

        for (int other_index : active_candidates) {
          if (other_index == triangle_index) {
            continue;
          }
          const Triangle& other = mesh.triangles[other_index];
          largest_area = std::max(largest_area, other.area);
          if (other.area >= smallest.area * 1.25 &&
              angle_degrees_unsigned(smallest.normal, other.normal) <= 1.5) {
            has_parallel_larger = true;
          }
        }

        return has_parallel_larger ||
               (largest_area > 1e-9 && smallest.area <= largest_area * 0.25);
      };

      while (candidates.size() > 2) {
        std::sort(candidates.begin(), candidates.end(), [&mesh](int lhs, int rhs) {
          if (std::abs(mesh.triangles[lhs].area - mesh.triangles[rhs].area) > 1e-9) {
            return mesh.triangles[lhs].area < mesh.triangles[rhs].area;
          }
          return lhs < rhs;
        });

        const int smallest_triangle = candidates.front();
        if (!should_remove_smallest(smallest_triangle, candidates)) {
          break;
        }

        triangles_to_remove.insert(smallest_triangle);
        candidates.erase(candidates.begin());
      }
    }

    if (triangles_to_remove.empty()) {
      break;
    }

    std::vector<Triangle> filtered;
    filtered.reserve(mesh.triangles.size() - triangles_to_remove.size());
    for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
      if (triangles_to_remove.count(static_cast<int>(triangle_index)) == 0) {
        filtered.push_back(mesh.triangles[triangle_index]);
      }
    }

    removed_total += mesh.triangles.size() - filtered.size();
    mesh.triangles = std::move(filtered);
    recompute_mesh_geometry(mesh);
  }

  return removed_total;
}

ComponentRemoval remove_tiny_components(MeshModel& mesh, const Tolerances& tolerances) {
  const AdjacencyData adjacency = build_adjacency(mesh);
  if (mesh.triangles.empty()) {
    return {};
  }

  std::vector<int> component_id(mesh.triangles.size(), -1);
  std::vector<std::vector<int>> components;
  std::vector<double> component_areas;
  std::vector<bool> visited(mesh.triangles.size(), false);

  for (std::size_t seed = 0; seed < mesh.triangles.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }
    const int id = static_cast<int>(components.size());
    components.push_back({});
    component_areas.push_back(0.0);

    std::queue<int> frontier;
    frontier.push(static_cast<int>(seed));
    visited[seed] = true;
    while (!frontier.empty()) {
      const int current = frontier.front();
      frontier.pop();
      component_id[current] = id;
      components[id].push_back(current);
      component_areas[id] += mesh.triangles[current].area;
      for (int neighbor : adjacency.triangle_graph[current]) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          frontier.push(neighbor);
        }
      }
    }
  }

  if (components.size() <= 1) {
    return {};
  }

  const double total_area =
      std::accumulate(component_areas.begin(), component_areas.end(), 0.0);
  const int largest_component =
      static_cast<int>(std::distance(component_areas.begin(),
                                     std::max_element(component_areas.begin(), component_areas.end())));

  std::vector<bool> keep_component(components.size(), true);
  for (std::size_t index = 0; index < components.size(); ++index) {
    if (static_cast<int>(index) == largest_component) {
      continue;
    }
    const bool below_triangle_threshold =
        components[index].size() < tolerances.min_component_triangles;
    const bool below_area_threshold =
        total_area > 0.0 &&
        (component_areas[index] / total_area) < tolerances.min_component_area_ratio;
    if (below_triangle_threshold || below_area_threshold) {
      keep_component[index] = false;
    }
  }

  std::vector<Triangle> filtered;
  filtered.reserve(mesh.triangles.size());
  ComponentRemoval removal;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    if (keep_component[component_id[triangle_index]]) {
      filtered.push_back(mesh.triangles[triangle_index]);
    } else {
      ++removal.triangles_removed;
    }
  }

  for (std::size_t index = 0; index < keep_component.size(); ++index) {
    if (!keep_component[index]) {
      ++removal.components_removed;
    }
  }

  mesh.triangles = std::move(filtered);
  compact_vertices(mesh);
  return removal;
}

std::size_t orient_triangles_consistently(MeshModel& mesh) {
  const AdjacencyData adjacency = build_adjacency(mesh);
  std::vector<bool> visited(mesh.triangles.size(), false);
  std::size_t flips = 0;

  for (std::size_t seed = 0; seed < mesh.triangles.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }
    std::queue<int> frontier;
    frontier.push(static_cast<int>(seed));
    visited[seed] = true;
    while (!frontier.empty()) {
      const int current = frontier.front();
      frontier.pop();
      for (int neighbor : adjacency.triangle_graph[current]) {
        if (visited[neighbor]) {
          continue;
        }
        const auto shared = shared_edge_key(mesh.triangles[current], mesh.triangles[neighbor]);
        if (shared && edge_direction_matches(mesh.triangles[current], *shared) ==
                          edge_direction_matches(mesh.triangles[neighbor], *shared)) {
          flip_triangle(mesh.triangles[neighbor]);
          ++flips;
        }
        visited[neighbor] = true;
        frontier.push(neighbor);
      }
    }
  }

  recompute_mesh_geometry(mesh);
  return flips;
}

std::array<std::array<double, 3>, 3> covariance_matrix(const std::vector<Vec3>& points,
                                                       const Vec3& centroid) {
  std::array<std::array<double, 3>, 3> covariance {};
  for (const Vec3& point : points) {
    const Vec3 delta = point - centroid;
    covariance[0][0] += delta.x * delta.x;
    covariance[0][1] += delta.x * delta.y;
    covariance[0][2] += delta.x * delta.z;
    covariance[1][0] += delta.y * delta.x;
    covariance[1][1] += delta.y * delta.y;
    covariance[1][2] += delta.y * delta.z;
    covariance[2][0] += delta.z * delta.x;
    covariance[2][1] += delta.z * delta.y;
    covariance[2][2] += delta.z * delta.z;
  }
  return covariance;
}

std::pair<std::array<double, 3>, std::array<Vec3, 3>> jacobi_eigenvectors(
    std::array<std::array<double, 3>, 3> matrix) {
  std::array<std::array<double, 3>, 3> eigenvectors {{
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  }};

  for (int iteration = 0; iteration < 25; ++iteration) {
    int p = 0;
    int q = 1;
    double max_value = std::abs(matrix[0][1]);
    for (int row = 0; row < 3; ++row) {
      for (int column = row + 1; column < 3; ++column) {
        const double value = std::abs(matrix[row][column]);
        if (value > max_value) {
          max_value = value;
          p = row;
          q = column;
        }
      }
    }

    if (max_value <= 1e-12) {
      break;
    }

    const double theta =
        0.5 * std::atan2(2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
    const double cosine = std::cos(theta);
    const double sine = std::sin(theta);

    for (int column = 0; column < 3; ++column) {
      const double p_value = matrix[p][column];
      const double q_value = matrix[q][column];
      matrix[p][column] = cosine * p_value - sine * q_value;
      matrix[q][column] = sine * p_value + cosine * q_value;
    }
    for (int row = 0; row < 3; ++row) {
      const double p_value = matrix[row][p];
      const double q_value = matrix[row][q];
      matrix[row][p] = cosine * p_value - sine * q_value;
      matrix[row][q] = sine * p_value + cosine * q_value;
    }
    for (int row = 0; row < 3; ++row) {
      const double p_value = eigenvectors[row][p];
      const double q_value = eigenvectors[row][q];
      eigenvectors[row][p] = cosine * p_value - sine * q_value;
      eigenvectors[row][q] = sine * p_value + cosine * q_value;
    }
  }

  std::array<double, 3> eigenvalues {matrix[0][0], matrix[1][1], matrix[2][2]};
  std::array<Vec3, 3> vectors {{
      normalized(Vec3 {eigenvectors[0][0], eigenvectors[1][0], eigenvectors[2][0]}),
      normalized(Vec3 {eigenvectors[0][1], eigenvectors[1][1], eigenvectors[2][1]}),
      normalized(Vec3 {eigenvectors[0][2], eigenvectors[1][2], eigenvectors[2][2]}),
  }};
  return {eigenvalues, vectors};
}

bool solve_linear_system_3x3(std::array<std::array<double, 3>, 3> matrix, Vec3 rhs, Vec3& solution) {
  std::array<double, 3> b {rhs.x, rhs.y, rhs.z};

  for (int pivot = 0; pivot < 3; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < 3; ++row) {
      if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot])) {
        best = row;
      }
    }
    if (std::abs(matrix[best][pivot]) <= 1e-12) {
      return false;
    }
    if (best != pivot) {
      std::swap(matrix[best], matrix[pivot]);
      std::swap(b[best], b[pivot]);
    }

    const double diagonal = matrix[pivot][pivot];
    for (int column = pivot; column < 3; ++column) {
      matrix[pivot][column] /= diagonal;
    }
    b[pivot] /= diagonal;

    for (int row = 0; row < 3; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = matrix[row][pivot];
      for (int column = pivot; column < 3; ++column) {
        matrix[row][column] -= factor * matrix[pivot][column];
      }
      b[row] -= factor * b[pivot];
    }
  }

  solution = {b[0], b[1], b[2]};
  return true;
}

std::vector<int> collect_unique_vertices(const MeshModel& mesh, const std::vector<int>& triangles) {
  std::vector<char> seen(mesh.vertices.size(), false);
  std::vector<int> vertices;
  for (int triangle_index : triangles) {
    for (int vertex : mesh.triangles[triangle_index].vertices) {
      if (!seen[vertex]) {
        seen[vertex] = true;
        vertices.push_back(vertex);
      }
    }
  }
  return vertices;
}

PlaneFit fit_plane_region(const MeshModel& mesh,
                         const std::vector<int>& triangle_indices,
                         double total_area,
                         const Tolerances& tolerances) {
  const std::vector<int> vertex_indices = collect_unique_vertices(mesh, triangle_indices);
  std::vector<Vec3> points;
  points.reserve(vertex_indices.size());

  Vec3 centroid {};
  for (int vertex_index : vertex_indices) {
    points.push_back(mesh.vertices[vertex_index]);
    centroid += mesh.vertices[vertex_index];
  }

  if (points.empty()) {
    throw std::runtime_error("Attempted to fit plane to empty region");
  }

  centroid = centroid / static_cast<double>(points.size());
  const auto covariance = covariance_matrix(points, centroid);
  const auto [eigenvalues, eigenvectors] = jacobi_eigenvectors(covariance);
  int smallest = 0;
  for (int index = 1; index < 3; ++index) {
    if (eigenvalues[index] < eigenvalues[smallest]) {
      smallest = index;
    }
  }

  Vec3 normal = eigenvectors[smallest];
  Vec3 average_normal {};
  double area = 0.0;
  for (int triangle_index : triangle_indices) {
    average_normal += mesh.triangles[triangle_index].normal * mesh.triangles[triangle_index].area;
    area += mesh.triangles[triangle_index].area;
  }

  if (length_squared(normal) <= 1e-12) {
    normal = normalized(average_normal);
  }
  if (dot(normal, average_normal) < 0.0) {
    normal = normal * -1.0;
  }
  if (length_squared(normal) <= 1e-12) {
    normal = {0.0, 0.0, 1.0};
  }

  PlaneFit fit;
  fit.normal = normalized(normal);
  fit.centroid = centroid;
  fit.d = -dot(fit.normal, centroid);

  double sum_squared_error = 0.0;
  double max_error = 0.0;
  double sum_angle = 0.0;
  double weighted_angle = 0.0;
  double area_weight_sum = 0.0;
  for (const Vec3& point : points) {
    const double distance = std::abs(plane_distance(fit, point));
    sum_squared_error += distance * distance;
    max_error = std::max(max_error, distance);
  }
  for (int triangle_index : triangle_indices) {
    const double angle = angle_degrees_unsigned(mesh.triangles[triangle_index].normal, fit.normal);
    sum_angle += angle;
    weighted_angle += angle * std::max(mesh.triangles[triangle_index].area, 1e-12);
    area_weight_sum += std::max(mesh.triangles[triangle_index].area, 1e-12);
  }

  fit.rms_error = std::sqrt(sum_squared_error / static_cast<double>(points.size()));
  fit.max_error = max_error;

  const double average_angle =
      area_weight_sum > 0.0 ? weighted_angle / area_weight_sum
                            : (triangle_indices.empty() ? 90.0
                                                        : sum_angle / triangle_indices.size());
  const double residual_score = clamp(1.0 - fit.rms_error / (tolerances.plane_distance * 1.5), 0.0, 1.0);
  const double max_error_score =
      clamp(1.0 - fit.max_error / (tolerances.plane_distance * 3.0), 0.0, 1.0);
  const double angle_score =
      clamp(1.0 - average_angle / (tolerances.normal_angle_degrees * 1.25), 0.0, 1.0);
  const double support_score =
      total_area > 0.0 ? clamp((area / total_area) * 8.0, 0.0, 1.0) : 0.0;
  fit.confidence =
      0.45 * residual_score + 0.25 * max_error_score + 0.20 * angle_score + 0.10 * support_score;
  return fit;
}

PlaneRegion build_region(int id,
                        const MeshModel& mesh,
                        const std::vector<int>& triangle_indices,
                        double total_area,
                        const Tolerances& tolerances) {
  PlaneRegion region;
  region.id = id;
  region.triangle_indices = triangle_indices;
  region.vertex_indices = collect_unique_vertices(mesh, triangle_indices);
  region.fit = fit_plane_region(mesh, triangle_indices, total_area, tolerances);
  region.area = 0.0;
  for (int triangle_index : triangle_indices) {
    region.area += mesh.triangles[triangle_index].area;
  }
  region.unresolved = triangle_indices.size() < 2 || region.fit.confidence < tolerances.min_region_confidence;
  return region;
}

bool triangle_is_compatible(const MeshModel& mesh,
                            int triangle_index,
                            const PlaneFit& fit,
                            const Tolerances& tolerances) {
  const Triangle& triangle = mesh.triangles[triangle_index];
  if (triangle.area <= 1e-12) {
    return false;
  }
  if (angle_degrees_unsigned(triangle.normal, fit.normal) > tolerances.normal_angle_degrees) {
    return false;
  }
  for (int vertex_index : triangle.vertices) {
    if (std::abs(plane_distance(fit, mesh.vertices[vertex_index])) >
        tolerances.plane_distance * 1.5) {
      return false;
    }
  }
  return true;
}

std::vector<PlaneRegion> segment_planar_regions(MeshModel& mesh, const Tolerances& tolerances) {
  const AdjacencyData adjacency = build_adjacency(mesh);
  const double total_area = std::accumulate(
      mesh.triangles.begin(), mesh.triangles.end(), 0.0,
      [](double sum, const Triangle& triangle) { return sum + triangle.area; });

  std::vector<int> order(mesh.triangles.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&mesh](int lhs, int rhs) {
    return mesh.triangles[lhs].area > mesh.triangles[rhs].area;
  });

  std::vector<int> triangle_to_region(mesh.triangles.size(), -1);
  std::vector<PlaneRegion> regions;

  for (int seed : order) {
    if (triangle_to_region[seed] != -1) {
      continue;
    }
    std::vector<int> triangles {seed};
    std::queue<int> frontier;
    frontier.push(seed);
    triangle_to_region[seed] = static_cast<int>(regions.size());
    PlaneRegion working = build_region(static_cast<int>(regions.size()), mesh, triangles, total_area, tolerances);

    while (!frontier.empty()) {
      const int current = frontier.front();
      frontier.pop();
      for (int neighbor : adjacency.triangle_graph[current]) {
        if (triangle_to_region[neighbor] != -1) {
          continue;
        }
        if (!triangle_is_compatible(mesh, neighbor, working.fit, tolerances)) {
          continue;
        }
        std::vector<int> candidate_triangles = triangles;
        candidate_triangles.push_back(neighbor);
        PlaneRegion candidate =
            build_region(static_cast<int>(regions.size()), mesh, candidate_triangles, total_area, tolerances);
        if (candidate.fit.rms_error > tolerances.plane_distance * 1.5 ||
            angle_degrees_unsigned(mesh.triangles[neighbor].normal, candidate.fit.normal) >
                tolerances.normal_angle_degrees * 1.25) {
          continue;
        }
        triangles = std::move(candidate_triangles);
        working = std::move(candidate);
        triangle_to_region[neighbor] = static_cast<int>(regions.size());
        frontier.push(neighbor);
      }
    }

    regions.push_back(working);
  }

  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    mesh.triangles[triangle_index].region_id = triangle_to_region[triangle_index];
  }

  std::vector<std::set<int>> neighbor_sets(regions.size());
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const int region_id = mesh.triangles[triangle_index].region_id;
    for (int neighbor : adjacency.triangle_graph[triangle_index]) {
      const int neighbor_region = mesh.triangles[neighbor].region_id;
      if (neighbor_region >= 0 && neighbor_region != region_id) {
        neighbor_sets[region_id].insert(neighbor_region);
      }
    }
  }
  for (std::size_t index = 0; index < regions.size(); ++index) {
    regions[index].neighbor_region_ids.assign(neighbor_sets[index].begin(), neighbor_sets[index].end());
  }
  return regions;
}

double coplanar_offset(const PlaneFit& lhs, const PlaneFit& rhs) {
  const double sign = dot(lhs.normal, rhs.normal) >= 0.0 ? 1.0 : -1.0;
  return std::abs(lhs.d - rhs.d * sign);
}

bool snap_parallel(PlaneRegion& anchor, PlaneRegion& moving) {
  Vec3 target = anchor.fit.normal;
  if (dot(target, moving.fit.normal) < 0.0) {
    target = target * -1.0;
  }
  if (length_squared(target) <= 1e-12) {
    return false;
  }
  moving.fit.normal = normalized(target);
  moving.fit.d = -dot(moving.fit.normal, moving.fit.centroid);
  return true;
}

bool snap_perpendicular(PlaneRegion& anchor, PlaneRegion& moving) {
  Vec3 candidate = moving.fit.normal - anchor.fit.normal * dot(moving.fit.normal, anchor.fit.normal);
  if (length_squared(candidate) <= 1e-12) {
    return false;
  }
  candidate = normalized(candidate);
  if (dot(candidate, moving.fit.normal) < 0.0) {
    candidate = candidate * -1.0;
  }
  moving.fit.normal = candidate;
  moving.fit.d = -dot(moving.fit.normal, moving.fit.centroid);
  return true;
}

std::vector<std::pair<int, int>> collect_region_pairs(const MeshModel& mesh) {
  const AdjacencyData adjacency = build_adjacency(mesh);
  std::set<std::pair<int, int>> pairs;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const int region_id = mesh.triangles[triangle_index].region_id;
    for (int neighbor : adjacency.triangle_graph[triangle_index]) {
      const int neighbor_region = mesh.triangles[neighbor].region_id;
      if (region_id >= 0 && neighbor_region >= 0 && region_id != neighbor_region) {
        pairs.insert({std::min(region_id, neighbor_region), std::max(region_id, neighbor_region)});
      }
    }
  }
  return {pairs.begin(), pairs.end()};
}

std::pair<std::vector<PlaneRegion>, ConstraintGraph> apply_constraints(
    MeshModel& mesh,
    const std::vector<PlaneRegion>& segmented_regions,
    const Tolerances& tolerances) {
  ConstraintGraph graph;
  UnionFind merge_groups(segmented_regions.size());

  for (const auto& [a, b] : collect_region_pairs(mesh)) {
    const PlaneRegion& lhs = segmented_regions[a];
    const PlaneRegion& rhs = segmented_regions[b];
    const double angle = angle_degrees_unsigned(lhs.fit.normal, rhs.fit.normal);
    const double offset = coplanar_offset(lhs.fit, rhs.fit);
    if (angle <= tolerances.merge_angle_degrees && offset <= tolerances.plane_merge_distance) {
      merge_groups.unite(a, b);
      const double score =
          clamp((lhs.fit.confidence + rhs.fit.confidence) * 0.5, 0.0, 1.0);
      graph.constraints.push_back(
          {a, b, ConstraintType::CoplanarMerge, true, score,
           "Merged adjacent regions after coplanar snap within angle and offset tolerances"});
    }
  }

  const double total_area = std::accumulate(
      mesh.triangles.begin(), mesh.triangles.end(), 0.0,
      [](double sum, const Triangle& triangle) { return sum + triangle.area; });

  std::unordered_map<int, std::vector<int>> root_to_triangles;
  for (const PlaneRegion& region : segmented_regions) {
    const int root = merge_groups.find(region.id);
    auto& triangles = root_to_triangles[root];
    triangles.insert(triangles.end(), region.triangle_indices.begin(), region.triangle_indices.end());
  }

  std::vector<PlaneRegion> merged_regions;
  std::vector<int> sorted_roots;
  sorted_roots.reserve(root_to_triangles.size());
  for (const auto& entry : root_to_triangles) {
    sorted_roots.push_back(entry.first);
  }
  std::sort(sorted_roots.begin(), sorted_roots.end());

  for (int root : sorted_roots) {
    auto& triangles = root_to_triangles[root];
    std::sort(triangles.begin(), triangles.end());
    triangles.erase(std::unique(triangles.begin(), triangles.end()), triangles.end());
    const int new_id = static_cast<int>(merged_regions.size());
    PlaneRegion region = build_region(new_id, mesh, triangles, total_area, tolerances);
    region.unresolved = region.fit.confidence < tolerances.min_region_confidence;
    merged_regions.push_back(std::move(region));
  }

  for (PlaneRegion& region : merged_regions) {
    for (int triangle_index : region.triangle_indices) {
      mesh.triangles[triangle_index].region_id = region.id;
    }
  }

  for (const auto& [a, b] : collect_region_pairs(mesh)) {
    PlaneRegion& lhs = merged_regions[a];
    PlaneRegion& rhs = merged_regions[b];
    const double angle = angle_degrees_unsigned(lhs.fit.normal, rhs.fit.normal);
    const double offset = coplanar_offset(lhs.fit, rhs.fit);

    if (angle <= tolerances.parallel_angle_degrees && offset > tolerances.plane_merge_distance) {
      PlaneRegion* anchor = lhs.fit.confidence >= rhs.fit.confidence ? &lhs : &rhs;
      PlaneRegion* moving = anchor == &lhs ? &rhs : &lhs;
      const bool applied = std::min(lhs.fit.confidence, rhs.fit.confidence) >= 0.55 &&
                           snap_parallel(*anchor, *moving);
      graph.constraints.push_back(
          {lhs.id, rhs.id, ConstraintType::Parallel, applied,
           clamp((lhs.fit.confidence + rhs.fit.confidence) * 0.5, 0.0, 1.0),
           applied ? "Snapped lower-confidence plane normal to parallel anchor"
                   : "Detected near-parallel planes but confidence was too low to enforce"});
    } else if (std::abs(90.0 - angle) <= tolerances.perpendicular_angle_degrees) {
      PlaneRegion* anchor = lhs.fit.confidence >= rhs.fit.confidence ? &lhs : &rhs;
      PlaneRegion* moving = anchor == &lhs ? &rhs : &lhs;
      const bool applied = std::min(lhs.fit.confidence, rhs.fit.confidence) >= 0.55 &&
                           snap_perpendicular(*anchor, *moving);
      graph.constraints.push_back(
          {lhs.id, rhs.id, ConstraintType::Perpendicular, applied,
           clamp((lhs.fit.confidence + rhs.fit.confidence) * 0.5, 0.0, 1.0),
           applied ? "Snapped lower-confidence plane normal to perpendicular anchor"
                   : "Detected near-perpendicular planes but confidence was too low to enforce"});
    }
  }

  std::vector<std::set<int>> neighbor_sets(merged_regions.size());
  for (const auto& [a, b] : collect_region_pairs(mesh)) {
    neighbor_sets[a].insert(b);
    neighbor_sets[b].insert(a);
  }
  for (std::size_t index = 0; index < merged_regions.size(); ++index) {
    merged_regions[index].neighbor_region_ids.assign(neighbor_sets[index].begin(),
                                                     neighbor_sets[index].end());
  }

  std::sort(merged_regions.begin(), merged_regions.end(), [](const PlaneRegion& lhs, const PlaneRegion& rhs) {
    return lhs.id < rhs.id;
  });
  return {merged_regions, graph};
}

Vec3 snap_vertex_to_incident_planes(const Vec3& original,
                                    const std::vector<int>& incident_regions,
                                    const std::vector<PlaneRegion>& regions,
                                    const Tolerances& tolerances) {
  struct CandidatePlane {
    int region_id {};
    double distance {};
  };

  std::vector<CandidatePlane> candidates;
  candidates.reserve(incident_regions.size());
  for (int region_id : incident_regions) {
    candidates.push_back({region_id, std::abs(plane_distance(regions[region_id].fit, original))});
  }
  std::sort(candidates.begin(), candidates.end(), [&regions](const CandidatePlane& lhs,
                                                             const CandidatePlane& rhs) {
    if (std::abs(lhs.distance - rhs.distance) > 1e-9) {
      return lhs.distance < rhs.distance;
    }
    if (std::abs(regions[lhs.region_id].fit.confidence - regions[rhs.region_id].fit.confidence) > 1e-9) {
      return regions[lhs.region_id].fit.confidence > regions[rhs.region_id].fit.confidence;
    }
    if (std::abs(regions[lhs.region_id].area - regions[rhs.region_id].area) > 1e-9) {
      return regions[lhs.region_id].area > regions[rhs.region_id].area;
    }
    return lhs.region_id < rhs.region_id;
  });

  const double distinct_angle = std::max(tolerances.parallel_angle_degrees * 2.0, 5.0);
  std::vector<int> selected;
  selected.reserve(3);
  for (const CandidatePlane& candidate : candidates) {
    bool distinct = true;
    for (int selected_region : selected) {
      if (angle_degrees_unsigned(regions[selected_region].fit.normal, regions[candidate.region_id].fit.normal) <
          distinct_angle) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      selected.push_back(candidate.region_id);
      if (selected.size() == 3) {
        break;
      }
    }
  }

  if (selected.empty() && !candidates.empty()) {
    selected.push_back(candidates.front().region_id);
  }

  Vec3 snapped = original;
  if (selected.size() == 1) {
    const PlaneFit& fit = regions[selected.front()].fit;
    snapped = original - fit.normal * plane_distance(fit, original);
  } else {
    bool solved_exact = false;
    if (selected.size() >= 3) {
      const PlaneFit& a = regions[selected[0]].fit;
      const PlaneFit& b = regions[selected[1]].fit;
      const PlaneFit& c = regions[selected[2]].fit;
      std::array<std::array<double, 3>, 3> exact_matrix {{
          {a.normal.x, a.normal.y, a.normal.z},
          {b.normal.x, b.normal.y, b.normal.z},
          {c.normal.x, c.normal.y, c.normal.z},
      }};
      Vec3 exact_rhs {-a.d, -b.d, -c.d};
      solved_exact = solve_linear_system_3x3(exact_matrix, exact_rhs, snapped);
    }

    if (!solved_exact) {
      std::array<std::array<double, 3>, 3> matrix {{
          {1.0, 0.0, 0.0},
          {0.0, 1.0, 0.0},
          {0.0, 0.0, 1.0},
      }};
      Vec3 rhs = original;
      for (int region_id : selected) {
        const Vec3& n = regions[region_id].fit.normal;
        matrix[0][0] += n.x * n.x;
        matrix[0][1] += n.x * n.y;
        matrix[0][2] += n.x * n.z;
        matrix[1][0] += n.y * n.x;
        matrix[1][1] += n.y * n.y;
        matrix[1][2] += n.y * n.z;
        matrix[2][0] += n.z * n.x;
        matrix[2][1] += n.z * n.y;
        matrix[2][2] += n.z * n.z;
        rhs = rhs - n * regions[region_id].fit.d;
      }

      if (!solve_linear_system_3x3(matrix, rhs, snapped)) {
        return original;
      }
    }
  }

  const double max_snap_distance =
      std::max(std::max(tolerances.plane_merge_distance, tolerances.vertex_weld * 4.0), 1e-6);
  if (length(snapped - original) > max_snap_distance) {
    return original;
  }
  return snapped;
}

std::vector<int> simplify_loop(const std::vector<int>& loop,
                               const std::vector<Vec3>& vertices,
                               const Vec3& normal,
                               double tolerance) {
  if (loop.size() < 3) {
    return {};
  }

  std::vector<int> simplified = loop;
  bool changed = true;
  const PlaneBasis basis = basis_from_normal(normal);

  while (changed && simplified.size() >= 3) {
    changed = false;
    for (std::size_t index = 0; index < simplified.size(); ++index) {
      const int previous = simplified[(index + simplified.size() - 1) % simplified.size()];
      const int current = simplified[index];
      const int next = simplified[(index + 1) % simplified.size()];
      if (previous == current || current == next || previous == next) {
        simplified.erase(simplified.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
      const Vec2 a = project_to_basis(vertices[previous], basis);
      const Vec2 b = project_to_basis(vertices[current], basis);
      const Vec2 c = project_to_basis(vertices[next], basis);
      const double previous_length = std::hypot(b.x - a.x, b.y - a.y);
      const double next_length = std::hypot(c.x - b.x, c.y - b.y);
      const double local_tolerance =
          std::min(tolerance, std::min(previous_length, next_length) * 0.2);
      const Vec2 ab {a.x - b.x, a.y - b.y};
      const Vec2 cb {c.x - b.x, c.y - b.y};
      const double straightness =
          (ab.x * cb.x + ab.y * cb.y) / std::max(previous_length * next_length, 1e-12);
      if (local_tolerance > 1e-9 &&
          point_line_distance_2d(b, a, c) <= local_tolerance &&
          straightness <= -0.995) {
        simplified.erase(simplified.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
    }
  }

  return simplified.size() >= 3 ? simplified : std::vector<int> {};
}

std::vector<int> fallback_convex_hull_loop(const PlaneRegion& region,
                                           const std::vector<int>& original_to_snapped,
                                           const std::vector<Vec3>& vertices,
                                           const Vec3& normal,
                                           double tolerance) {
  struct ProjectedVertex {
    Vec2 point;
    int vertex_id {};
  };

  std::vector<ProjectedVertex> projected_vertices;
  projected_vertices.reserve(region.vertex_indices.size());
  std::set<int> unique_vertices;
  const PlaneBasis basis = basis_from_normal(normal);

  for (int original_vertex : region.vertex_indices) {
    const int snapped_vertex = original_to_snapped[original_vertex];
    if (snapped_vertex < 0 || !unique_vertices.insert(snapped_vertex).second) {
      continue;
    }
    projected_vertices.push_back({project_to_basis(vertices[snapped_vertex], basis), snapped_vertex});
  }

  if (projected_vertices.size() < 3) {
    return {};
  }

  std::sort(projected_vertices.begin(), projected_vertices.end(), [](const ProjectedVertex& lhs,
                                                                     const ProjectedVertex& rhs) {
    if (std::abs(lhs.point.x - rhs.point.x) > 1e-9) {
      return lhs.point.x < rhs.point.x;
    }
    if (std::abs(lhs.point.y - rhs.point.y) > 1e-9) {
      return lhs.point.y < rhs.point.y;
    }
    return lhs.vertex_id < rhs.vertex_id;
  });

  projected_vertices.erase(
      std::unique(projected_vertices.begin(), projected_vertices.end(),
                  [](const ProjectedVertex& lhs, const ProjectedVertex& rhs) {
                    return std::abs(lhs.point.x - rhs.point.x) <= 1e-9 &&
                           std::abs(lhs.point.y - rhs.point.y) <= 1e-9;
                  }),
      projected_vertices.end());

  if (projected_vertices.size() < 3) {
    return {};
  }

  std::vector<ProjectedVertex> hull;
  hull.reserve(projected_vertices.size() * 2);
  for (const ProjectedVertex& vertex : projected_vertices) {
    while (hull.size() >= 2 &&
           cross_2d(hull[hull.size() - 2].point, hull.back().point, vertex.point) <= 1e-9) {
      hull.pop_back();
    }
    hull.push_back(vertex);
  }

  const std::size_t lower_size = hull.size();
  for (std::size_t index = projected_vertices.size(); index-- > 0;) {
    const ProjectedVertex& vertex = projected_vertices[index];
    while (hull.size() > lower_size &&
           cross_2d(hull[hull.size() - 2].point, hull.back().point, vertex.point) <= 1e-9) {
      hull.pop_back();
    }
    hull.push_back(vertex);
  }

  if (!hull.empty()) {
    hull.pop_back();
  }
  if (hull.size() < 3) {
    return {};
  }

  std::vector<int> loop;
  loop.reserve(hull.size());
  for (const ProjectedVertex& vertex : hull) {
    loop.push_back(vertex.vertex_id);
  }
  return simplify_loop(loop, vertices, normal, tolerance);
}

void add_or_cancel_oriented_edge(
    std::unordered_map<DirectedEdgeKey, int, DirectedEdgeKeyHash>& oriented_edges,
    int from,
    int to) {
  if (from == to) {
    return;
  }

  const DirectedEdgeKey reverse {to, from};
  const auto reverse_it = oriented_edges.find(reverse);
  if (reverse_it != oriented_edges.end()) {
    if (--reverse_it->second <= 0) {
      oriented_edges.erase(reverse_it);
    }
    return;
  }

  ++oriented_edges[DirectedEdgeKey {from, to}];
}

std::vector<std::pair<int, int>> build_region_boundary_edges_from_triangles(
    const PlaneRegion& region,
    const MeshModel& mesh,
    const std::vector<int>& original_to_snapped) {
  std::unordered_map<DirectedEdgeKey, int, DirectedEdgeKeyHash> oriented_edges;

  for (int triangle_index : region.triangle_indices) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    std::array<int, 3> snapped_vertices {
        original_to_snapped[triangle.vertices[0]],
        original_to_snapped[triangle.vertices[1]],
        original_to_snapped[triangle.vertices[2]],
    };
    if (snapped_vertices[0] < 0 || snapped_vertices[1] < 0 || snapped_vertices[2] < 0) {
      continue;
    }
    if (snapped_vertices[0] == snapped_vertices[1] || snapped_vertices[1] == snapped_vertices[2] ||
        snapped_vertices[0] == snapped_vertices[2]) {
      continue;
    }
    if (dot(triangle.normal, region.fit.normal) < 0.0) {
      std::swap(snapped_vertices[1], snapped_vertices[2]);
    }

    add_or_cancel_oriented_edge(oriented_edges, snapped_vertices[0], snapped_vertices[1]);
    add_or_cancel_oriented_edge(oriented_edges, snapped_vertices[1], snapped_vertices[2]);
    add_or_cancel_oriented_edge(oriented_edges, snapped_vertices[2], snapped_vertices[0]);
  }

  std::vector<std::pair<int, int>> edges;
  edges.reserve(oriented_edges.size());
  for (const auto& [edge, count] : oriented_edges) {
    if (count > 0) {
      edges.push_back({edge.from, edge.to});
    }
  }
  std::sort(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.first != rhs.first) {
      return lhs.first < rhs.first;
    }
    return lhs.second < rhs.second;
  });
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

int choose_next_edge(const std::vector<std::pair<int, int>>& edges,
                     const std::vector<bool>& used,
                     const std::vector<int>& candidates,
                     int previous,
                     int current,
                     const std::vector<Vec3>& vertices,
                     const PlaneBasis& basis) {
  if (candidates.empty()) {
    return -1;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  const Vec2 previous_point = project_to_basis(vertices[previous], basis);
  const Vec2 current_point = project_to_basis(vertices[current], basis);
  const Vec2 previous_direction {current_point.x - previous_point.x, current_point.y - previous_point.y};
  const double previous_length = std::hypot(previous_direction.x, previous_direction.y);
  if (previous_length <= 1e-12) {
    return candidates.front();
  }
  const Vec2 normalized_previous {previous_direction.x / previous_length,
                                  previous_direction.y / previous_length};

  int best_index = candidates.front();
  double best_turn = std::numeric_limits<double>::max();
  for (int candidate : candidates) {
    if (used[candidate]) {
      continue;
    }
    const Vec2 next_point = project_to_basis(vertices[edges[candidate].second], basis);
    Vec2 direction {next_point.x - current_point.x, next_point.y - current_point.y};
    const double direction_length = std::hypot(direction.x, direction.y);
    if (direction_length <= 1e-12) {
      continue;
    }
    direction.x /= direction_length;
    direction.y /= direction_length;

    double turn = std::atan2(normalized_previous.x * direction.y - normalized_previous.y * direction.x,
                             normalized_previous.x * direction.x + normalized_previous.y * direction.y);
    if (turn <= 1e-9) {
      turn += 2.0 * kPi;
    }
    if (turn < best_turn) {
      best_turn = turn;
      best_index = candidate;
    }
  }
  return best_index;
}

int choose_next_neighbor(const std::vector<int>& candidates,
                         int previous,
                         int current,
                         const std::vector<Vec3>& vertices,
                         const PlaneBasis& basis) {
  if (candidates.empty()) {
    return -1;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  const Vec2 previous_point = project_to_basis(vertices[previous], basis);
  const Vec2 current_point = project_to_basis(vertices[current], basis);
  const Vec2 previous_direction {current_point.x - previous_point.x, current_point.y - previous_point.y};
  const double previous_length = std::hypot(previous_direction.x, previous_direction.y);
  if (previous_length <= 1e-12) {
    return candidates.front();
  }
  const Vec2 normalized_previous {previous_direction.x / previous_length,
                                  previous_direction.y / previous_length};

  int best_vertex = candidates.front();
  double best_turn = std::numeric_limits<double>::max();
  for (int candidate : candidates) {
    const Vec2 next_point = project_to_basis(vertices[candidate], basis);
    Vec2 direction {next_point.x - current_point.x, next_point.y - current_point.y};
    const double direction_length = std::hypot(direction.x, direction.y);
    if (direction_length <= 1e-12) {
      continue;
    }
    direction.x /= direction_length;
    direction.y /= direction_length;

    double turn = std::atan2(normalized_previous.x * direction.y - normalized_previous.y * direction.x,
                             normalized_previous.x * direction.x + normalized_previous.y * direction.y);
    if (turn <= 1e-9) {
      turn += 2.0 * kPi;
    }
    if (turn < best_turn) {
      best_turn = turn;
      best_vertex = candidate;
    }
  }
  return best_vertex;
}

std::size_t total_loop_edges(const std::vector<std::vector<int>>& loops) {
  std::size_t total = 0;
  for (const auto& loop : loops) {
    total += loop.size();
  }
  return total;
}

double polygon_area(const std::vector<int>& loop, const std::vector<Vec3>& vertices, const Vec3& normal);

double loops_face_area_estimate(const std::vector<std::vector<int>>& loops,
                                const std::vector<Vec3>& vertices,
                                const Vec3& normal) {
  if (loops.empty()) {
    return 0.0;
  }

  std::vector<double> areas;
  areas.reserve(loops.size());
  for (const auto& loop : loops) {
    if (loop.size() >= 3) {
      areas.push_back(polygon_area(loop, vertices, normal));
    }
  }
  if (areas.empty()) {
    return 0.0;
  }

  std::sort(areas.begin(), areas.end(), std::greater<double>());
  double total = areas.front();
  for (std::size_t index = 1; index < areas.size(); ++index) {
    total -= areas[index];
  }
  return std::max(total, 0.0);
}

struct LoopCandidate {
  std::vector<std::vector<int>> loops;
  std::size_t edge_coverage {};
  double area {};
  double area_error {std::numeric_limits<double>::infinity()};
  bool plausible {};
};

LoopCandidate score_loop_candidate(std::vector<std::vector<int>> loops,
                                   const std::vector<Vec3>& vertices,
                                   const Vec3& normal,
                                   double expected_area) {
  LoopCandidate candidate;
  candidate.edge_coverage = total_loop_edges(loops);
  candidate.area = loops_face_area_estimate(loops, vertices, normal);
  if (expected_area > 1e-9) {
    candidate.area_error = std::abs(candidate.area - expected_area);
    const double area_ratio = candidate.area / expected_area;
    candidate.plausible = area_ratio >= 0.45 && area_ratio <= 1.35;
  } else {
    candidate.area_error = candidate.area;
    candidate.plausible = candidate.area > 0.0;
  }
  candidate.loops = std::move(loops);
  return candidate;
}

bool is_better_loop_candidate(const LoopCandidate& candidate, const LoopCandidate& best) {
  if (candidate.loops.empty()) {
    return false;
  }
  if (best.loops.empty()) {
    return true;
  }
  if (candidate.plausible != best.plausible) {
    return candidate.plausible;
  }
  if (candidate.edge_coverage != best.edge_coverage) {
    return candidate.edge_coverage > best.edge_coverage;
  }
  if (std::abs(candidate.area_error - best.area_error) > 1e-9) {
    return candidate.area_error < best.area_error;
  }
  return candidate.loops.size() < best.loops.size();
}

std::vector<std::vector<int>> extract_loops_for_region(const std::vector<std::pair<int, int>>& edges,
                                                       const std::vector<Vec3>& vertices,
                                                       const Vec3& normal,
                                                       double tolerance) {
  std::vector<std::vector<int>> loops;
  if (edges.empty()) {
    return loops;
  }

  std::unordered_map<int, std::vector<int>> outgoing;
  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    outgoing[edges[edge_index].first].push_back(static_cast<int>(edge_index));
  }

  std::vector<bool> used(edges.size(), false);
  const PlaneBasis basis = basis_from_normal(normal);

  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    if (used[edge_index]) {
      continue;
    }
    std::vector<int> loop;
    int start = edges[edge_index].first;
    int previous = start;
    int current = edges[edge_index].second;
    used[edge_index] = true;
    loop.push_back(start);

    int safety = 0;
    while (true) {
      if (++safety > static_cast<int>(edges.size()) + 4) {
        loop.clear();
        break;
      }
      loop.push_back(current);
      if (current == start) {
        loop.pop_back();
        break;
      }
      const auto candidates_it = outgoing.find(current);
      if (candidates_it == outgoing.end()) {
        loop.clear();
        break;
      }
      const int next_edge =
          choose_next_edge(edges, used, candidates_it->second, previous, current, vertices, basis);
      if (next_edge < 0 || used[next_edge]) {
        loop.clear();
        break;
      }
      used[next_edge] = true;
      previous = current;
      current = edges[next_edge].second;
    }

    if (loop.size() >= 3) {
      std::vector<int> simplified = simplify_loop(loop, vertices, normal, tolerance);
      if (simplified.size() >= 3) {
        loops.push_back(std::move(simplified));
      }
    }
  }

  return loops;
}

std::vector<std::vector<int>> extract_loops_for_region_undirected(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<Vec3>& vertices,
    const Vec3& normal,
    double tolerance) {
  std::vector<std::vector<int>> loops;
  if (edges.empty()) {
    return loops;
  }

  std::unordered_map<int, std::vector<int>> adjacency;
  std::vector<EdgeKey> edge_keys;
  edge_keys.reserve(edges.size());
  for (const auto& [from, to] : edges) {
    adjacency[from].push_back(to);
    adjacency[to].push_back(from);
    edge_keys.push_back(make_edge_key(from, to));
  }

  std::vector<bool> used(edge_keys.size(), false);
  const PlaneBasis basis = basis_from_normal(normal);

  auto try_walk = [&](int start_index, bool reverse_start) -> std::vector<int> {
    std::vector<bool> local_used = used;
    const int start = reverse_start ? edges[start_index].second : edges[start_index].first;
    int previous = start;
    int current = reverse_start ? edges[start_index].first : edges[start_index].second;
    local_used[start_index] = true;

    std::vector<int> loop;
    loop.push_back(start);
    int safety = 0;
    while (true) {
      if (++safety > static_cast<int>(edges.size()) + 4) {
        return {};
      }
      loop.push_back(current);
      if (current == start) {
        loop.pop_back();
        used = std::move(local_used);
        return loop;
      }

      const auto adjacency_it = adjacency.find(current);
      if (adjacency_it == adjacency.end()) {
        return {};
      }

      std::vector<int> candidates;
      for (int candidate : adjacency_it->second) {
        if (candidate == current) {
          continue;
        }
        const EdgeKey key = make_edge_key(current, candidate);
        bool already_used = false;
        for (std::size_t edge_index = 0; edge_index < edge_keys.size(); ++edge_index) {
          if (edge_keys[edge_index] == key && local_used[edge_index]) {
            already_used = true;
            break;
          }
        }
        if (!already_used) {
          candidates.push_back(candidate);
        }
      }

      const int next = choose_next_neighbor(candidates, previous, current, vertices, basis);
      if (next < 0) {
        return {};
      }

      bool marked = false;
      const EdgeKey key = make_edge_key(current, next);
      for (std::size_t edge_index = 0; edge_index < edge_keys.size(); ++edge_index) {
        if (edge_keys[edge_index] == key && !local_used[edge_index]) {
          local_used[edge_index] = true;
          marked = true;
          break;
        }
      }
      if (!marked) {
        return {};
      }

      previous = current;
      current = next;
    }
  };

  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    if (used[edge_index]) {
      continue;
    }

    std::vector<int> loop = try_walk(static_cast<int>(edge_index), false);
    if (loop.size() < 3) {
      loop = try_walk(static_cast<int>(edge_index), true);
    }
    if (loop.size() < 3) {
      continue;
    }

    std::vector<int> simplified = simplify_loop(loop, vertices, normal, tolerance);
    if (simplified.size() >= 3) {
      loops.push_back(std::move(simplified));
    }
  }

  return loops;
}

double polygon_area(const std::vector<int>& loop, const std::vector<Vec3>& vertices, const Vec3& normal) {
  const PlaneBasis basis = basis_from_normal(normal);
  std::vector<Vec2> projected;
  projected.reserve(loop.size());
  for (int vertex : loop) {
    projected.push_back(project_to_basis(vertices[vertex], basis));
  }
  return std::abs(signed_area_2d(projected));
}

double signed_polygon_area(const std::vector<int>& loop,
                           const std::vector<Vec3>& vertices,
                           const Vec3& normal) {
  const PlaneBasis basis = basis_from_normal(normal);
  std::vector<Vec2> projected;
  projected.reserve(loop.size());
  for (int vertex : loop) {
    projected.push_back(project_to_basis(vertices[vertex], basis));
  }
  return signed_area_2d(projected);
}

bool point_in_polygon_2d(const Vec2& point, const std::vector<Vec2>& polygon) {
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1; index < polygon.size(); previous = index++) {
    const Vec2& a = polygon[index];
    const Vec2& b = polygon[previous];
    const bool crosses = ((a.y > point.y) != (b.y > point.y)) &&
                         (point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) + 1e-12) + a.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

void orient_loop_to_sign(std::vector<int>& loop,
                         const std::vector<Vec3>& vertices,
                         const Vec3& normal,
                         double target_sign) {
  if (loop.size() < 3) {
    return;
  }
  const double area = signed_polygon_area(loop, vertices, normal);
  if ((area < 0.0 && target_sign > 0.0) || (area > 0.0 && target_sign < 0.0)) {
    std::reverse(loop.begin(), loop.end());
  }
}

double face_total_area(const ReconstructedFace& face,
                       const std::vector<Vec3>& vertices,
                       const Vec3& normal) {
  if (face.loops.empty()) {
    return 0.0;
  }
  double total = polygon_area(face.loops.front(), vertices, normal);
  for (std::size_t index = 1; index < face.loops.size(); ++index) {
    total -= polygon_area(face.loops[index], vertices, normal);
  }
  return std::max(total, 0.0);
}

void suppress_duplicate_coplanar_faces(std::vector<ReconstructedFace>& faces,
                                       const Tolerances& tolerances) {
  std::vector<bool> keep(faces.size(), true);

  for (std::size_t lhs_index = 0; lhs_index < faces.size(); ++lhs_index) {
    if (!keep[lhs_index]) {
      continue;
    }
    for (std::size_t rhs_index = lhs_index + 1; rhs_index < faces.size(); ++rhs_index) {
      if (!keep[rhs_index]) {
        continue;
      }

      const ReconstructedFace& lhs = faces[lhs_index];
      const ReconstructedFace& rhs = faces[rhs_index];
      const double angle = angle_degrees_unsigned(lhs.fit.normal, rhs.fit.normal);
      const double offset = coplanar_offset(lhs.fit, rhs.fit);
      const double centroid_distance = length(lhs.fit.centroid - rhs.fit.centroid);
      const double area_scale = std::max(std::max(lhs.area, rhs.area), 1e-9);
      const double area_ratio = std::min(lhs.area, rhs.area) / area_scale;

      if (angle > 0.5 || offset > tolerances.plane_merge_distance * 0.25 ||
          centroid_distance > tolerances.vertex_weld * 2.0 || area_ratio < 0.95) {
        continue;
      }

      const bool keep_lhs = lhs.confidence > rhs.confidence + 1e-9 ||
                            (std::abs(lhs.confidence - rhs.confidence) <= 1e-9 &&
                             lhs.region_id <= rhs.region_id);
      if (keep_lhs) {
        keep[rhs_index] = false;
      } else {
        keep[lhs_index] = false;
        break;
      }
    }
  }

  std::vector<ReconstructedFace> filtered;
  filtered.reserve(faces.size());
  for (std::size_t index = 0; index < faces.size(); ++index) {
    if (keep[index]) {
      filtered.push_back(std::move(faces[index]));
    }
  }
  faces = std::move(filtered);
}

std::size_t face_edge_count(const ReconstructedFace& face) {
  std::size_t count = 0;
  for (const auto& loop : face.loops) {
    count += loop.size();
  }
  return count;
}

void prune_dangling_faces(std::vector<ReconstructedFace>& faces, const Tolerances& tolerances) {
  for (int iteration = 0; iteration < 4; ++iteration) {
    std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
    std::vector<std::vector<EdgeKey>> face_edges(faces.size());
    for (std::size_t face_index = 0; face_index < faces.size(); ++face_index) {
      for (const auto& loop : faces[face_index].loops) {
        for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
          const EdgeKey key = make_edge_key(loop[edge_index], loop[(edge_index + 1) % loop.size()]);
          face_edges[face_index].push_back(key);
          ++edge_use_count[key];
        }
      }
    }

    const auto mismatch_score = [&]() -> std::size_t {
      std::size_t score = 0;
      for (const auto& [edge, count] : edge_use_count) {
        (void)edge;
        if (count == 1 || count > 2) {
          ++score;
        }
      }
      return score;
    };

    const std::size_t baseline_score = mismatch_score();
    std::optional<std::size_t> best_face_index;
    std::size_t best_score = baseline_score;

    for (std::size_t face_index = 0; face_index < faces.size(); ++face_index) {
      const std::size_t total_edges = face_edge_count(faces[face_index]);
      if (total_edges == 0) {
        continue;
      }

      std::size_t unmatched_edges = 0;
      for (const EdgeKey& edge : face_edges[face_index]) {
        const int count = edge_use_count[edge];
        if (count == 1 || count > 2) {
          ++unmatched_edges;
        }
      }

      if (unmatched_edges + 1 < total_edges) {
        continue;
      }
      if (faces[face_index].area > tolerances.plane_distance * tolerances.plane_distance * 4.0) {
        continue;
      }

      for (const EdgeKey& edge : face_edges[face_index]) {
        auto it = edge_use_count.find(edge);
        if (it == edge_use_count.end()) {
          continue;
        }
        if (--it->second <= 0) {
          edge_use_count.erase(it);
        }
      }

      const std::size_t candidate_score = mismatch_score();
      if (candidate_score < best_score) {
        best_score = candidate_score;
        best_face_index = face_index;
      }

      for (const EdgeKey& edge : face_edges[face_index]) {
        ++edge_use_count[edge];
      }
    }

    if (!best_face_index.has_value()) {
      break;
    }

    faces.erase(faces.begin() + static_cast<long>(*best_face_index));
  }
}

Vec3 newell_loop_normal(const std::vector<int>& loop, const std::vector<Vec3>& vertices) {
  Vec3 normal {};
  for (std::size_t index = 0; index < loop.size(); ++index) {
    const Vec3& current = vertices[loop[index]];
    const Vec3& next = vertices[loop[(index + 1) % loop.size()]];
    normal.x += (current.y - next.y) * (current.z + next.z);
    normal.y += (current.z - next.z) * (current.x + next.x);
    normal.z += (current.x - next.x) * (current.y + next.y);
  }
  return normalized(normal);
}

void add_small_planar_gap_caps(std::vector<ReconstructedFace>& faces,
                               const std::vector<Vec3>& vertices,
                               const Tolerances& tolerances,
                               double max_cap_area) {
  if (faces.size() < 20) {
    return;
  }

  std::unordered_map<EdgeKey, std::vector<std::pair<int, int>>, EdgeKeyHash> edge_orientations;
  for (const ReconstructedFace& face : faces) {
    for (const auto& loop : face.loops) {
      for (std::size_t index = 0; index < loop.size(); ++index) {
        const int from = loop[index];
        const int to = loop[(index + 1) % loop.size()];
        edge_orientations[make_edge_key(from, to)].push_back({from, to});
      }
    }
  }

  std::unordered_map<int, std::vector<int>> adjacency;
  std::unordered_map<EdgeKey, std::pair<int, int>, EdgeKeyHash> open_orientations;
  std::vector<EdgeKey> open_edges;
  for (const auto& [edge, orientations] : edge_orientations) {
    if (orientations.size() != 1) {
      continue;
    }
    open_edges.push_back(edge);
    open_orientations[edge] = orientations.front();
    adjacency[edge.a].push_back(edge.b);
    adjacency[edge.b].push_back(edge.a);
  }

  if (open_edges.empty()) {
    return;
  }

  std::unordered_set<int> visited_vertices;
  auto estimate_component_normal = [&](const std::vector<int>& component_vertices) -> Vec3 {
    for (std::size_t i = 0; i < component_vertices.size(); ++i) {
      const Vec3& a = vertices[component_vertices[i]];
      for (std::size_t j = i + 1; j < component_vertices.size(); ++j) {
        const Vec3& b = vertices[component_vertices[j]];
        for (std::size_t k = j + 1; k < component_vertices.size(); ++k) {
          const Vec3& c = vertices[component_vertices[k]];
          const Vec3 normal = cross(b - a, c - a);
          if (length_squared(normal) > 1e-12) {
            return normalized(normal);
          }
        }
      }
    }
    return Vec3 {0.0, 0.0, 1.0};
  };

  for (const auto& [seed_vertex, neighbors] : adjacency) {
    (void)neighbors;
    if (visited_vertices.count(seed_vertex) > 0) {
      continue;
    }

    std::vector<int> component_vertices;
    std::vector<int> frontier {seed_vertex};
    visited_vertices.insert(seed_vertex);
    while (!frontier.empty()) {
      const int current = frontier.back();
      frontier.pop_back();
      component_vertices.push_back(current);
      for (int neighbor : adjacency[current]) {
        if (visited_vertices.insert(neighbor).second) {
          frontier.push_back(neighbor);
        }
      }
    }

    std::set<int> component_vertex_set(component_vertices.begin(), component_vertices.end());
    std::vector<std::pair<int, int>> component_edges;
    component_edges.reserve(open_edges.size());
    for (const EdgeKey& edge : open_edges) {
      if (component_vertex_set.count(edge.a) != 0 && component_vertex_set.count(edge.b) != 0) {
        const auto orientation_it = open_orientations.find(edge);
        if (orientation_it != open_orientations.end()) {
          component_edges.push_back(orientation_it->second);
        }
      }
    }

    if (component_edges.size() < 3 || component_edges.size() > 16) {
      continue;
    }

    const Vec3 seed_normal = estimate_component_normal(component_vertices);
    std::vector<std::vector<int>> loops;
    bool degree_two_cycle = true;
    for (int vertex_index : component_vertices) {
      if (adjacency[vertex_index].size() != 2) {
        degree_two_cycle = false;
        break;
      }
    }
    if (degree_two_cycle) {
      const int start = component_vertices.front();
      int previous = -1;
      int current = start;
      std::vector<int> loop;
      bool closed = false;
      for (int safety = 0; safety < static_cast<int>(component_vertices.size()) + 2; ++safety) {
        loop.push_back(current);
        const std::vector<int>& neighbors_for_vertex = adjacency[current];
        int next = neighbors_for_vertex.front();
        if (next == previous && neighbors_for_vertex.size() > 1) {
          next = neighbors_for_vertex[1];
        }
        previous = current;
        current = next;
        if (current == start) {
          closed = true;
          break;
        }
      }
      if (closed && loop.size() >= 3) {
        loops.push_back(std::move(loop));
      }
    }
    if (loops.empty()) {
      const double gap_loop_tolerance = std::max(tolerances.vertex_weld * 0.1, 1e-6);
      loops = extract_loops_for_region_undirected(component_edges, vertices, seed_normal,
                                                  gap_loop_tolerance);
    }
    if (loops.empty()) {
      continue;
    }

    Vec3 normal = seed_normal;
    for (const auto& loop : loops) {
      const Vec3 candidate = newell_loop_normal(loop, vertices);
      if (length_squared(candidate) > 1e-12) {
        normal = candidate;
        break;
      }
    }
    if (length_squared(normal) <= 1e-12) {
      continue;
    }

    std::sort(loops.begin(), loops.end(), [&vertices, &normal](const auto& lhs, const auto& rhs) {
      return polygon_area(lhs, vertices, normal) > polygon_area(rhs, vertices, normal);
    });

    Vec3 centroid {};
    for (int vertex_index : component_vertex_set) {
      centroid += vertices[vertex_index];
    }
    centroid = centroid / static_cast<double>(std::max<std::size_t>(component_vertex_set.size(), 1));

    double max_distance = 0.0;
    double sum_squared = 0.0;
    for (int vertex_index : component_vertex_set) {
      const double distance = std::abs(dot(normal, vertices[vertex_index] - centroid));
      max_distance = std::max(max_distance, distance);
      sum_squared += distance * distance;
    }

    ReconstructedFace face;
    face.region_id = -1;
    face.fit.normal = normal;
    face.fit.centroid = centroid;
    face.fit.d = -dot(face.fit.normal, face.fit.centroid);
    face.fit.max_error = max_distance;
    face.fit.rms_error = std::sqrt(sum_squared /
                                   static_cast<double>(std::max<std::size_t>(component_vertex_set.size(), 1)));
    face.fit.confidence = 0.55;
    face.confidence = face.fit.confidence;
    face.loops = std::move(loops);
    orient_loop_to_sign(face.loops.front(), vertices, face.fit.normal, 1.0);
    for (std::size_t loop_index = 1; loop_index < face.loops.size(); ++loop_index) {
      orient_loop_to_sign(face.loops[loop_index], vertices, face.fit.normal, -1.0);
    }
    face.area = face_total_area(face, vertices, face.fit.normal);
    if (face.area <= 1e-9 || face.area > max_cap_area) {
      continue;
    }

    if (max_distance > tolerances.plane_distance * 0.25) {
      if (face.loops.size() != 1 || face.loops.front().size() < 4 || face.loops.front().size() > 6 ||
          max_distance > tolerances.plane_distance * 1.25) {
        continue;
      }

      const std::vector<int>& loop = face.loops.front();
      for (std::size_t index = 1; index + 1 < loop.size(); ++index) {
        ReconstructedFace triangle_face;
        triangle_face.region_id = -1;
        triangle_face.confidence = 0.45;
        triangle_face.loops.push_back({loop[0], loop[index], loop[index + 1]});
        Vec3 triangle_normal = newell_loop_normal(triangle_face.loops.front(), vertices);
        if (length_squared(triangle_normal) <= 1e-12) {
          continue;
        }
        if (dot(triangle_normal, face.fit.normal) < 0.0) {
          std::reverse(triangle_face.loops.front().begin(), triangle_face.loops.front().end());
          triangle_normal = triangle_normal * -1.0;
        }
        Vec3 triangle_centroid {};
        for (int vertex_index : triangle_face.loops.front()) {
          triangle_centroid += vertices[vertex_index];
        }
        triangle_centroid = triangle_centroid / 3.0;
        triangle_face.fit.normal = triangle_normal;
        triangle_face.fit.centroid = triangle_centroid;
        triangle_face.fit.d = -dot(triangle_face.fit.normal, triangle_face.fit.centroid);
        triangle_face.fit.max_error = 0.0;
        triangle_face.fit.rms_error = 0.0;
        triangle_face.fit.confidence = triangle_face.confidence;
        triangle_face.area = face_total_area(triangle_face, vertices, triangle_face.fit.normal);
        if (triangle_face.area > 1e-9) {
          faces.push_back(std::move(triangle_face));
        }
      }
      continue;
    }

    faces.push_back(std::move(face));
  }
}

std::vector<ReconstructedFace> build_faces_from_region_loops(const PlaneRegion& region,
                                                             const std::vector<std::vector<int>>& loops,
                                                             const std::vector<Vec3>& vertices) {
  struct ProjectedLoop {
    std::size_t index {};
    double area {};
    std::vector<Vec2> projected;
    Vec2 sample_point {};
  };

  std::vector<ProjectedLoop> projected_loops;
  projected_loops.reserve(loops.size());
  const PlaneBasis basis = basis_from_normal(region.fit.normal);

  for (std::size_t loop_index = 0; loop_index < loops.size(); ++loop_index) {
    if (loops[loop_index].size() < 3) {
      continue;
    }
    std::vector<Vec2> projected;
    projected.reserve(loops[loop_index].size());
    Vec2 sample_point {};
    for (int vertex_index : loops[loop_index]) {
      const Vec2 point = project_to_basis(vertices[vertex_index], basis);
      projected.push_back(point);
      sample_point.x += point.x;
      sample_point.y += point.y;
    }
    sample_point.x /= static_cast<double>(projected.size());
    sample_point.y /= static_cast<double>(projected.size());
    projected_loops.push_back(
        {loop_index, std::abs(signed_area_2d(projected)), std::move(projected), sample_point});
  }

  std::sort(projected_loops.begin(), projected_loops.end(),
            [](const ProjectedLoop& lhs, const ProjectedLoop& rhs) {
              if (std::abs(lhs.area - rhs.area) > 1e-9) {
                return lhs.area > rhs.area;
              }
              return lhs.index < rhs.index;
            });

  std::vector<int> parent(loops.size(), -1);
  std::vector<int> depth(loops.size(), 0);
  for (std::size_t candidate_index = 0; candidate_index < projected_loops.size(); ++candidate_index) {
    const ProjectedLoop& candidate = projected_loops[candidate_index];
    for (std::size_t container_index = 0; container_index < candidate_index; ++container_index) {
      const ProjectedLoop& container = projected_loops[container_index];
      if (point_in_polygon_2d(candidate.sample_point, container.projected)) {
        parent[candidate.index] = static_cast<int>(container.index);
        depth[candidate.index] = depth[container.index] + 1;
        break;
      }
    }
  }

  std::unordered_map<int, std::vector<std::size_t>> outer_to_holes;
  std::vector<std::size_t> outer_indices;
  for (const ProjectedLoop& projected_loop : projected_loops) {
    const std::size_t loop_index = projected_loop.index;
    if (depth[loop_index] % 2 == 0) {
      outer_indices.push_back(loop_index);
      continue;
    }

    int outer_index = parent[loop_index];
    while (outer_index >= 0 && depth[static_cast<std::size_t>(outer_index)] % 2 == 1) {
      outer_index = parent[static_cast<std::size_t>(outer_index)];
    }
    if (outer_index >= 0) {
      outer_to_holes[outer_index].push_back(loop_index);
    }
  }

  std::vector<ReconstructedFace> faces;
  faces.reserve(outer_indices.size());
  for (std::size_t outer_index : outer_indices) {
    ReconstructedFace face;
    face.region_id = region.id;
    face.fit = region.fit;
    face.confidence = region.fit.confidence;
    face.loops.push_back(loops[outer_index]);
    orient_loop_to_sign(face.loops.front(), vertices, region.fit.normal, 1.0);

    auto holes_it = outer_to_holes.find(static_cast<int>(outer_index));
    if (holes_it != outer_to_holes.end()) {
      std::sort(holes_it->second.begin(), holes_it->second.end());
      for (std::size_t hole_index : holes_it->second) {
        std::vector<int> hole = loops[hole_index];
        orient_loop_to_sign(hole, vertices, region.fit.normal, -1.0);
        face.loops.push_back(std::move(hole));
      }
    }

    face.area = face_total_area(face, vertices, region.fit.normal);
    faces.push_back(std::move(face));
  }

  return faces;
}

template <typename Fn>
void for_each_face_edge(const ReconstructedFace& face, Fn&& fn) {
  for (const auto& loop : face.loops) {
    for (std::size_t index = 0; index < loop.size(); ++index) {
      fn(loop[index], loop[(index + 1) % loop.size()]);
    }
  }
}

std::vector<int> sanitize_loop_indices(std::vector<int> loop) {
  if (loop.size() < 3) {
    return {};
  }
  if (loop.size() >= 2 && loop.front() == loop.back()) {
    loop.pop_back();
  }

  bool changed = true;
  while (changed && loop.size() >= 3) {
    changed = false;

    std::vector<int> compacted;
    compacted.reserve(loop.size());
    for (int vertex : loop) {
      if (!compacted.empty() && compacted.back() == vertex) {
        changed = true;
        continue;
      }
      compacted.push_back(vertex);
    }
    if (compacted.size() >= 2 && compacted.front() == compacted.back()) {
      compacted.pop_back();
      changed = true;
    }
    loop = std::move(compacted);
    if (loop.size() < 3) {
      break;
    }

    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int previous = loop[(index + loop.size() - 1) % loop.size()];
      const int next = loop[(index + 1) % loop.size()];
      if (previous == next) {
        loop.erase(loop.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
    }
  }

  return loop.size() >= 3 ? loop : std::vector<int> {};
}

std::vector<int> refine_loop_with_shared_vertices(const std::vector<int>& loop,
                                                  const std::vector<Vec3>& vertices,
                                                  const std::vector<int>& candidate_vertices,
                                                  double tolerance,
                                                  std::size_t& insertion_count) {
  if (loop.size() < 3) {
    return loop;
  }

  std::vector<int> refined;
  refined.reserve(loop.size() * 2);
  const double endpoint_epsilon = std::max(tolerance * 0.5, 1e-8);

  for (std::size_t index = 0; index < loop.size(); ++index) {
    const int from = loop[index];
    const int to = loop[(index + 1) % loop.size()];
    refined.push_back(from);

    std::vector<std::pair<double, int>> inserts;
    inserts.reserve(4);
    for (int candidate : candidate_vertices) {
      if (candidate == from || candidate == to) {
        continue;
      }
      double t = 0.0;
      const double distance =
          point_segment_distance_3d(vertices[candidate], vertices[from], vertices[to], t);
      if (distance > tolerance || t <= endpoint_epsilon || t >= 1.0 - endpoint_epsilon) {
        continue;
      }
      inserts.push_back({t, candidate});
    }

    std::sort(inserts.begin(), inserts.end(), [](const auto& lhs, const auto& rhs) {
      if (std::abs(lhs.first - rhs.first) > 1e-9) {
        return lhs.first < rhs.first;
      }
      return lhs.second < rhs.second;
    });

    int last_added = from;
    double last_t = -1.0;
    for (const auto& [t, candidate] : inserts) {
      if (candidate == last_added || std::abs(t - last_t) <= 1e-9) {
        continue;
      }
      refined.push_back(candidate);
      ++insertion_count;
      last_added = candidate;
      last_t = t;
    }
  }

  std::vector<int> compacted;
  compacted.reserve(refined.size());
  for (int vertex : refined) {
    if (!compacted.empty() && compacted.back() == vertex) {
      continue;
    }
    compacted.push_back(vertex);
  }
  if (compacted.size() >= 2 && compacted.front() == compacted.back()) {
    compacted.pop_back();
  }

  compacted = sanitize_loop_indices(std::move(compacted));
  return compacted.size() >= 3 ? compacted : loop;
}

void refine_face_loops_with_shared_vertices(std::vector<ReconstructedFace>& faces,
                                            const std::vector<Vec3>& vertices,
                                            double tolerance,
                                            std::size_t& insertion_count) {
  std::set<int> boundary_vertex_set;
  for (const ReconstructedFace& face : faces) {
    for (const auto& loop : face.loops) {
      boundary_vertex_set.insert(loop.begin(), loop.end());
    }
  }
  const std::vector<int> candidate_vertices {boundary_vertex_set.begin(), boundary_vertex_set.end()};

  for (ReconstructedFace& face : faces) {
    for (std::vector<int>& loop : face.loops) {
      loop = refine_loop_with_shared_vertices(loop, vertices, candidate_vertices, tolerance,
                                              insertion_count);
    }
  }
}

void sanitize_face_loops(std::vector<ReconstructedFace>& faces) {
  for (ReconstructedFace& face : faces) {
    std::vector<std::vector<int>> sanitized;
    sanitized.reserve(face.loops.size());
    for (std::vector<int>& loop : face.loops) {
      std::vector<int> cleaned = sanitize_loop_indices(std::move(loop));
      if (cleaned.size() >= 3) {
        sanitized.push_back(std::move(cleaned));
      }
    }
    face.loops = std::move(sanitized);
  }
}

double signed_loop_volume(const std::vector<int>& loop, const std::vector<Vec3>& vertices) {
  if (loop.size() < 3) {
    return 0.0;
  }
  double volume = 0.0;
  const Vec3& a = vertices[loop[0]];
  for (std::size_t index = 1; index + 1 < loop.size(); ++index) {
    const Vec3& b = vertices[loop[index]];
    const Vec3& c = vertices[loop[index + 1]];
    volume += dot(a, cross(b, c)) / 6.0;
  }
  return volume;
}

double signed_polyhedron_volume(const std::vector<ReconstructedFace>& faces,
                                const std::vector<Vec3>& vertices) {
  double volume = 0.0;
  for (const ReconstructedFace& face : faces) {
    if (face.loops.empty()) {
      continue;
    }
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const double contribution = signed_loop_volume(face.loops[loop_index], vertices);
      volume += contribution;
    }
  }
  return volume;
}

void append_loop_candidate(std::vector<LoopCandidate>& candidates,
                           std::vector<std::vector<int>> loops,
                           const std::vector<Vec3>& vertices,
                           const Vec3& normal,
                           double expected_area) {
  LoopCandidate candidate =
      score_loop_candidate(std::move(loops), vertices, normal, expected_area);
  if (!candidate.loops.empty()) {
    candidates.push_back(std::move(candidate));
  }
}

std::vector<LoopCandidate> collect_region_loop_candidates(
    const PlaneRegion& region,
    const std::unordered_map<int, std::vector<std::pair<int, int>>>& boundary_edges,
    const MeshModel& mesh,
    const std::vector<int>& original_to_snapped,
    const std::vector<Vec3>& vertices,
    const Tolerances& tolerances) {
  std::vector<LoopCandidate> candidates;

  const auto topological_edges_it = boundary_edges.find(region.id);
  if (topological_edges_it != boundary_edges.end()) {
    append_loop_candidate(candidates,
                          extract_loops_for_region(topological_edges_it->second, vertices,
                                                   region.fit.normal,
                                                   tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
    append_loop_candidate(candidates,
                          extract_loops_for_region_undirected(topological_edges_it->second, vertices,
                                                              region.fit.normal,
                                                              tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
  }

  const std::vector<std::pair<int, int>> triangle_boundary_edges =
      build_region_boundary_edges_from_triangles(region, mesh, original_to_snapped);
  if (!triangle_boundary_edges.empty()) {
    append_loop_candidate(candidates,
                          extract_loops_for_region(triangle_boundary_edges, vertices,
                                                   region.fit.normal,
                                                   tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
    append_loop_candidate(candidates,
                          extract_loops_for_region_undirected(triangle_boundary_edges, vertices,
                                                              region.fit.normal,
                                                              tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
  }

  if (candidates.empty()) {
    std::vector<int> hull =
        fallback_convex_hull_loop(region, original_to_snapped, vertices, region.fit.normal,
                                  tolerances.collinear_distance);
    const double hull_area =
        hull.size() >= 3 ? polygon_area(hull, vertices, region.fit.normal) : 0.0;
    const double hull_ratio = region.area > 1e-9 ? hull_area / region.area : 0.0;
    if (hull.size() >= 3 && hull_ratio >= 0.7 && hull_ratio <= 1.15) {
      append_loop_candidate(candidates, {std::move(hull)}, vertices, region.fit.normal, region.area);
    }
  }

  return candidates;
}

std::size_t choose_initial_candidate_index(const std::vector<LoopCandidate>& candidates) {
  if (candidates.empty()) {
    return std::numeric_limits<std::size_t>::max();
  }

  std::size_t best_index = 0;
  LoopCandidate best_candidate = candidates.front();
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    if (is_better_loop_candidate(candidates[index], best_candidate)) {
      best_candidate = candidates[index];
      best_index = index;
    }
  }
  return best_index;
}

void build_faces_from_selected_candidates(const std::vector<PlaneRegion>& regions,
                                          const std::vector<std::vector<LoopCandidate>>& candidates,
                                          const std::vector<std::size_t>& selected_indices,
                                          const std::vector<Vec3>& vertices,
                                          ReconstructionResult& result) {
  result.faces.clear();
  result.omitted_region_ids.clear();
  result.edge_split_insertions = 0;

  for (std::size_t region_index = 0; region_index < regions.size(); ++region_index) {
    const PlaneRegion& region = regions[region_index];
    if (region_index >= selected_indices.size() ||
        selected_indices[region_index] >= candidates[region_index].size()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }

    std::vector<ReconstructedFace> faces =
        build_faces_from_region_loops(region, candidates[region_index][selected_indices[region_index]].loops,
                                      vertices);
    if (faces.empty()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }

    bool emitted_face = false;
    for (ReconstructedFace& face : faces) {
      if (!face.loops.empty() && face.loops.front().size() >= 3) {
        result.faces.push_back(std::move(face));
        emitted_face = true;
      }
    }
    if (!emitted_face) {
      result.omitted_region_ids.push_back(region.id);
    }
  }
}

void finalize_shell_reconstruction(ReconstructionResult& result,
                                   const MeshModel& mesh,
                                   const Tolerances& tolerances,
                                   double solid_threshold) {
  result.failure_reasons.clear();
  result.problematic_regions.clear();
  result.open_edge_count = 0;
  result.non_manifold_edge_count = 0;
  result.shell_gap_score = 1.0;
  result.confidence = 0.0;

  if (result.faces.empty()) {
    result.failure_reasons.push_back("Unable to reconstruct any planar shell faces");
    result.outcome = ReconstructionOutcome::AnalysisOnly;
    return;
  }

  refine_face_loops_with_shared_vertices(result.faces, result.vertices, tolerances.collinear_distance,
                                         result.edge_split_insertions);
  for (ReconstructedFace& face : result.faces) {
    if (face.loops.empty()) {
      continue;
    }
    orient_loop_to_sign(face.loops.front(), result.vertices, face.fit.normal, 1.0);
    for (std::size_t loop_index = 1; loop_index < face.loops.size(); ++loop_index) {
      orient_loop_to_sign(face.loops[loop_index], result.vertices, face.fit.normal, -1.0);
    }
    face.area = face_total_area(face, result.vertices, face.fit.normal);
  }
  suppress_duplicate_coplanar_faces(result.faces, tolerances);
  prune_dangling_faces(result.faces, tolerances);
  const double shell_cap_area_limit =
      std::min(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.015,
               tolerances.plane_distance * tolerances.plane_distance * 40.0);
  add_small_planar_gap_caps(result.faces, result.vertices, tolerances, shell_cap_area_limit);
  sanitize_face_loops(result.faces);

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
  std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHash> edge_regions;
  for (const ReconstructedFace& face : result.faces) {
    for_each_face_edge(face, [&](int from, int to) {
      const EdgeKey key = make_edge_key(from, to);
      ++edge_use_count[key];
      edge_regions[key].push_back(face.region_id);
    });
  }

  std::size_t open_shell_edges = 0;
  std::size_t non_manifold_shell_edges = 0;
  std::unordered_map<int, std::size_t> problematic_region_counts;
  for (const auto& [edge_key, count] : edge_use_count) {
    if (count == 1) {
      ++open_shell_edges;
      for (int region_id : edge_regions[edge_key]) {
        ++problematic_region_counts[region_id];
      }
    } else if (count != 2) {
      ++non_manifold_shell_edges;
      for (int region_id : edge_regions[edge_key]) {
        ++problematic_region_counts[region_id];
      }
    }
  }

  result.open_edge_count = open_shell_edges;
  result.non_manifold_edge_count = non_manifold_shell_edges;
  result.problematic_regions.reserve(problematic_region_counts.size());
  for (const auto& [region_id, edge_count] : problematic_region_counts) {
    result.problematic_regions.push_back({region_id, edge_count});
  }
  std::sort(result.problematic_regions.begin(), result.problematic_regions.end(),
            [](const RegionTopologyIssue& lhs, const RegionTopologyIssue& rhs) {
              if (lhs.edge_count != rhs.edge_count) {
                return lhs.edge_count > rhs.edge_count;
              }
              return lhs.region_id < rhs.region_id;
            });

  result.shell_gap_score =
      edge_use_count.empty()
          ? 1.0
          : static_cast<double>(open_shell_edges + non_manifold_shell_edges) /
                static_cast<double>(edge_use_count.size());

  for (const ReconstructedFace& face : result.faces) {
    result.confidence += face.confidence;
  }
  result.confidence /= static_cast<double>(result.faces.size());

  if (open_shell_edges > 0 || non_manifold_shell_edges > 0) {
    result.failure_reasons.push_back("Reconstructed shell is open or topologically inconsistent");
  }
  if (result.faces.size() < 4) {
    result.failure_reasons.push_back("Too few reconstructed faces for a closed solid");
  }

  const double volume = std::abs(signed_polyhedron_volume(result.faces, result.vertices));
  const double min_volume = std::pow(std::max(bounds_diagonal(mesh.bounds), 1e-6), 3.0) * 1e-6;
  if (volume <= min_volume) {
    result.failure_reasons.push_back("Reconstructed shell volume is too small or inconsistent");
  }

  if (open_shell_edges == 0 && non_manifold_shell_edges == 0 && result.faces.size() >= 4 &&
      volume > min_volume && result.confidence >= solid_threshold) {
    result.outcome = ReconstructionOutcome::SolidCreated;
  } else if (!result.faces.empty()) {
    result.outcome = ReconstructionOutcome::ShellOnly;
    if (result.confidence < solid_threshold) {
      result.failure_reasons.push_back("Solid export was gated by low reconstruction confidence");
    }
  } else {
    result.outcome = ReconstructionOutcome::AnalysisOnly;
  }
}

bool is_better_reconstruction_result(const ReconstructionResult& candidate,
                                     const ReconstructionResult& current) {
  const auto outcome_rank = [](ReconstructionOutcome outcome) {
    switch (outcome) {
      case ReconstructionOutcome::SolidCreated:
        return 3;
      case ReconstructionOutcome::ShellOnly:
        return 2;
      case ReconstructionOutcome::AnalysisOnly:
      default:
        return 1;
    }
  };

  if (outcome_rank(candidate.outcome) != outcome_rank(current.outcome)) {
    return outcome_rank(candidate.outcome) > outcome_rank(current.outcome);
  }
  if (candidate.non_manifold_edge_count != current.non_manifold_edge_count) {
    return candidate.non_manifold_edge_count < current.non_manifold_edge_count;
  }
  if (candidate.open_edge_count != current.open_edge_count) {
    return candidate.open_edge_count < current.open_edge_count;
  }
  if (candidate.omitted_region_ids.size() != current.omitted_region_ids.size()) {
    return candidate.omitted_region_ids.size() < current.omitted_region_ids.size();
  }
  if (std::abs(candidate.shell_gap_score - current.shell_gap_score) > 1e-9) {
    return candidate.shell_gap_score < current.shell_gap_score;
  }
  if (std::abs(candidate.confidence - current.confidence) > 1e-9) {
    return candidate.confidence > current.confidence;
  }
  return candidate.faces.size() > current.faces.size();
}

std::vector<std::size_t> collect_problem_region_indices(
    const ReconstructionResult& result,
    const std::unordered_map<int, std::size_t>& region_id_to_index,
    std::size_t max_regions) {
  std::vector<std::size_t> indices;
  std::unordered_set<std::size_t> seen;

  for (const RegionTopologyIssue& issue : result.problematic_regions) {
    const auto it = region_id_to_index.find(issue.region_id);
    if (it != region_id_to_index.end() && seen.insert(it->second).second) {
      indices.push_back(it->second);
      if (indices.size() >= max_regions) {
        return indices;
      }
    }
  }
  for (int region_id : result.omitted_region_ids) {
    const auto it = region_id_to_index.find(region_id);
    if (it != region_id_to_index.end() && seen.insert(it->second).second) {
      indices.push_back(it->second);
      if (indices.size() >= max_regions) {
        return indices;
      }
    }
  }
  return indices;
}

void optimize_region_loop_choices(ReconstructionResult& result,
                                  const MeshModel& mesh,
                                  const std::vector<PlaneRegion>& regions,
                                  const std::vector<std::vector<LoopCandidate>>& candidates,
                                  std::vector<std::size_t>& selected_indices,
                                  const Tolerances& tolerances,
                                  double solid_threshold) {
  std::unordered_map<int, std::size_t> region_id_to_index;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    region_id_to_index[regions[index].id] = index;
  }

  for (int pass = 0; pass < 2; ++pass) {
    bool improved = false;
    const std::vector<std::size_t> priority =
        collect_problem_region_indices(result, region_id_to_index, 24);
    for (std::size_t region_index : priority) {
      if (region_index >= candidates.size() || candidates[region_index].size() <= 1) {
        continue;
      }

      ReconstructionResult best_result = result;
      std::size_t best_choice = selected_indices[region_index];
      for (std::size_t candidate_index = 0; candidate_index < candidates[region_index].size();
           ++candidate_index) {
        if (candidate_index == best_choice) {
          continue;
        }

        std::vector<std::size_t> trial_indices = selected_indices;
        trial_indices[region_index] = candidate_index;

        ReconstructionResult trial;
        trial.vertices = result.vertices;
        build_faces_from_selected_candidates(regions, candidates, trial_indices, trial.vertices, trial);
        finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
        if (is_better_reconstruction_result(trial, best_result)) {
          best_result = std::move(trial);
          best_choice = candidate_index;
        }
      }

      if (best_choice != selected_indices[region_index]) {
        selected_indices[region_index] = best_choice;
        result = std::move(best_result);
        improved = true;
      }
    }

    if (!improved || result.outcome == ReconstructionOutcome::SolidCreated) {
      break;
    }
  }
}

void resolve_tiny_conflict_faces(ReconstructionResult& result,
                                 const MeshModel& mesh,
                                 const Tolerances& tolerances,
                                 double solid_threshold) {
  if (result.non_manifold_edge_count == 0 || result.faces.empty()) {
    return;
  }

  const double shell_cap_area_limit =
      std::min(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.015,
               tolerances.plane_distance * tolerances.plane_distance * 40.0);

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
  std::vector<std::unordered_set<EdgeKey, EdgeKeyHash>> face_edges(result.faces.size());
  for (std::size_t face_index = 0; face_index < result.faces.size(); ++face_index) {
    for_each_face_edge(result.faces[face_index], [&](int from, int to) {
      const EdgeKey key = make_edge_key(from, to);
      ++edge_use_count[key];
      face_edges[face_index].insert(key);
    });
  }

  std::vector<std::size_t> candidate_faces;
  candidate_faces.reserve(result.faces.size());
  for (std::size_t face_index = 0; face_index < result.faces.size(); ++face_index) {
    const ReconstructedFace& face = result.faces[face_index];
    if (face.region_id < 0 || face.area > shell_cap_area_limit || face.loops.size() != 1 ||
        face.loops.front().size() > 8) {
      continue;
    }
    bool touches_non_manifold = false;
    for (const EdgeKey& edge : face_edges[face_index]) {
      const auto edge_it = edge_use_count.find(edge);
      if (edge_it != edge_use_count.end() && edge_it->second > 2) {
        touches_non_manifold = true;
        break;
      }
    }
    if (touches_non_manifold) {
      candidate_faces.push_back(face_index);
    }
  }

  if (candidate_faces.empty() || candidate_faces.size() > 6) {
    return;
  }

  ReconstructionResult best_result = result;
  const std::size_t subset_limit = std::min<std::size_t>(3, candidate_faces.size());
  const std::size_t total_masks = std::size_t {1} << candidate_faces.size();
  for (std::size_t mask = 1; mask < total_masks; ++mask) {
    if (static_cast<std::size_t>(__builtin_popcountll(static_cast<unsigned long long>(mask))) >
        subset_limit) {
      continue;
    }

    ReconstructionResult trial = result;
    std::vector<ReconstructedFace> kept_faces;
    kept_faces.reserve(result.faces.size());
    std::unordered_set<int> removed_region_ids;
    for (std::size_t face_index = 0; face_index < result.faces.size(); ++face_index) {
      bool remove_face = false;
      for (std::size_t candidate_index = 0; candidate_index < candidate_faces.size(); ++candidate_index) {
        if (((mask >> candidate_index) & 1U) != 0U && candidate_faces[candidate_index] == face_index) {
          remove_face = true;
          removed_region_ids.insert(result.faces[face_index].region_id);
          break;
        }
      }
      if (!remove_face) {
        kept_faces.push_back(result.faces[face_index]);
      }
    }
    trial.faces = std::move(kept_faces);
    for (int region_id : removed_region_ids) {
      if (std::find(trial.omitted_region_ids.begin(), trial.omitted_region_ids.end(), region_id) ==
          trial.omitted_region_ids.end()) {
        trial.omitted_region_ids.push_back(region_id);
      }
    }
    finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
    if (is_better_reconstruction_result(trial, best_result)) {
      best_result = std::move(trial);
    }
  }

  if (is_better_reconstruction_result(best_result, result)) {
    result = std::move(best_result);
  }
}

ReconstructionResult reconstruct_shell(const MeshModel& mesh,
                                       const std::vector<PlaneRegion>& regions,
                                       const Tolerances& tolerances,
                                       double solid_threshold) {
  ReconstructionResult result;
  if (regions.empty()) {
    result.failure_reasons.push_back("No planar regions were detected");
    return result;
  }

  const AdjacencyData adjacency = build_adjacency(mesh);
  std::vector<std::vector<int>> incident_regions(mesh.vertices.size());
  for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index) {
    std::set<int> ids;
    for (int triangle_index : adjacency.vertex_to_triangles[vertex_index]) {
      if (mesh.triangles[triangle_index].region_id >= 0) {
        ids.insert(mesh.triangles[triangle_index].region_id);
      }
    }
    incident_regions[vertex_index] = {ids.begin(), ids.end()};
  }

  std::vector<int> original_to_snapped(mesh.vertices.size(), -1);
  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> snapped_lookup;
  const double snap_weld_tolerance = std::max(tolerances.vertex_weld * 0.10, 1e-7);
  auto quantize = [snap_weld_tolerance](double value) -> long long {
    return static_cast<long long>(std::llround(value / snap_weld_tolerance));
  };

  for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index) {
    Vec3 snapped = mesh.vertices[vertex_index];
    if (!incident_regions[vertex_index].empty()) {
      snapped = snap_vertex_to_incident_planes(mesh.vertices[vertex_index], incident_regions[vertex_index],
                                               regions, tolerances);
    }
    const GridKey key {quantize(snapped.x), quantize(snapped.y), quantize(snapped.z)};
    const std::optional<int> found =
        find_nearby_grid_vertex(snapped_lookup, result.vertices, key, snapped, snap_weld_tolerance);
    if (found.has_value()) {
      original_to_snapped[vertex_index] = *found;
    } else {
      const int snapped_index = static_cast<int>(result.vertices.size());
      result.vertices.push_back(snapped);
      snapped_lookup[key].push_back(snapped_index);
      original_to_snapped[vertex_index] = snapped_index;
    }
  }

  std::unordered_map<int, std::vector<std::pair<int, int>>> boundary_edges;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    for (int edge_index = 0; edge_index < 3; ++edge_index) {
      const int original_from = triangle.vertices[edge_index];
      const int original_to = triangle.vertices[(edge_index + 1) % 3];
      const EdgeKey edge_key = make_edge_key(original_from, original_to);
      const auto occurrences_it = adjacency.edge_occurrences.find(edge_key);
      bool shared_with_same_region = false;
      if (occurrences_it != adjacency.edge_occurrences.end()) {
        for (const TriangleEdgeOccurrence& occurrence : occurrences_it->second) {
          if (occurrence.triangle == static_cast<int>(triangle_index)) {
            continue;
          }
          if (mesh.triangles[occurrence.triangle].region_id == triangle.region_id) {
            shared_with_same_region = true;
            break;
          }
        }
      }
      if (shared_with_same_region) {
        continue;
      }
      const int snapped_from = original_to_snapped[original_from];
      const int snapped_to = original_to_snapped[original_to];
      if (snapped_from == snapped_to) {
        continue;
      }
      boundary_edges[triangle.region_id].push_back({snapped_from, snapped_to});
    }
  }

  for (const PlaneRegion& region : regions) {
    LoopCandidate best_candidate;
    const auto topological_edges_it = boundary_edges.find(region.id);
    if (topological_edges_it != boundary_edges.end()) {
      LoopCandidate directed_candidate =
          score_loop_candidate(extract_loops_for_region(topological_edges_it->second, result.vertices,
                                                        region.fit.normal,
                                                        tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(directed_candidate, best_candidate)) {
        best_candidate = std::move(directed_candidate);
      }

      LoopCandidate undirected_candidate =
          score_loop_candidate(extract_loops_for_region_undirected(topological_edges_it->second,
                                                                   result.vertices, region.fit.normal,
                                                                   tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(undirected_candidate, best_candidate)) {
        best_candidate = std::move(undirected_candidate);
      }
    }

    const std::vector<std::pair<int, int>> triangle_boundary_edges =
        build_region_boundary_edges_from_triangles(region, mesh, original_to_snapped);
    if (!triangle_boundary_edges.empty()) {
      LoopCandidate triangle_directed =
          score_loop_candidate(extract_loops_for_region(triangle_boundary_edges, result.vertices,
                                                        region.fit.normal,
                                                        tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(triangle_directed, best_candidate)) {
        best_candidate = std::move(triangle_directed);
      }

      LoopCandidate triangle_undirected =
          score_loop_candidate(extract_loops_for_region_undirected(triangle_boundary_edges,
                                                                   result.vertices, region.fit.normal,
                                                                   tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(triangle_undirected, best_candidate)) {
        best_candidate = std::move(triangle_undirected);
      }
    }

    std::vector<std::vector<int>> loops = std::move(best_candidate.loops);
    if (loops.empty()) {
      std::vector<int> hull =
          fallback_convex_hull_loop(region, original_to_snapped, result.vertices, region.fit.normal,
                                    tolerances.collinear_distance);
      const double hull_area =
          hull.size() >= 3 ? polygon_area(hull, result.vertices, region.fit.normal) : 0.0;
      const double hull_ratio = region.area > 1e-9 ? hull_area / region.area : 0.0;
      if (hull.size() >= 3 && hull_ratio >= 0.7 && hull_ratio <= 1.15) {
        loops.push_back(std::move(hull));
      }
    }
    if (loops.empty()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }

    std::vector<ReconstructedFace> faces = build_faces_from_region_loops(region, loops, result.vertices);
    if (faces.empty()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }
    for (ReconstructedFace& face : faces) {
      if (!face.loops.empty() && face.loops.front().size() >= 3) {
        result.faces.push_back(std::move(face));
      } else {
        result.omitted_region_ids.push_back(region.id);
      }
    }
  }

  if (result.faces.empty()) {
    result.failure_reasons.push_back("Unable to reconstruct any planar shell faces");
    result.outcome = ReconstructionOutcome::AnalysisOnly;
    return result;
  }

  refine_face_loops_with_shared_vertices(result.faces, result.vertices, tolerances.collinear_distance,
                                         result.edge_split_insertions);
  for (ReconstructedFace& face : result.faces) {
    if (face.loops.empty()) {
      continue;
    }
    orient_loop_to_sign(face.loops.front(), result.vertices, face.fit.normal, 1.0);
    for (std::size_t loop_index = 1; loop_index < face.loops.size(); ++loop_index) {
      orient_loop_to_sign(face.loops[loop_index], result.vertices, face.fit.normal, -1.0);
    }
    face.area = face_total_area(face, result.vertices, face.fit.normal);
  }
  suppress_duplicate_coplanar_faces(result.faces, tolerances);
  prune_dangling_faces(result.faces, tolerances);
  const double shell_cap_area_limit =
      std::min(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.015,
               tolerances.plane_distance * tolerances.plane_distance * 40.0);
  add_small_planar_gap_caps(result.faces, result.vertices, tolerances, shell_cap_area_limit);
  sanitize_face_loops(result.faces);

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
  std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHash> edge_regions;
  for (const ReconstructedFace& face : result.faces) {
    for_each_face_edge(face, [&](int from, int to) {
      const EdgeKey key = make_edge_key(from, to);
      ++edge_use_count[key];
      edge_regions[key].push_back(face.region_id);
    });
  }

  std::size_t open_shell_edges = 0;
  std::size_t non_manifold_shell_edges = 0;
  std::unordered_map<int, std::size_t> problematic_region_counts;
  for (const auto& [edge_key, count] : edge_use_count) {
    if (count == 1) {
      ++open_shell_edges;
      for (int region_id : edge_regions[edge_key]) {
        ++problematic_region_counts[region_id];
      }
    } else if (count != 2) {
      ++non_manifold_shell_edges;
      for (int region_id : edge_regions[edge_key]) {
        ++problematic_region_counts[region_id];
      }
    }
  }
  result.open_edge_count = open_shell_edges;
  result.non_manifold_edge_count = non_manifold_shell_edges;
  result.problematic_regions.clear();
  result.problematic_regions.reserve(problematic_region_counts.size());
  for (const auto& [region_id, edge_count] : problematic_region_counts) {
    result.problematic_regions.push_back({region_id, edge_count});
  }
  std::sort(result.problematic_regions.begin(), result.problematic_regions.end(),
            [](const RegionTopologyIssue& lhs, const RegionTopologyIssue& rhs) {
              if (lhs.edge_count != rhs.edge_count) {
                return lhs.edge_count > rhs.edge_count;
              }
              return lhs.region_id < rhs.region_id;
            });
  result.shell_gap_score =
      edge_use_count.empty()
          ? 1.0
          : static_cast<double>(open_shell_edges + non_manifold_shell_edges) /
                static_cast<double>(edge_use_count.size());

  result.confidence = 0.0;
  for (const ReconstructedFace& face : result.faces) {
    result.confidence += face.confidence;
  }
  result.confidence /= static_cast<double>(result.faces.size());

  if (open_shell_edges > 0 || non_manifold_shell_edges > 0) {
    result.failure_reasons.push_back("Reconstructed shell is open or topologically inconsistent");
  }
  if (result.faces.size() < 4) {
    result.failure_reasons.push_back("Too few reconstructed faces for a closed solid");
  }

  const double volume = std::abs(signed_polyhedron_volume(result.faces, result.vertices));
  const double min_volume = std::pow(std::max(bounds_diagonal(mesh.bounds), 1e-6), 3.0) * 1e-6;
  if (volume <= min_volume) {
    result.failure_reasons.push_back("Reconstructed shell volume is too small or inconsistent");
  }

  if (open_shell_edges == 0 && non_manifold_shell_edges == 0 && result.faces.size() >= 4 &&
      volume > min_volume && result.confidence >= solid_threshold) {
    result.outcome = ReconstructionOutcome::SolidCreated;
  } else if (!result.faces.empty()) {
    result.outcome = ReconstructionOutcome::ShellOnly;
    if (result.confidence < solid_threshold) {
      result.failure_reasons.push_back("Solid export was gated by low reconstruction confidence");
    }
  } else {
    result.outcome = ReconstructionOutcome::AnalysisOnly;
  }

  if (result.outcome != ReconstructionOutcome::SolidCreated &&
      result.non_manifold_edge_count > 0 &&
      result.open_edge_count <= 12) {
    resolve_tiny_conflict_faces(result, mesh, tolerances, solid_threshold);
  }

  if (result.outcome != ReconstructionOutcome::SolidCreated &&
      result.non_manifold_edge_count > 0 &&
      result.open_edge_count <= 12) {
    std::vector<std::vector<LoopCandidate>> candidates;
    candidates.reserve(regions.size());
    std::vector<std::size_t> selected_indices;
    selected_indices.reserve(regions.size());
    for (const PlaneRegion& region : regions) {
      candidates.push_back(collect_region_loop_candidates(region, boundary_edges, mesh,
                                                          original_to_snapped, result.vertices,
                                                          tolerances));
      selected_indices.push_back(choose_initial_candidate_index(candidates.back()));
    }

    optimize_region_loop_choices(result, mesh, regions, candidates, selected_indices, tolerances,
                                 solid_threshold);
    ReconstructionResult candidate_result;
    candidate_result.vertices = result.vertices;
    build_faces_from_selected_candidates(regions, candidates, selected_indices, candidate_result.vertices,
                                         candidate_result);
    finalize_shell_reconstruction(candidate_result, mesh, tolerances, solid_threshold);
    if (is_better_reconstruction_result(candidate_result, result)) {
      result = std::move(candidate_result);
    }
  }
  return result;
}

std::string step_header() {
  std::ostringstream output;
  output << "ISO-10303-21;\n";
  output << "HEADER;\n";
  output << "FILE_DESCRIPTION(('mesh2solid advanced brep'),'2;1');\n";
  output << "FILE_NAME('reconstruction.step','2026-03-25T00:00:00',('Codex'),('OpenAI'),"
         << "'mesh2solid','mesh2solid','');\n";
  output << "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\n";
  output << "ENDSEC;\n";
  output << "DATA;\n";
  return output.str();
}

class StepBuilder {
 public:
  int add(const std::string& body) {
    const int id = next_id_++;
    entities_.push_back("#" + std::to_string(id) + "=" + body + ";");
    return id;
  }

  std::string ref(int id) const { return "#" + std::to_string(id); }

  std::string emit() const {
    std::ostringstream output;
    for (const std::string& entity : entities_) {
      output << entity << "\n";
    }
    return output.str();
  }

 private:
  int next_id_ {1};
  std::vector<std::string> entities_;
};

bool write_step_file(const std::filesystem::path& path, const ReconstructionResult& reconstruction) {
  if (reconstruction.outcome != ReconstructionOutcome::SolidCreated) {
    return false;
  }

  StepBuilder step;
  const int application_context = step.add(
      "APPLICATION_CONTEXT('configuration controlled 3d designs of mechanical parts and assemblies')");
  const int application_protocol =
      step.add("APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994," +
               step.ref(application_context) + ")");
  (void)application_protocol;
  const int design_context =
      step.add("DESIGN_CONTEXT(''," + step.ref(application_context) + ",'design')");
  const int mechanical_context =
      step.add("MECHANICAL_CONTEXT(''," + step.ref(application_context) + ",'mechanical')");
  const int product = step.add("PRODUCT('mesh2solid','mesh2solid','',(" + step.ref(mechanical_context) + "))");
  const int product_formation = step.add(
      "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('',''," + step.ref(product) + ",.NOT_KNOWN.)");
  const int product_definition =
      step.add("PRODUCT_DEFINITION('',''," + step.ref(product_formation) + "," + step.ref(design_context) +
               ")");
  const int product_shape =
      step.add("PRODUCT_DEFINITION_SHAPE('',''," + step.ref(product_definition) + ")");
  const int length_unit = step.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
  const int angle_unit = step.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
  const int solid_angle_unit = step.add("(NAMED_UNIT(*) SOLID_ANGLE_UNIT() SI_UNIT($,.STERADIAN.))");
  const int uncertainty =
      step.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06)," + step.ref(length_unit) +
               ",'distance_accuracy_value','confusion accuracy')");
  const int context = step.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3) "
                               "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" +
                               step.ref(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
                               step.ref(length_unit) + "," + step.ref(angle_unit) + "," +
                               step.ref(solid_angle_unit) + ")) REPRESENTATION_CONTEXT('',''))");

  const int origin = step.add("CARTESIAN_POINT('',(0.0,0.0,0.0))");
  const int z_direction = step.add("DIRECTION('',(0.0,0.0,1.0))");
  const int x_direction = step.add("DIRECTION('',(1.0,0.0,0.0))");
  const int global_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin) + "," + step.ref(z_direction) + "," +
               step.ref(x_direction) + ")");

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
    const Vec3 delta = end - start;
    const double edge_length = length(delta);
    if (edge_length <= 1e-12) {
      throw std::runtime_error("Cannot export zero-length STEP edge");
    }

    const Vec3 direction = normalized(delta);
    const int direction_id =
        step.add("DIRECTION('',(" + format_double(direction.x) + "," + format_double(direction.y) +
                 "," + format_double(direction.z) + "))");
    const int vector_id = step.add("VECTOR(''," + step.ref(direction_id) + "," +
                                   format_double(edge_length) + ")");
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," + step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  std::vector<int> face_ids;
  face_ids.reserve(reconstruction.faces.size());
  for (const ReconstructedFace& face : reconstruction.faces) {
    const int plane_origin = step.add("CARTESIAN_POINT('',(" + format_double(face.fit.centroid.x) + "," +
                                   format_double(face.fit.centroid.y) + "," +
                                   format_double(face.fit.centroid.z) + "))");
    const int face_normal = step.add("DIRECTION('',(" + format_double(face.fit.normal.x) + "," +
                                   format_double(face.fit.normal.y) + "," +
                                   format_double(face.fit.normal.z) + "))");
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    const int face_ref = step.add("DIRECTION('',(" + format_double(basis.u.x) + "," +
                                format_double(basis.u.y) + "," + format_double(basis.u.z) + "))");
    const int axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                              step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const auto& loop = face.loops[loop_index];
      std::ostringstream edge_refs;
      for (std::size_t index = 0; index < loop.size(); ++index) {
        const int from = loop[index];
        const int to = loop[(index + 1) % loop.size()];
        const int edge_curve_id = get_edge_curve_id(from, to);
        const EdgeKey key = make_edge_key(from, to);
        const bool same_orientation = key.a == from && key.b == to;
        const int oriented_edge_id =
            step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                     std::string(same_orientation ? ".T." : ".F.") + ")");
        if (index > 0) {
          edge_refs << ",";
        }
        edge_refs << step.ref(oriented_edge_id);
      }
      const int edge_loop = step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    const int advanced_face =
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)");
    face_ids.push_back(advanced_face);
  }

  std::ostringstream shell_faces;
  for (std::size_t index = 0; index < face_ids.size(); ++index) {
    if (index > 0) {
      shell_faces << ",";
    }
    shell_faces << step.ref(face_ids[index]);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(global_axis) + ")," + step.ref(context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(product_shape) + "," +
           step.ref(shape_representation) + ")");

  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write STEP file: " + path.string());
  }
  output << step_header();
  output << step.emit();
  output << "ENDSEC;\n";
  output << "END-ISO-10303-21;\n";
  return true;
}

void write_ascii_stl(const std::filesystem::path& path, const MeshModel& mesh) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write STL file: " + path.string());
  }

  output << "solid cleaned_mesh\n";
  for (const Triangle& triangle : mesh.triangles) {
    output << "  facet normal " << format_double(triangle.normal.x) << " "
           << format_double(triangle.normal.y) << " " << format_double(triangle.normal.z) << "\n";
    output << "    outer loop\n";
    for (int vertex_index : triangle.vertices) {
      const Vec3& vertex = mesh.vertices[vertex_index];
      output << "      vertex " << format_double(vertex.x) << " " << format_double(vertex.y) << " "
             << format_double(vertex.z) << "\n";
    }
    output << "    endloop\n";
    output << "  endfacet\n";
  }
  output << "endsolid cleaned_mesh\n";
}

std::string backend_description() {
#if defined(MESH2SOLID_WITH_CGAL) && defined(MESH2SOLID_WITH_OCCT)
  return "internal_fallback + cgal + opencascade";
#elif defined(MESH2SOLID_WITH_CGAL)
  return "internal_fallback + cgal";
#elif defined(MESH2SOLID_WITH_OCCT)
  return "internal_fallback + opencascade";
#else
  return "internal_fallback";
#endif
}

std::string regions_json(const std::vector<PlaneRegion>& regions) {
  std::ostringstream output;
  output << "{\n  \"regions\": [\n";
  for (std::size_t index = 0; index < regions.size(); ++index) {
    const PlaneRegion& region = regions[index];
    output << "    {\n";
    output << "      \"id\": " << region.id << ",\n";
    output << "      \"triangle_count\": " << region.triangle_indices.size() << ",\n";
    output << "      \"vertex_count\": " << region.vertex_indices.size() << ",\n";
    output << "      \"area\": " << format_double(region.area) << ",\n";
    output << "      \"unresolved\": " << bool_json(region.unresolved) << ",\n";
    output << "      \"fit\": {\n";
    output << "        \"normal\": " << vec3_json(region.fit.normal) << ",\n";
    output << "        \"centroid\": " << vec3_json(region.fit.centroid) << ",\n";
    output << "        \"d\": " << format_double(region.fit.d) << ",\n";
    output << "        \"rms_error\": " << format_double(region.fit.rms_error) << ",\n";
    output << "        \"max_error\": " << format_double(region.fit.max_error) << ",\n";
    output << "        \"confidence\": " << format_double(region.fit.confidence) << "\n";
    output << "      },\n";
    output << "      \"neighbors\": [";
    for (std::size_t neighbor = 0; neighbor < region.neighbor_region_ids.size(); ++neighbor) {
      if (neighbor > 0) {
        output << ",";
      }
      output << region.neighbor_region_ids[neighbor];
    }
    output << "]\n";
    output << "    }";
    if (index + 1 < regions.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string constraints_json(const ConstraintGraph& graph) {
  std::ostringstream output;
  output << "{\n  \"constraints\": [\n";
  for (std::size_t index = 0; index < graph.constraints.size(); ++index) {
    const Constraint& constraint = graph.constraints[index];
    output << "    {\n";
    output << "      \"region_a\": " << constraint.region_a << ",\n";
    output << "      \"region_b\": " << constraint.region_b << ",\n";
    output << "      \"type\": \"" << escape_json(constraint_type_to_string(constraint.type)) << "\",\n";
    output << "      \"applied\": " << bool_json(constraint.applied) << ",\n";
    output << "      \"score\": " << format_double(constraint.score) << ",\n";
    output << "      \"rationale\": \"" << escape_json(constraint.rationale) << "\"\n";
    output << "    }";
    if (index + 1 < graph.constraints.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string reconstruction_debug_json(const ReconstructionResult& reconstruction) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"vertices\": [\n";
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    output << "    " << vec3_json(reconstruction.vertices[index]);
    if (index + 1 < reconstruction.vertices.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ],\n";
  output << "  \"faces\": [\n";
  for (std::size_t index = 0; index < reconstruction.faces.size(); ++index) {
    const ReconstructedFace& face = reconstruction.faces[index];
    output << "    {\n";
    output << "      \"region_id\": " << face.region_id << ",\n";
    output << "      \"area\": " << format_double(face.area) << ",\n";
    output << "      \"confidence\": " << format_double(face.confidence) << ",\n";
    output << "      \"fit\": {\n";
    output << "        \"normal\": " << vec3_json(face.fit.normal) << ",\n";
    output << "        \"centroid\": " << vec3_json(face.fit.centroid) << ",\n";
    output << "        \"d\": " << format_double(face.fit.d) << "\n";
    output << "      },\n";
    output << "      \"loops\": [\n";
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      output << "        [";
      for (std::size_t vertex_index = 0; vertex_index < face.loops[loop_index].size(); ++vertex_index) {
        if (vertex_index > 0) {
          output << ",";
        }
        output << face.loops[loop_index][vertex_index];
      }
      output << "]";
      if (loop_index + 1 < face.loops.size()) {
        output << ",";
      }
      output << "\n";
    }
    output << "      ]\n";
    output << "    }";
    if (index + 1 < reconstruction.faces.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string report_json(const RunReport& report, const AnalyzeOptions& options) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"input\": \"" << escape_json(options.input_path.string()) << "\",\n";
  output << "  \"preset\": \"" << escape_json(options.preset) << "\",\n";
  output << "  \"backend\": \"" << escape_json(report.backend) << "\",\n";
  output << "  \"mesh_stats_before\": {\n";
  output << "    \"vertex_count\": " << report.repair.before.vertex_count << ",\n";
  output << "    \"triangle_count\": " << report.repair.before.triangle_count << ",\n";
  output << "    \"connected_components\": " << report.repair.before.connected_components << ",\n";
  output << "    \"open_edge_count\": " << report.repair.before.open_edge_count << ",\n";
  output << "    \"non_manifold_edge_count\": " << report.repair.before.non_manifold_edge_count << ",\n";
  output << "    \"surface_area\": " << format_double(report.repair.before.surface_area) << ",\n";
  output << "    \"bbox_diagonal\": " << format_double(report.repair.before.bbox_diagonal) << "\n";
  output << "  },\n";
  output << "  \"mesh_stats_after\": {\n";
  output << "    \"vertex_count\": " << report.repair.after.vertex_count << ",\n";
  output << "    \"triangle_count\": " << report.repair.after.triangle_count << ",\n";
  output << "    \"connected_components\": " << report.repair.after.connected_components << ",\n";
  output << "    \"open_edge_count\": " << report.repair.after.open_edge_count << ",\n";
  output << "    \"non_manifold_edge_count\": " << report.repair.after.non_manifold_edge_count << ",\n";
  output << "    \"surface_area\": " << format_double(report.repair.after.surface_area) << ",\n";
  output << "    \"bbox_diagonal\": " << format_double(report.repair.after.bbox_diagonal) << "\n";
  output << "  },\n";
  output << "  \"repair\": {\n";
  output << "    \"welded_vertices_removed\": " << report.repair.welded_vertices_removed << ",\n";
  output << "    \"duplicate_triangles_removed\": " << report.repair.duplicate_triangles_removed << ",\n";
  output << "    \"degenerate_triangles_removed\": " << report.repair.degenerate_triangles_removed << ",\n";
  output << "    \"tiny_components_removed\": " << report.repair.tiny_components_removed << ",\n";
  output << "    \"tiny_component_triangles_removed\": "
         << report.repair.tiny_component_triangles_removed << ",\n";
  output << "    \"orientation_flips\": " << report.repair.orientation_flips << ",\n";
  output << "    \"warnings\": [";
  for (std::size_t index = 0; index < report.repair.warnings.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << "\"" << escape_json(report.repair.warnings[index]) << "\"";
  }
  output << "]\n";
  output << "  },\n";
  output << "  \"tolerances\": {\n";
  output << "    \"vertex_weld\": " << format_double(report.tolerances.vertex_weld) << ",\n";
  output << "    \"plane_distance\": " << format_double(report.tolerances.plane_distance) << ",\n";
  output << "    \"plane_merge_distance\": " << format_double(report.tolerances.plane_merge_distance) << ",\n";
  output << "    \"collinear_distance\": " << format_double(report.tolerances.collinear_distance) << ",\n";
  output << "    \"normal_angle_degrees\": " << format_double(report.tolerances.normal_angle_degrees) << ",\n";
  output << "    \"merge_angle_degrees\": " << format_double(report.tolerances.merge_angle_degrees) << ",\n";
  output << "    \"parallel_angle_degrees\": " << format_double(report.tolerances.parallel_angle_degrees)
         << ",\n";
  output << "    \"perpendicular_angle_degrees\": "
         << format_double(report.tolerances.perpendicular_angle_degrees) << "\n";
  output << "  },\n";
  output << "  \"regions\": {\n";
  output << "    \"count\": " << report.regions.size() << ",\n";
  output << "    \"unresolved_count\": "
         << std::count_if(report.regions.begin(), report.regions.end(),
                          [](const PlaneRegion& region) { return region.unresolved; })
         << "\n";
  output << "  },\n";
  output << "  \"constraints\": {\n";
  output << "    \"count\": " << report.constraint_graph.constraints.size() << "\n";
  output << "  },\n";
  output << "  \"reconstruction\": {\n";
  output << "    \"outcome\": \""
         << escape_json(reconstruction_outcome_to_string(report.reconstruction.outcome)) << "\",\n";
  output << "    \"face_count\": " << report.reconstruction.faces.size() << ",\n";
  output << "    \"vertex_count\": " << report.reconstruction.vertices.size() << ",\n";
  output << "    \"edge_split_insertions\": " << report.reconstruction.edge_split_insertions << ",\n";
  output << "    \"open_edge_count\": " << report.reconstruction.open_edge_count << ",\n";
  output << "    \"non_manifold_edge_count\": " << report.reconstruction.non_manifold_edge_count << ",\n";
  output << "    \"omitted_region_ids\": [";
  for (std::size_t index = 0; index < report.reconstruction.omitted_region_ids.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << report.reconstruction.omitted_region_ids[index];
  }
  output << "],\n";
  output << "    \"problematic_regions\": [";
  for (std::size_t index = 0; index < report.reconstruction.problematic_regions.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    const RegionTopologyIssue& issue = report.reconstruction.problematic_regions[index];
    output << "{\"region_id\":" << issue.region_id << ",\"edge_count\":" << issue.edge_count << "}";
  }
  output << "],\n";
  output << "    \"shell_gap_score\": " << format_double(report.reconstruction.shell_gap_score) << ",\n";
  output << "    \"confidence\": " << format_double(report.reconstruction.confidence) << ",\n";
  output << "    \"step_written\": " << bool_json(report.reconstruction.step_written) << ",\n";
  output << "    \"failure_reasons\": [";
  for (std::size_t index = 0; index < report.reconstruction.failure_reasons.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << "\"" << escape_json(report.reconstruction.failure_reasons[index]) << "\"";
  }
  output << "]\n";
  output << "  }\n";
  output << "}\n";
  return output.str();
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write file: " + path.string());
  }
  output << text;
}

}  // namespace

std::string reconstruction_outcome_to_string(ReconstructionOutcome outcome) {
  switch (outcome) {
    case ReconstructionOutcome::AnalysisOnly:
      return "analysis_only";
    case ReconstructionOutcome::ShellOnly:
      return "shell_only";
    case ReconstructionOutcome::SolidCreated:
      return "solid_created";
  }
  return "analysis_only";
}

std::string constraint_type_to_string(ConstraintType type) {
  switch (type) {
    case ConstraintType::CoplanarMerge:
      return "coplanar_merge";
    case ConstraintType::Parallel:
      return "parallel";
    case ConstraintType::Perpendicular:
      return "perpendicular";
  }
  return "coplanar_merge";
}

RunReport analyze(const AnalyzeOptions& options) {
  MeshModel mesh = load_mesh(options.input_path);
  RunReport report;
  report.backend = backend_description();

  Tolerances repair_tolerances = build_tolerances(mesh, options.preset);
  report.repair.before = compute_mesh_stats(mesh);

  report.repair.welded_vertices_removed = weld_vertices(mesh, repair_tolerances.vertex_weld);
  report.repair.duplicate_triangles_removed = remove_duplicate_triangles(mesh);
  report.repair.degenerate_triangles_removed = remove_degenerate_triangles(mesh);
  compact_vertices(mesh);

  const ComponentRemoval removal = remove_tiny_components(mesh, repair_tolerances);
  report.repair.tiny_components_removed = removal.components_removed;
  report.repair.tiny_component_triangles_removed = removal.triangles_removed;
  report.repair.orientation_flips = orient_triangles_consistently(mesh);
  compact_vertices(mesh);
  const std::size_t non_manifold_slivers_removed = remove_non_manifold_sliver_triangles(mesh);
  compact_vertices(mesh);

  report.tolerances = build_tolerances(mesh, options.preset);
  report.repair.after = compute_mesh_stats(mesh);

  if (non_manifold_slivers_removed > 0) {
    report.repair.warnings.push_back("Removed " + std::to_string(non_manifold_slivers_removed) +
                                     " sliver triangles while resolving non-manifold edges");
  }

  if (report.repair.after.open_edge_count > 0) {
    report.repair.warnings.push_back("Mesh still has open boundary edges after repair");
  }
  if (report.repair.after.non_manifold_edge_count > 0) {
    report.repair.warnings.push_back("Mesh still has non-manifold edges after repair");
  }
  if (mesh.triangles.empty()) {
    throw std::runtime_error("No triangles remain after repair");
  }

  std::vector<PlaneRegion> segmented = segment_planar_regions(mesh, report.tolerances);
  auto [constrained_regions, constraint_graph] = apply_constraints(mesh, segmented, report.tolerances);
  report.cleaned_mesh = mesh;
  report.regions = std::move(constrained_regions);
  report.constraint_graph = std::move(constraint_graph);
  report.reconstruction =
      reconstruct_shell(report.cleaned_mesh, report.regions, report.tolerances, options.solid_threshold);
  return report;
}

void write_outputs(const AnalyzeOptions& options, RunReport& report) {
  std::filesystem::create_directories(options.output_dir);
  write_ascii_stl(options.output_dir / "cleaned_mesh.stl", report.cleaned_mesh);
  write_text(options.output_dir / "regions.json", regions_json(report.regions));
  write_text(options.output_dir / "constraints.json", constraints_json(report.constraint_graph));
  write_text(options.output_dir / "reconstruction_debug.json",
             reconstruction_debug_json(report.reconstruction));

  if (report.reconstruction.outcome == ReconstructionOutcome::SolidCreated) {
    report.reconstruction.step_written =
        write_step_file(options.output_dir / "reconstruction.step", report.reconstruction);
  } else {
    report.reconstruction.step_written = false;
  }

  write_text(options.output_dir / "report.json", report_json(report, options));
}

}  // namespace mesh2solid
