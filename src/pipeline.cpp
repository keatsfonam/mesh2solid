#include "stl2solid/pipeline.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
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

namespace stl2solid {

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
      canonical.emplace(key, canonical_index);
      welded.push_back(vertex);
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
                                    const std::vector<PlaneRegion>& regions) {
  std::array<std::array<double, 3>, 3> matrix {{
      {1.0, 0.0, 0.0},
      {0.0, 1.0, 0.0},
      {0.0, 0.0, 1.0},
  }};
  Vec3 rhs = original;

  for (int region_id : incident_regions) {
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

  Vec3 snapped = original;
  if (solve_linear_system_3x3(matrix, rhs, snapped)) {
    return snapped;
  }
  return original;
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
      if (point_line_distance_2d(b, a, c) <= tolerance) {
        simplified.erase(simplified.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
    }
  }

  return simplified.size() >= 3 ? simplified : std::vector<int> {};
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
        std::vector<Vec2> projected;
        projected.reserve(simplified.size());
        for (int vertex : simplified) {
          projected.push_back(project_to_basis(vertices[vertex], basis));
        }
        if (signed_area_2d(projected) < 0.0) {
          std::reverse(simplified.begin(), simplified.end());
        }
        loops.push_back(std::move(simplified));
      }
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

double signed_polyhedron_volume(const std::vector<ReconstructedFace>& faces,
                                const std::vector<Vec3>& vertices) {
  double volume = 0.0;
  for (const ReconstructedFace& face : faces) {
    if (face.vertex_ids.size() < 3) {
      continue;
    }
    const Vec3& a = vertices[face.vertex_ids[0]];
    for (std::size_t index = 1; index + 1 < face.vertex_ids.size(); ++index) {
      const Vec3& b = vertices[face.vertex_ids[index]];
      const Vec3& c = vertices[face.vertex_ids[index + 1]];
      volume += dot(a, cross(b, c)) / 6.0;
    }
  }
  return volume;
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
  std::unordered_map<GridKey, int, GridKeyHash> snapped_lookup;
  auto quantize = [tolerances](double value) -> long long {
    return static_cast<long long>(std::llround(value / std::max(tolerances.collinear_distance, 1e-7)));
  };

  for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index) {
    Vec3 snapped = mesh.vertices[vertex_index];
    if (!incident_regions[vertex_index].empty()) {
      snapped = snap_vertex_to_incident_planes(mesh.vertices[vertex_index], incident_regions[vertex_index],
                                               regions);
    }
    const GridKey key {quantize(snapped.x), quantize(snapped.y), quantize(snapped.z)};
    const auto found = snapped_lookup.find(key);
    if (found != snapped_lookup.end() &&
        length(result.vertices[found->second] - snapped) <= tolerances.collinear_distance * 2.0) {
      original_to_snapped[vertex_index] = found->second;
    } else {
      const int snapped_index = static_cast<int>(result.vertices.size());
      result.vertices.push_back(snapped);
      snapped_lookup.emplace(key, snapped_index);
      original_to_snapped[vertex_index] = snapped_index;
    }
  }

  std::unordered_map<int, std::vector<std::pair<int, int>>> boundary_edges;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    for (int edge_index = 0; edge_index < 3; ++edge_index) {
      const int neighbor = adjacency.neighbors[triangle_index][edge_index];
      if (neighbor >= 0 && mesh.triangles[neighbor].region_id == triangle.region_id) {
        continue;
      }
      const int original_from = triangle.vertices[edge_index];
      const int original_to = triangle.vertices[(edge_index + 1) % 3];
      const int snapped_from = original_to_snapped[original_from];
      const int snapped_to = original_to_snapped[original_to];
      if (snapped_from == snapped_to) {
        continue;
      }
      boundary_edges[triangle.region_id].push_back({snapped_from, snapped_to});
    }
  }

  bool unsupported_holes = false;
  for (const PlaneRegion& region : regions) {
    const auto edges_it = boundary_edges.find(region.id);
    if (edges_it == boundary_edges.end()) {
      continue;
    }
    std::vector<std::vector<int>> loops =
        extract_loops_for_region(edges_it->second, result.vertices, region.fit.normal,
                                 tolerances.collinear_distance);
    if (loops.empty()) {
      continue;
    }

    std::sort(loops.begin(), loops.end(), [&result, &region](const auto& lhs, const auto& rhs) {
      return polygon_area(lhs, result.vertices, region.fit.normal) >
             polygon_area(rhs, result.vertices, region.fit.normal);
    });

    if (loops.size() > 1) {
      unsupported_holes = true;
    }

    ReconstructedFace face;
    face.region_id = region.id;
    face.fit = region.fit;
    face.vertex_ids = loops.front();
    face.area = polygon_area(face.vertex_ids, result.vertices, region.fit.normal);
    face.confidence = region.fit.confidence;
    if (face.vertex_ids.size() >= 3) {
      result.faces.push_back(std::move(face));
    }
  }

  if (result.faces.empty()) {
    result.failure_reasons.push_back("Unable to reconstruct any planar shell faces");
    result.outcome = ReconstructionOutcome::AnalysisOnly;
    return result;
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
  for (const ReconstructedFace& face : result.faces) {
    for (std::size_t index = 0; index < face.vertex_ids.size(); ++index) {
      const int from = face.vertex_ids[index];
      const int to = face.vertex_ids[(index + 1) % face.vertex_ids.size()];
      ++edge_use_count[make_edge_key(from, to)];
    }
  }

  std::size_t open_shell_edges = 0;
  for (const auto& [edge_key, count] : edge_use_count) {
    (void)edge_key;
    if (count != 2) {
      ++open_shell_edges;
    }
  }
  result.shell_gap_score =
      edge_use_count.empty() ? 1.0
                             : static_cast<double>(open_shell_edges) / static_cast<double>(edge_use_count.size());

  result.confidence = 0.0;
  for (const ReconstructedFace& face : result.faces) {
    result.confidence += face.confidence;
  }
  result.confidence /= static_cast<double>(result.faces.size());

  if (unsupported_holes) {
    result.failure_reasons.push_back("Detected faces with multiple boundary loops; holes are not exported in v1");
  }
  if (open_shell_edges > 0) {
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

  if (open_shell_edges == 0 && !unsupported_holes && result.faces.size() >= 4 &&
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
  return result;
}

std::string step_header() {
  std::ostringstream output;
  output << "ISO-10303-21;\n";
  output << "HEADER;\n";
  output << "FILE_DESCRIPTION(('stl2solid advanced brep'),'2;1');\n";
  output << "FILE_NAME('reconstruction.step','2026-03-25T00:00:00',('Codex'),('OpenAI'),"
         << "'stl2solid','stl2solid','');\n";
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
  const int product = step.add("PRODUCT('stl2solid','stl2solid','',(" + step.ref(mechanical_context) + "))");
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

    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < face.vertex_ids.size(); ++index) {
      const int from = face.vertex_ids[index];
      const int to = face.vertex_ids[(index + 1) % face.vertex_ids.size()];
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
    const int outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)");
    const int advanced_face =
        step.add("ADVANCED_FACE('',(" + step.ref(outer_bound) + ")," + step.ref(plane) + ",.T.)");
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
      step.add("MANIFOLD_SOLID_BREP('stl2solid'," + step.ref(closed_shell) + ")");
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
#if defined(STL2SOLID_WITH_CGAL) && defined(STL2SOLID_WITH_OCCT)
  return "internal_fallback + cgal + opencascade";
#elif defined(STL2SOLID_WITH_CGAL)
  return "internal_fallback + cgal";
#elif defined(STL2SOLID_WITH_OCCT)
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
  MeshModel mesh = load_stl(options.input_path);
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

  report.tolerances = build_tolerances(mesh, options.preset);
  report.repair.after = compute_mesh_stats(mesh);

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

  if (report.reconstruction.outcome == ReconstructionOutcome::SolidCreated) {
    report.reconstruction.step_written =
        write_step_file(options.output_dir / "reconstruction.step", report.reconstruction);
  } else {
    report.reconstruction.step_written = false;
  }

  write_text(options.output_dir / "report.json", report_json(report, options));
}

}  // namespace stl2solid
