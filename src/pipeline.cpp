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
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <zlib.h>

#if defined(MESH2SOLID_WITH_CGAL)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_regularization/regularize_planes.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <boost/property_map/property_map.hpp>
#endif

#if defined(MESH2SOLID_WITH_OCCT)
#include <BRepLib.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Writer.hxx>
#include <Standard_Failure.hxx>
#include <ShapeFix_Shape.hxx>
#include <ShapeFix_ShapeTolerance.hxx>
#include <ShapeFix_Shell.hxx>
#include <ShapeFix_Solid.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Shell.hxx>
#include <gp_Pnt.hxx>
#endif

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

// Keep the implementation in one translation unit for low-risk refactors, but split
// the heavy subsystems into focused fragments so the main pipeline file stays navigable.
#include "pipeline/io.inc"

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

#include "pipeline/cgal.inc"

#include "pipeline/reconstruction.inc"

#include "pipeline/fallback.inc"

#include "pipeline/output.inc"

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

std::string reconstruction_method_to_string(ReconstructionMethod method) {
  switch (method) {
    case ReconstructionMethod::AnalyticPlanar:
      return "analytic_planar";
    case ReconstructionMethod::FacetedMeshFallback:
      return "faceted_mesh_fallback";
    case ReconstructionMethod::OcctFacetedMeshFallback:
      return "occt_faceted_mesh_fallback";
  }
  return "analytic_planar";
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

bool mesh_stats_are_watertight(const MeshStats& stats) {
  return stats.open_edge_count == 0 && stats.non_manifold_edge_count == 0;
}

RunReport analyze(const AnalyzeOptions& options) {
  MeshModel mesh = load_mesh(options.input_path);
  const MeshModel original_mesh = mesh;
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
  report.plane_regularization =
      regularize_plane_regions_with_cgal(mesh, segmented, report.tolerances);
  auto [constrained_regions, constraint_graph] = apply_constraints(mesh, segmented, report.tolerances);
  report.cleaned_mesh = mesh;
  report.reconstruction_mesh = report.cleaned_mesh;
  report.regions = std::move(constrained_regions);
  report.constraint_graph = std::move(constraint_graph);
  report.reconstruction =
      reconstruct_shell(report.cleaned_mesh, report.regions, report.tolerances, options.solid_threshold);
  if (report.reconstruction.outcome != ReconstructionOutcome::SolidCreated) {
    ReconstructionResult best_reconstruction = report.reconstruction;
    MeshModel best_reconstruction_mesh = report.reconstruction_mesh;

    ReconstructionResult cleaned_fallback =
        reconstruct_faceted_mesh_fallback(report.cleaned_mesh, report.tolerances,
                                          options.solid_threshold,
                                          mesh_stats_are_watertight(report.repair.after));
    if (is_better_reconstruction_result(cleaned_fallback, best_reconstruction)) {
      best_reconstruction = std::move(cleaned_fallback);
      best_reconstruction_mesh = report.cleaned_mesh;
    }

    if (mesh_stats_are_watertight(report.repair.before)) {
      ReconstructionResult original_fallback =
          reconstruct_faceted_mesh_fallback(original_mesh, repair_tolerances,
                                            options.solid_threshold, true);
      if (is_better_reconstruction_result(original_fallback, best_reconstruction)) {
        best_reconstruction = std::move(original_fallback);
        best_reconstruction_mesh = original_mesh;
      }
    }

    report.reconstruction = std::move(best_reconstruction);
    report.reconstruction_mesh = std::move(best_reconstruction_mesh);
  }
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
    const Tolerances reconstruction_tolerances =
        build_tolerances(report.reconstruction_mesh, options.preset);
    const std::filesystem::path step_path = options.output_dir / "reconstruction.step";
    if (report.reconstruction.method == ReconstructionMethod::OcctFacetedMeshFallback) {
      report.reconstruction.step_written =
          write_step_file_occt(step_path, report.reconstruction_mesh, reconstruction_tolerances);
      if (!report.reconstruction.step_written) {
        report.reconstruction.step_written =
            write_step_file(step_path, report.reconstruction, report.reconstruction_mesh,
                            reconstruction_tolerances);
      }
    } else {
      report.reconstruction.step_written =
          write_step_file(step_path, report.reconstruction, report.reconstruction_mesh,
                          reconstruction_tolerances);
    }
  } else {
    report.reconstruction.step_written = false;
  }

  write_text(options.output_dir / "report.json", report_json(report, options));
}

}  // namespace mesh2solid
