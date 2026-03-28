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

std::optional<std::vector<SimpleCylinderPrimitive>> collect_simple_cylinder_primitives(
    const MeshModel& mesh,
    const Tolerances& tolerances) {
  const std::vector<std::vector<int>> components = collect_triangle_components(mesh);
  if (components.empty()) {
    return std::nullopt;
  }

  std::vector<SimpleCylinderPrimitive> cylinders;
  cylinders.reserve(components.size());
  for (const std::vector<int>& component : components) {
    const MeshModel component_mesh = extract_triangle_component_mesh(mesh, component);
    const std::optional<SimpleCylinderPrimitive> cylinder =
        detect_simple_cylinder_component(component_mesh, tolerances);
    if (!cylinder.has_value()) {
      return std::nullopt;
    }
    cylinders.push_back(*cylinder);
  }

  return cylinders;
}

double projected_loop_area(const std::vector<int>& loop,
                           const std::vector<Vec3>& vertices,
                           const PlaneBasis& basis) {
  if (loop.size() < 3) {
    return 0.0;
  }

  double twice_area = 0.0;
  for (std::size_t index = 0; index < loop.size(); ++index) {
    const Vec3& current = vertices[loop[index]];
    const Vec3& next = vertices[loop[(index + 1) % loop.size()]];
    const double current_u = dot(current, basis.u);
    const double current_v = dot(current, basis.v);
    const double next_u = dot(next, basis.u);
    const double next_v = dot(next, basis.v);
    twice_area += current_u * next_v - next_u * current_v;
  }
  return 0.5 * twice_area;
}

std::vector<Vec2> project_loop_to_basis(const std::vector<int>& loop,
                                        const std::vector<Vec3>& vertices,
                                        const PlaneBasis& basis) {
  std::vector<Vec2> projected;
  projected.reserve(loop.size());
  for (int vertex_index : loop) {
    projected.push_back(project_to_basis(vertices[vertex_index], basis));
  }
  return projected;
}

using ClassifiedFaceLoops = std::tuple<std::vector<int>, std::vector<int>, double, double>;

struct ClassifiedFaceMultiLoops {
  std::vector<int> outer_loop;
  double outer_area {};
  std::vector<std::vector<int>> inner_loops;
  std::vector<double> inner_areas;
};

std::optional<ClassifiedFaceLoops> classify_face_loops_by_area(const ReconstructedFace& face,
                                                               const std::vector<Vec3>& vertices,
                                                               const PlaneBasis& basis) {
  if (face.loops.size() != 2) {
    return std::nullopt;
  }
  const double area0 = projected_loop_area(face.loops[0], vertices, basis);
  const double area1 = projected_loop_area(face.loops[1], vertices, basis);
  if (std::abs(area0) <= 1e-9 || std::abs(area1) <= 1e-9) {
    return std::nullopt;
  }
  if (std::abs(area0) >= std::abs(area1)) {
    return ClassifiedFaceLoops {face.loops[0], face.loops[1], area0, area1};
  }
  return ClassifiedFaceLoops {face.loops[1], face.loops[0], area1, area0};
}

std::optional<ClassifiedFaceMultiLoops> classify_face_loops_outer_inner(
    const ReconstructedFace& face,
    const std::vector<Vec3>& vertices,
    const PlaneBasis& basis) {
  if (face.loops.size() < 2) {
    return std::nullopt;
  }

  int outer_index = -1;
  double outer_area = 0.0;
  for (std::size_t index = 0; index < face.loops.size(); ++index) {
    const double area = projected_loop_area(face.loops[index], vertices, basis);
    if (std::abs(area) <= 1e-9) {
      return std::nullopt;
    }
    if (outer_index < 0 || std::abs(area) > std::abs(outer_area)) {
      outer_index = static_cast<int>(index);
      outer_area = area;
    }
  }
  if (outer_index < 0) {
    return std::nullopt;
  }

  ClassifiedFaceMultiLoops classified;
  classified.outer_loop = face.loops[outer_index];
  classified.outer_area = outer_area;
  classified.inner_loops.reserve(face.loops.size() - 1);
  classified.inner_areas.reserve(face.loops.size() - 1);
  for (std::size_t index = 0; index < face.loops.size(); ++index) {
    if (static_cast<int>(index) == outer_index) {
      continue;
    }
    classified.inner_loops.push_back(face.loops[index]);
    classified.inner_areas.push_back(projected_loop_area(face.loops[index], vertices, basis));
  }
  if (classified.inner_loops.empty()) {
    return std::nullopt;
  }
  return classified;
}

bool circle_fit_matches(const CircleFit2D& lhs,
                        const CircleFit2D& rhs,
                        double radius_tolerance) {
  const Vec2 center_delta {lhs.center.x - rhs.center.x, lhs.center.y - rhs.center.y};
  return std::abs(lhs.radius - rhs.radius) <= radius_tolerance &&
         lhs.max_error <= radius_tolerance && rhs.max_error <= radius_tolerance &&
         std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) <=
             radius_tolerance;
}

bool circle_fit_centers_coincident(const CircleFit2D& lhs,
                                   const CircleFit2D& rhs,
                                   double center_tolerance) {
  const Vec2 center_delta {lhs.center.x - rhs.center.x, lhs.center.y - rhs.center.y};
  return std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) <=
         center_tolerance;
}

struct ObroundFit2D {
  Vec2 center {};
  Vec2 major {};
  Vec2 minor {};
  double radius {};
  double half_span {};
  double max_error {};
};

std::optional<ObroundFit2D> fit_obround_2d(const std::vector<Vec2>& points) {
  if (points.size() < 6) {
    return std::nullopt;
  }

  Vec2 centroid {};
  for (const Vec2& point : points) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x /= static_cast<double>(points.size());
  centroid.y /= static_cast<double>(points.size());

  double xx = 0.0;
  double xy = 0.0;
  double yy = 0.0;
  for (const Vec2& point : points) {
    const double dx = point.x - centroid.x;
    const double dy = point.y - centroid.y;
    xx += dx * dx;
    xy += dx * dy;
    yy += dy * dy;
  }
  if (xx + yy <= 1e-12) {
    return std::nullopt;
  }

  auto evaluate_basis = [&](const Vec2& major, const Vec2& minor) -> std::optional<ObroundFit2D> {
    std::vector<Vec2> local_points;
    local_points.reserve(points.size());
    for (const Vec2& point : points) {
      const Vec2 delta {point.x - centroid.x, point.y - centroid.y};
      local_points.push_back(
          {delta.x * major.x + delta.y * major.y, delta.x * minor.x + delta.y * minor.y});
    }

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const Vec2& point : local_points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    const Vec2 local_center {0.5 * (min_x + max_x), 0.5 * (min_y + max_y)};
    const double radius = 0.5 * (max_y - min_y);
    if (radius <= 1e-6) {
      return std::nullopt;
    }
    const double half_extent_x = 0.5 * (max_x - min_x);
    const double half_span = half_extent_x - radius;
    if (half_span <= std::max(radius * 0.08, 0.2)) {
      return std::nullopt;
    }

    double max_error = 0.0;
    bool saw_left_cap = false;
    bool saw_right_cap = false;
    bool saw_upper_flank = false;
    bool saw_lower_flank = false;
    for (const Vec2& point : local_points) {
      const double x = point.x - local_center.x;
      const double y = point.y - local_center.y;
      double error = 0.0;
      if (x > half_span) {
        error = std::abs(std::sqrt((x - half_span) * (x - half_span) + y * y) - radius);
        saw_right_cap = true;
      } else if (x < -half_span) {
        error = std::abs(std::sqrt((x + half_span) * (x + half_span) + y * y) - radius);
        saw_left_cap = true;
      } else {
        error = std::abs(std::abs(y) - radius);
        if (y >= 0.0) {
          saw_upper_flank = true;
        } else {
          saw_lower_flank = true;
        }
      }
      max_error = std::max(max_error, error);
    }
    if (max_error > std::max(radius * 0.12, 0.45) || !saw_left_cap || !saw_right_cap ||
        !saw_upper_flank || !saw_lower_flank) {
      return std::nullopt;
    }

    ObroundFit2D fit;
    fit.center = {
        centroid.x + major.x * local_center.x + minor.x * local_center.y,
        centroid.y + major.y * local_center.x + minor.y * local_center.y,
    };
    fit.major = major;
    fit.minor = minor;
    fit.radius = radius;
    fit.half_span = half_span;
    fit.max_error = max_error;
    return fit;
  };

  const double theta = 0.5 * std::atan2(2.0 * xy, xx - yy);
  const Vec2 major {std::cos(theta), std::sin(theta)};
  const Vec2 minor {-major.y, major.x};

  if (const std::optional<ObroundFit2D> fit = evaluate_basis(major, minor); fit.has_value()) {
    return fit;
  }
  return evaluate_basis({-major.x, -major.y}, {-minor.x, -minor.y});
}

struct SimplePrismaticBoreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBore {
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive bore {};
};

struct SimplePrismaticMultiBoreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBore> bores;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBlindBoreExport {
  int top_face_index {};
  int floor_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> floor_loop;
  double top_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBlindBore {
  double top_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive bore {};
};

struct SimplePrismaticMultiBlindBoreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBlindBore> bores;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBlindCounterbore {
  double top_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
};

struct SimplePrismaticMultiBlindCountersink {
  double top_inner_area {};
  double floor_area {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive bore {};
};

struct SimplePrismaticMultiBlindCountersinkExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBlindCountersink> countersinks;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBlindCounterboreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBlindCounterbore> counterbores;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBlindCounterboreExport {
  int top_face_index {};
  int step_face_index {};
  int floor_face_index {};
  std::vector<int> top_outer_loop;
  double top_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBlindCountersinkExport {
  int top_face_index {};
  int floor_face_index {};
  std::vector<int> top_outer_loop;
  double top_inner_area {};
  double floor_area {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticCounterboreExport {
  int top_face_index {};
  int bottom_face_index {};
  int step_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticCountersinkExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

// Phase-1 canonical prismatic section model:
// shared-cap axial features all reduce to the same 2.5D description:
// paired cap loops plus one or more feature columns whose radius profile is
// constant, stepped, or tapered along the shared axis.
enum class PrismaticAxialThroughFeatureKind {
  bore,
  countersink,
  counterbore,
};

struct SimplePrismaticAxialThroughFeature {
  double top_inner_area {};
  double bottom_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  PrismaticAxialThroughFeatureKind kind {PrismaticAxialThroughFeatureKind::bore};
  SimpleCylinderPrimitive bore {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
};

struct SimplePrismaticAxialThroughFeatureExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticAxialThroughFeature> features;
  std::vector<int> outer_face_indices;
};

using PrismaticAxialSectionColumn = SimplePrismaticAxialThroughFeature;
using PrismaticAxialSectionModel = SimplePrismaticAxialThroughFeatureExport;

struct PrismaticBlindSectionModel {
  int top_face_index {};
  int support_bottom_face_index {-1};
  std::vector<int> top_outer_loop;
  std::vector<int> support_bottom_outer_loop;
  std::vector<PrismaticAxialSectionColumn> features;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBossExport {
  int base_face_index {};
  int top_face_index {};
  std::vector<int> base_outer_loop;
  double base_inner_area {};
  double top_area {};
  SimpleCylinderPrimitive boss {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBoss {
  double base_inner_area {};
  double top_area {};
  SimpleCylinderPrimitive boss {};
};

struct SimplePrismaticMultiBossExport {
  int bottom_face_index {};
  int base_face_index {};
  std::vector<int> bottom_outer_loop;
  std::vector<int> base_outer_loop;
  std::vector<SimplePrismaticMultiBoss> bosses;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticStandoffExport {
  int top_face_index {};
  int base_face_index {};
  int bottom_face_index {};
  std::vector<int> base_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_outer_area {};
  double top_inner_area {};
  double base_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive outer_cylinder {};
  SimpleCylinderPrimitive inner_cylinder {};
  std::vector<int> outer_face_indices;
};

struct PrismaticSupportSectionColumn {
  double base_inner_area {};
  double top_outer_area {};
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive outer_cylinder {};
  bool has_inner_cylinder {};
  SimpleCylinderPrimitive inner_cylinder {};
};

struct PrismaticSupportSectionModel {
  int base_face_index {};
  int support_bottom_face_index {-1};
  std::vector<int> base_outer_loop;
  std::vector<int> support_bottom_outer_loop;
  std::vector<PrismaticSupportSectionColumn> features;
  std::vector<int> outer_face_indices;
};

std::vector<double> projected_face_loop_areas(const ReconstructedFace& face,
                                              const std::vector<Vec3>& vertices,
                                              const PlaneBasis& basis) {
  std::vector<double> areas;
  areas.reserve(face.loops.size());
  for (const std::vector<int>& loop : face.loops) {
    areas.push_back(projected_loop_area(loop, vertices, basis));
  }
  return areas;
}

std::uint64_t make_face_loop_key(int face_index, std::size_t loop_index) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(face_index)) << 32U) |
         static_cast<std::uint64_t>(loop_index);
}

struct LoopSubsequenceMatch {
  std::size_t start {};
  std::size_t span {};
  bool forward {};
};

std::vector<int> ordered_open_chain_from_directed_edges(const std::vector<std::pair<int, int>>& edges) {
  std::unordered_map<int, std::vector<int>> outgoing;
  std::unordered_map<int, int> incoming_counts;
  for (const auto& edge : edges) {
    outgoing[edge.first].push_back(edge.second);
    incoming_counts[edge.second] += 1;
    incoming_counts.try_emplace(edge.first, 0);
  }

  int start_vertex = -1;
  for (const auto& entry : outgoing) {
    if (incoming_counts[entry.first] == 0) {
      start_vertex = entry.first;
      break;
    }
  }
  if (start_vertex < 0) {
    return {};
  }

  std::vector<int> chain;
  std::unordered_set<int> visited_vertices;
  chain.push_back(start_vertex);
  visited_vertices.insert(start_vertex);
  int current = start_vertex;
  while (true) {
    const auto outgoing_it = outgoing.find(current);
    if (outgoing_it == outgoing.end() || outgoing_it->second.empty()) {
      break;
    }
    if (outgoing_it->second.size() != 1) {
      return {};
    }
    const int next = outgoing_it->second.front();
    if (visited_vertices.contains(next)) {
      return {};
    }
    chain.push_back(next);
    visited_vertices.insert(next);
    current = next;
  }
  return chain;
}

std::optional<LoopSubsequenceMatch> find_loop_subsequence(const std::vector<int>& loop,
                                                          const std::vector<int>& chain) {
  if (loop.empty() || chain.size() < 2 || chain.size() > loop.size() + 1) {
    return std::nullopt;
  }

  const std::size_t loop_size = loop.size();
  const std::size_t chain_size = chain.size();
  for (std::size_t start = 0; start < loop_size; ++start) {
    bool forward_match = true;
    for (std::size_t index = 0; index < chain_size; ++index) {
      if (loop[(start + index) % loop_size] != chain[index]) {
        forward_match = false;
        break;
      }
    }
    if (forward_match) {
      return LoopSubsequenceMatch {start, chain_size - 1, true};
    }

    bool reverse_match = true;
    for (std::size_t index = 0; index < chain_size; ++index) {
      if (loop[(start + index) % loop_size] != chain[chain_size - 1 - index]) {
        reverse_match = false;
        break;
      }
    }
    if (reverse_match) {
      return LoopSubsequenceMatch {start, chain_size - 1, false};
    }
  }
  return std::nullopt;
}

std::vector<Vec2> project_chain_to_basis(const std::vector<int>& chain,
                                         const std::vector<Vec3>& vertices,
                                         const PlaneBasis& basis) {
  std::vector<Vec2> projected;
  projected.reserve(chain.size());
  for (int vertex_index : chain) {
    projected.push_back(project_to_basis(vertices[vertex_index], basis));
  }
  return projected;
}

double polyline_length_2d(const std::vector<Vec2>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double dx = points[index].x - points[index - 1].x;
    const double dy = points[index].y - points[index - 1].y;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

std::vector<Vec2> smooth_profile_points(const std::vector<Vec2>& points) {
  if (points.size() <= 4) {
    return points;
  }

  std::vector<Vec2> smoothed = points;
  for (std::size_t index = 1; index + 1 < points.size(); ++index) {
    smoothed[index].x =
        0.25 * points[index - 1].x + 0.5 * points[index].x + 0.25 * points[index + 1].x;
    smoothed[index].y =
        0.25 * points[index - 1].y + 0.5 * points[index].y + 0.25 * points[index + 1].y;
  }
  return smoothed;
}

std::vector<Vec2> resample_polyline_2d(const std::vector<Vec2>& points, std::size_t target_count) {
  if (points.size() < 2 || target_count <= 2) {
    return points;
  }

  std::vector<double> cumulative(points.size(), 0.0);
  for (std::size_t index = 1; index < points.size(); ++index) {
    const double dx = points[index].x - points[index - 1].x;
    const double dy = points[index].y - points[index - 1].y;
    cumulative[index] = cumulative[index - 1] + std::sqrt(dx * dx + dy * dy);
  }
  const double total_length = cumulative.back();
  if (total_length <= 1e-9) {
    return points;
  }

  std::vector<Vec2> resampled;
  resampled.reserve(target_count);
  for (std::size_t sample_index = 0; sample_index < target_count; ++sample_index) {
    const double target_length =
        total_length * static_cast<double>(sample_index) / static_cast<double>(target_count - 1);
    auto upper = std::lower_bound(cumulative.begin(), cumulative.end(), target_length);
    if (upper == cumulative.begin()) {
      resampled.push_back(points.front());
      continue;
    }
    if (upper == cumulative.end()) {
      resampled.push_back(points.back());
      continue;
    }

    const std::size_t upper_index = static_cast<std::size_t>(upper - cumulative.begin());
    const std::size_t lower_index = upper_index - 1;
    const double span = cumulative[upper_index] - cumulative[lower_index];
    const double t = span <= 1e-12 ? 0.0 : (target_length - cumulative[lower_index]) / span;
    resampled.push_back(Vec2 {
        points[lower_index].x * (1.0 - t) + points[upper_index].x * t,
        points[lower_index].y * (1.0 - t) + points[upper_index].y * t,
    });
  }
  return resampled;
}

double polyline_line_fit_error_2d(const std::vector<Vec2>& points) {
  if (points.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  double max_error = 0.0;
  for (const Vec2& point : points) {
    max_error = std::max(max_error, point_line_distance_2d(point, points.front(), points.back()));
  }
  return max_error;
}

#include "output_profile_sweep_detection.inc"

std::optional<std::size_t> outer_loop_index_from_areas(const std::vector<double>& areas);

struct CylinderShellFaceMatch {
  int top_vertices {};
  int bottom_vertices {};
  int intermediate_vertices {};
};

std::optional<CylinderShellFaceMatch> match_face_to_cylinder_shell(
    const ReconstructedFace& face,
    const std::vector<Vec3>& vertices,
    const Vec3& axis_origin,
    const Vec3& axis,
    double radius,
    double bottom_plane,
    double top_plane,
    double tolerance) {
  if (face.loops.size() != 1 || face.loops.front().size() < 3) {
    return std::nullopt;
  }

  CylinderShellFaceMatch match;
  std::unordered_set<int> unique_vertices;
  for (int vertex_index : face.loops.front()) {
    unique_vertices.insert(vertex_index);
  }

  for (int vertex_index : unique_vertices) {
    const Vec3& point = vertices[vertex_index];
    const double coordinate = dot(point, axis);
    if (coordinate < bottom_plane - tolerance || coordinate > top_plane + tolerance) {
      return std::nullopt;
    }

    const Vec3 center_point = axis_origin + axis * coordinate;
    const double radial_distance = length(point - center_point);
    if (std::abs(radial_distance - radius) > tolerance) {
      return std::nullopt;
    }

    if (std::abs(coordinate - top_plane) <= tolerance) {
      ++match.top_vertices;
    } else if (std::abs(coordinate - bottom_plane) <= tolerance) {
      ++match.bottom_vertices;
    } else {
      ++match.intermediate_vertices;
    }
  }

  const int strata =
      (match.top_vertices > 0 ? 1 : 0) + (match.bottom_vertices > 0 ? 1 : 0) +
      (match.intermediate_vertices > 0 ? 1 : 0);
  if (strata < 2) {
    return std::nullopt;
  }

  return match;
}

bool has_intermediate_matching_circular_section(const ReconstructionResult& reconstruction,
                                                int top_face_index,
                                                int bottom_face_index,
                                                const Vec3& axis,
                                                const PlaneBasis& axis_basis,
                                                double top_plane,
                                                double bottom_plane,
                                                const Vec2& center,
                                                double radius,
                                                double tolerance) {
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (static_cast<int>(face_index) == top_face_index ||
        static_cast<int>(face_index) == bottom_face_index) {
      continue;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (angle_degrees_unsigned(face.fit.normal, axis) > 8.0) {
      continue;
    }

    const double plane = dot(face.fit.centroid, axis);
    if (plane >= top_plane - tolerance || plane <= bottom_plane + tolerance) {
      continue;
    }

    auto loop_matches = [&](const std::vector<int>& loop) -> bool {
      if (loop.size() < 8) {
        return false;
      }
      const std::optional<CircleFit2D> circle =
          fit_circle_2d(project_loop_to_basis(loop, reconstruction.vertices, axis_basis));
      if (!circle.has_value()) {
        return false;
      }
      const double circle_tolerance = std::max((circle->radius + radius) * 0.015, 1e-4);
      const Vec2 center_delta {circle->center.x - center.x, circle->center.y - center.y};
      return std::abs(circle->radius - radius) <= circle_tolerance &&
             circle->max_error <= std::max(circle->radius * 0.03, 1e-4) &&
             std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) <=
                 circle_tolerance;
    };

    if (face.loops.size() == 1) {
      if (loop_matches(face.loops.front())) {
        return true;
      }
      continue;
    }

    const std::vector<double> areas =
        projected_face_loop_areas(face, reconstruction.vertices, axis_basis);
    const std::optional<std::size_t> outer_loop_index = outer_loop_index_from_areas(areas);
    if (!outer_loop_index.has_value()) {
      continue;
    }
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      if (loop_index == *outer_loop_index) {
        continue;
      }
      if (loop_matches(face.loops[loop_index])) {
        return true;
      }
    }
  }

  return false;
}

std::optional<std::size_t> outer_loop_index_from_areas(const std::vector<double>& areas) {
  if (areas.empty()) {
    return std::nullopt;
  }

  std::size_t outer_index = 0;
  double outer_area = areas[0];
  if (std::abs(outer_area) <= 1e-9) {
    return std::nullopt;
  }

  for (std::size_t index = 1; index < areas.size(); ++index) {
    if (std::abs(areas[index]) <= 1e-9) {
      return std::nullopt;
    }
    if (std::abs(areas[index]) > std::abs(outer_area)) {
      outer_area = areas[index];
      outer_index = index;
    }
  }

  return outer_index;
}

#include "output_cylindrical_detection.inc"

std::optional<SimplePrismaticBossExport> detect_simple_prismatic_boss_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 8 || reconstruction.vertices.size() < 16) {
    return std::nullopt;
  }

  std::vector<int> base_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      base_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (base_candidates.empty()) {
    return std::nullopt;
  }

  for (int base_face_index : base_candidates) {
    const ReconstructedFace& base_face = reconstruction.faces[base_face_index];
    Vec3 axis = normalized(base_face.fit.normal);
    if (length_squared(axis) <= 1e-12) {
      continue;
    }

    const PlaneBasis axis_basis = basis_from_normal(axis);
    const std::optional<ClassifiedFaceLoops> loops =
        classify_face_loops_by_area(base_face, reconstruction.vertices, axis_basis);
    if (!loops.has_value()) {
      continue;
    }

    const auto& [base_outer_loop, base_inner_loop, base_outer_area, base_inner_area] = *loops;
    (void)base_outer_area;
    if (base_inner_loop.size() < 8) {
      continue;
    }

    const std::optional<CircleFit2D> base_circle =
        fit_circle_2d(project_loop_to_basis(base_inner_loop, reconstruction.vertices, axis_basis));
    if (!base_circle.has_value() || base_circle->radius <= 1e-6) {
      continue;
    }

    const double radius_tolerance = std::max(base_circle->radius * 0.03, 1e-4);
    if (base_circle->max_error > radius_tolerance) {
      continue;
    }

    const double base_plane = dot(base_face.fit.centroid, axis);
    for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
      if (static_cast<int>(face_index) == base_face_index) {
        continue;
      }

      const ReconstructedFace& top_face = reconstruction.faces[face_index];
      if (top_face.loops.size() != 1 || top_face.loops.front().size() < 8) {
        continue;
      }
      if (angle_degrees_unsigned(base_face.fit.normal, top_face.fit.normal) > 8.0) {
        continue;
      }

      const std::vector<int>& top_loop = top_face.loops.front();
      const std::optional<CircleFit2D> top_circle =
          fit_circle_2d(project_loop_to_basis(top_loop, reconstruction.vertices, axis_basis));
      if (!top_circle.has_value()) {
        continue;
      }
      if (std::abs(top_circle->radius - base_circle->radius) > radius_tolerance ||
          top_circle->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 center_delta {top_circle->center.x - base_circle->center.x,
                               top_circle->center.y - base_circle->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          radius_tolerance) {
        continue;
      }

      const double top_plane = dot(top_face.fit.centroid, axis);
      if (top_plane <= base_plane + radius_tolerance) {
        continue;
      }
      const double height = top_plane - base_plane;
      if (height <= radius_tolerance) {
        continue;
      }

      std::unordered_set<int> cylinder_vertices;
      cylinder_vertices.insert(base_inner_loop.begin(), base_inner_loop.end());
      cylinder_vertices.insert(top_loop.begin(), top_loop.end());

      std::vector<int> outer_face_indices;
      int boss_face_count = 0;
      bool valid = true;
      for (std::size_t other_index = 0; other_index < reconstruction.faces.size(); ++other_index) {
        if (static_cast<int>(other_index) == base_face_index ||
            static_cast<int>(other_index) == static_cast<int>(face_index)) {
          continue;
        }
        const ReconstructedFace& face = reconstruction.faces[other_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 3) {
          valid = false;
          break;
        }

        bool all_cylinder = true;
        for (int vertex_index : face.loops.front()) {
          if (!cylinder_vertices.contains(vertex_index)) {
            all_cylinder = false;
            break;
          }
        }

        if (all_cylinder) {
          int base_vertices = 0;
          int top_vertices = 0;
          for (int vertex_index : face.loops.front()) {
            const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
            if (std::abs(coordinate - base_plane) <= radius_tolerance) {
              ++base_vertices;
            } else if (std::abs(coordinate - top_plane) <= radius_tolerance) {
              ++top_vertices;
            }
          }
          if (base_vertices != 2 || top_vertices != 2) {
            valid = false;
            break;
          }
          ++boss_face_count;
        } else {
          outer_face_indices.push_back(static_cast<int>(other_index));
        }
      }

      if (!valid || boss_face_count < 8 || outer_face_indices.size() < 5) {
        continue;
      }

      const Vec2 center {
          (base_circle->center.x + top_circle->center.x) * 0.5,
          (base_circle->center.y + top_circle->center.y) * 0.5,
      };
      const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
      const Vec3 base_center = axis_origin + axis * base_plane;
      const Vec3 seam_vector = reconstruction.vertices[base_inner_loop.front()] - base_center;
      const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
      if (length_squared(radial) <= 1e-12) {
        continue;
      }

      SimplePrismaticBossExport boss;
      boss.base_face_index = base_face_index;
      boss.top_face_index = static_cast<int>(face_index);
      boss.base_outer_loop = base_outer_loop;
      boss.base_inner_area = base_inner_area;
      boss.top_area = projected_loop_area(top_loop, reconstruction.vertices, axis_basis);
      boss.boss = SimpleCylinderPrimitive {base_center, axis, normalized(radial), base_circle->radius, height};
      boss.outer_face_indices = std::move(outer_face_indices);
      return boss;
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticMultiBossExport> detect_simple_prismatic_multi_boss_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> base_candidates;
  std::vector<int> bottom_candidates;
  std::vector<int> top_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() >= 3) {
      base_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() == 4) {
      bottom_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 8) {
      top_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (base_candidates.empty() || bottom_candidates.empty() || top_candidates.size() < 2) {
    return std::nullopt;
  }

  for (int base_face_index : base_candidates) {
    const ReconstructedFace& base_face = reconstruction.faces[base_face_index];
    Vec3 rough_axis = normalized(base_face.fit.normal);
    if (length_squared(rough_axis) <= 1e-12) {
      continue;
    }

    for (int bottom_face_index : bottom_candidates) {
      if (bottom_face_index == base_face_index) {
        continue;
      }

      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];
      if (angle_degrees_unsigned(base_face.fit.normal, bottom_face.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = rough_axis;
      if (dot(base_face.fit.centroid - bottom_face.fit.centroid, axis) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);
      const std::optional<ClassifiedFaceMultiLoops> base_loops =
          classify_face_loops_outer_inner(base_face, reconstruction.vertices, axis_basis);
      if (!base_loops.has_value() || base_loops->inner_loops.size() < 2) {
        continue;
      }

      const double base_plane = dot(base_face.fit.centroid, axis);
      const double bottom_plane = dot(bottom_face.fit.centroid, axis);
      if (base_plane <= bottom_plane + 1e-4) {
        continue;
      }

      const std::vector<int>& bottom_outer_loop = bottom_face.loops.front();
      std::vector<CircleFit2D> base_circles;
      base_circles.reserve(base_loops->inner_loops.size());
      bool fit_valid = true;
      for (const std::vector<int>& loop : base_loops->inner_loops) {
        const std::optional<CircleFit2D> circle =
            fit_circle_2d(project_loop_to_basis(loop, reconstruction.vertices, axis_basis));
        if (!circle.has_value() || circle->radius <= 1e-6) {
          fit_valid = false;
          break;
        }
        const double tolerance = std::max(circle->radius * 0.03, 1e-4);
        if (circle->max_error > tolerance) {
          fit_valid = false;
          break;
        }
        base_circles.push_back(*circle);
      }
      if (!fit_valid) {
        continue;
      }

      struct PendingBoss {
        std::size_t base_loop_index {};
        std::size_t top_face_index {};
        std::vector<int> top_loop;
        CircleFit2D top_circle {};
        double top_area {};
        double top_plane {};
      };

      std::vector<bool> matched_top(top_candidates.size(), false);
      std::vector<PendingBoss> pending;
      pending.reserve(base_loops->inner_loops.size());

      for (std::size_t base_index = 0; base_index < base_loops->inner_loops.size(); ++base_index) {
        const CircleFit2D& base_circle = base_circles[base_index];
        std::size_t best_top = top_candidates.size();
        double best_score = std::numeric_limits<double>::infinity();
        PendingBoss best_pending;
        for (std::size_t top_index = 0; top_index < top_candidates.size(); ++top_index) {
          if (matched_top[top_index]) {
            continue;
          }

          const ReconstructedFace& top_face = reconstruction.faces[top_candidates[top_index]];
          if (angle_degrees_unsigned(base_face.fit.normal, top_face.fit.normal) > 8.0) {
            continue;
          }
          const double top_plane = dot(top_face.fit.centroid, axis);
          if (top_plane <= base_plane + 1e-4) {
            continue;
          }
          const std::vector<int>& top_loop = top_face.loops.front();
          const std::optional<CircleFit2D> top_circle =
              fit_circle_2d(project_loop_to_basis(top_loop, reconstruction.vertices, axis_basis));
          if (!top_circle.has_value()) {
            continue;
          }

          const double tolerance = std::max((base_circle.radius + top_circle->radius) * 0.015, 1e-4);
          const Vec2 delta {base_circle.center.x - top_circle->center.x,
                            base_circle.center.y - top_circle->center.y};
          const double center_distance = std::sqrt(delta.x * delta.x + delta.y * delta.y);
          if (std::abs(base_circle.radius - top_circle->radius) > tolerance ||
              top_circle->max_error > tolerance || center_distance > tolerance) {
            continue;
          }
          const double score = center_distance + std::abs(base_circle.radius - top_circle->radius);
          if (score < best_score) {
            best_score = score;
            best_top = top_index;
            best_pending = PendingBoss {
                base_index,
                top_index,
                top_loop,
                *top_circle,
                projected_loop_area(top_loop, reconstruction.vertices, axis_basis),
                top_plane,
            };
          }
        }
        if (best_top == top_candidates.size()) {
          fit_valid = false;
          break;
        }
        matched_top[best_top] = true;
        pending.push_back(std::move(best_pending));
      }
      if (!fit_valid || pending.size() != base_loops->inner_loops.size()) {
        continue;
      }

      std::unordered_set<int> all_cylinder_vertices;
      std::vector<std::unordered_set<int>> cylinder_vertex_sets;
      cylinder_vertex_sets.reserve(pending.size());
      for (const PendingBoss& boss : pending) {
        std::unordered_set<int> cylinder_vertices;
        cylinder_vertices.insert(base_loops->inner_loops[boss.base_loop_index].begin(),
                                 base_loops->inner_loops[boss.base_loop_index].end());
        cylinder_vertices.insert(boss.top_loop.begin(), boss.top_loop.end());
        all_cylinder_vertices.insert(cylinder_vertices.begin(), cylinder_vertices.end());
        cylinder_vertex_sets.push_back(std::move(cylinder_vertices));
      }

      std::vector<int> outer_face_indices;
      std::vector<int> cylinder_face_counts(pending.size(), 0);
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == base_face_index ||
            static_cast<int>(face_index) == bottom_face_index) {
          continue;
        }

        bool is_top = false;
        for (const PendingBoss& boss : pending) {
          if (static_cast<int>(face_index) == top_candidates[boss.top_face_index]) {
            is_top = true;
            break;
          }
        }
        if (is_top) {
          continue;
        }

        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 3) {
          fit_valid = false;
          break;
        }

        bool all_cylinder = true;
        for (int vertex_index : face.loops.front()) {
          if (!all_cylinder_vertices.contains(vertex_index)) {
            all_cylinder = false;
            break;
          }
        }
        if (!all_cylinder) {
          outer_face_indices.push_back(static_cast<int>(face_index));
          continue;
        }

        std::size_t matched_boss = pending.size();
        for (std::size_t boss_index = 0; boss_index < pending.size(); ++boss_index) {
          bool belongs = true;
          for (int vertex_index : face.loops.front()) {
            if (!cylinder_vertex_sets[boss_index].contains(vertex_index)) {
              belongs = false;
              break;
            }
          }
          if (belongs) {
            matched_boss = boss_index;
            break;
          }
        }
        if (matched_boss == pending.size()) {
          fit_valid = false;
          break;
        }
        ++cylinder_face_counts[matched_boss];
      }

      if (!fit_valid || outer_face_indices.size() < 4) {
        continue;
      }
      if (std::any_of(cylinder_face_counts.begin(), cylinder_face_counts.end(),
                      [](int count) { return count < 8; })) {
        continue;
      }

      std::vector<SimplePrismaticMultiBoss> bosses;
      bosses.reserve(pending.size());
      for (const PendingBoss& pending_boss : pending) {
        const CircleFit2D& base_circle = base_circles[pending_boss.base_loop_index];
        const Vec2 center {
            0.5 * (base_circle.center.x + pending_boss.top_circle.center.x),
            0.5 * (base_circle.center.y + pending_boss.top_circle.center.y),
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const Vec3 base_center = axis_origin + axis * base_plane;
        const Vec3 seam_vector =
            reconstruction.vertices[base_loops->inner_loops[pending_boss.base_loop_index].front()] -
            base_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          fit_valid = false;
          break;
        }

        SimplePrismaticMultiBoss boss;
        boss.base_inner_area = base_loops->inner_areas[pending_boss.base_loop_index];
        boss.top_area = pending_boss.top_area;
        boss.boss = SimpleCylinderPrimitive {
            base_center,
            axis,
            normalized(radial),
            base_circle.radius,
            pending_boss.top_plane - base_plane,
        };
        bosses.push_back(std::move(boss));
      }
      if (!fit_valid) {
        continue;
      }

      SimplePrismaticMultiBossExport export_result;
      export_result.bottom_face_index = bottom_face_index;
      export_result.base_face_index = base_face_index;
      export_result.bottom_outer_loop = bottom_outer_loop;
      export_result.base_outer_loop = base_loops->outer_loop;
      export_result.bosses = std::move(bosses);
      export_result.outer_face_indices = std::move(outer_face_indices);
      return export_result;
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticStandoffExport> detect_simple_prismatic_standoff_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 9 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 3) {
    return std::nullopt;
  }

  for (int top_face_index : cap_candidates) {
    const ReconstructedFace& raw_top_face = reconstruction.faces[top_face_index];
    Vec3 rough_axis = normalized(raw_top_face.fit.normal);
    if (length_squared(rough_axis) <= 1e-12) {
      continue;
    }

    const PlaneBasis rough_basis = basis_from_normal(rough_axis);
    const std::optional<ClassifiedFaceLoops> rough_top_loops =
        classify_face_loops_by_area(raw_top_face, reconstruction.vertices, rough_basis);
    if (!rough_top_loops.has_value()) {
      continue;
    }

    const auto& [rough_top_outer_loop, rough_top_inner_loop, rough_top_outer_area, rough_top_inner_area] =
        *rough_top_loops;
    if (rough_top_outer_loop.size() < 8 || rough_top_inner_loop.size() < 8) {
      continue;
    }

    const std::optional<CircleFit2D> rough_top_outer_circle = fit_circle_2d(
        project_loop_to_basis(rough_top_outer_loop, reconstruction.vertices, rough_basis));
    const std::optional<CircleFit2D> rough_top_inner_circle = fit_circle_2d(
        project_loop_to_basis(rough_top_inner_loop, reconstruction.vertices, rough_basis));
    if (!rough_top_outer_circle.has_value() || !rough_top_inner_circle.has_value()) {
      continue;
    }

    const double initial_radius_tolerance =
        std::max(rough_top_outer_circle->radius * 0.03, 1e-4);
    if (rough_top_outer_circle->radius <= rough_top_inner_circle->radius + initial_radius_tolerance ||
        rough_top_outer_circle->max_error > initial_radius_tolerance ||
        rough_top_inner_circle->max_error > initial_radius_tolerance ||
        !circle_fit_centers_coincident(*rough_top_outer_circle,
                                       *rough_top_inner_circle,
                                       initial_radius_tolerance)) {
      continue;
    }

    for (int bottom_face_index : cap_candidates) {
      if (bottom_face_index == top_face_index) {
        continue;
      }

      const ReconstructedFace& raw_bottom_face = reconstruction.faces[bottom_face_index];
      if (angle_degrees_unsigned(raw_top_face.fit.normal, raw_bottom_face.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(raw_top_face.fit.normal);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(raw_top_face.fit.centroid, axis) < dot(raw_bottom_face.fit.centroid, axis)) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      const std::optional<ClassifiedFaceLoops> top_loops =
          classify_face_loops_by_area(raw_top_face, reconstruction.vertices, axis_basis);
      const std::optional<ClassifiedFaceLoops> bottom_loops =
          classify_face_loops_by_area(raw_bottom_face, reconstruction.vertices, axis_basis);
      if (!top_loops.has_value() || !bottom_loops.has_value()) {
        continue;
      }

      const auto& [top_outer_loop, top_inner_loop, top_outer_area, top_inner_area] = *top_loops;
      const auto& [bottom_outer_loop, bottom_inner_loop, bottom_outer_area, bottom_inner_area] =
          *bottom_loops;
      (void)bottom_outer_area;
      if (top_outer_loop.size() < 8 || top_inner_loop.size() < 8 || bottom_inner_loop.size() < 8) {
        continue;
      }

      const std::optional<CircleFit2D> top_outer_circle = fit_circle_2d(
          project_loop_to_basis(top_outer_loop, reconstruction.vertices, axis_basis));
      const std::optional<CircleFit2D> top_inner_circle = fit_circle_2d(
          project_loop_to_basis(top_inner_loop, reconstruction.vertices, axis_basis));
      const std::optional<CircleFit2D> bottom_inner_circle = fit_circle_2d(
          project_loop_to_basis(bottom_inner_loop, reconstruction.vertices, axis_basis));
      if (!top_outer_circle.has_value() || !top_inner_circle.has_value() ||
          !bottom_inner_circle.has_value()) {
        continue;
      }

      const double radius_tolerance = std::max(top_outer_circle->radius * 0.03, 1e-4);
      if (top_outer_circle->radius <= top_inner_circle->radius + radius_tolerance ||
          top_outer_circle->max_error > radius_tolerance ||
          top_inner_circle->max_error > radius_tolerance ||
          bottom_inner_circle->max_error > radius_tolerance ||
          !circle_fit_centers_coincident(*top_outer_circle, *top_inner_circle, radius_tolerance) ||
          !circle_fit_matches(*top_inner_circle, *bottom_inner_circle, radius_tolerance)) {
        continue;
      }

      const double top_plane = dot(raw_top_face.fit.centroid, axis);
      const double bottom_plane = dot(raw_bottom_face.fit.centroid, axis);
      if (top_plane <= bottom_plane + radius_tolerance) {
        continue;
      }

      for (int base_face_index : cap_candidates) {
        if (base_face_index == top_face_index || base_face_index == bottom_face_index) {
          continue;
        }

        const ReconstructedFace& raw_base_face = reconstruction.faces[base_face_index];
        if (angle_degrees_unsigned(raw_top_face.fit.normal, raw_base_face.fit.normal) > 8.0) {
          continue;
        }

        const double base_plane = dot(raw_base_face.fit.centroid, axis);
        if (base_plane <= bottom_plane + radius_tolerance ||
            base_plane >= top_plane - radius_tolerance) {
          continue;
        }

        const std::optional<ClassifiedFaceLoops> base_loops =
            classify_face_loops_by_area(raw_base_face, reconstruction.vertices, axis_basis);
        if (!base_loops.has_value()) {
          continue;
        }

        const auto& [base_outer_loop, base_inner_loop, base_outer_area, base_inner_area] =
            *base_loops;
        (void)base_outer_area;
        if (base_inner_loop.size() < 8) {
          continue;
        }

        const std::optional<CircleFit2D> base_inner_circle = fit_circle_2d(
            project_loop_to_basis(base_inner_loop, reconstruction.vertices, axis_basis));
        if (!base_inner_circle.has_value() || base_inner_circle->max_error > radius_tolerance ||
            !circle_fit_matches(*top_outer_circle, *base_inner_circle, radius_tolerance)) {
          continue;
        }

        std::unordered_set<int> outer_vertices;
        outer_vertices.insert(top_outer_loop.begin(), top_outer_loop.end());
        outer_vertices.insert(base_inner_loop.begin(), base_inner_loop.end());

        std::unordered_set<int> inner_vertices;
        inner_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
        inner_vertices.insert(bottom_inner_loop.begin(), bottom_inner_loop.end());

        std::vector<int> outer_face_indices;
        int outer_cylinder_face_count = 0;
        int inner_cylinder_face_count = 0;
        bool valid = true;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == base_face_index ||
              static_cast<int>(face_index) == bottom_face_index) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 3) {
            valid = false;
            break;
          }

          bool all_outer = true;
          bool all_inner = true;
          for (int vertex_index : face.loops.front()) {
            if (!outer_vertices.contains(vertex_index)) {
              all_outer = false;
            }
            if (!inner_vertices.contains(vertex_index)) {
              all_inner = false;
            }
          }

          if (all_outer) {
            int top_vertices = 0;
            int base_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= radius_tolerance) {
                ++top_vertices;
              } else if (std::abs(coordinate - base_plane) <= radius_tolerance) {
                ++base_vertices;
              }
            }
            if (top_vertices != 2 || base_vertices != 2) {
              valid = false;
              break;
            }
            ++outer_cylinder_face_count;
          } else if (all_inner) {
            int top_vertices = 0;
            int bottom_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= radius_tolerance) {
                ++top_vertices;
              } else if (std::abs(coordinate - bottom_plane) <= radius_tolerance) {
                ++bottom_vertices;
              }
            }
            if (top_vertices != 2 || bottom_vertices != 2) {
              valid = false;
              break;
            }
            ++inner_cylinder_face_count;
          } else {
            outer_face_indices.push_back(static_cast<int>(face_index));
          }
        }

        if (!valid || outer_cylinder_face_count < 8 || inner_cylinder_face_count < 8 ||
            outer_face_indices.size() < 4) {
          continue;
        }

        const Vec2 averaged_center {
            (top_outer_circle->center.x + top_inner_circle->center.x + base_inner_circle->center.x +
             bottom_inner_circle->center.x) * 0.25,
            (top_outer_circle->center.y + top_inner_circle->center.y + base_inner_circle->center.y +
             bottom_inner_circle->center.y) * 0.25,
        };
        const Vec3 axis_origin = axis_basis.u * averaged_center.x + axis_basis.v * averaged_center.y;
        const Vec3 outer_base_center = axis_origin + axis * base_plane;
        const Vec3 outer_seam_vector = reconstruction.vertices[base_inner_loop.front()] - outer_base_center;
        const Vec3 outer_radial = outer_seam_vector - axis * dot(outer_seam_vector, axis);
        const Vec3 inner_base_center = axis_origin + axis * bottom_plane;
        const Vec3 inner_seam_vector = reconstruction.vertices[bottom_inner_loop.front()] - inner_base_center;
        const Vec3 inner_radial = inner_seam_vector - axis * dot(inner_seam_vector, axis);
        if (length_squared(outer_radial) <= 1e-12 || length_squared(inner_radial) <= 1e-12) {
          continue;
        }

        SimplePrismaticStandoffExport standoff;
        standoff.top_face_index = top_face_index;
        standoff.base_face_index = base_face_index;
        standoff.bottom_face_index = bottom_face_index;
        standoff.base_outer_loop = base_outer_loop;
        standoff.bottom_outer_loop = bottom_outer_loop;
        standoff.top_outer_area = top_outer_area;
        standoff.top_inner_area = top_inner_area;
        standoff.base_inner_area = base_inner_area;
        standoff.bottom_inner_area = bottom_inner_area;
        standoff.outer_cylinder = SimpleCylinderPrimitive {
            outer_base_center,
            axis,
            normalized(outer_radial),
            top_outer_circle->radius,
            top_plane - base_plane,
        };
        standoff.inner_cylinder = SimpleCylinderPrimitive {
            inner_base_center,
            axis,
            normalized(inner_radial),
            top_inner_circle->radius,
            top_plane - bottom_plane,
        };
        standoff.outer_face_indices = std::move(outer_face_indices);
        return standoff;
      }
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticBlindCounterboreExport>
detect_simple_prismatic_blind_counterbore_export(const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  std::vector<int> floor_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 8) {
      floor_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2 || floor_candidates.empty()) {
    return std::nullopt;
  }

  for (int top_face_index : cap_candidates) {
    const ReconstructedFace& raw_top_face = reconstruction.faces[top_face_index];
    Vec3 axis = normalized(raw_top_face.fit.normal);
    if (length_squared(axis) <= 1e-12) {
      continue;
    }
    const PlaneBasis axis_basis = basis_from_normal(axis);
    const std::optional<ClassifiedFaceLoops> top_loops =
        classify_face_loops_by_area(raw_top_face, reconstruction.vertices, axis_basis);
    if (!top_loops.has_value()) {
      continue;
    }

    const auto& [top_outer_loop, top_inner_loop, top_outer_area, top_inner_area] = *top_loops;
    (void)top_outer_area;
    if (top_inner_loop.size() < 8) {
      continue;
    }

    const std::optional<CircleFit2D> top_inner_circle = fit_circle_2d(
        project_loop_to_basis(top_inner_loop, reconstruction.vertices, axis_basis));
    if (!top_inner_circle.has_value() || top_inner_circle->radius <= 1e-6) {
      continue;
    }
    const double radius_tolerance = std::max(top_inner_circle->radius * 0.03, 1e-4);
    if (top_inner_circle->max_error > radius_tolerance) {
      continue;
    }

    const double top_plane = dot(raw_top_face.fit.centroid, axis);
    for (int step_face_index : cap_candidates) {
      if (step_face_index == top_face_index) {
        continue;
      }
      const ReconstructedFace& raw_step_face = reconstruction.faces[step_face_index];
      if (angle_degrees_unsigned(raw_top_face.fit.normal, raw_step_face.fit.normal) > 8.0) {
        continue;
      }

      const double step_plane = dot(raw_step_face.fit.centroid, axis);
      if (step_plane >= top_plane - radius_tolerance) {
        continue;
      }

      const std::optional<ClassifiedFaceLoops> step_loops =
          classify_face_loops_by_area(raw_step_face, reconstruction.vertices, axis_basis);
      if (!step_loops.has_value()) {
        continue;
      }

      const auto& [step_outer_loop, step_inner_loop, step_outer_area, step_inner_area] = *step_loops;
      if (step_outer_loop.size() < 8 || step_inner_loop.size() < 8) {
        continue;
      }

      const std::optional<CircleFit2D> step_outer_circle = fit_circle_2d(
          project_loop_to_basis(step_outer_loop, reconstruction.vertices, axis_basis));
      const std::optional<CircleFit2D> step_inner_circle = fit_circle_2d(
          project_loop_to_basis(step_inner_loop, reconstruction.vertices, axis_basis));
      if (!step_outer_circle.has_value() || !step_inner_circle.has_value()) {
        continue;
      }
      if (step_outer_circle->radius <= step_inner_circle->radius + radius_tolerance ||
          !circle_fit_matches(*top_inner_circle, *step_outer_circle, radius_tolerance) ||
          step_inner_circle->max_error > radius_tolerance) {
        continue;
      }

      for (int floor_face_index : floor_candidates) {
        if (floor_face_index == top_face_index || floor_face_index == step_face_index) {
          continue;
        }

        const ReconstructedFace& raw_floor_face = reconstruction.faces[floor_face_index];
        if (angle_degrees_unsigned(raw_top_face.fit.normal, raw_floor_face.fit.normal) > 8.0) {
          continue;
        }

        const double floor_plane = dot(raw_floor_face.fit.centroid, axis);
        if (floor_plane >= step_plane - radius_tolerance) {
          continue;
        }

        const std::vector<int>& floor_loop = raw_floor_face.loops.front();
        const std::optional<CircleFit2D> floor_circle = fit_circle_2d(
            project_loop_to_basis(floor_loop, reconstruction.vertices, axis_basis));
        if (!floor_circle.has_value() ||
            !circle_fit_matches(*step_inner_circle, *floor_circle, radius_tolerance)) {
          continue;
        }

        std::unordered_set<int> upper_vertices;
        upper_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
        upper_vertices.insert(step_outer_loop.begin(), step_outer_loop.end());

        std::unordered_set<int> lower_vertices;
        lower_vertices.insert(step_inner_loop.begin(), step_inner_loop.end());
        lower_vertices.insert(floor_loop.begin(), floor_loop.end());

        std::vector<int> outer_face_indices;
        int upper_face_count = 0;
        int lower_face_count = 0;
        bool valid = true;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == step_face_index ||
              static_cast<int>(face_index) == floor_face_index) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 3) {
            valid = false;
            break;
          }

          bool all_upper = true;
          bool all_lower = true;
          for (int vertex_index : face.loops.front()) {
            if (!upper_vertices.contains(vertex_index)) {
              all_upper = false;
            }
            if (!lower_vertices.contains(vertex_index)) {
              all_lower = false;
            }
          }

          if (all_upper) {
            int top_vertices = 0;
            int step_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= radius_tolerance) {
                ++top_vertices;
              } else if (std::abs(coordinate - step_plane) <= radius_tolerance) {
                ++step_vertices;
              }
            }
            if (top_vertices != 2 || step_vertices != 2) {
              valid = false;
              break;
            }
            ++upper_face_count;
          } else if (all_lower) {
            int step_vertices = 0;
            int floor_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - step_plane) <= radius_tolerance) {
                ++step_vertices;
              } else if (std::abs(coordinate - floor_plane) <= radius_tolerance) {
                ++floor_vertices;
              }
            }
            if (step_vertices != 2 || floor_vertices != 2) {
              valid = false;
              break;
            }
            ++lower_face_count;
          } else {
            outer_face_indices.push_back(static_cast<int>(face_index));
          }
        }

        if (!valid || upper_face_count < 8 || lower_face_count < 8 ||
            outer_face_indices.size() < 5) {
          continue;
        }

        const Vec2 center {
            (top_inner_circle->center.x + step_outer_circle->center.x + step_inner_circle->center.x +
             floor_circle->center.x) * 0.25,
            (top_inner_circle->center.y + step_outer_circle->center.y + step_inner_circle->center.y +
             floor_circle->center.y) * 0.25,
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;

        const Vec3 upper_base_center = axis_origin + axis * step_plane;
        const Vec3 upper_seam_vector =
            reconstruction.vertices[step_outer_loop.front()] - upper_base_center;
        const Vec3 upper_radial = upper_seam_vector - axis * dot(upper_seam_vector, axis);
        const Vec3 lower_base_center = axis_origin + axis * floor_plane;
        const Vec3 lower_seam_vector =
            reconstruction.vertices[floor_loop.front()] - lower_base_center;
        const Vec3 lower_radial = lower_seam_vector - axis * dot(lower_seam_vector, axis);
        if (length_squared(upper_radial) <= 1e-12 || length_squared(lower_radial) <= 1e-12) {
          continue;
        }

        SimplePrismaticBlindCounterboreExport counterbore;
        counterbore.top_face_index = top_face_index;
        counterbore.step_face_index = step_face_index;
        counterbore.floor_face_index = floor_face_index;
        counterbore.top_outer_loop = top_outer_loop;
        counterbore.top_inner_area = top_inner_area;
        counterbore.step_outer_area = step_outer_area;
        counterbore.step_inner_area = step_inner_area;
        counterbore.floor_area = projected_loop_area(floor_loop, reconstruction.vertices, axis_basis);
        counterbore.upper_bore = SimpleCylinderPrimitive {
            upper_base_center,
            axis,
            normalized(upper_radial),
            top_inner_circle->radius,
            top_plane - step_plane,
        };
        counterbore.lower_bore = SimpleCylinderPrimitive {
            lower_base_center,
            axis,
            normalized(lower_radial),
            step_inner_circle->radius,
            step_plane - floor_plane,
        };
        counterbore.outer_face_indices = std::move(outer_face_indices);
        return counterbore;
      }
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticBlindCountersinkExport>
detect_simple_prismatic_blind_countersink_export(const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 9 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  std::vector<int> floor_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 8) {
      floor_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.empty() || floor_candidates.empty()) {
    return std::nullopt;
  }

  for (int top_face_index : cap_candidates) {
    const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
    Vec3 axis = normalized(top_face.fit.normal);
    if (length_squared(axis) <= 1e-12) {
      continue;
    }

    const PlaneBasis axis_basis = basis_from_normal(axis);
    const std::optional<ClassifiedFaceLoops> top_loops =
        classify_face_loops_by_area(top_face, reconstruction.vertices, axis_basis);
    if (!top_loops.has_value()) {
      continue;
    }

    const auto& [top_outer_loop, top_inner_loop, top_outer_area, top_inner_area] = *top_loops;
    (void)top_outer_area;
    if (top_inner_loop.size() < 8) {
      continue;
    }

    const std::optional<CircleFit2D> top_circle =
        fit_circle_2d(project_loop_to_basis(top_inner_loop, reconstruction.vertices, axis_basis));
    if (!top_circle.has_value() || top_circle->radius <= 1e-6) {
      continue;
    }

    const double radius_tolerance = std::max(top_circle->radius * 0.03, 1e-4);
    if (top_circle->max_error > radius_tolerance) {
      continue;
    }

    const double top_plane = dot(top_face.fit.centroid, axis);
    for (int floor_face_index : floor_candidates) {
      if (floor_face_index == top_face_index) {
        continue;
      }

      const ReconstructedFace& floor_face = reconstruction.faces[floor_face_index];
      if (angle_degrees_unsigned(top_face.fit.normal, floor_face.fit.normal) > 8.0) {
        continue;
      }

      const std::vector<int>& floor_loop = floor_face.loops.front();
      const std::optional<CircleFit2D> floor_circle =
          fit_circle_2d(project_loop_to_basis(floor_loop, reconstruction.vertices, axis_basis));
      if (!floor_circle.has_value() || floor_circle->radius <= 1e-6) {
        continue;
      }

      if (top_circle->radius <= floor_circle->radius + radius_tolerance ||
          floor_circle->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 center_delta {floor_circle->center.x - top_circle->center.x,
                               floor_circle->center.y - top_circle->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          radius_tolerance) {
        continue;
      }

      const double floor_plane = dot(floor_face.fit.centroid, axis);
      if (floor_plane >= top_plane - radius_tolerance) {
        continue;
      }

      const double plane_bucket_tolerance = std::max(radius_tolerance * 0.5, 1e-4);
      std::map<long long, std::vector<int>> plane_buckets;
      for (std::size_t vertex_index = 0; vertex_index < reconstruction.vertices.size(); ++vertex_index) {
        const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
        if (coordinate >= top_plane - plane_bucket_tolerance ||
            coordinate <= floor_plane + plane_bucket_tolerance) {
          continue;
        }
        const long long bucket =
            static_cast<long long>(std::llround(coordinate / plane_bucket_tolerance));
        plane_buckets[bucket].push_back(static_cast<int>(vertex_index));
      }

      for (const auto& [bucket, bucket_vertices] : plane_buckets) {
        if (bucket_vertices.size() < 8) {
          continue;
        }

        std::unordered_set<int> unique_step_vertices(bucket_vertices.begin(), bucket_vertices.end());
        if (unique_step_vertices.size() < 8) {
          continue;
        }

        const double step_plane = static_cast<double>(bucket) * plane_bucket_tolerance;
        if (step_plane <= floor_plane + radius_tolerance ||
            step_plane >= top_plane - radius_tolerance) {
          continue;
        }

        std::vector<Vec2> step_points;
        step_points.reserve(unique_step_vertices.size());
        for (int vertex_index : unique_step_vertices) {
          step_points.push_back(project_to_basis(reconstruction.vertices[vertex_index], axis_basis));
        }
        const std::optional<CircleFit2D> step_circle = fit_circle_2d(step_points);
        if (!step_circle.has_value()) {
          continue;
        }

        if (std::abs(step_circle->radius - floor_circle->radius) > radius_tolerance ||
            step_circle->max_error > radius_tolerance ||
            !circle_fit_centers_coincident(*step_circle, *floor_circle, radius_tolerance)) {
          continue;
        }

        std::unordered_set<int> upper_vertices;
        upper_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
        upper_vertices.insert(unique_step_vertices.begin(), unique_step_vertices.end());

        std::unordered_set<int> lower_vertices;
        lower_vertices.insert(unique_step_vertices.begin(), unique_step_vertices.end());
        lower_vertices.insert(floor_loop.begin(), floor_loop.end());

        std::vector<int> outer_face_indices;
        int upper_face_count = 0;
        int lower_face_count = 0;
        bool valid = true;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == floor_face_index) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 3) {
            valid = false;
            break;
          }

          bool all_upper = true;
          bool all_lower = true;
          for (int vertex_index : face.loops.front()) {
            if (!upper_vertices.contains(vertex_index)) {
              all_upper = false;
            }
            if (!lower_vertices.contains(vertex_index)) {
              all_lower = false;
            }
          }

          if (all_upper) {
            int top_vertices = 0;
            int step_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= plane_bucket_tolerance) {
                ++top_vertices;
              } else if (std::abs(coordinate - step_plane) <= plane_bucket_tolerance) {
                ++step_vertices;
              }
            }
            if (top_vertices != 2 || step_vertices != 2) {
              valid = false;
              break;
            }
            ++upper_face_count;
          } else if (all_lower) {
            int step_vertices = 0;
            int floor_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - step_plane) <= plane_bucket_tolerance) {
                ++step_vertices;
              } else if (std::abs(coordinate - floor_plane) <= plane_bucket_tolerance) {
                ++floor_vertices;
              }
            }
            if (step_vertices != 2 || floor_vertices != 2) {
              valid = false;
              break;
            }
            ++lower_face_count;
          } else {
            outer_face_indices.push_back(static_cast<int>(face_index));
          }
        }

        if (!valid || upper_face_count < 8 || lower_face_count < 8 ||
            outer_face_indices.size() < 5) {
          continue;
        }

        const Vec2 center {
            (top_circle->center.x + floor_circle->center.x + step_circle->center.x) / 3.0,
            (top_circle->center.y + floor_circle->center.y + step_circle->center.y) / 3.0,
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const Vec3 step_center = axis_origin + axis * step_plane;
        const Vec3 floor_center = axis_origin + axis * floor_plane;

        const int reference_vertex = *unique_step_vertices.begin();
        const Vec3 seam_vector = reconstruction.vertices[reference_vertex] - step_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          continue;
        }

        SimplePrismaticBlindCountersinkExport countersink;
        countersink.top_face_index = top_face_index;
        countersink.floor_face_index = floor_face_index;
        countersink.top_outer_loop = top_outer_loop;
        countersink.top_inner_area = top_inner_area;
        countersink.floor_area = projected_loop_area(floor_loop, reconstruction.vertices, axis_basis);
        countersink.countersink = SimpleConicalFrustumPrimitive {
            step_center,
            axis,
            normalized(radial),
            floor_circle->radius,
            top_circle->radius,
            top_plane - step_plane,
        };
        countersink.bore = SimpleCylinderPrimitive {
            floor_center,
            axis,
            normalized(radial),
            floor_circle->radius,
            step_plane - floor_plane,
        };
        countersink.outer_face_indices = std::move(outer_face_indices);
        return countersink;
      }
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticBlindBoreExport> detect_simple_prismatic_blind_bore_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 8 || reconstruction.vertices.size() < 16) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.empty()) {
    return std::nullopt;
  }

  for (int top_face_index : cap_candidates) {
    const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
    Vec3 axis = normalized(top_face.fit.normal);
    if (length_squared(axis) <= 1e-12) {
      continue;
    }

    const PlaneBasis axis_basis = basis_from_normal(axis);
    const std::optional<ClassifiedFaceLoops> loops =
        classify_face_loops_by_area(top_face, reconstruction.vertices, axis_basis);
    if (!loops.has_value()) {
      continue;
    }

    const auto& [top_outer_loop, top_inner_loop, top_outer_area, top_inner_area] = *loops;
    (void)top_outer_area;
    if (top_inner_loop.size() < 8) {
      continue;
    }

    const std::optional<CircleFit2D> top_circle =
        fit_circle_2d(project_loop_to_basis(top_inner_loop, reconstruction.vertices, axis_basis));
    if (!top_circle.has_value() || top_circle->radius <= 1e-6) {
      continue;
    }

    const double radius_tolerance = std::max(top_circle->radius * 0.03, 1e-4);
    if (top_circle->max_error > radius_tolerance) {
      continue;
    }

    const double top_plane = dot(top_face.fit.centroid, axis);
    for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
      if (static_cast<int>(face_index) == top_face_index) {
        continue;
      }

      const ReconstructedFace& floor_face = reconstruction.faces[face_index];
      if (floor_face.loops.size() != 1 || floor_face.loops.front().size() < 8) {
        continue;
      }
      if (angle_degrees_unsigned(top_face.fit.normal, floor_face.fit.normal) > 8.0) {
        continue;
      }

      const std::vector<int>& floor_loop = floor_face.loops.front();
      const std::optional<CircleFit2D> floor_circle =
          fit_circle_2d(project_loop_to_basis(floor_loop, reconstruction.vertices, axis_basis));
      if (!floor_circle.has_value()) {
        continue;
      }
      if (std::abs(floor_circle->radius - top_circle->radius) > radius_tolerance ||
          floor_circle->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 center_delta {floor_circle->center.x - top_circle->center.x,
                               floor_circle->center.y - top_circle->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          radius_tolerance) {
        continue;
      }

      const double floor_plane = dot(floor_face.fit.centroid, axis);
      if (floor_plane >= top_plane - radius_tolerance) {
        continue;
      }
      const double height = top_plane - floor_plane;
      if (height <= radius_tolerance) {
        continue;
      }

      std::unordered_set<int> inner_vertices;
      inner_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
      inner_vertices.insert(floor_loop.begin(), floor_loop.end());

      std::vector<int> outer_face_indices;
      int bore_face_count = 0;
      bool valid = true;
      for (std::size_t other_index = 0; other_index < reconstruction.faces.size(); ++other_index) {
        if (static_cast<int>(other_index) == top_face_index ||
            static_cast<int>(other_index) == static_cast<int>(face_index)) {
          continue;
        }
        const ReconstructedFace& face = reconstruction.faces[other_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 3) {
          valid = false;
          break;
        }

        bool all_inner = true;
        for (int vertex_index : face.loops.front()) {
          if (!inner_vertices.contains(vertex_index)) {
            all_inner = false;
            break;
          }
        }

        if (all_inner) {
          int top_vertices = 0;
          int floor_vertices = 0;
          for (int vertex_index : face.loops.front()) {
            const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
            if (std::abs(coordinate - top_plane) <= radius_tolerance) {
              ++top_vertices;
            } else if (std::abs(coordinate - floor_plane) <= radius_tolerance) {
              ++floor_vertices;
            }
          }
          if (top_vertices != 2 || floor_vertices != 2) {
            valid = false;
            break;
          }
          ++bore_face_count;
        } else {
          outer_face_indices.push_back(static_cast<int>(other_index));
        }
      }

      if (!valid || bore_face_count < 8 || outer_face_indices.size() < 5) {
        continue;
      }

      const Vec2 center {
          (top_circle->center.x + floor_circle->center.x) * 0.5,
          (top_circle->center.y + floor_circle->center.y) * 0.5,
      };
      const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
      const Vec3 base_center = axis_origin + axis * floor_plane;
      const Vec3 seam_vector = reconstruction.vertices[floor_loop.front()] - base_center;
      const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
      if (length_squared(radial) <= 1e-12) {
        continue;
      }

      SimplePrismaticBlindBoreExport bore;
      bore.top_face_index = top_face_index;
      bore.floor_face_index = static_cast<int>(face_index);
      bore.top_outer_loop = top_outer_loop;
      bore.floor_loop = floor_loop;
      bore.top_inner_area = top_inner_area;
      bore.floor_area = projected_loop_area(floor_loop, reconstruction.vertices, axis_basis);
      bore.bore = SimpleCylinderPrimitive {base_center, axis, normalized(radial), top_circle->radius, height};
      bore.outer_face_indices = std::move(outer_face_indices);
      return bore;
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticMultiBlindBoreExport> detect_simple_prismatic_multi_blind_bore_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> top_candidates;
  std::vector<int> bottom_candidates;
  std::vector<int> floor_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() >= 3) {
      top_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 4) {
      bottom_candidates.push_back(static_cast<int>(face_index));
      if (face.loops.front().size() >= 8) {
        floor_candidates.push_back(static_cast<int>(face_index));
      }
    }
  }
  if (top_candidates.empty() || bottom_candidates.empty() || floor_candidates.size() < 2) {
    return std::nullopt;
  }

  for (int top_face_index : top_candidates) {
    const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
    Vec3 rough_axis = normalized(top_face.fit.normal);
    if (length_squared(rough_axis) <= 1e-12) {
      continue;
    }

    for (int bottom_face_index : bottom_candidates) {
      if (bottom_face_index == top_face_index) {
        continue;
      }

      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];
      if (angle_degrees_unsigned(top_face.fit.normal, bottom_face.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = rough_axis;
      if (dot(top_face.fit.centroid - bottom_face.fit.centroid, axis) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);
      const std::optional<ClassifiedFaceMultiLoops> top_loops =
          classify_face_loops_outer_inner(top_face, reconstruction.vertices, axis_basis);
      if (!top_loops.has_value() || top_loops->inner_loops.size() < 2) {
        continue;
      }

      const double top_plane = dot(top_face.fit.centroid, axis);
      const double bottom_plane = dot(bottom_face.fit.centroid, axis);
      if (top_plane <= bottom_plane + 1e-4) {
        continue;
      }

      const std::vector<int>& bottom_outer_loop = bottom_face.loops.front();
      std::vector<CircleFit2D> top_circles;
      top_circles.reserve(top_loops->inner_loops.size());
      bool fit_valid = true;
      for (const std::vector<int>& loop : top_loops->inner_loops) {
        const std::optional<CircleFit2D> circle =
            fit_circle_2d(project_loop_to_basis(loop, reconstruction.vertices, axis_basis));
        if (!circle.has_value() || circle->radius <= 1e-6) {
          fit_valid = false;
          break;
        }
        const double tolerance = std::max(circle->radius * 0.03, 1e-4);
        if (circle->max_error > tolerance) {
          fit_valid = false;
          break;
        }
        top_circles.push_back(*circle);
      }
      if (!fit_valid) {
        continue;
      }

      struct PendingBore {
        std::size_t top_loop_index {};
        int floor_face_index {};
        std::vector<int> floor_loop;
        CircleFit2D floor_circle {};
        double floor_area {};
        double floor_plane {};
      };

      std::vector<bool> matched_floor(reconstruction.faces.size(), false);
      std::vector<PendingBore> pending;
      pending.reserve(top_loops->inner_loops.size());

      for (std::size_t top_index = 0; top_index < top_loops->inner_loops.size(); ++top_index) {
        const CircleFit2D& top_circle = top_circles[top_index];
        int best_floor_face = -1;
        double best_score = std::numeric_limits<double>::infinity();
        PendingBore best_pending;
        for (int floor_face_index : floor_candidates) {
          if (matched_floor[floor_face_index] ||
              floor_face_index == top_face_index ||
              floor_face_index == bottom_face_index) {
            continue;
          }

          const ReconstructedFace& floor_face = reconstruction.faces[floor_face_index];
          if (angle_degrees_unsigned(top_face.fit.normal, floor_face.fit.normal) > 8.0) {
            continue;
          }
          const double floor_plane = dot(floor_face.fit.centroid, axis);
          if (floor_plane >= top_plane - 1e-4 || floor_plane <= bottom_plane + 1e-4) {
            continue;
          }
          const std::vector<int>& floor_loop = floor_face.loops.front();
          const std::optional<CircleFit2D> floor_circle =
              fit_circle_2d(project_loop_to_basis(floor_loop, reconstruction.vertices, axis_basis));
          if (!floor_circle.has_value()) {
            continue;
          }

          const double tolerance = std::max((top_circle.radius + floor_circle->radius) * 0.015, 1e-4);
          const Vec2 delta {top_circle.center.x - floor_circle->center.x,
                            top_circle.center.y - floor_circle->center.y};
          const double center_distance = std::sqrt(delta.x * delta.x + delta.y * delta.y);
          if (std::abs(top_circle.radius - floor_circle->radius) > tolerance ||
              floor_circle->max_error > tolerance || center_distance > tolerance) {
            continue;
          }
          const double score = center_distance + std::abs(top_circle.radius - floor_circle->radius);
          if (score < best_score) {
            best_score = score;
            best_floor_face = floor_face_index;
            best_pending = PendingBore {
                top_index,
                floor_face_index,
                floor_loop,
                *floor_circle,
                projected_loop_area(floor_loop, reconstruction.vertices, axis_basis),
                floor_plane,
            };
          }
        }

        if (best_floor_face < 0) {
          fit_valid = false;
          break;
        }
        matched_floor[best_floor_face] = true;
        pending.push_back(std::move(best_pending));
      }
      if (!fit_valid || pending.size() != top_loops->inner_loops.size()) {
        continue;
      }

      std::unordered_set<int> all_inner_vertices;
      std::vector<std::unordered_set<int>> inner_vertex_sets;
      inner_vertex_sets.reserve(pending.size());
      for (const PendingBore& bore : pending) {
        std::unordered_set<int> inner_vertices;
        inner_vertices.insert(top_loops->inner_loops[bore.top_loop_index].begin(),
                              top_loops->inner_loops[bore.top_loop_index].end());
        inner_vertices.insert(bore.floor_loop.begin(), bore.floor_loop.end());
        all_inner_vertices.insert(inner_vertices.begin(), inner_vertices.end());
        inner_vertex_sets.push_back(std::move(inner_vertices));
      }

      std::vector<int> outer_face_indices;
      std::vector<int> bore_face_counts(pending.size(), 0);
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == top_face_index ||
            static_cast<int>(face_index) == bottom_face_index) {
          continue;
        }

        bool is_floor = false;
        for (const PendingBore& bore : pending) {
          if (static_cast<int>(face_index) == bore.floor_face_index) {
            is_floor = true;
            break;
          }
        }
        if (is_floor) {
          continue;
        }

        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 3) {
          fit_valid = false;
          break;
        }

        bool all_inner = true;
        for (int vertex_index : face.loops.front()) {
          if (!all_inner_vertices.contains(vertex_index)) {
            all_inner = false;
            break;
          }
        }
        if (!all_inner) {
          outer_face_indices.push_back(static_cast<int>(face_index));
          continue;
        }

        std::size_t matched_bore = pending.size();
        for (std::size_t bore_index = 0; bore_index < pending.size(); ++bore_index) {
          bool belongs = true;
          for (int vertex_index : face.loops.front()) {
            if (!inner_vertex_sets[bore_index].contains(vertex_index)) {
              belongs = false;
              break;
            }
          }
          if (belongs) {
            matched_bore = bore_index;
            break;
          }
        }
        if (matched_bore == pending.size()) {
          fit_valid = false;
          break;
        }
        ++bore_face_counts[matched_bore];
      }

      if (!fit_valid || outer_face_indices.size() < 4) {
        continue;
      }
      if (std::any_of(bore_face_counts.begin(), bore_face_counts.end(),
                      [](int count) { return count < 8; })) {
        continue;
      }

      std::vector<SimplePrismaticMultiBlindBore> bores;
      bores.reserve(pending.size());
      for (const PendingBore& pending_bore : pending) {
        const CircleFit2D& top_circle = top_circles[pending_bore.top_loop_index];
        const Vec2 center {
            0.5 * (top_circle.center.x + pending_bore.floor_circle.center.x),
            0.5 * (top_circle.center.y + pending_bore.floor_circle.center.y),
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const Vec3 base_center = axis_origin + axis * pending_bore.floor_plane;
        const Vec3 seam_vector =
            reconstruction.vertices[top_loops->inner_loops[pending_bore.top_loop_index].front()] - base_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          fit_valid = false;
          break;
        }

        SimplePrismaticMultiBlindBore bore;
        bore.top_inner_area = top_loops->inner_areas[pending_bore.top_loop_index];
        bore.floor_area = pending_bore.floor_area;
        bore.bore = SimpleCylinderPrimitive {
            base_center,
            axis,
            normalized(radial),
            top_circle.radius,
            top_plane - pending_bore.floor_plane,
        };
        bores.push_back(std::move(bore));
      }
      if (!fit_valid) {
        continue;
      }

      SimplePrismaticMultiBlindBoreExport export_result;
      export_result.top_face_index = top_face_index;
      export_result.bottom_face_index = bottom_face_index;
      export_result.top_outer_loop = top_loops->outer_loop;
      export_result.bottom_outer_loop = bottom_outer_loop;
      export_result.bores = std::move(bores);
      export_result.outer_face_indices = std::move(outer_face_indices);
      return export_result;
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticMultiBlindCounterboreExport>
detect_simple_prismatic_multi_blind_counterbore_export(const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 12 || reconstruction.vertices.size() < 32) {
    return std::nullopt;
  }

  std::vector<int> top_candidates;
  std::vector<int> bottom_candidates;
  std::vector<int> step_candidates;
  std::vector<int> floor_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() >= 3) {
      top_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 2) {
      step_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 4) {
      bottom_candidates.push_back(static_cast<int>(face_index));
      if (face.loops.front().size() >= 8) {
        floor_candidates.push_back(static_cast<int>(face_index));
      }
    }
  }
  if (top_candidates.empty() || bottom_candidates.empty() || step_candidates.size() < 2 ||
      floor_candidates.size() < 2) {
    return std::nullopt;
  }

  for (int top_face_index : top_candidates) {
    const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
    Vec3 rough_axis = normalized(top_face.fit.normal);
    if (length_squared(rough_axis) <= 1e-12) {
      continue;
    }

    for (int bottom_face_index : bottom_candidates) {
      if (bottom_face_index == top_face_index) {
        continue;
      }

      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];
      if (angle_degrees_unsigned(top_face.fit.normal, bottom_face.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = rough_axis;
      if (dot(top_face.fit.centroid - bottom_face.fit.centroid, axis) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);
      const std::optional<ClassifiedFaceMultiLoops> top_loops =
          classify_face_loops_outer_inner(top_face, reconstruction.vertices, axis_basis);
      if (!top_loops.has_value() || top_loops->inner_loops.size() < 2) {
        continue;
      }

      const double top_plane = dot(top_face.fit.centroid, axis);
      const double bottom_plane = dot(bottom_face.fit.centroid, axis);
      if (top_plane <= bottom_plane + 1e-4) {
        continue;
      }

      std::vector<CircleFit2D> top_circles;
      top_circles.reserve(top_loops->inner_loops.size());
      bool fit_valid = true;
      for (const std::vector<int>& loop : top_loops->inner_loops) {
        const std::optional<CircleFit2D> circle =
            fit_circle_2d(project_loop_to_basis(loop, reconstruction.vertices, axis_basis));
        if (!circle.has_value() || circle->radius <= 1e-6) {
          fit_valid = false;
          break;
        }
        const double tolerance = std::max(circle->radius * 0.03, 1e-4);
        if (circle->max_error > tolerance) {
          fit_valid = false;
          break;
        }
        top_circles.push_back(*circle);
      }
      if (!fit_valid) {
        continue;
      }

      struct PendingCounterbore {
        std::size_t top_loop_index {};
        int step_face_index {};
        std::vector<int> step_outer_loop;
        std::vector<int> step_inner_loop;
        CircleFit2D step_outer_circle {};
        CircleFit2D step_inner_circle {};
        double step_outer_area {};
        double step_inner_area {};
        double step_plane {};
        int floor_face_index {};
        std::vector<int> floor_loop;
        CircleFit2D floor_circle {};
        double floor_area {};
        double floor_plane {};
      };

      std::vector<bool> used_step(reconstruction.faces.size(), false);
      std::vector<bool> used_floor(reconstruction.faces.size(), false);
      std::vector<PendingCounterbore> pending;
      pending.reserve(top_loops->inner_loops.size());

      for (std::size_t top_index = 0; top_index < top_loops->inner_loops.size(); ++top_index) {
        const CircleFit2D& top_circle = top_circles[top_index];
        bool found_match = false;
        double best_score = std::numeric_limits<double>::infinity();
        PendingCounterbore best_pending;

        for (int step_face_index : step_candidates) {
          if (used_step[step_face_index] || step_face_index == top_face_index ||
              step_face_index == bottom_face_index) {
            continue;
          }

          const ReconstructedFace& step_face = reconstruction.faces[step_face_index];
          if (angle_degrees_unsigned(top_face.fit.normal, step_face.fit.normal) > 8.0) {
            continue;
          }
          const double step_plane = dot(step_face.fit.centroid, axis);
          if (step_plane >= top_plane - 1e-4 || step_plane <= bottom_plane + 1e-4) {
            continue;
          }

          const std::optional<ClassifiedFaceLoops> step_loops =
              classify_face_loops_by_area(step_face, reconstruction.vertices, axis_basis);
          if (!step_loops.has_value()) {
            continue;
          }
          const auto& [step_outer_loop, step_inner_loop, step_outer_area, step_inner_area] = *step_loops;
          if (step_outer_loop.size() < 8 || step_inner_loop.size() < 8) {
            continue;
          }

          const std::optional<CircleFit2D> step_outer_circle = fit_circle_2d(
              project_loop_to_basis(step_outer_loop, reconstruction.vertices, axis_basis));
          const std::optional<CircleFit2D> step_inner_circle = fit_circle_2d(
              project_loop_to_basis(step_inner_loop, reconstruction.vertices, axis_basis));
          if (!step_outer_circle.has_value() || !step_inner_circle.has_value()) {
            continue;
          }

          const double upper_tolerance =
              std::max((top_circle.radius + step_outer_circle->radius) * 0.015, 1e-4);
          const Vec2 upper_delta {top_circle.center.x - step_outer_circle->center.x,
                                  top_circle.center.y - step_outer_circle->center.y};
          const double upper_center_distance =
              std::sqrt(upper_delta.x * upper_delta.x + upper_delta.y * upper_delta.y);
          if (std::abs(top_circle.radius - step_outer_circle->radius) > upper_tolerance ||
              step_outer_circle->max_error > upper_tolerance ||
              step_inner_circle->max_error > upper_tolerance ||
              upper_center_distance > upper_tolerance ||
              step_outer_circle->radius <= step_inner_circle->radius + upper_tolerance) {
            continue;
          }

          for (int floor_face_index : floor_candidates) {
            if (used_floor[floor_face_index] || floor_face_index == top_face_index ||
                floor_face_index == bottom_face_index || floor_face_index == step_face_index) {
              continue;
            }

            const ReconstructedFace& floor_face = reconstruction.faces[floor_face_index];
            if (angle_degrees_unsigned(top_face.fit.normal, floor_face.fit.normal) > 8.0) {
              continue;
            }
            const double floor_plane = dot(floor_face.fit.centroid, axis);
            if (floor_plane >= step_plane - 1e-4 || floor_plane <= bottom_plane + 1e-4) {
              continue;
            }

            const std::vector<int>& floor_loop = floor_face.loops.front();
            const std::optional<CircleFit2D> floor_circle =
                fit_circle_2d(project_loop_to_basis(floor_loop, reconstruction.vertices, axis_basis));
            if (!floor_circle.has_value()) {
              continue;
            }

            const double lower_tolerance =
                std::max((step_inner_circle->radius + floor_circle->radius) * 0.015, 1e-4);
            const Vec2 lower_delta {step_inner_circle->center.x - floor_circle->center.x,
                                    step_inner_circle->center.y - floor_circle->center.y};
            const double lower_center_distance =
                std::sqrt(lower_delta.x * lower_delta.x + lower_delta.y * lower_delta.y);
            if (std::abs(step_inner_circle->radius - floor_circle->radius) > lower_tolerance ||
                floor_circle->max_error > lower_tolerance || lower_center_distance > lower_tolerance) {
              continue;
            }

            const double score =
                upper_center_distance + lower_center_distance +
                std::abs(top_circle.radius - step_outer_circle->radius) +
                std::abs(step_inner_circle->radius - floor_circle->radius);
            if (score < best_score) {
              best_score = score;
              found_match = true;
              best_pending = PendingCounterbore {
                  top_index,
                  step_face_index,
                  step_outer_loop,
                  step_inner_loop,
                  *step_outer_circle,
                  *step_inner_circle,
                  step_outer_area,
                  step_inner_area,
                  step_plane,
                  floor_face_index,
                  floor_loop,
                  *floor_circle,
                  projected_loop_area(floor_loop, reconstruction.vertices, axis_basis),
                  floor_plane,
              };
            }
          }
        }

        if (!found_match) {
          fit_valid = false;
          break;
        }
        used_step[best_pending.step_face_index] = true;
        used_floor[best_pending.floor_face_index] = true;
        pending.push_back(std::move(best_pending));
      }
      if (!fit_valid || pending.size() != top_loops->inner_loops.size()) {
        continue;
      }

      std::vector<std::unordered_set<int>> upper_vertex_sets;
      std::vector<std::unordered_set<int>> lower_vertex_sets;
      std::unordered_set<int> all_feature_vertices;
      upper_vertex_sets.reserve(pending.size());
      lower_vertex_sets.reserve(pending.size());
      for (const PendingCounterbore& counterbore : pending) {
        std::unordered_set<int> upper_vertices;
        upper_vertices.insert(top_loops->inner_loops[counterbore.top_loop_index].begin(),
                              top_loops->inner_loops[counterbore.top_loop_index].end());
        upper_vertices.insert(counterbore.step_outer_loop.begin(), counterbore.step_outer_loop.end());
        all_feature_vertices.insert(upper_vertices.begin(), upper_vertices.end());
        upper_vertex_sets.push_back(std::move(upper_vertices));

        std::unordered_set<int> lower_vertices;
        lower_vertices.insert(counterbore.step_inner_loop.begin(), counterbore.step_inner_loop.end());
        lower_vertices.insert(counterbore.floor_loop.begin(), counterbore.floor_loop.end());
        all_feature_vertices.insert(lower_vertices.begin(), lower_vertices.end());
        lower_vertex_sets.push_back(std::move(lower_vertices));
      }

      std::vector<int> outer_face_indices;
      std::vector<int> upper_face_counts(pending.size(), 0);
      std::vector<int> lower_face_counts(pending.size(), 0);
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == top_face_index ||
            static_cast<int>(face_index) == bottom_face_index) {
          continue;
        }

        bool is_internal_cap = false;
        for (const PendingCounterbore& counterbore : pending) {
          if (static_cast<int>(face_index) == counterbore.step_face_index ||
              static_cast<int>(face_index) == counterbore.floor_face_index) {
            is_internal_cap = true;
            break;
          }
        }
        if (is_internal_cap) {
          continue;
        }

        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 3) {
          fit_valid = false;
          break;
        }

        bool all_feature = true;
        for (int vertex_index : face.loops.front()) {
          if (!all_feature_vertices.contains(vertex_index)) {
            all_feature = false;
            break;
          }
        }
        if (!all_feature) {
          outer_face_indices.push_back(static_cast<int>(face_index));
          continue;
        }

        bool matched = false;
        for (std::size_t feature_index = 0; feature_index < pending.size(); ++feature_index) {
          bool all_upper = true;
          bool all_lower = true;
          for (int vertex_index : face.loops.front()) {
            if (!upper_vertex_sets[feature_index].contains(vertex_index)) {
              all_upper = false;
            }
            if (!lower_vertex_sets[feature_index].contains(vertex_index)) {
              all_lower = false;
            }
          }

          if (all_upper) {
            int top_vertices = 0;
            int step_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= 1e-4) {
                ++top_vertices;
              } else if (std::abs(coordinate - pending[feature_index].step_plane) <= 1e-4) {
                ++step_vertices;
              }
            }
            if (top_vertices == 2 && step_vertices == 2) {
              ++upper_face_counts[feature_index];
              matched = true;
              break;
            }
          } else if (all_lower) {
            int step_vertices = 0;
            int floor_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - pending[feature_index].step_plane) <= 1e-4) {
                ++step_vertices;
              } else if (std::abs(coordinate - pending[feature_index].floor_plane) <= 1e-4) {
                ++floor_vertices;
              }
            }
            if (step_vertices == 2 && floor_vertices == 2) {
              ++lower_face_counts[feature_index];
              matched = true;
              break;
            }
          }
        }
        if (!matched) {
          fit_valid = false;
          break;
        }
      }

      if (!fit_valid || outer_face_indices.size() < 4) {
        continue;
      }
      if (std::any_of(upper_face_counts.begin(), upper_face_counts.end(),
                      [](int count) { return count < 8; }) ||
          std::any_of(lower_face_counts.begin(), lower_face_counts.end(),
                      [](int count) { return count < 8; })) {
        continue;
      }

      std::vector<SimplePrismaticMultiBlindCounterbore> counterbores;
      counterbores.reserve(pending.size());
      for (const PendingCounterbore& pending_counterbore : pending) {
        const Vec2 center {
            (top_circles[pending_counterbore.top_loop_index].center.x +
             pending_counterbore.step_outer_circle.center.x +
             pending_counterbore.step_inner_circle.center.x +
             pending_counterbore.floor_circle.center.x) * 0.25,
            (top_circles[pending_counterbore.top_loop_index].center.y +
             pending_counterbore.step_outer_circle.center.y +
             pending_counterbore.step_inner_circle.center.y +
             pending_counterbore.floor_circle.center.y) * 0.25,
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;

        const Vec3 upper_base_center = axis_origin + axis * pending_counterbore.step_plane;
        const Vec3 upper_seam_vector =
            reconstruction.vertices[pending_counterbore.step_outer_loop.front()] - upper_base_center;
        const Vec3 upper_radial = upper_seam_vector - axis * dot(upper_seam_vector, axis);
        const Vec3 lower_base_center = axis_origin + axis * pending_counterbore.floor_plane;
        const Vec3 lower_seam_vector =
            reconstruction.vertices[pending_counterbore.floor_loop.front()] - lower_base_center;
        const Vec3 lower_radial = lower_seam_vector - axis * dot(lower_seam_vector, axis);
        if (length_squared(upper_radial) <= 1e-12 || length_squared(lower_radial) <= 1e-12) {
          fit_valid = false;
          break;
        }

        SimplePrismaticMultiBlindCounterbore counterbore;
        counterbore.top_inner_area = top_loops->inner_areas[pending_counterbore.top_loop_index];
        counterbore.step_outer_area = pending_counterbore.step_outer_area;
        counterbore.step_inner_area = pending_counterbore.step_inner_area;
        counterbore.floor_area = pending_counterbore.floor_area;
        counterbore.upper_bore = SimpleCylinderPrimitive {
            upper_base_center,
            axis,
            normalized(upper_radial),
            top_circles[pending_counterbore.top_loop_index].radius,
            top_plane - pending_counterbore.step_plane,
        };
        counterbore.lower_bore = SimpleCylinderPrimitive {
            lower_base_center,
            axis,
            normalized(lower_radial),
            pending_counterbore.step_inner_circle.radius,
            pending_counterbore.step_plane - pending_counterbore.floor_plane,
        };
        counterbores.push_back(std::move(counterbore));
      }
      if (!fit_valid) {
        continue;
      }

      SimplePrismaticMultiBlindCounterboreExport export_result;
      export_result.top_face_index = top_face_index;
      export_result.bottom_face_index = bottom_face_index;
      export_result.top_outer_loop = top_loops->outer_loop;
      export_result.bottom_outer_loop = bottom_face.loops.front();
      export_result.counterbores = std::move(counterbores);
      export_result.outer_face_indices = std::move(outer_face_indices);
      return export_result;
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticMultiBlindCountersinkExport>
detect_simple_prismatic_multi_blind_countersink_export(const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 11 || reconstruction.vertices.size() < 32) {
    return std::nullopt;
  }

  std::vector<int> top_candidates;
  std::vector<int> bottom_candidates;
  std::vector<int> floor_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() >= 3) {
      top_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 4) {
      bottom_candidates.push_back(static_cast<int>(face_index));
      if (face.loops.front().size() >= 8) {
        floor_candidates.push_back(static_cast<int>(face_index));
      }
    }
  }
  if (top_candidates.empty() || bottom_candidates.empty() || floor_candidates.size() < 2) {
    return std::nullopt;
  }

  for (int top_face_index : top_candidates) {
    const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
    Vec3 rough_axis = normalized(top_face.fit.normal);
    if (length_squared(rough_axis) <= 1e-12) {
      continue;
    }

    for (int bottom_face_index : bottom_candidates) {
      if (bottom_face_index == top_face_index) {
        continue;
      }

      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];
      if (angle_degrees_unsigned(top_face.fit.normal, bottom_face.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = rough_axis;
      if (dot(top_face.fit.centroid - bottom_face.fit.centroid, axis) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);
      const std::optional<ClassifiedFaceMultiLoops> top_loops =
          classify_face_loops_outer_inner(top_face, reconstruction.vertices, axis_basis);
      if (!top_loops.has_value() || top_loops->inner_loops.size() < 2) {
        continue;
      }

      const double top_plane = dot(top_face.fit.centroid, axis);
      const double bottom_plane = dot(bottom_face.fit.centroid, axis);
      if (top_plane <= bottom_plane + 1e-4) {
        continue;
      }

      std::vector<CircleFit2D> top_circles;
      top_circles.reserve(top_loops->inner_loops.size());
      bool fit_valid = true;
      double max_top_radius = 0.0;
      for (const std::vector<int>& loop : top_loops->inner_loops) {
        const std::optional<CircleFit2D> circle =
            fit_circle_2d(project_loop_to_basis(loop, reconstruction.vertices, axis_basis));
        if (!circle.has_value() || circle->radius <= 1e-6) {
          fit_valid = false;
          break;
        }
        const double tolerance = std::max(circle->radius * 0.03, 1e-4);
        if (circle->max_error > tolerance) {
          fit_valid = false;
          break;
        }
        top_circles.push_back(*circle);
        max_top_radius = std::max(max_top_radius, circle->radius);
      }
      if (!fit_valid || max_top_radius <= 1e-6) {
        continue;
      }

      struct PendingCountersink {
        std::size_t top_loop_index {};
        int floor_face_index {};
        std::vector<int> floor_loop;
        CircleFit2D top_circle {};
        CircleFit2D floor_circle {};
        CircleFit2D step_circle {};
        std::unordered_set<int> step_vertices;
        double floor_area {};
        double floor_plane {};
      };

      std::vector<bool> matched_floor(reconstruction.faces.size(), false);
      std::vector<PendingCountersink> pending;
      pending.reserve(top_loops->inner_loops.size());
      const double pair_tolerance = std::max(max_top_radius * 0.03, 1e-4);

      for (std::size_t top_index = 0; top_index < top_loops->inner_loops.size(); ++top_index) {
        const CircleFit2D& top_circle = top_circles[top_index];
        int best_floor_face = -1;
        double best_score = std::numeric_limits<double>::infinity();
        PendingCountersink best_pending;

        for (int floor_face_index : floor_candidates) {
          if (matched_floor[floor_face_index] ||
              floor_face_index == top_face_index ||
              floor_face_index == bottom_face_index) {
            continue;
          }

          const ReconstructedFace& floor_face = reconstruction.faces[floor_face_index];
          if (angle_degrees_unsigned(top_face.fit.normal, floor_face.fit.normal) > 8.0) {
            continue;
          }
          const double floor_plane = dot(floor_face.fit.centroid, axis);
          if (floor_plane >= top_plane - 1e-4 || floor_plane <= bottom_plane + 1e-4) {
            continue;
          }

          const std::vector<int>& floor_loop = floor_face.loops.front();
          const std::optional<CircleFit2D> floor_circle =
              fit_circle_2d(project_loop_to_basis(floor_loop, reconstruction.vertices, axis_basis));
          if (!floor_circle.has_value()) {
            continue;
          }

          const Vec2 center_delta {top_circle.center.x - floor_circle->center.x,
                                   top_circle.center.y - floor_circle->center.y};
          const double center_distance =
              std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
          if (floor_circle->radius >= top_circle.radius - pair_tolerance ||
              floor_circle->max_error > pair_tolerance ||
              center_distance > pair_tolerance) {
            continue;
          }

          const double score =
              center_distance + std::abs(top_circle.radius - floor_circle->radius);
          if (score < best_score) {
            best_score = score;
            best_floor_face = floor_face_index;
            best_pending = PendingCountersink {
                top_index,
                floor_face_index,
                floor_loop,
                top_circle,
                *floor_circle,
                CircleFit2D {},
                {},
                projected_loop_area(floor_loop, reconstruction.vertices, axis_basis),
                floor_plane,
            };
          }
        }

        if (best_floor_face < 0) {
          fit_valid = false;
          break;
        }
        matched_floor[best_floor_face] = true;
        pending.push_back(std::move(best_pending));
      }
      if (!fit_valid || pending.size() != top_loops->inner_loops.size()) {
        continue;
      }

      const double plane_bucket_tolerance = std::max(pair_tolerance * 0.5, 1e-4);
      std::map<long long, std::vector<int>> plane_buckets;
      for (std::size_t vertex_index = 0; vertex_index < reconstruction.vertices.size(); ++vertex_index) {
        const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
        if (coordinate >= top_plane - plane_bucket_tolerance ||
            coordinate <= bottom_plane + plane_bucket_tolerance) {
          continue;
        }
        const long long bucket =
            static_cast<long long>(std::llround(coordinate / plane_bucket_tolerance));
        plane_buckets[bucket].push_back(static_cast<int>(vertex_index));
      }

      for (const auto& [bucket, bucket_vertices] : plane_buckets) {
        if (bucket_vertices.size() < pending.size() * 8U) {
          continue;
        }

        const double step_plane = static_cast<double>(bucket) * plane_bucket_tolerance;
        if (step_plane <= bottom_plane + pair_tolerance ||
            step_plane >= top_plane - pair_tolerance) {
          continue;
        }

        std::unordered_set<int> assigned_step_vertices;
        bool step_valid = true;
        for (PendingCountersink& countersink : pending) {
          std::unordered_set<int> step_vertices;
          std::vector<Vec2> step_points;
          for (int vertex_index : bucket_vertices) {
            const Vec2 point = project_to_basis(reconstruction.vertices[vertex_index], axis_basis);
            const Vec2 center_delta {point.x - countersink.floor_circle.center.x,
                                     point.y - countersink.floor_circle.center.y};
            const double radial_distance =
                std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
            if (std::abs(radial_distance - countersink.floor_circle.radius) <= pair_tolerance * 1.5) {
              step_vertices.insert(vertex_index);
            }
          }
          if (step_vertices.size() < 8) {
            step_valid = false;
            break;
          }
          for (int vertex_index : step_vertices) {
            step_points.push_back(project_to_basis(reconstruction.vertices[vertex_index], axis_basis));
          }
          const std::optional<CircleFit2D> step_circle = fit_circle_2d(step_points);
          if (!step_circle.has_value() ||
              std::abs(step_circle->radius - countersink.floor_circle.radius) > pair_tolerance ||
              step_circle->max_error > pair_tolerance ||
              !circle_fit_centers_coincident(*step_circle, countersink.floor_circle, pair_tolerance)) {
            step_valid = false;
            break;
          }
          for (int vertex_index : step_vertices) {
            if (!assigned_step_vertices.insert(vertex_index).second) {
              step_valid = false;
              break;
            }
          }
          if (!step_valid) {
            break;
          }
          countersink.step_circle = *step_circle;
          countersink.step_vertices = std::move(step_vertices);
        }
        if (!step_valid) {
          continue;
        }

        std::vector<int> outer_face_indices;
        std::vector<int> upper_face_counts(pending.size(), 0);
        std::vector<int> lower_face_counts(pending.size(), 0);
        bool valid = true;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == bottom_face_index) {
            continue;
          }

          bool is_floor_face = false;
          for (const PendingCountersink& countersink : pending) {
            if (static_cast<int>(face_index) == countersink.floor_face_index) {
              is_floor_face = true;
              break;
            }
          }
          if (is_floor_face) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 3) {
            valid = false;
            break;
          }

          bool matched_feature = false;
          for (std::size_t countersink_index = 0; countersink_index < pending.size(); ++countersink_index) {
            const PendingCountersink& countersink = pending[countersink_index];
            std::unordered_set<int> upper_vertices;
            upper_vertices.insert(top_loops->inner_loops[countersink.top_loop_index].begin(),
                                  top_loops->inner_loops[countersink.top_loop_index].end());
            upper_vertices.insert(countersink.step_vertices.begin(), countersink.step_vertices.end());

            std::unordered_set<int> lower_vertices;
            lower_vertices.insert(countersink.step_vertices.begin(), countersink.step_vertices.end());
            lower_vertices.insert(countersink.floor_loop.begin(), countersink.floor_loop.end());

            bool all_upper = true;
            bool all_lower = true;
            for (int vertex_index : face.loops.front()) {
              if (!upper_vertices.contains(vertex_index)) {
                all_upper = false;
              }
              if (!lower_vertices.contains(vertex_index)) {
                all_lower = false;
              }
            }

            if (all_upper) {
              int top_vertices = 0;
              int step_vertices = 0;
              for (int vertex_index : face.loops.front()) {
                const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
                if (std::abs(coordinate - top_plane) <= plane_bucket_tolerance) {
                  ++top_vertices;
                } else if (std::abs(coordinate - step_plane) <= plane_bucket_tolerance) {
                  ++step_vertices;
                }
              }
              if (top_vertices != 2 || step_vertices != 2) {
                valid = false;
              } else {
                ++upper_face_counts[countersink_index];
                matched_feature = true;
              }
              break;
            }

            if (all_lower) {
              int step_vertices = 0;
              int floor_vertices = 0;
              for (int vertex_index : face.loops.front()) {
                const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
                if (std::abs(coordinate - step_plane) <= plane_bucket_tolerance) {
                  ++step_vertices;
                } else if (std::abs(coordinate - countersink.floor_plane) <= plane_bucket_tolerance) {
                  ++floor_vertices;
                }
              }
              if (step_vertices != 2 || floor_vertices != 2) {
                valid = false;
              } else {
                ++lower_face_counts[countersink_index];
                matched_feature = true;
              }
              break;
            }
          }

          if (!valid) {
            break;
          }
          if (!matched_feature) {
            outer_face_indices.push_back(static_cast<int>(face_index));
          }
        }

        if (!valid || outer_face_indices.size() < 4) {
          continue;
        }
        for (std::size_t index = 0; index < pending.size(); ++index) {
          if (upper_face_counts[index] < 8 || lower_face_counts[index] < 8) {
            valid = false;
            break;
          }
        }
        if (!valid) {
          continue;
        }

        std::vector<SimplePrismaticMultiBlindCountersink> countersinks;
        countersinks.reserve(pending.size());
        for (const PendingCountersink& pending_countersink : pending) {
          const Vec2 center {
              (pending_countersink.top_circle.center.x +
               pending_countersink.floor_circle.center.x +
               pending_countersink.step_circle.center.x) / 3.0,
              (pending_countersink.top_circle.center.y +
               pending_countersink.floor_circle.center.y +
               pending_countersink.step_circle.center.y) / 3.0,
          };
          const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
          const Vec3 step_center = axis_origin + axis * step_plane;
          const Vec3 floor_center = axis_origin + axis * pending_countersink.floor_plane;
          const int reference_vertex = *pending_countersink.step_vertices.begin();
          const Vec3 seam_vector = reconstruction.vertices[reference_vertex] - step_center;
          const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
          if (length_squared(radial) <= 1e-12) {
            valid = false;
            break;
          }

          SimplePrismaticMultiBlindCountersink countersink;
          countersink.top_inner_area = top_loops->inner_areas[pending_countersink.top_loop_index];
          countersink.floor_area = pending_countersink.floor_area;
          countersink.countersink = SimpleConicalFrustumPrimitive {
              step_center,
              axis,
              normalized(radial),
              pending_countersink.floor_circle.radius,
              pending_countersink.top_circle.radius,
              top_plane - step_plane,
          };
          countersink.bore = SimpleCylinderPrimitive {
              floor_center,
              axis,
              normalized(radial),
              pending_countersink.floor_circle.radius,
              step_plane - pending_countersink.floor_plane,
          };
          countersinks.push_back(std::move(countersink));
        }
        if (!valid) {
          continue;
        }

        SimplePrismaticMultiBlindCountersinkExport export_result;
        export_result.top_face_index = top_face_index;
        export_result.bottom_face_index = bottom_face_index;
        export_result.top_outer_loop = top_loops->outer_loop;
        export_result.bottom_outer_loop = bottom_face.loops.front();
        export_result.countersinks = std::move(countersinks);
        export_result.outer_face_indices = std::move(outer_face_indices);
        return export_result;
      }
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticCounterboreExport> detect_simple_prismatic_counterbore_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 9 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 3) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.centroid - face_b.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, face_a.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      const std::optional<ClassifiedFaceLoops> loops_a =
          classify_face_loops_by_area(face_a, reconstruction.vertices, axis_basis);
      const std::optional<ClassifiedFaceLoops> loops_b =
          classify_face_loops_by_area(face_b, reconstruction.vertices, axis_basis);
      if (!loops_a.has_value() || !loops_b.has_value()) {
        continue;
      }

      const std::optional<CircleFit2D> circle_a =
          fit_circle_2d(project_loop_to_basis(std::get<1>(*loops_a), reconstruction.vertices, axis_basis));
      const std::optional<CircleFit2D> circle_b =
          fit_circle_2d(project_loop_to_basis(std::get<1>(*loops_b), reconstruction.vertices, axis_basis));
      if (!circle_a.has_value() || !circle_b.has_value()) {
        continue;
      }

      const double initial_radius_tolerance =
          std::max(std::max(circle_a->radius, circle_b->radius) * 0.03, 1e-4);
      if (std::abs(circle_a->radius - circle_b->radius) <= initial_radius_tolerance) {
        continue;
      }

      const bool a_is_top = circle_a->radius > circle_b->radius;
      const int top_face_index = a_is_top ? cap_candidates[i] : cap_candidates[j];
      const int bottom_face_index = a_is_top ? cap_candidates[j] : cap_candidates[i];
      const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];

      axis = normalized(top_face.fit.centroid - bottom_face.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, top_face.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis ordered_axis_basis = basis_from_normal(axis);
      const std::optional<ClassifiedFaceLoops> ordered_top_loops =
          classify_face_loops_by_area(top_face, reconstruction.vertices, ordered_axis_basis);
      const std::optional<ClassifiedFaceLoops> ordered_bottom_loops =
          classify_face_loops_by_area(bottom_face, reconstruction.vertices, ordered_axis_basis);
      if (!ordered_top_loops.has_value() || !ordered_bottom_loops.has_value()) {
        continue;
      }

      const auto& [top_outer_loop, top_inner_loop, top_outer_area, top_inner_area] =
          *ordered_top_loops;
      const auto& [bottom_outer_loop, bottom_inner_loop, bottom_outer_area, bottom_inner_area] =
          *ordered_bottom_loops;
      (void)top_outer_area;
      (void)bottom_outer_area;
      if (top_inner_loop.size() < 8 || bottom_inner_loop.size() < 8) {
        continue;
      }

      const std::optional<CircleFit2D> top_circle = fit_circle_2d(
          project_loop_to_basis(top_inner_loop, reconstruction.vertices, ordered_axis_basis));
      const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(
          project_loop_to_basis(bottom_inner_loop, reconstruction.vertices, ordered_axis_basis));
      if (!top_circle.has_value() || !bottom_circle.has_value()) {
        continue;
      }

      const double radius_tolerance = std::max(top_circle->radius * 0.03, 1e-4);
      if (top_circle->radius <= bottom_circle->radius + radius_tolerance ||
          top_circle->max_error > radius_tolerance ||
          bottom_circle->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 top_bottom_delta {top_circle->center.x - bottom_circle->center.x,
                                   top_circle->center.y - bottom_circle->center.y};
      if (std::sqrt(top_bottom_delta.x * top_bottom_delta.x +
                    top_bottom_delta.y * top_bottom_delta.y) > radius_tolerance) {
        continue;
      }

      const double top_plane = dot(top_face.fit.centroid, axis);
      const double bottom_plane = dot(bottom_face.fit.centroid, axis);

      for (int step_face_index : cap_candidates) {
        if (step_face_index == top_face_index || step_face_index == bottom_face_index) {
          continue;
        }
        const ReconstructedFace& step_face = reconstruction.faces[step_face_index];
        if (angle_degrees_unsigned(top_face.fit.normal, step_face.fit.normal) > 8.0) {
          continue;
        }

        const double step_plane = dot(step_face.fit.centroid, axis);
        if (step_plane <= bottom_plane + radius_tolerance ||
            step_plane >= top_plane - radius_tolerance) {
          continue;
        }

        const std::optional<ClassifiedFaceLoops> step_loops =
            classify_face_loops_by_area(step_face, reconstruction.vertices, ordered_axis_basis);
        if (!step_loops.has_value()) {
          continue;
        }
        const auto& [step_outer_loop, step_inner_loop, step_outer_area, step_inner_area] =
            *step_loops;
        if (step_outer_loop.size() < 8 || step_inner_loop.size() < 8) {
          continue;
        }

        const std::optional<CircleFit2D> step_outer_circle = fit_circle_2d(
            project_loop_to_basis(step_outer_loop, reconstruction.vertices, ordered_axis_basis));
        const std::optional<CircleFit2D> step_inner_circle = fit_circle_2d(
            project_loop_to_basis(step_inner_loop, reconstruction.vertices, ordered_axis_basis));
        if (!step_outer_circle.has_value() || !step_inner_circle.has_value()) {
          continue;
        }

        const auto circle_matches = [&](const CircleFit2D& lhs, const CircleFit2D& rhs) {
          const Vec2 delta {lhs.center.x - rhs.center.x, lhs.center.y - rhs.center.y};
          return std::abs(lhs.radius - rhs.radius) <= radius_tolerance &&
                 lhs.max_error <= radius_tolerance &&
                 rhs.max_error <= radius_tolerance &&
                 std::sqrt(delta.x * delta.x + delta.y * delta.y) <= radius_tolerance;
        };

        if (!circle_matches(*top_circle, *step_outer_circle) ||
            !circle_matches(*bottom_circle, *step_inner_circle)) {
          continue;
        }

        std::unordered_set<int> upper_vertices;
        upper_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
        upper_vertices.insert(step_outer_loop.begin(), step_outer_loop.end());

        std::unordered_set<int> lower_vertices;
        lower_vertices.insert(step_inner_loop.begin(), step_inner_loop.end());
        lower_vertices.insert(bottom_inner_loop.begin(), bottom_inner_loop.end());

        std::vector<int> outer_face_indices;
        int upper_face_count = 0;
        int lower_face_count = 0;
        bool valid = true;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == bottom_face_index ||
              static_cast<int>(face_index) == step_face_index) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 3) {
            valid = false;
            break;
          }

          bool all_upper = true;
          bool all_lower = true;
          for (int vertex_index : face.loops.front()) {
            if (!upper_vertices.contains(vertex_index)) {
              all_upper = false;
            }
            if (!lower_vertices.contains(vertex_index)) {
              all_lower = false;
            }
          }

          if (all_upper) {
            int top_vertices = 0;
            int step_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= radius_tolerance) {
                ++top_vertices;
              } else if (std::abs(coordinate - step_plane) <= radius_tolerance) {
                ++step_vertices;
              }
            }
            if (top_vertices != 2 || step_vertices != 2) {
              valid = false;
              break;
            }
            ++upper_face_count;
          } else if (all_lower) {
            int step_vertices = 0;
            int bottom_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - step_plane) <= radius_tolerance) {
                ++step_vertices;
              } else if (std::abs(coordinate - bottom_plane) <= radius_tolerance) {
                ++bottom_vertices;
              }
            }
            if (step_vertices != 2 || bottom_vertices != 2) {
              valid = false;
              break;
            }
            ++lower_face_count;
          } else {
            outer_face_indices.push_back(static_cast<int>(face_index));
          }
        }

        if (!valid || upper_face_count < 8 || lower_face_count < 8 ||
            outer_face_indices.size() < 4) {
          continue;
        }

        const Vec2 center {
            (top_circle->center.x + bottom_circle->center.x + step_outer_circle->center.x +
             step_inner_circle->center.x) * 0.25,
            (top_circle->center.y + bottom_circle->center.y + step_outer_circle->center.y +
             step_inner_circle->center.y) * 0.25,
        };
        const Vec3 axis_origin = ordered_axis_basis.u * center.x + ordered_axis_basis.v * center.y;

        const Vec3 upper_base_center = axis_origin + axis * step_plane;
        const Vec3 upper_seam_vector = reconstruction.vertices[step_outer_loop.front()] - upper_base_center;
        const Vec3 upper_radial = upper_seam_vector - axis * dot(upper_seam_vector, axis);
        const Vec3 lower_base_center = axis_origin + axis * bottom_plane;
        const Vec3 lower_seam_vector = reconstruction.vertices[bottom_inner_loop.front()] - lower_base_center;
        const Vec3 lower_radial = lower_seam_vector - axis * dot(lower_seam_vector, axis);
        if (length_squared(upper_radial) <= 1e-12 || length_squared(lower_radial) <= 1e-12) {
          continue;
        }

        SimplePrismaticCounterboreExport counterbore;
        counterbore.top_face_index = top_face_index;
        counterbore.bottom_face_index = bottom_face_index;
        counterbore.step_face_index = step_face_index;
        counterbore.top_outer_loop = top_outer_loop;
        counterbore.bottom_outer_loop = bottom_outer_loop;
        counterbore.top_inner_area = top_inner_area;
        counterbore.bottom_inner_area = bottom_inner_area;
        counterbore.step_outer_area = step_outer_area;
        counterbore.step_inner_area = step_inner_area;
        counterbore.upper_bore = SimpleCylinderPrimitive {
            upper_base_center,
            axis,
            normalized(upper_radial),
            top_circle->radius,
            top_plane - step_plane,
        };
        counterbore.lower_bore = SimpleCylinderPrimitive {
            lower_base_center,
            axis,
            normalized(lower_radial),
            bottom_circle->radius,
            step_plane - bottom_plane,
        };
        counterbore.outer_face_indices = std::move(outer_face_indices);
        return counterbore;
      }
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticCountersinkExport> detect_simple_prismatic_countersink_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 8 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.centroid - face_b.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, face_a.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      const std::optional<ClassifiedFaceLoops> loops_a =
          classify_face_loops_by_area(face_a, reconstruction.vertices, axis_basis);
      const std::optional<ClassifiedFaceLoops> loops_b =
          classify_face_loops_by_area(face_b, reconstruction.vertices, axis_basis);
      if (!loops_a.has_value() || !loops_b.has_value()) {
        continue;
      }

      const std::optional<CircleFit2D> circle_a =
          fit_circle_2d(project_loop_to_basis(std::get<1>(*loops_a), reconstruction.vertices, axis_basis));
      const std::optional<CircleFit2D> circle_b =
          fit_circle_2d(project_loop_to_basis(std::get<1>(*loops_b), reconstruction.vertices, axis_basis));
      if (!circle_a.has_value() || !circle_b.has_value()) {
        continue;
      }

      const double initial_radius_tolerance =
          std::max(std::max(circle_a->radius, circle_b->radius) * 0.03, 1e-4);
      if (std::abs(circle_a->radius - circle_b->radius) <= initial_radius_tolerance) {
        continue;
      }

      const bool a_is_top = circle_a->radius > circle_b->radius;
      const int top_face_index = a_is_top ? cap_candidates[i] : cap_candidates[j];
      const int bottom_face_index = a_is_top ? cap_candidates[j] : cap_candidates[i];
      const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];

      axis = normalized(top_face.fit.centroid - bottom_face.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, top_face.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis ordered_axis_basis = basis_from_normal(axis);
      const std::optional<ClassifiedFaceLoops> ordered_top_loops =
          classify_face_loops_by_area(top_face, reconstruction.vertices, ordered_axis_basis);
      const std::optional<ClassifiedFaceLoops> ordered_bottom_loops =
          classify_face_loops_by_area(bottom_face, reconstruction.vertices, ordered_axis_basis);
      if (!ordered_top_loops.has_value() || !ordered_bottom_loops.has_value()) {
        continue;
      }

      const auto& [top_outer_loop, top_inner_loop, top_outer_area, top_inner_area] =
          *ordered_top_loops;
      const auto& [bottom_outer_loop, bottom_inner_loop, bottom_outer_area, bottom_inner_area] =
          *ordered_bottom_loops;
      (void)top_outer_area;
      (void)bottom_outer_area;
      if (top_inner_loop.size() < 8 || bottom_inner_loop.size() < 8) {
        continue;
      }

      const std::optional<CircleFit2D> top_circle = fit_circle_2d(
          project_loop_to_basis(top_inner_loop, reconstruction.vertices, ordered_axis_basis));
      const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(
          project_loop_to_basis(bottom_inner_loop, reconstruction.vertices, ordered_axis_basis));
      if (!top_circle.has_value() || !bottom_circle.has_value()) {
        continue;
      }

      const double radius_tolerance = std::max(top_circle->radius * 0.03, 1e-4);
      if (top_circle->radius <= bottom_circle->radius + radius_tolerance ||
          top_circle->max_error > radius_tolerance ||
          bottom_circle->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 top_bottom_delta {top_circle->center.x - bottom_circle->center.x,
                                   top_circle->center.y - bottom_circle->center.y};
      if (std::sqrt(top_bottom_delta.x * top_bottom_delta.x +
                    top_bottom_delta.y * top_bottom_delta.y) > radius_tolerance) {
        continue;
      }

      const double top_plane = dot(top_face.fit.centroid, axis);
      const double bottom_plane = dot(bottom_face.fit.centroid, axis);
      if (top_plane <= bottom_plane + radius_tolerance) {
        continue;
      }

      const double plane_bucket_tolerance = std::max(radius_tolerance * 0.5, 1e-4);
      std::map<long long, std::vector<int>> plane_buckets;
      for (std::size_t vertex_index = 0; vertex_index < reconstruction.vertices.size(); ++vertex_index) {
        const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
        if (coordinate >= top_plane - plane_bucket_tolerance ||
            coordinate <= bottom_plane + plane_bucket_tolerance) {
          continue;
        }
        const long long bucket =
            static_cast<long long>(std::llround(coordinate / plane_bucket_tolerance));
        plane_buckets[bucket].push_back(static_cast<int>(vertex_index));
      }

      for (const auto& [bucket, bucket_vertices] : plane_buckets) {
        if (bucket_vertices.size() < 8) {
          continue;
        }

        std::unordered_set<int> unique_step_vertices(bucket_vertices.begin(), bucket_vertices.end());
        if (unique_step_vertices.size() < 8) {
          continue;
        }

        const double step_plane = static_cast<double>(bucket) * plane_bucket_tolerance;
        if (step_plane <= bottom_plane + radius_tolerance ||
            step_plane >= top_plane - radius_tolerance) {
          continue;
        }

        std::vector<Vec2> step_points;
        step_points.reserve(unique_step_vertices.size());
        for (int vertex_index : unique_step_vertices) {
          step_points.push_back(project_to_basis(reconstruction.vertices[vertex_index], ordered_axis_basis));
        }
        const std::optional<CircleFit2D> step_circle = fit_circle_2d(step_points);
        if (!step_circle.has_value()) {
          continue;
        }

        if (std::abs(step_circle->radius - bottom_circle->radius) > radius_tolerance ||
            step_circle->max_error > radius_tolerance ||
            !circle_fit_centers_coincident(*step_circle, *bottom_circle, radius_tolerance)) {
          continue;
        }

        std::unordered_set<int> upper_vertices;
        upper_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
        upper_vertices.insert(unique_step_vertices.begin(), unique_step_vertices.end());

        std::unordered_set<int> lower_vertices;
        lower_vertices.insert(unique_step_vertices.begin(), unique_step_vertices.end());
        lower_vertices.insert(bottom_inner_loop.begin(), bottom_inner_loop.end());

        std::vector<int> outer_face_indices;
        int upper_face_count = 0;
        int lower_face_count = 0;
        bool valid = true;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == bottom_face_index) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 3) {
            valid = false;
            break;
          }

          bool all_upper = true;
          bool all_lower = true;
          for (int vertex_index : face.loops.front()) {
            if (!upper_vertices.contains(vertex_index)) {
              all_upper = false;
            }
            if (!lower_vertices.contains(vertex_index)) {
              all_lower = false;
            }
          }

          if (all_upper) {
            int top_vertices = 0;
            int step_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - top_plane) <= plane_bucket_tolerance) {
                ++top_vertices;
              } else if (std::abs(coordinate - step_plane) <= plane_bucket_tolerance) {
                ++step_vertices;
              }
            }
            if (top_vertices != 2 || step_vertices != 2) {
              valid = false;
              break;
            }
            ++upper_face_count;
          } else if (all_lower) {
            int step_vertices = 0;
            int bottom_vertices = 0;
            for (int vertex_index : face.loops.front()) {
              const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
              if (std::abs(coordinate - step_plane) <= plane_bucket_tolerance) {
                ++step_vertices;
              } else if (std::abs(coordinate - bottom_plane) <= plane_bucket_tolerance) {
                ++bottom_vertices;
              }
            }
            if (step_vertices != 2 || bottom_vertices != 2) {
              valid = false;
              break;
            }
            ++lower_face_count;
          } else {
            outer_face_indices.push_back(static_cast<int>(face_index));
          }
        }

        if (!valid || upper_face_count < 8 || lower_face_count < 8 ||
            outer_face_indices.size() < 4) {
          continue;
        }

        const Vec2 center {
            (top_circle->center.x + bottom_circle->center.x + step_circle->center.x) / 3.0,
            (top_circle->center.y + bottom_circle->center.y + step_circle->center.y) / 3.0,
        };
        const Vec3 axis_origin = ordered_axis_basis.u * center.x + ordered_axis_basis.v * center.y;
        const Vec3 step_center = axis_origin + axis * step_plane;
        const Vec3 bottom_center = axis_origin + axis * bottom_plane;

        const int reference_vertex = *unique_step_vertices.begin();
        const Vec3 seam_vector = reconstruction.vertices[reference_vertex] - step_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          continue;
        }

        SimplePrismaticCountersinkExport countersink;
        countersink.top_face_index = top_face_index;
        countersink.bottom_face_index = bottom_face_index;
        countersink.top_outer_loop = top_outer_loop;
        countersink.bottom_outer_loop = bottom_outer_loop;
        countersink.top_inner_area = top_inner_area;
        countersink.bottom_inner_area = bottom_inner_area;
        countersink.countersink = SimpleConicalFrustumPrimitive {
            step_center,
            axis,
            normalized(radial),
            bottom_circle->radius,
            top_circle->radius,
            top_plane - step_plane,
        };
        countersink.bore = SimpleCylinderPrimitive {
            bottom_center,
            axis,
            normalized(radial),
            bottom_circle->radius,
            step_plane - bottom_plane,
        };
        countersink.outer_face_indices = std::move(outer_face_indices);
        return countersink;
      }
    }
  }

  return std::nullopt;
}


struct AxialLoopCircle {
  std::size_t loop_index {};
  CircleFit2D circle {};
};

struct AxialPendingThroughFeature {
  std::size_t top_loop_index {};
  std::size_t bottom_loop_index {};
  PrismaticAxialThroughFeatureKind kind {PrismaticAxialThroughFeatureKind::bore};
  CircleFit2D top_circle {};
  CircleFit2D bottom_circle {};
  std::size_t step_candidate_index {};
  CircleFit2D step_outer_circle {};
  CircleFit2D step_inner_circle {};
  CircleFit2D step_circle {};
  double step_outer_area {};
  double step_inner_area {};
  double step_plane {};
  std::vector<int> step_outer_loop;
  std::vector<int> step_inner_loop;
  std::unordered_set<int> step_vertices;
};

struct AxialCapPairAnalysis {
  int top_face_index {};
  int bottom_face_index {};
  Vec3 axis {};
  Vec3 cap_normal {};
  PlaneBasis axis_basis {};
  double top_plane {};
  double bottom_plane {};
  double pair_tolerance {};
  double plane_bucket_tolerance {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<std::vector<int>> top_inner_loops;
  std::vector<std::vector<int>> bottom_inner_loops;
  std::vector<double> top_inner_areas;
  std::vector<double> bottom_inner_areas;
  std::vector<AxialLoopCircle> fitted_top;
  std::vector<AxialLoopCircle> fitted_bottom;
  std::map<long long, std::vector<int>> plane_buckets;
};

struct AxialFeatureFaceClassification {
  std::vector<int> outer_face_indices;
  std::vector<int> bore_face_counts;
  std::vector<int> upper_face_counts;
  std::vector<int> lower_face_counts;
};

std::optional<AxialCapPairAnalysis> analyze_axial_cap_pair(
    const ReconstructionResult& reconstruction,
    int face_a_index,
    int face_b_index,
    const Vec3& axis) {
  const ReconstructedFace& face_a = reconstruction.faces[face_a_index];
  const ReconstructedFace& face_b = reconstruction.faces[face_b_index];
  const PlaneBasis axis_basis = basis_from_normal(axis);

  const std::optional<ClassifiedFaceMultiLoops> loops_a =
      classify_face_loops_outer_inner(face_a, reconstruction.vertices, axis_basis);
  const std::optional<ClassifiedFaceMultiLoops> loops_b =
      classify_face_loops_outer_inner(face_b, reconstruction.vertices, axis_basis);
  if (!loops_a.has_value() || !loops_b.has_value()) {
    return std::nullopt;
  }
  if (loops_a->inner_loops.size() != loops_b->inner_loops.size() ||
      loops_a->inner_loops.size() < 2) {
    return std::nullopt;
  }

  AxialCapPairAnalysis analysis;
  analysis.axis = axis;
  analysis.axis_basis = axis_basis;

  const double plane_a = dot(face_a.fit.centroid, axis);
  const double plane_b = dot(face_b.fit.centroid, axis);
  const bool a_is_top = plane_a >= plane_b;
  analysis.top_plane = std::max(plane_a, plane_b);
  analysis.bottom_plane = std::min(plane_a, plane_b);
  analysis.top_face_index = a_is_top ? face_a_index : face_b_index;
  analysis.bottom_face_index = a_is_top ? face_b_index : face_a_index;
  analysis.cap_normal = reconstruction.faces[analysis.top_face_index].fit.normal;
  analysis.top_outer_loop = a_is_top ? loops_a->outer_loop : loops_b->outer_loop;
  analysis.bottom_outer_loop = a_is_top ? loops_b->outer_loop : loops_a->outer_loop;
  analysis.top_inner_loops = a_is_top ? loops_a->inner_loops : loops_b->inner_loops;
  analysis.bottom_inner_loops = a_is_top ? loops_b->inner_loops : loops_a->inner_loops;
  analysis.top_inner_areas = a_is_top ? loops_a->inner_areas : loops_b->inner_areas;
  analysis.bottom_inner_areas = a_is_top ? loops_b->inner_areas : loops_a->inner_areas;

  double max_top_radius = 0.0;
  analysis.fitted_top.reserve(analysis.top_inner_loops.size());
  analysis.fitted_bottom.reserve(analysis.bottom_inner_loops.size());
  for (std::size_t loop_index = 0; loop_index < analysis.top_inner_loops.size(); ++loop_index) {
    const std::optional<CircleFit2D> top_circle = fit_circle_2d(
        project_loop_to_basis(analysis.top_inner_loops[loop_index], reconstruction.vertices, axis_basis));
    const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(
        project_loop_to_basis(analysis.bottom_inner_loops[loop_index], reconstruction.vertices, axis_basis));
    if (!top_circle.has_value() || !bottom_circle.has_value()) {
      return std::nullopt;
    }
    analysis.fitted_top.push_back({loop_index, *top_circle});
    analysis.fitted_bottom.push_back({loop_index, *bottom_circle});
    max_top_radius = std::max(max_top_radius, top_circle->radius);
  }
  if (max_top_radius <= 1e-6) {
    return std::nullopt;
  }

  analysis.pair_tolerance = std::max(max_top_radius * 0.03, 1e-4);
  analysis.plane_bucket_tolerance = std::max(analysis.pair_tolerance * 0.5, 1e-4);
  for (std::size_t vertex_index = 0; vertex_index < reconstruction.vertices.size(); ++vertex_index) {
    const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
    if (coordinate >= analysis.top_plane - analysis.plane_bucket_tolerance ||
        coordinate <= analysis.bottom_plane + analysis.plane_bucket_tolerance) {
      continue;
    }
    const long long bucket = static_cast<long long>(
        std::llround(coordinate / analysis.plane_bucket_tolerance));
    analysis.plane_buckets[bucket].push_back(static_cast<int>(vertex_index));
  }

  return analysis;
}

std::optional<std::vector<AxialPendingThroughFeature>> match_axial_through_features(
    const ReconstructionResult& reconstruction,
    const std::vector<int>& step_candidates,
    const AxialCapPairAnalysis& analysis) {
  std::vector<bool> matched_bottom(analysis.fitted_bottom.size(), false);
  std::vector<bool> matched_step(step_candidates.size(), false);
  std::vector<AxialPendingThroughFeature> pending;
  pending.reserve(analysis.fitted_top.size());
  std::unordered_set<int> assigned_step_vertices;
  bool saw_profile_change = false;

  for (const AxialLoopCircle& top_loop : analysis.fitted_top) {
    if (top_loop.circle.radius <= 1e-6 ||
        top_loop.circle.max_error > analysis.pair_tolerance) {
      return std::nullopt;
    }

    std::size_t bottom_match = analysis.fitted_bottom.size();
    double best_bottom_score = std::numeric_limits<double>::infinity();
    for (std::size_t bottom_index = 0; bottom_index < analysis.fitted_bottom.size(); ++bottom_index) {
      if (matched_bottom[bottom_index]) {
        continue;
      }
      const AxialLoopCircle& bottom_loop = analysis.fitted_bottom[bottom_index];
      const Vec2 center_delta {top_loop.circle.center.x - bottom_loop.circle.center.x,
                               top_loop.circle.center.y - bottom_loop.circle.center.y};
      const double center_distance =
          std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
      if (bottom_loop.circle.max_error > analysis.pair_tolerance ||
          bottom_loop.circle.radius > top_loop.circle.radius + analysis.pair_tolerance ||
          center_distance > analysis.pair_tolerance) {
        continue;
      }
      const double score =
          center_distance + std::abs(top_loop.circle.radius - bottom_loop.circle.radius);
      if (score < best_bottom_score) {
        best_bottom_score = score;
        bottom_match = bottom_index;
      }
    }
    if (bottom_match == analysis.fitted_bottom.size()) {
      return std::nullopt;
    }

    matched_bottom[bottom_match] = true;
    const AxialLoopCircle& bottom_loop = analysis.fitted_bottom[bottom_match];
    if (std::abs(top_loop.circle.radius - bottom_loop.circle.radius) <= analysis.pair_tolerance) {
      pending.push_back(AxialPendingThroughFeature {
          top_loop.loop_index,
          bottom_loop.loop_index,
          PrismaticAxialThroughFeatureKind::bore,
          top_loop.circle,
          bottom_loop.circle,
          0,
          CircleFit2D {},
          CircleFit2D {},
          CircleFit2D {},
          0.0,
          0.0,
          0.0,
          {},
          {},
          {},
      });
      continue;
    }

    std::size_t step_match = step_candidates.size();
    AxialPendingThroughFeature best_counterbore;
    double best_step_score = std::numeric_limits<double>::infinity();
    for (std::size_t step_index = 0; step_index < step_candidates.size(); ++step_index) {
      if (matched_step[step_index]) {
        continue;
      }
      const ReconstructedFace& step_face = reconstruction.faces[step_candidates[step_index]];
      if (angle_degrees_unsigned(analysis.cap_normal, step_face.fit.normal) > 8.0) {
        continue;
      }
      const double step_plane = dot(step_face.fit.centroid, analysis.axis);
      if (step_plane >= analysis.top_plane - analysis.pair_tolerance ||
          step_plane <= analysis.bottom_plane + analysis.pair_tolerance) {
        continue;
      }

      const std::optional<ClassifiedFaceLoops> step_loops =
          classify_face_loops_by_area(step_face, reconstruction.vertices, analysis.axis_basis);
      if (!step_loops.has_value()) {
        continue;
      }
      const auto& [step_outer_loop, step_inner_loop, step_outer_area, step_inner_area] = *step_loops;
      const std::optional<CircleFit2D> step_outer_circle = fit_circle_2d(
          project_loop_to_basis(step_outer_loop, reconstruction.vertices, analysis.axis_basis));
      const std::optional<CircleFit2D> step_inner_circle = fit_circle_2d(
          project_loop_to_basis(step_inner_loop, reconstruction.vertices, analysis.axis_basis));
      if (!step_outer_circle.has_value() || !step_inner_circle.has_value()) {
        continue;
      }

      const Vec2 outer_delta {top_loop.circle.center.x - step_outer_circle->center.x,
                              top_loop.circle.center.y - step_outer_circle->center.y};
      const Vec2 inner_delta {bottom_loop.circle.center.x - step_inner_circle->center.x,
                              bottom_loop.circle.center.y - step_inner_circle->center.y};
      const double outer_center_distance =
          std::sqrt(outer_delta.x * outer_delta.x + outer_delta.y * outer_delta.y);
      const double inner_center_distance =
          std::sqrt(inner_delta.x * inner_delta.x + inner_delta.y * inner_delta.y);
      if (std::abs(top_loop.circle.radius - step_outer_circle->radius) > analysis.pair_tolerance ||
          std::abs(bottom_loop.circle.radius - step_inner_circle->radius) > analysis.pair_tolerance ||
          step_outer_circle->max_error > analysis.pair_tolerance ||
          step_inner_circle->max_error > analysis.pair_tolerance ||
          outer_center_distance > analysis.pair_tolerance ||
          inner_center_distance > analysis.pair_tolerance) {
        continue;
      }

      const double score = outer_center_distance + inner_center_distance +
                           std::abs(top_loop.circle.radius - step_outer_circle->radius) +
                           std::abs(bottom_loop.circle.radius - step_inner_circle->radius);
      if (score < best_step_score) {
        best_step_score = score;
        best_counterbore = AxialPendingThroughFeature {
            top_loop.loop_index,
            bottom_loop.loop_index,
            PrismaticAxialThroughFeatureKind::counterbore,
            top_loop.circle,
            bottom_loop.circle,
            step_index,
            *step_outer_circle,
            *step_inner_circle,
            CircleFit2D {},
            step_outer_area,
            step_inner_area,
            step_plane,
            step_outer_loop,
            step_inner_loop,
            {},
        };
        step_match = step_index;
      }
    }
    if (step_match != step_candidates.size()) {
      matched_step[step_match] = true;
      saw_profile_change = true;
      pending.push_back(std::move(best_counterbore));
      continue;
    }

    double best_countersink_score = std::numeric_limits<double>::infinity();
    std::optional<AxialPendingThroughFeature> best_countersink;
    for (const auto& [bucket, bucket_vertices] : analysis.plane_buckets) {
      const double step_plane = static_cast<double>(bucket) * analysis.plane_bucket_tolerance;
      if (step_plane <= analysis.bottom_plane + analysis.pair_tolerance ||
          step_plane >= analysis.top_plane - analysis.pair_tolerance) {
        continue;
      }

      std::unordered_set<int> step_vertices;
      std::vector<Vec2> step_points;
      for (int vertex_index : bucket_vertices) {
        if (assigned_step_vertices.contains(vertex_index)) {
          continue;
        }
        const Vec2 point =
            project_to_basis(reconstruction.vertices[vertex_index], analysis.axis_basis);
        const Vec2 center_delta {point.x - bottom_loop.circle.center.x,
                                 point.y - bottom_loop.circle.center.y};
        const double radial_distance =
            std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
        if (std::abs(radial_distance - bottom_loop.circle.radius) <= analysis.pair_tolerance * 1.5) {
          step_vertices.insert(vertex_index);
          step_points.push_back(point);
        }
      }
      if (step_vertices.size() < 8) {
        continue;
      }

      const std::optional<CircleFit2D> step_circle = fit_circle_2d(step_points);
      if (!step_circle.has_value() ||
          std::abs(step_circle->radius - bottom_loop.circle.radius) > analysis.pair_tolerance ||
          step_circle->max_error > analysis.pair_tolerance ||
          !circle_fit_centers_coincident(*step_circle, bottom_loop.circle, analysis.pair_tolerance)) {
        continue;
      }

      const double score = step_circle->max_error + std::abs(analysis.top_plane - step_plane);
      if (score < best_countersink_score) {
        best_countersink_score = score;
        best_countersink = AxialPendingThroughFeature {
            top_loop.loop_index,
            bottom_loop.loop_index,
            PrismaticAxialThroughFeatureKind::countersink,
            top_loop.circle,
            bottom_loop.circle,
            0,
            CircleFit2D {},
            CircleFit2D {},
            *step_circle,
            0.0,
            0.0,
            step_plane,
            {},
            {},
            std::move(step_vertices),
        };
      }
    }
    if (!best_countersink.has_value()) {
      return std::nullopt;
    }

    for (int vertex_index : best_countersink->step_vertices) {
      assigned_step_vertices.insert(vertex_index);
    }
    saw_profile_change = true;
    pending.push_back(std::move(*best_countersink));
  }

  if (pending.size() != analysis.fitted_top.size() || pending.size() < 2 || !saw_profile_change) {
    return std::nullopt;
  }
  return pending;
}

std::optional<AxialFeatureFaceClassification> classify_axial_through_feature_faces(
    const ReconstructionResult& reconstruction,
    const std::vector<int>& step_candidates,
    const AxialCapPairAnalysis& analysis,
    const std::vector<AxialPendingThroughFeature>& pending) {
  AxialFeatureFaceClassification classification;
  classification.bore_face_counts.assign(pending.size(), 0);
  classification.upper_face_counts.assign(pending.size(), 0);
  classification.lower_face_counts.assign(pending.size(), 0);

  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (static_cast<int>(face_index) == analysis.top_face_index ||
        static_cast<int>(face_index) == analysis.bottom_face_index) {
      continue;
    }

    bool is_step = false;
    for (const AxialPendingThroughFeature& feature : pending) {
      if (feature.kind == PrismaticAxialThroughFeatureKind::counterbore &&
          static_cast<int>(face_index) == step_candidates[feature.step_candidate_index]) {
        is_step = true;
        break;
      }
    }
    if (is_step) {
      continue;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() != 1 || face.loops.front().size() < 4) {
      return std::nullopt;
    }

    bool matched_feature = false;
    for (std::size_t feature_index = 0; feature_index < pending.size(); ++feature_index) {
      const AxialPendingThroughFeature& feature = pending[feature_index];
      if (feature.kind == PrismaticAxialThroughFeatureKind::bore) {
        std::unordered_set<int> bore_vertices;
        bore_vertices.insert(analysis.top_inner_loops[feature.top_loop_index].begin(),
                             analysis.top_inner_loops[feature.top_loop_index].end());
        bore_vertices.insert(analysis.bottom_inner_loops[feature.bottom_loop_index].begin(),
                             analysis.bottom_inner_loops[feature.bottom_loop_index].end());
        bool all_bore = true;
        for (int vertex_index : face.loops.front()) {
          if (!bore_vertices.contains(vertex_index)) {
            all_bore = false;
            break;
          }
        }
        if (!all_bore) {
          continue;
        }

        int top_vertices = 0;
        int bottom_vertices = 0;
        for (int vertex_index : face.loops.front()) {
          const double coordinate = dot(reconstruction.vertices[vertex_index], analysis.axis);
          if (std::abs(coordinate - analysis.top_plane) <= analysis.pair_tolerance) {
            ++top_vertices;
          } else if (std::abs(coordinate - analysis.bottom_plane) <= analysis.pair_tolerance) {
            ++bottom_vertices;
          }
        }
        if (top_vertices != 2 || bottom_vertices != 2) {
          return std::nullopt;
        }
        ++classification.bore_face_counts[feature_index];
        matched_feature = true;
        break;
      }

      std::unordered_set<int> upper_vertices;
      std::unordered_set<int> lower_vertices;
      if (feature.kind == PrismaticAxialThroughFeatureKind::counterbore) {
        upper_vertices.insert(analysis.top_inner_loops[feature.top_loop_index].begin(),
                              analysis.top_inner_loops[feature.top_loop_index].end());
        upper_vertices.insert(feature.step_outer_loop.begin(), feature.step_outer_loop.end());
        lower_vertices.insert(feature.step_inner_loop.begin(), feature.step_inner_loop.end());
        lower_vertices.insert(analysis.bottom_inner_loops[feature.bottom_loop_index].begin(),
                              analysis.bottom_inner_loops[feature.bottom_loop_index].end());
      } else {
        upper_vertices.insert(analysis.top_inner_loops[feature.top_loop_index].begin(),
                              analysis.top_inner_loops[feature.top_loop_index].end());
        upper_vertices.insert(feature.step_vertices.begin(), feature.step_vertices.end());
        lower_vertices.insert(feature.step_vertices.begin(), feature.step_vertices.end());
        lower_vertices.insert(analysis.bottom_inner_loops[feature.bottom_loop_index].begin(),
                              analysis.bottom_inner_loops[feature.bottom_loop_index].end());
      }

      bool all_upper = true;
      bool all_lower = true;
      for (int vertex_index : face.loops.front()) {
        if (!upper_vertices.contains(vertex_index)) {
          all_upper = false;
        }
        if (!lower_vertices.contains(vertex_index)) {
          all_lower = false;
        }
      }

      if (all_upper) {
        int top_vertices = 0;
        int step_vertices = 0;
        for (int vertex_index : face.loops.front()) {
          const double coordinate = dot(reconstruction.vertices[vertex_index], analysis.axis);
          if (std::abs(coordinate - analysis.top_plane) <= analysis.pair_tolerance) {
            ++top_vertices;
          } else if (std::abs(coordinate - feature.step_plane) <= analysis.pair_tolerance) {
            ++step_vertices;
          }
        }
        if (top_vertices != 2 || step_vertices != 2) {
          return std::nullopt;
        }
        ++classification.upper_face_counts[feature_index];
        matched_feature = true;
        break;
      }

      if (all_lower) {
        int step_vertices = 0;
        int bottom_vertices = 0;
        for (int vertex_index : face.loops.front()) {
          const double coordinate = dot(reconstruction.vertices[vertex_index], analysis.axis);
          if (std::abs(coordinate - feature.step_plane) <= analysis.pair_tolerance) {
            ++step_vertices;
          } else if (std::abs(coordinate - analysis.bottom_plane) <= analysis.pair_tolerance) {
            ++bottom_vertices;
          }
        }
        if (step_vertices != 2 || bottom_vertices != 2) {
          return std::nullopt;
        }
        ++classification.lower_face_counts[feature_index];
        matched_feature = true;
        break;
      }
    }

    if (!matched_feature) {
      classification.outer_face_indices.push_back(static_cast<int>(face_index));
    }
  }

  if (classification.outer_face_indices.size() < 4) {
    return std::nullopt;
  }
  return classification;
}

std::optional<std::vector<SimplePrismaticAxialThroughFeature>> build_axial_through_feature_export(
    const ReconstructionResult& reconstruction,
    const AxialCapPairAnalysis& analysis,
    const std::vector<AxialPendingThroughFeature>& pending,
    const AxialFeatureFaceClassification& classification) {
  std::vector<SimplePrismaticAxialThroughFeature> features;
  features.reserve(pending.size());

  for (std::size_t feature_index = 0; feature_index < pending.size(); ++feature_index) {
    const AxialPendingThroughFeature& feature = pending[feature_index];
    if (feature.kind == PrismaticAxialThroughFeatureKind::bore) {
      if (classification.bore_face_counts[feature_index] < 8) {
        return std::nullopt;
      }
      const double radius = 0.5 * (feature.top_circle.radius + feature.bottom_circle.radius);
      const Vec2 center {
          0.5 * (feature.top_circle.center.x + feature.bottom_circle.center.x),
          0.5 * (feature.top_circle.center.y + feature.bottom_circle.center.y),
      };
      const Vec3 axis_origin = analysis.axis_basis.u * center.x + analysis.axis_basis.v * center.y;
      const Vec3 base_center = axis_origin + analysis.axis * analysis.bottom_plane;
      const Vec3 seam_vector =
          reconstruction.vertices[analysis.bottom_inner_loops[feature.bottom_loop_index].front()] - base_center;
      const Vec3 radial = seam_vector - analysis.axis * dot(seam_vector, analysis.axis);
      if (length_squared(radial) <= 1e-12) {
        return std::nullopt;
      }

      SimplePrismaticAxialThroughFeature exported;
      exported.top_inner_area = analysis.top_inner_areas[feature.top_loop_index];
      exported.bottom_inner_area = analysis.bottom_inner_areas[feature.bottom_loop_index];
      exported.kind = PrismaticAxialThroughFeatureKind::bore;
      exported.bore = SimpleCylinderPrimitive {
          base_center,
          analysis.axis,
          normalized(radial),
          radius,
          analysis.top_plane - analysis.bottom_plane,
      };
      features.push_back(std::move(exported));
      continue;
    }

    if (feature.kind == PrismaticAxialThroughFeatureKind::countersink) {
      if (classification.upper_face_counts[feature_index] < 8 ||
          classification.lower_face_counts[feature_index] < 8 ||
          feature.step_vertices.empty()) {
        return std::nullopt;
      }
      const Vec2 center {
          (feature.top_circle.center.x + feature.bottom_circle.center.x + feature.step_circle.center.x) / 3.0,
          (feature.top_circle.center.y + feature.bottom_circle.center.y + feature.step_circle.center.y) / 3.0,
      };
      const Vec3 axis_origin = analysis.axis_basis.u * center.x + analysis.axis_basis.v * center.y;
      const Vec3 step_center = axis_origin + analysis.axis * feature.step_plane;
      const Vec3 bottom_center = axis_origin + analysis.axis * analysis.bottom_plane;
      const int reference_vertex = *feature.step_vertices.begin();
      const Vec3 seam_vector = reconstruction.vertices[reference_vertex] - step_center;
      const Vec3 radial = seam_vector - analysis.axis * dot(seam_vector, analysis.axis);
      if (length_squared(radial) <= 1e-12) {
        return std::nullopt;
      }

      SimplePrismaticAxialThroughFeature exported;
      exported.top_inner_area = analysis.top_inner_areas[feature.top_loop_index];
      exported.bottom_inner_area = analysis.bottom_inner_areas[feature.bottom_loop_index];
      exported.kind = PrismaticAxialThroughFeatureKind::countersink;
      exported.countersink = SimpleConicalFrustumPrimitive {
          step_center,
          analysis.axis,
          normalized(radial),
          feature.bottom_circle.radius,
          feature.top_circle.radius,
          analysis.top_plane - feature.step_plane,
      };
      exported.bore = SimpleCylinderPrimitive {
          bottom_center,
          analysis.axis,
          normalized(radial),
          feature.bottom_circle.radius,
          feature.step_plane - analysis.bottom_plane,
      };
      features.push_back(std::move(exported));
      continue;
    }

    if (classification.upper_face_counts[feature_index] < 8 ||
        classification.lower_face_counts[feature_index] < 8) {
      return std::nullopt;
    }
    const Vec2 center {
        (feature.top_circle.center.x + feature.bottom_circle.center.x +
         feature.step_outer_circle.center.x + feature.step_inner_circle.center.x) * 0.25,
        (feature.top_circle.center.y + feature.bottom_circle.center.y +
         feature.step_outer_circle.center.y + feature.step_inner_circle.center.y) * 0.25,
    };
    const Vec3 axis_origin = analysis.axis_basis.u * center.x + analysis.axis_basis.v * center.y;
    const Vec3 upper_base_center = axis_origin + analysis.axis * feature.step_plane;
    const Vec3 upper_seam_vector =
        reconstruction.vertices[feature.step_outer_loop.front()] - upper_base_center;
    const Vec3 upper_radial = upper_seam_vector - analysis.axis * dot(upper_seam_vector, analysis.axis);
    const Vec3 lower_base_center = axis_origin + analysis.axis * analysis.bottom_plane;
    const Vec3 lower_seam_vector =
        reconstruction.vertices[analysis.bottom_inner_loops[feature.bottom_loop_index].front()] - lower_base_center;
    const Vec3 lower_radial = lower_seam_vector - analysis.axis * dot(lower_seam_vector, analysis.axis);
    if (length_squared(upper_radial) <= 1e-12 || length_squared(lower_radial) <= 1e-12) {
      return std::nullopt;
    }

    SimplePrismaticAxialThroughFeature exported;
    exported.top_inner_area = analysis.top_inner_areas[feature.top_loop_index];
    exported.bottom_inner_area = analysis.bottom_inner_areas[feature.bottom_loop_index];
    exported.kind = PrismaticAxialThroughFeatureKind::counterbore;
    exported.step_outer_area = feature.step_outer_area;
    exported.step_inner_area = feature.step_inner_area;
    exported.upper_bore = SimpleCylinderPrimitive {
        upper_base_center,
        analysis.axis,
        normalized(upper_radial),
        feature.top_circle.radius,
        analysis.top_plane - feature.step_plane,
    };
    exported.lower_bore = SimpleCylinderPrimitive {
        lower_base_center,
        analysis.axis,
        normalized(lower_radial),
        feature.bottom_circle.radius,
        feature.step_plane - analysis.bottom_plane,
    };
    features.push_back(std::move(exported));
  }

  return features;
}

std::optional<SimplePrismaticAxialThroughFeatureExport>
detect_simple_prismatic_axial_through_feature_export(const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 32) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  std::vector<int> step_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() >= 3) {
      cap_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 2) {
      step_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      const Vec3 centroid_axis = normalized(face_a.fit.centroid - face_b.fit.centroid);
      if (length_squared(centroid_axis) <= 1e-12) {
        continue;
      }

      for (const Vec3& axis : std::array<Vec3, 2> {centroid_axis, centroid_axis * -1.0}) {
        const std::optional<AxialCapPairAnalysis> analysis =
            analyze_axial_cap_pair(reconstruction, cap_candidates[i], cap_candidates[j], axis);
        if (!analysis.has_value()) {
          continue;
        }

        const std::optional<std::vector<AxialPendingThroughFeature>> pending =
            match_axial_through_features(reconstruction, step_candidates, *analysis);
        if (!pending.has_value()) {
          continue;
        }

        const std::optional<AxialFeatureFaceClassification> classification =
            classify_axial_through_feature_faces(reconstruction, step_candidates, *analysis, *pending);
        if (!classification.has_value()) {
          continue;
        }

        const std::optional<std::vector<SimplePrismaticAxialThroughFeature>> features =
            build_axial_through_feature_export(reconstruction, *analysis, *pending, *classification);
        if (!features.has_value()) {
          continue;
        }

        SimplePrismaticAxialThroughFeatureExport export_result;
        export_result.top_face_index = analysis->top_face_index;
        export_result.bottom_face_index = analysis->bottom_face_index;
        export_result.top_outer_loop = analysis->top_outer_loop;
        export_result.bottom_outer_loop = analysis->bottom_outer_loop;
        export_result.features = *features;
        export_result.outer_face_indices = classification->outer_face_indices;
        return export_result;
      }
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticBoreExport> detect_simple_prismatic_bore_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 7 || reconstruction.vertices.size() < 16) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.centroid - face_b.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, face_a.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      const std::optional<ClassifiedFaceLoops> loops_a =
          classify_face_loops_by_area(face_a, reconstruction.vertices, axis_basis);
      const std::optional<ClassifiedFaceLoops> loops_b =
          classify_face_loops_by_area(face_b, reconstruction.vertices, axis_basis);
      if (!loops_a.has_value() || !loops_b.has_value()) {
        continue;
      }

      const auto& [outer_a, inner_a, outer_area_a, inner_area_a] = *loops_a;
      const auto& [outer_b, inner_b, outer_area_b, inner_area_b] = *loops_b;
      if (inner_a.size() < 8 || inner_b.size() < 8) {
        continue;
      }

      const std::optional<CircleFit2D> circle_a =
          fit_circle_2d(project_loop_to_basis(inner_a, reconstruction.vertices, axis_basis));
      const std::optional<CircleFit2D> circle_b =
          fit_circle_2d(project_loop_to_basis(inner_b, reconstruction.vertices, axis_basis));
      if (!circle_a.has_value() || !circle_b.has_value()) {
        continue;
      }

      const double radius = (circle_a->radius + circle_b->radius) * 0.5;
      if (radius <= 1e-6) {
        continue;
      }

      const double radius_tolerance = std::max(radius * 0.03, 1e-4);
      if (std::abs(circle_a->radius - circle_b->radius) > radius_tolerance ||
          circle_a->max_error > radius_tolerance || circle_b->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 center_delta {circle_a->center.x - circle_b->center.x,
                               circle_a->center.y - circle_b->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          radius_tolerance) {
        continue;
      }

      const double plane_a = dot(face_a.fit.centroid, axis);
      const double plane_b = dot(face_b.fit.centroid, axis);
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
      const double height = top_plane - bottom_plane;
      if (height <= radius_tolerance) {
        continue;
      }

      const bool a_is_top = plane_a >= plane_b;
      const int top_face_index = a_is_top ? cap_candidates[i] : cap_candidates[j];
      const int bottom_face_index = a_is_top ? cap_candidates[j] : cap_candidates[i];
      const std::vector<int>& top_outer_loop = a_is_top ? outer_a : outer_b;
      const std::vector<int>& bottom_outer_loop = a_is_top ? outer_b : outer_a;
      const std::vector<int>& top_inner_loop = a_is_top ? inner_a : inner_b;
      const std::vector<int>& bottom_inner_loop = a_is_top ? inner_b : inner_a;
      const double top_inner_area = a_is_top ? inner_area_a : inner_area_b;
      const double bottom_inner_area = a_is_top ? inner_area_b : inner_area_a;

      std::unordered_set<int> inner_vertices;
      inner_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
      inner_vertices.insert(bottom_inner_loop.begin(), bottom_inner_loop.end());

      std::vector<int> outer_face_indices;
      int bore_face_count = 0;
      bool valid = true;
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == top_face_index ||
            static_cast<int>(face_index) == bottom_face_index) {
          continue;
        }
        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 4) {
          valid = false;
          break;
        }

        bool all_inner = true;
        for (int vertex_index : face.loops.front()) {
          if (!inner_vertices.contains(vertex_index)) {
            all_inner = false;
            break;
          }
        }

        if (all_inner) {
          int top_vertices = 0;
          int bottom_vertices = 0;
          for (int vertex_index : face.loops.front()) {
            const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
            if (std::abs(coordinate - top_plane) <= radius_tolerance) {
              ++top_vertices;
            } else if (std::abs(coordinate - bottom_plane) <= radius_tolerance) {
              ++bottom_vertices;
            }
          }
          if (top_vertices != 2 || bottom_vertices != 2) {
            valid = false;
            break;
          }
          ++bore_face_count;
        } else {
          outer_face_indices.push_back(static_cast<int>(face_index));
        }
      }

      if (!valid || bore_face_count < 8 || outer_face_indices.size() < 4) {
        continue;
      }

      const Vec2 center {
          (circle_a->center.x + circle_b->center.x) * 0.5,
          (circle_a->center.y + circle_b->center.y) * 0.5,
      };
      const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
      const Vec3 base_center = axis_origin + axis * bottom_plane;
      const Vec3 seam_vector =
          reconstruction.vertices[bottom_inner_loop.front()] - base_center;
      const Vec3 radial =
          seam_vector - axis * dot(seam_vector, axis);
      if (length_squared(radial) <= 1e-12) {
        continue;
      }

      SimplePrismaticBoreExport bore;
      bore.top_face_index = top_face_index;
      bore.bottom_face_index = bottom_face_index;
      bore.top_outer_loop = top_outer_loop;
      bore.bottom_outer_loop = bottom_outer_loop;
      bore.top_inner_area = top_inner_area;
      bore.bottom_inner_area = bottom_inner_area;
      bore.bore = SimpleCylinderPrimitive {base_center, axis, normalized(radial), radius, height};
      bore.outer_face_indices = std::move(outer_face_indices);
      return bore;
    }
  }

  return std::nullopt;
}

std::optional<SimplePrismaticMultiBoreExport> detect_simple_prismatic_multi_bore_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 8 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() >= 3) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.centroid - face_b.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, face_a.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      const std::optional<ClassifiedFaceMultiLoops> loops_a =
          classify_face_loops_outer_inner(face_a, reconstruction.vertices, axis_basis);
      const std::optional<ClassifiedFaceMultiLoops> loops_b =
          classify_face_loops_outer_inner(face_b, reconstruction.vertices, axis_basis);
      if (!loops_a.has_value() || !loops_b.has_value()) {
        continue;
      }
      if (loops_a->inner_loops.size() != loops_b->inner_loops.size() || loops_a->inner_loops.size() < 2) {
        continue;
      }

      const double plane_a = dot(face_a.fit.centroid, axis);
      const double plane_b = dot(face_b.fit.centroid, axis);
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
      const bool a_is_top = plane_a >= plane_b;

      const int top_face_index = a_is_top ? cap_candidates[i] : cap_candidates[j];
      const int bottom_face_index = a_is_top ? cap_candidates[j] : cap_candidates[i];
      const std::vector<int>& top_outer_loop = a_is_top ? loops_a->outer_loop : loops_b->outer_loop;
      const std::vector<int>& bottom_outer_loop = a_is_top ? loops_b->outer_loop : loops_a->outer_loop;
      const std::vector<std::vector<int>>& top_inner_loops =
          a_is_top ? loops_a->inner_loops : loops_b->inner_loops;
      const std::vector<std::vector<int>>& bottom_inner_loops =
          a_is_top ? loops_b->inner_loops : loops_a->inner_loops;
      const std::vector<double>& top_inner_areas =
          a_is_top ? loops_a->inner_areas : loops_b->inner_areas;
      const std::vector<double>& bottom_inner_areas =
          a_is_top ? loops_b->inner_areas : loops_a->inner_areas;

      struct FittedLoop {
        std::size_t loop_index {};
        CircleFit2D circle {};
      };

      std::vector<FittedLoop> fitted_top;
      std::vector<FittedLoop> fitted_bottom;
      fitted_top.reserve(top_inner_loops.size());
      fitted_bottom.reserve(bottom_inner_loops.size());

      bool fit_valid = true;
      for (std::size_t loop_index = 0; loop_index < top_inner_loops.size(); ++loop_index) {
        const std::optional<CircleFit2D> top_circle =
            fit_circle_2d(project_loop_to_basis(top_inner_loops[loop_index], reconstruction.vertices, axis_basis));
        const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(
            project_loop_to_basis(bottom_inner_loops[loop_index], reconstruction.vertices, axis_basis));
        if (!top_circle.has_value() || !bottom_circle.has_value()) {
          fit_valid = false;
          break;
        }
        fitted_top.push_back({loop_index, *top_circle});
        fitted_bottom.push_back({loop_index, *bottom_circle});
      }
      if (!fit_valid) {
        continue;
      }

      std::vector<bool> matched_bottom(fitted_bottom.size(), false);
      std::vector<SimplePrismaticMultiBore> bores;
      bores.reserve(fitted_top.size());
      std::vector<std::unordered_set<int>> bore_vertex_sets;
      bore_vertex_sets.reserve(fitted_top.size());

      for (const FittedLoop& top_loop : fitted_top) {
        const double radius_tolerance = std::max(top_loop.circle.radius * 0.03, 1e-4);
        if (top_loop.circle.radius <= 1e-6 || top_loop.circle.max_error > radius_tolerance) {
          fit_valid = false;
          break;
        }

        std::size_t best_index = fitted_bottom.size();
        double best_score = std::numeric_limits<double>::infinity();
        for (std::size_t bottom_index = 0; bottom_index < fitted_bottom.size(); ++bottom_index) {
          if (matched_bottom[bottom_index]) {
            continue;
          }
          const FittedLoop& bottom_loop = fitted_bottom[bottom_index];
          const double candidate_tolerance =
              std::max((top_loop.circle.radius + bottom_loop.circle.radius) * 0.015, 1e-4);
          const Vec2 center_delta {top_loop.circle.center.x - bottom_loop.circle.center.x,
                                   top_loop.circle.center.y - bottom_loop.circle.center.y};
          const double center_distance =
              std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
          if (std::abs(top_loop.circle.radius - bottom_loop.circle.radius) > candidate_tolerance ||
              bottom_loop.circle.max_error > candidate_tolerance || center_distance > candidate_tolerance) {
            continue;
          }
          const double score = center_distance + std::abs(top_loop.circle.radius - bottom_loop.circle.radius);
          if (score < best_score) {
            best_score = score;
            best_index = bottom_index;
          }
        }
        if (best_index == fitted_bottom.size()) {
          fit_valid = false;
          break;
        }

        matched_bottom[best_index] = true;
        const FittedLoop& bottom_loop = fitted_bottom[best_index];
        const double radius = 0.5 * (top_loop.circle.radius + bottom_loop.circle.radius);
        const Vec2 center {
            0.5 * (top_loop.circle.center.x + bottom_loop.circle.center.x),
            0.5 * (top_loop.circle.center.y + bottom_loop.circle.center.y),
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const Vec3 base_center = axis_origin + axis * bottom_plane;
        const Vec3 seam_vector =
            reconstruction.vertices[bottom_inner_loops[bottom_loop.loop_index].front()] - base_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          fit_valid = false;
          break;
        }

        SimplePrismaticMultiBore bore;
        bore.top_inner_area = top_inner_areas[top_loop.loop_index];
        bore.bottom_inner_area = bottom_inner_areas[bottom_loop.loop_index];
        bore.bore = SimpleCylinderPrimitive {base_center, axis, normalized(radial), radius, top_plane - bottom_plane};
        bores.push_back(std::move(bore));

        std::unordered_set<int> bore_vertices;
        bore_vertices.insert(top_inner_loops[top_loop.loop_index].begin(), top_inner_loops[top_loop.loop_index].end());
        bore_vertices.insert(
            bottom_inner_loops[bottom_loop.loop_index].begin(), bottom_inner_loops[bottom_loop.loop_index].end());
        bore_vertex_sets.push_back(std::move(bore_vertices));
      }
      if (!fit_valid) {
        continue;
      }

      std::unordered_set<int> all_inner_vertices;
      for (const auto& vertex_set : bore_vertex_sets) {
        all_inner_vertices.insert(vertex_set.begin(), vertex_set.end());
      }

      std::vector<int> outer_face_indices;
      std::vector<int> bore_face_counts(bores.size(), 0);
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == top_face_index ||
            static_cast<int>(face_index) == bottom_face_index) {
          continue;
        }
        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 4) {
          fit_valid = false;
          break;
        }

        bool all_inner = true;
        for (int vertex_index : face.loops.front()) {
          if (!all_inner_vertices.contains(vertex_index)) {
            all_inner = false;
            break;
          }
        }
        if (!all_inner) {
          outer_face_indices.push_back(static_cast<int>(face_index));
          continue;
        }

        std::size_t matching_bore = bores.size();
        for (std::size_t bore_index = 0; bore_index < bore_vertex_sets.size(); ++bore_index) {
          bool belongs = true;
          for (int vertex_index : face.loops.front()) {
            if (!bore_vertex_sets[bore_index].contains(vertex_index)) {
              belongs = false;
              break;
            }
          }
          if (belongs) {
            matching_bore = bore_index;
            break;
          }
        }
        if (matching_bore == bores.size()) {
          fit_valid = false;
          break;
        }
        ++bore_face_counts[matching_bore];
      }
      if (!fit_valid || outer_face_indices.size() < 4) {
        continue;
      }
      if (std::any_of(bore_face_counts.begin(), bore_face_counts.end(), [](int count) { return count < 8; })) {
        continue;
      }

      SimplePrismaticMultiBoreExport export_result;
      export_result.top_face_index = top_face_index;
      export_result.bottom_face_index = bottom_face_index;
      export_result.top_outer_loop = top_outer_loop;
      export_result.bottom_outer_loop = bottom_outer_loop;
      export_result.bores = std::move(bores);
      export_result.outer_face_indices = std::move(outer_face_indices);
      return export_result;
    }
  }

  return std::nullopt;
}

#include "output_step_types.inc"

std::optional<StepSmoothBandBundle> add_step_cylindrical_band_bundle(
    StepBuilder& step,
    const GenericSmoothExtrudedBand& band) {
  if (band.top_profile_points.size() < 3 ||
      band.top_profile_points.size() != band.bottom_profile_points.size()) {
    return std::nullopt;
  }

  const std::optional<CircleFit2D> top_circle = fit_circle_2d(band.top_profile_points);
  const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(band.bottom_profile_points);
  if (!top_circle.has_value() || !bottom_circle.has_value()) {
    return std::nullopt;
  }

  const double radius = 0.5 * (top_circle->radius + bottom_circle->radius);
  if (!std::isfinite(radius) || radius <= 1e-6) {
    return std::nullopt;
  }

  const double radius_tolerance =
      band.prefer_cylindrical_surface ? std::max(radius * 0.03, 0.08) : std::max(radius * 0.025, 1e-4);
  if (std::abs(top_circle->radius - bottom_circle->radius) > radius_tolerance ||
      top_circle->max_error > radius_tolerance ||
      bottom_circle->max_error > radius_tolerance ||
      !circle_fit_centers_coincident(*top_circle, *bottom_circle, radius_tolerance)) {
    return std::nullopt;
  }

  const Vec2 center {
      0.5 * (top_circle->center.x + bottom_circle->center.x),
      0.5 * (top_circle->center.y + bottom_circle->center.y),
  };

  auto project_point_to_circle = [&](const Vec2& point) {
    Vec2 radial {point.x - center.x, point.y - center.y};
    const double radial_length = std::sqrt(radial.x * radial.x + radial.y * radial.y);
    if (radial_length <= 1e-9) {
      return Vec2 {center.x + radius, center.y};
    }
    radial.x /= radial_length;
    radial.y /= radial_length;
    return Vec2 {center.x + radial.x * radius, center.y + radial.y * radius};
  };

  const Vec2 top_start_2d = project_point_to_circle(band.top_profile_points.front());
  const Vec2 top_end_2d = project_point_to_circle(band.top_profile_points.back());
  const Vec2 bottom_start_2d = project_point_to_circle(band.bottom_profile_points.front());
  const Vec2 bottom_end_2d = project_point_to_circle(band.bottom_profile_points.back());

  auto lift_radial = [&](const Vec2& point) {
    return band.basis.u * (point.x - center.x) + band.basis.v * (point.y - center.y);
  };

  Vec3 start_radial = lift_radial(top_start_2d) + lift_radial(bottom_start_2d);
  if (length_squared(start_radial) <= 1e-12) {
    start_radial = lift_radial(top_start_2d);
  }
  Vec3 end_radial = lift_radial(top_end_2d) + lift_radial(bottom_end_2d);
  if (length_squared(end_radial) <= 1e-12) {
    end_radial = lift_radial(top_end_2d);
  }
  start_radial = normalized(start_radial);
  end_radial = normalized(end_radial);
  if (length_squared(start_radial) <= 1e-12 || length_squared(end_radial) <= 1e-12) {
    return std::nullopt;
  }

  const Vec3 axis = normalized(band.axis);
  if (length_squared(axis) <= 1e-12) {
    return std::nullopt;
  }

  const double sweep_radians = std::acos(clamp(dot(start_radial, end_radial), -1.0, 1.0));
  const double sweep_degrees = sweep_radians * 180.0 / kPi;
  if (!std::isfinite(sweep_degrees) || sweep_degrees < 12.0 || sweep_degrees > 180.0) {
    return std::nullopt;
  }

  const bool same_sense = dot(cross(start_radial, end_radial), axis) >= 0.0;
  const Vec3 center_offset = band.basis.u * center.x + band.basis.v * center.y;
  const Vec3 bottom_center = center_offset + axis * band.bottom_plane;
  const Vec3 top_center = center_offset + axis * band.top_plane;
  const Vec3 bottom_start = bottom_center + start_radial * radius;
  const Vec3 bottom_end = bottom_center + end_radial * radius;
  const Vec3 top_start = top_center + start_radial * radius;
  const Vec3 top_end = top_center + end_radial * radius;

  auto add_point_vertex = [&](const Vec3& point) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," +
                                  format_double(point.z) + "))");
    return std::pair<int, int> {point_id, step.add("VERTEX_POINT(''," + step.ref(point_id) + ")")};
  };

  const auto [bottom_center_point, _bottom_center_vertex_unused] = add_point_vertex(bottom_center);
  const auto [top_center_point, _top_center_vertex_unused] = add_point_vertex(top_center);
  const auto [bottom_start_point, bottom_start_vertex] = add_point_vertex(bottom_start);
  const auto [bottom_end_point, bottom_end_vertex] = add_point_vertex(bottom_end);
  const auto [top_start_point, top_start_vertex] = add_point_vertex(top_start);
  const auto [top_end_point, top_end_vertex] = add_point_vertex(top_end);
  (void)_bottom_center_vertex_unused;
  (void)_top_center_vertex_unused;

  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," +
               format_double(axis.y) + "," + format_double(axis.z) + "))");
  const int ref_direction =
      step.add("DIRECTION('',(" + format_double(start_radial.x) + "," +
               format_double(start_radial.y) + "," + format_double(start_radial.z) + "))");

  const int cylinder_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface =
      step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
               format_double(radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");

  const int top_circle_id = step.add("CIRCLE(''," + step.ref(top_axis) + "," + format_double(radius) + ")");
  const int bottom_circle_id =
      step.add("CIRCLE(''," + step.ref(bottom_axis) + "," + format_double(radius) + ")");
  const int top_trimmed_curve =
      step.add("TRIMMED_CURVE(''," + step.ref(top_circle_id) + ",(" + step.ref(top_start_point) + "),(" +
               step.ref(top_end_point) + ")," + std::string(same_sense ? ".T." : ".F.") + ",.CARTESIAN.)");
  const int bottom_trimmed_curve =
      step.add("TRIMMED_CURVE(''," + step.ref(bottom_circle_id) + ",(" + step.ref(bottom_start_point) + "),(" +
               step.ref(bottom_end_point) + ")," + std::string(same_sense ? ".T." : ".F.") + ",.CARTESIAN.)");

  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_start_vertex) + "," + step.ref(top_end_vertex) + "," +
               step.ref(top_trimmed_curve) + ",.T.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_start_vertex) + "," +
               step.ref(bottom_end_vertex) + "," + step.ref(bottom_trimmed_curve) + ",.T.)");

  auto add_line_edge = [&](const Vec3& start,
                           int from_point,
                           int from_vertex,
                           const Vec3& end,
                           int to_vertex) {
    const Vec3 delta = end - start;
    const double edge_length = length(delta);
    if (edge_length <= 1e-12) {
      throw std::runtime_error("Cannot export zero-length cylindrical band seam edge");
    }
    const Vec3 direction = normalized(delta);
    const int direction_id =
        step.add("DIRECTION('',(" + format_double(direction.x) + "," + format_double(direction.y) + "," +
                 format_double(direction.z) + "))");
    const int vector_id = step.add("VECTOR(''," + step.ref(direction_id) + "," +
                                   format_double(edge_length) + ")");
    const int line_id = step.add("LINE(''," + step.ref(from_point) + "," + step.ref(vector_id) + ")");
    return step.add("EDGE_CURVE(''," + step.ref(from_vertex) + "," + step.ref(to_vertex) + "," +
                    step.ref(line_id) + ",.T.)");
  };

  const int start_seam_edge_curve =
      add_line_edge(bottom_start, bottom_start_point, bottom_start_vertex, top_start, top_start_vertex);
  const int end_seam_edge_curve =
      add_line_edge(bottom_end, bottom_end_point, bottom_end_vertex, top_end, top_end_vertex);

  const int top_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.T.)");
  const int end_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(end_seam_edge_curve) + ",.F.)");
  const int bottom_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.F.)");
  const int start_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(start_seam_edge_curve) + ",.T.)");
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(top_oriented_edge) + "," +
                                    step.ref(end_seam_oriented) + "," +
                                    step.ref(bottom_oriented_edge) + "," +
                                    step.ref(start_seam_oriented) + "))");
  const int lateral_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  return StepSmoothBandBundle {
      top_edge_curve,
      bottom_edge_curve,
      start_seam_edge_curve,
      end_seam_edge_curve,
      lateral_face,
  };
}

std::pair<std::vector<int>, std::vector<double>> open_uniform_knot_data(int control_point_count,
                                                                        int degree) {
  if (control_point_count < 2) {
    return {{}, {}};
  }
  const int clamped_degree = std::max(1, std::min(degree, control_point_count - 1));
  const int distinct_knot_count = control_point_count - clamped_degree + 1;
  std::vector<int> multiplicities(distinct_knot_count, 1);
  multiplicities.front() = clamped_degree + 1;
  multiplicities.back() = clamped_degree + 1;
  std::vector<double> knots;
  knots.reserve(distinct_knot_count);
  for (int index = 0; index < distinct_knot_count; ++index) {
    knots.push_back(static_cast<double>(index));
  }
  return {multiplicities, knots};
}

int add_step_b_spline_curve(StepBuilder& step,
                            const std::vector<Vec3>& points,
                            int degree = 3) {
  if (points.size() < 2) {
    throw std::runtime_error("Cannot export B-spline curve with fewer than 2 points");
  }

  const int clamped_degree = std::max(1, std::min(degree, static_cast<int>(points.size()) - 1));
  std::ostringstream point_refs;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Vec3& point = points[index];
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    if (index > 0) {
      point_refs << ",";
    }
    point_refs << step.ref(point_id);
  }

  const auto [multiplicities, knots] = open_uniform_knot_data(static_cast<int>(points.size()), clamped_degree);
  if (multiplicities.empty() || knots.empty()) {
    throw std::runtime_error("Unable to compute B-spline knot data");
  }

  std::ostringstream multiplicity_values;
  for (std::size_t index = 0; index < multiplicities.size(); ++index) {
    if (index > 0) {
      multiplicity_values << ",";
    }
    multiplicity_values << multiplicities[index];
  }
  std::ostringstream knot_values;
  for (std::size_t index = 0; index < knots.size(); ++index) {
    if (index > 0) {
      knot_values << ",";
    }
    knot_values << format_double(knots[index]);
  }

  return step.add("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(clamped_degree) + ",(" +
                  point_refs.str() + "),.UNSPECIFIED.,.F.,.F.,(" + multiplicity_values.str() +
                  "),(" + knot_values.str() + "),.UNSPECIFIED.)");
}

int add_step_b_spline_surface(StepBuilder& step,
                              const std::vector<Vec3>& bottom_points,
                              const std::vector<Vec3>& top_points,
                              int degree = 3) {
  if (bottom_points.size() < 2 || bottom_points.size() != top_points.size()) {
    throw std::runtime_error("Cannot export B-spline surface without matching profile rows");
  }

  const int u_degree = std::max(1, std::min(degree, static_cast<int>(bottom_points.size()) - 1));
  const int v_degree = 1;
  std::ostringstream control_rows;
  for (std::size_t index = 0; index < bottom_points.size(); ++index) {
    const int bottom_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_points[index].x) + "," +
                 format_double(bottom_points[index].y) + "," +
                 format_double(bottom_points[index].z) + "))");
    const int top_point = step.add("CARTESIAN_POINT('',(" + format_double(top_points[index].x) + "," +
                                      format_double(top_points[index].y) + "," +
                                      format_double(top_points[index].z) + "))");
    if (index > 0) {
      control_rows << ",";
    }
    control_rows << "(" << step.ref(bottom_point) << "," << step.ref(top_point) << ")";
  }

  const auto [u_multiplicities, u_knots] =
      open_uniform_knot_data(static_cast<int>(bottom_points.size()), u_degree);
  const auto [v_multiplicities, v_knots] = open_uniform_knot_data(2, v_degree);
  if (u_multiplicities.empty() || u_knots.empty() ||
      v_multiplicities.empty() || v_knots.empty()) {
    throw std::runtime_error("Unable to compute B-spline surface knot data");
  }

  auto emit_int_list = [](const std::vector<int>& values) {
    std::ostringstream output;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output << ",";
      }
      output << values[index];
    }
    return output.str();
  };
  auto emit_double_list = [](const std::vector<double>& values) {
    std::ostringstream output;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output << ",";
      }
      output << format_double(values[index]);
    }
    return output.str();
  };

  return step.add("B_SPLINE_SURFACE_WITH_KNOTS(''," + std::to_string(u_degree) + "," +
                  std::to_string(v_degree) + ",(" + control_rows.str() + "),.UNSPECIFIED.,.F.,.F.,.F.,(" +
                  emit_int_list(u_multiplicities) + "),(" + emit_int_list(v_multiplicities) +
                  "),(" + emit_double_list(u_knots) + "),(" + emit_double_list(v_knots) +
                  "),.UNSPECIFIED.)");
}

StepExportContext build_step_export_context(StepBuilder& step) {
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
  const int context_2d = step.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) "
                                  "PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )");

  const int origin = step.add("CARTESIAN_POINT('',(0.0,0.0,0.0))");
  const int z_direction = step.add("DIRECTION('',(0.0,0.0,1.0))");
  const int x_direction = step.add("DIRECTION('',(1.0,0.0,0.0))");
  const int global_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin) + "," + step.ref(z_direction) + "," +
               step.ref(x_direction) + ")");

  return StepExportContext {product_shape, context, context_2d, global_axis};
}

StepPlaneSurface add_step_plane_surface(StepBuilder& step,
                                        const Vec3& origin,
                                        const Vec3& normal,
                                        const Vec3& ref_direction) {
  const Vec3 local_normal = normalized(normal);
  Vec3 local_ref = ref_direction - local_normal * dot(ref_direction, local_normal);
  if (length_squared(local_ref) <= 1e-12) {
    local_ref = basis_from_normal(local_normal).u;
  } else {
    local_ref = normalized(local_ref);
  }
  const Vec3 local_v = normalized(cross(local_normal, local_ref));
  const int origin_id = step.add("CARTESIAN_POINT('',(" + format_double(origin.x) + "," +
                               format_double(origin.y) + "," + format_double(origin.z) + "))");
  const int normal_id = step.add("DIRECTION('',(" + format_double(local_normal.x) + "," +
                               format_double(local_normal.y) + "," +
                               format_double(local_normal.z) + "))");
  const int ref_id = step.add("DIRECTION('',(" + format_double(local_ref.x) + "," +
                            format_double(local_ref.y) + "," + format_double(local_ref.z) + "))");
  const int axis_id = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin_id) + "," +
                               step.ref(normal_id) + "," + step.ref(ref_id) + ")");
  return StepPlaneSurface {
      step.add("PLANE(''," + step.ref(axis_id) + ")"),
      origin,
      local_normal,
      PlaneBasis {local_ref, local_v},
  };
}

StepCylinderBundle add_step_cylinder_bundle(StepBuilder& step,
                                            const StepExportContext& context_ids,
                                            const SimpleCylinderPrimitive& cylinder,
                                            const StepPlaneSurface& top_plane,
                                            const StepPlaneSurface& bottom_plane) {
  const Vec3 top_center = cylinder.base_center + cylinder.axis * cylinder.height;
  const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
  const Vec3 top_seam = top_center + cylinder.ref_direction * cylinder.radius;
  const double circumference = 2.0 * kPi * cylinder.radius;

  const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                     format_double(top_seam.y) + "," +
                                     format_double(top_seam.z) + "))");
  const int bottom_seam_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
               format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
  const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
  const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

  const int base_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
               format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
  const int top_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
               format_double(top_center.y) + "," + format_double(top_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
               format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
  const int ref_direction =
      step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
               format_double(cylinder.ref_direction.y) + "," +
               format_double(cylinder.ref_direction.z) + "))");

  const int cylinder_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface =
      step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
               format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");

  auto add_plane_circle_pcurve = [&](const StepPlaneSurface& plane,
                                     const Vec3& circle_center) -> int {
    const Vec3 delta = circle_center - plane.origin;
    const double center_u = dot(delta, plane.basis.u);
    const double center_v = dot(delta, plane.basis.v);
    const double ref_u_raw = dot(cylinder.ref_direction, plane.basis.u);
    const double ref_v_raw = dot(cylinder.ref_direction, plane.basis.v);
    const double ref_scale = std::sqrt(ref_u_raw * ref_u_raw + ref_v_raw * ref_v_raw);
    const double ref_u = ref_scale <= 1e-12 ? 1.0 : ref_u_raw / ref_scale;
    const double ref_v = ref_scale <= 1e-12 ? 0.0 : ref_v_raw / ref_scale;

    const int center_2d =
        step.add("CARTESIAN_POINT('',(" + format_double(center_u) + "," +
                 format_double(center_v) + "))");
    const int ref_2d = step.add("DIRECTION('',(" + format_double(ref_u) + "," +
                                format_double(ref_v) + "))");
    const int circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(center_2d) + "," + step.ref(ref_2d) + ")");
    const int circle_2d =
        step.add("CIRCLE(''," + step.ref(circle_axis_2d) + "," + format_double(cylinder.radius) + ")");
    const int circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    return step.add("PCURVE(''," + step.ref(plane.plane_id) + "," + step.ref(circle_repr) + ")");
  };

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec = step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");
  const int top_plane_pcurve = add_plane_circle_pcurve(top_plane, top_center);
  const int top_surface_curve =
      step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) + "," +
               step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line =
      step.add("LINE(''," + step.ref(bottom_seam_point) + "," + step.ref(seam_line_vec) + ")");
  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line =
      step.add("LINE(''," + step.ref(seam_u0_point) + "," + step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line =
      step.add("LINE(''," + step.ref(seam_u1_point) + "," + step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
  const int seam_curve =
      step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" + step.ref(seam_u1_pcurve) + "," +
               step.ref(seam_u0_pcurve) + "),.PCURVE_S1.)");
  const int seam_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
               step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

  const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis) + "," +
                                     format_double(cylinder.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
               step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_pcurve = add_plane_circle_pcurve(bottom_plane, cylinder.base_center);
  const int bottom_surface_curve =
      step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
               step.ref(bottom_cyl_pcurve) + "," + step.ref(bottom_plane_pcurve) +
               "),.PCURVE_S1.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
               step.ref(bottom_seam_vertex) + "," + step.ref(bottom_surface_curve) + ",.T.)");

  const int lateral_top_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
  const int lateral_seam_forward =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.F.)");
  const int lateral_bottom_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.T.)");
  const int lateral_seam_backward =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.T.)");
  const int lateral_loop =
      step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
               step.ref(lateral_seam_forward) + "," + step.ref(lateral_bottom_oriented) + "," +
               step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face =
      step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
               step.ref(cylinder_surface) + ",.T.)");

  return StepCylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
}

StepSmoothBandBundle add_step_smooth_band_bundle(StepBuilder& step,
                                                 const GenericSmoothExtrudedBand& band) {
  if (band.top_profile_points.size() < 2 || band.top_profile_points.size() != band.bottom_profile_points.size()) {
    throw std::runtime_error("Invalid smooth band profile");
  }

  if (const std::optional<StepSmoothBandBundle> cylindrical_bundle =
          add_step_cylindrical_band_bundle(step, band);
      cylindrical_bundle.has_value()) {
    return *cylindrical_bundle;
  }

  auto lift_profile = [&](const std::vector<Vec2>& profile, double plane) {
    std::vector<Vec3> points;
    points.reserve(profile.size());
    for (const Vec2& point : profile) {
      points.push_back(band.basis.u * point.x + band.basis.v * point.y + band.axis * plane);
    }
    return points;
  };

  const std::vector<Vec3> top_points = lift_profile(band.top_profile_points, band.top_plane);
  const std::vector<Vec3> bottom_points = lift_profile(band.bottom_profile_points, band.bottom_plane);

  const int top_curve = add_step_b_spline_curve(step, top_points);
  const int bottom_curve = add_step_b_spline_curve(step, bottom_points);
  const int surface = add_step_b_spline_surface(step, bottom_points, top_points);

  auto add_vertex_point = [&](const Vec3& point) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    return std::pair<int, int> {point_id, step.add("VERTEX_POINT(''," + step.ref(point_id) + ")")};
  };

  const auto [top_start_point, top_start_vertex] = add_vertex_point(top_points.front());
  const auto [top_end_point, top_end_vertex] = add_vertex_point(top_points.back());
  const auto [bottom_start_point, bottom_start_vertex] = add_vertex_point(bottom_points.front());
  const auto [bottom_end_point, bottom_end_vertex] = add_vertex_point(bottom_points.back());

  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_start_vertex) + "," + step.ref(top_end_vertex) + "," +
               step.ref(top_curve) + ",.T.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_start_vertex) + "," +
               step.ref(bottom_end_vertex) + "," + step.ref(bottom_curve) + ",.T.)");

  auto add_line_edge = [&](const Vec3& start,
                           int from_point,
                           int from_vertex,
                           const Vec3& end,
                           int /*to_point*/,
                           int to_vertex) {
    const Vec3 delta = end - start;
    const double edge_length = length(delta);
    if (edge_length <= 1e-12) {
      throw std::runtime_error("Cannot export zero-length smooth band seam edge");
    }
    const Vec3 direction = normalized(delta);
    const int direction_id =
        step.add("DIRECTION('',(" + format_double(direction.x) + "," + format_double(direction.y) +
                 "," + format_double(direction.z) + "))");
    const int vector_id = step.add("VECTOR(''," + step.ref(direction_id) + "," +
                                   format_double(edge_length) + ")");
    const int line_id = step.add("LINE(''," + step.ref(from_point) + "," + step.ref(vector_id) + ")");
    return step.add("EDGE_CURVE(''," + step.ref(from_vertex) + "," + step.ref(to_vertex) + "," +
                    step.ref(line_id) + ",.T.)");
  };

  const int start_seam_edge_curve = add_line_edge(bottom_points.front(),
                                                  bottom_start_point,
                                                  bottom_start_vertex,
                                                  top_points.front(),
                                                  top_start_point,
                                                  top_start_vertex);
  const int end_seam_edge_curve = add_line_edge(bottom_points.back(),
                                                bottom_end_point,
                                                bottom_end_vertex,
                                                top_points.back(),
                                                top_end_point,
                                                top_end_vertex);

  const int top_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.T.)");
  const int end_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(end_seam_edge_curve) + ",.F.)");
  const int bottom_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.F.)");
  const int start_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(start_seam_edge_curve) + ",.T.)");
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(top_oriented_edge) + "," +
                                    step.ref(end_seam_oriented) + "," +
                                    step.ref(bottom_oriented_edge) + "," +
                                    step.ref(start_seam_oriented) + "))");
  const int lateral_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(surface) + ",.T.)");

  return StepSmoothBandBundle {
      top_edge_curve,
      bottom_edge_curve,
      start_seam_edge_curve,
      end_seam_edge_curve,
      lateral_face,
  };
}

#include "output_cylinder_solids.inc"

#include "output_step_loop.inc"
#include "output_axial.inc"
#include "output_prismatic_sections.inc"
#include "output_prismatic_blind_sections.inc"
#include "output_prismatic_support_sections.inc"
#include "output_obround.inc"
#include "output_profile_sweeps.inc"
#include "output_cylindrical.inc"

#include "output_fallback_step.inc"

bool write_step_file(const std::filesystem::path& path,
                     const ReconstructionResult& reconstruction,
                     const MeshModel& mesh,
                     const Tolerances& tolerances) {
  if (reconstruction.outcome != ReconstructionOutcome::SolidCreated) {
    return false;
  }

  if (const std::optional<std::vector<SimpleCylinderPrimitive>> cylinders =
          collect_simple_cylinder_primitives(mesh, tolerances);
      cylinders.has_value()) {
    return write_step_file_simple_cylinders(path, *cylinders);
  }

  if (const std::optional<PrismaticBlindSectionModel> blind_sections =
          detect_prismatic_blind_section_model(reconstruction);
      blind_sections.has_value()) {
    return write_step_file_prismatic_blind_section_model(path, reconstruction, *blind_sections);
  }

  if (const std::optional<PrismaticSupportSectionModel> support_sections =
          detect_prismatic_support_section_model(reconstruction);
      support_sections.has_value()) {
    return write_step_file_prismatic_support_section_model(path, reconstruction, *support_sections);
  }

  const std::optional<GenericCylindricalThroughHoleExport> generic_holes =
      detect_generic_cylindrical_through_hole_export(reconstruction);
  const std::optional<GenericCylindricalBlindFeatureExport> generic_blind_features =
      detect_generic_cylindrical_blind_feature_export(reconstruction);
  const std::optional<GenericSmoothExtrudedBandExport> generic_smooth_bands =
      detect_generic_smooth_extruded_band_export(reconstruction);

  if (const std::optional<PrismaticProfileSweepModel> profile_sweep_model =
          detect_prismatic_profile_sweep_model(
              reconstruction,
              generic_smooth_bands,
              !generic_holes.has_value() && !generic_blind_features.has_value());
      profile_sweep_model.has_value()) {
    return write_step_file_prismatic_profile_sweep_model(path, reconstruction, *profile_sweep_model);
  }

  const std::optional<GenericCylindricalFeatureExport> generic_features =
      merge_generic_cylindrical_feature_exports(generic_holes, generic_blind_features, generic_smooth_bands);
  const bool generic_features_cover_nontrivial_side_geometry =
      generic_features.has_value() &&
      (!generic_features->blind_features.empty() || !generic_features->smooth_bands.empty());

  if (!generic_features_cover_nontrivial_side_geometry) {
    if (const std::optional<PrismaticAxialSectionModel> axial_sections =
            detect_prismatic_axial_section_model(reconstruction);
        axial_sections.has_value()) {
      return write_step_file_prismatic_axial_section_model(path, reconstruction, *axial_sections);
    }
  }

  if (generic_features.has_value()) {
    return write_step_file_generic_cylindrical_features(path, reconstruction, *generic_features);
  }

  if (const std::optional<PrismaticAxialSectionModel> axial_sections =
          detect_prismatic_axial_section_model(reconstruction);
      axial_sections.has_value()) {
    return write_step_file_prismatic_axial_section_model(path, reconstruction, *axial_sections);
  }

  return write_step_file_generic_planar(path, reconstruction);
}

#include "output_reporting.inc"
