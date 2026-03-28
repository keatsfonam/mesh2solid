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

#include "output_prismatic_types.inc"

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

#include "output_prismatic_support_detection.inc"

#include "output_prismatic_blind_detection.inc"
#include "output_prismatic_axial_detection.inc"

#include "output_step_types.inc"
#include "output_step_helpers.inc"
#include "output_step_bundles.inc"

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
