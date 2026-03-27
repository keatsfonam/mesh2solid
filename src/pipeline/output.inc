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

struct SimplePrismaticMultiCounterbore {
  double top_inner_area {};
  double bottom_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
};

struct SimplePrismaticMultiCounterboreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiCounterbore> counterbores;
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

struct SimplePrismaticObroundSlotExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  Vec3 slot_origin {};
  Vec3 axis {};
  Vec3 major_direction {};
  Vec3 minor_direction {};
  double top_plane {};
  double bottom_plane {};
  double radius {};
  double half_span {};
  std::vector<int> outer_face_indices;
};

struct GenericCylindricalThroughHole {
  int top_face_index {};
  int bottom_face_index {};
  std::size_t top_loop_index {};
  std::size_t bottom_loop_index {};
  double top_loop_area {};
  double bottom_loop_area {};
  SimpleCylinderPrimitive cylinder {};
  std::vector<int> side_face_indices;
};

struct GenericCylindricalThroughHoleExport {
  std::vector<GenericCylindricalThroughHole> holes;
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

std::optional<SimplePrismaticObroundSlotExport> detect_simple_prismatic_obround_slot_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 16) {
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
      if (outer_a.size() != 4 || outer_b.size() != 4 || inner_a.size() < 6 || inner_b.size() < 6) {
        continue;
      }

      const std::optional<ObroundFit2D> fit_a =
          fit_obround_2d(project_loop_to_basis(inner_a, reconstruction.vertices, axis_basis));
      const std::optional<ObroundFit2D> fit_b =
          fit_obround_2d(project_loop_to_basis(inner_b, reconstruction.vertices, axis_basis));
      if (!fit_a.has_value() || !fit_b.has_value()) {
        continue;
      }

      const double radius = 0.5 * (fit_a->radius + fit_b->radius);
      const double span = 0.5 * (fit_a->half_span + fit_b->half_span);
      const double radius_tolerance = std::max(radius * 0.08, 0.2);
      const double span_tolerance = std::max(radius * 0.12, 0.25);
      const double center_tolerance = std::max(radius * 0.12, 0.25);
      if (std::abs(fit_a->radius - fit_b->radius) > radius_tolerance ||
          std::abs(fit_a->half_span - fit_b->half_span) > span_tolerance) {
        continue;
      }

      const Vec2 center_delta {fit_a->center.x - fit_b->center.x, fit_a->center.y - fit_b->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          center_tolerance) {
        continue;
      }

      const double basis_alignment =
          std::abs(fit_a->major.x * fit_b->major.x + fit_a->major.y * fit_b->major.y);
      if (basis_alignment < std::cos(6.0 * kPi / 180.0)) {
        continue;
      }

      const double plane_a = dot(face_a.fit.centroid, axis);
      const double plane_b = dot(face_b.fit.centroid, axis);
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
      if (top_plane - bottom_plane <= radius_tolerance) {
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
      int inner_face_count = 0;
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
          ++inner_face_count;
        } else {
          if (angle_degrees_unsigned(face.fit.normal, axis) < 2.0) {
            valid = false;
            break;
          }
          outer_face_indices.push_back(static_cast<int>(face_index));
        }
      }

      if (!valid || inner_face_count < 4 || outer_face_indices.size() != 4) {
        continue;
      }

      Vec2 averaged_major = fit_a->major;
      if (fit_a->major.x * fit_b->major.x + fit_a->major.y * fit_b->major.y < 0.0) {
        averaged_major.x += -fit_b->major.x;
        averaged_major.y += -fit_b->major.y;
      } else {
        averaged_major.x += fit_b->major.x;
        averaged_major.y += fit_b->major.y;
      }
      const double major_length =
          std::sqrt(averaged_major.x * averaged_major.x + averaged_major.y * averaged_major.y);
      if (major_length <= 1e-12) {
        continue;
      }
      averaged_major.x /= major_length;
      averaged_major.y /= major_length;
      const Vec2 averaged_minor {-averaged_major.y, averaged_major.x};

      Vec3 major_direction =
          normalized(axis_basis.u * averaged_major.x + axis_basis.v * averaged_major.y);
      Vec3 minor_direction =
          normalized(axis_basis.u * averaged_minor.x + axis_basis.v * averaged_minor.y);
      if (dot(cross(major_direction, minor_direction), axis) < 0.0) {
        minor_direction = minor_direction * -1.0;
      }

      const Vec2 averaged_center {
          0.5 * (fit_a->center.x + fit_b->center.x),
          0.5 * (fit_a->center.y + fit_b->center.y),
      };

      SimplePrismaticObroundSlotExport slot;
      slot.top_face_index = top_face_index;
      slot.bottom_face_index = bottom_face_index;
      slot.top_outer_loop = top_outer_loop;
      slot.bottom_outer_loop = bottom_outer_loop;
      slot.top_inner_area = top_inner_area;
      slot.bottom_inner_area = bottom_inner_area;
      slot.slot_origin = axis_basis.u * averaged_center.x + axis_basis.v * averaged_center.y;
      slot.axis = axis;
      slot.major_direction = major_direction;
      slot.minor_direction = minor_direction;
      slot.top_plane = top_plane;
      slot.bottom_plane = bottom_plane;
      slot.radius = radius;
      slot.half_span = span;
      slot.outer_face_indices = std::move(outer_face_indices);
      return slot;
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

std::optional<SimplePrismaticMultiCounterboreExport> detect_simple_prismatic_multi_counterbore_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 12 || reconstruction.vertices.size() < 32) {
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
  if (cap_candidates.size() < 2 || step_candidates.size() < 2) {
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
      const bool a_is_top = plane_a >= plane_b;
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
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

      struct LoopCircle {
        std::size_t loop_index {};
        CircleFit2D circle {};
      };
      std::vector<LoopCircle> fitted_top;
      std::vector<LoopCircle> fitted_bottom;
      fitted_top.reserve(top_inner_loops.size());
      fitted_bottom.reserve(bottom_inner_loops.size());
      bool fit_valid = true;
      for (std::size_t loop_index = 0; loop_index < top_inner_loops.size(); ++loop_index) {
        const std::optional<CircleFit2D> top_circle =
            fit_circle_2d(project_loop_to_basis(top_inner_loops[loop_index], reconstruction.vertices, axis_basis));
        const std::optional<CircleFit2D> bottom_circle =
            fit_circle_2d(project_loop_to_basis(bottom_inner_loops[loop_index], reconstruction.vertices, axis_basis));
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

      struct PendingCounterbore {
        std::size_t top_loop_index {};
        std::size_t bottom_loop_index {};
        std::size_t step_face_index {};
        CircleFit2D top_circle {};
        CircleFit2D step_outer_circle {};
        CircleFit2D step_inner_circle {};
        CircleFit2D bottom_circle {};
        double step_outer_area {};
        double step_inner_area {};
        double step_plane {};
        std::vector<int> step_outer_loop;
        std::vector<int> step_inner_loop;
      };

      std::vector<bool> matched_bottom(fitted_bottom.size(), false);
      std::vector<bool> matched_step(step_candidates.size(), false);
      std::vector<PendingCounterbore> pending;
      pending.reserve(fitted_top.size());

      for (const LoopCircle& top_loop : fitted_top) {
        if (top_loop.circle.radius <= 1e-6) {
          fit_valid = false;
          break;
        }
        std::size_t bottom_match = fitted_bottom.size();
        double best_bottom_score = std::numeric_limits<double>::infinity();
        for (std::size_t bottom_index = 0; bottom_index < fitted_bottom.size(); ++bottom_index) {
          if (matched_bottom[bottom_index]) {
            continue;
          }
          const LoopCircle& bottom_loop = fitted_bottom[bottom_index];
          const double center_tolerance =
              std::max((top_loop.circle.radius + bottom_loop.circle.radius) * 0.03, 1e-4);
          const Vec2 center_delta {top_loop.circle.center.x - bottom_loop.circle.center.x,
                                   top_loop.circle.center.y - bottom_loop.circle.center.y};
          const double center_distance =
              std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
          if (bottom_loop.circle.max_error > center_tolerance || center_distance > center_tolerance) {
            continue;
          }
          const double score = center_distance + std::abs(top_loop.circle.radius - bottom_loop.circle.radius);
          if (score < best_bottom_score) {
            best_bottom_score = score;
            bottom_match = bottom_index;
          }
        }
        if (bottom_match == fitted_bottom.size()) {
          fit_valid = false;
          break;
        }

        const LoopCircle& bottom_loop = fitted_bottom[bottom_match];
        if (top_loop.circle.radius <= bottom_loop.circle.radius + std::max(bottom_loop.circle.radius * 0.05, 1e-4)) {
          fit_valid = false;
          break;
        }

        std::size_t step_match = step_candidates.size();
        PendingCounterbore best_pending;
        double best_step_score = std::numeric_limits<double>::infinity();
        for (std::size_t step_index = 0; step_index < step_candidates.size(); ++step_index) {
          if (matched_step[step_index]) {
            continue;
          }
          const ReconstructedFace& step_face = reconstruction.faces[step_candidates[step_index]];
          if (angle_degrees_unsigned(face_a.fit.normal, step_face.fit.normal) > 8.0) {
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
          const std::optional<CircleFit2D> step_outer_circle =
              fit_circle_2d(project_loop_to_basis(step_outer_loop, reconstruction.vertices, axis_basis));
          const std::optional<CircleFit2D> step_inner_circle =
              fit_circle_2d(project_loop_to_basis(step_inner_loop, reconstruction.vertices, axis_basis));
          if (!step_outer_circle.has_value() || !step_inner_circle.has_value()) {
            continue;
          }

          const double outer_tolerance =
              std::max((top_loop.circle.radius + step_outer_circle->radius) * 0.015, 1e-4);
          const double inner_tolerance =
              std::max((bottom_loop.circle.radius + step_inner_circle->radius) * 0.015, 1e-4);
          const Vec2 outer_delta {top_loop.circle.center.x - step_outer_circle->center.x,
                                  top_loop.circle.center.y - step_outer_circle->center.y};
          const Vec2 inner_delta {bottom_loop.circle.center.x - step_inner_circle->center.x,
                                  bottom_loop.circle.center.y - step_inner_circle->center.y};
          const double outer_center_distance =
              std::sqrt(outer_delta.x * outer_delta.x + outer_delta.y * outer_delta.y);
          const double inner_center_distance =
              std::sqrt(inner_delta.x * inner_delta.x + inner_delta.y * inner_delta.y);
          if (std::abs(top_loop.circle.radius - step_outer_circle->radius) > outer_tolerance ||
              std::abs(bottom_loop.circle.radius - step_inner_circle->radius) > inner_tolerance ||
              step_outer_circle->max_error > outer_tolerance || step_inner_circle->max_error > inner_tolerance ||
              outer_center_distance > outer_tolerance || inner_center_distance > inner_tolerance) {
            continue;
          }

          const double score = outer_center_distance + inner_center_distance +
                               std::abs(top_loop.circle.radius - step_outer_circle->radius) +
                               std::abs(bottom_loop.circle.radius - step_inner_circle->radius);
          if (score < best_step_score) {
            best_step_score = score;
            best_pending = PendingCounterbore {
                top_loop.loop_index,
                bottom_loop.loop_index,
                step_index,
                top_loop.circle,
                *step_outer_circle,
                *step_inner_circle,
                bottom_loop.circle,
                step_outer_area,
                step_inner_area,
                step_plane,
                step_outer_loop,
                step_inner_loop,
            };
            step_match = step_index;
          }
        }
        if (step_match == step_candidates.size()) {
          fit_valid = false;
          break;
        }

        matched_bottom[bottom_match] = true;
        matched_step[step_match] = true;
        pending.push_back(std::move(best_pending));
      }

      if (!fit_valid || pending.size() != top_inner_loops.size()) {
        continue;
      }

      std::unordered_set<int> all_internal_vertices;
      std::vector<std::unordered_set<int>> upper_vertex_sets;
      std::vector<std::unordered_set<int>> lower_vertex_sets;
      upper_vertex_sets.reserve(pending.size());
      lower_vertex_sets.reserve(pending.size());
      for (const PendingCounterbore& counterbore : pending) {
        std::unordered_set<int> upper_vertices;
        upper_vertices.insert(top_inner_loops[counterbore.top_loop_index].begin(),
                              top_inner_loops[counterbore.top_loop_index].end());
        upper_vertices.insert(counterbore.step_outer_loop.begin(), counterbore.step_outer_loop.end());
        std::unordered_set<int> lower_vertices;
        lower_vertices.insert(bottom_inner_loops[counterbore.bottom_loop_index].begin(),
                              bottom_inner_loops[counterbore.bottom_loop_index].end());
        lower_vertices.insert(counterbore.step_inner_loop.begin(), counterbore.step_inner_loop.end());
        all_internal_vertices.insert(upper_vertices.begin(), upper_vertices.end());
        all_internal_vertices.insert(lower_vertices.begin(), lower_vertices.end());
        upper_vertex_sets.push_back(std::move(upper_vertices));
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
        bool is_step = false;
        for (const PendingCounterbore& counterbore : pending) {
          if (static_cast<int>(face_index) == step_candidates[counterbore.step_face_index]) {
            is_step = true;
            break;
          }
        }
        if (is_step) {
          continue;
        }

        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 3) {
          fit_valid = false;
          break;
        }

        bool all_internal = true;
        for (int vertex_index : face.loops.front()) {
          if (!all_internal_vertices.contains(vertex_index)) {
            all_internal = false;
            break;
          }
        }
        if (!all_internal) {
          outer_face_indices.push_back(static_cast<int>(face_index));
          continue;
        }

        std::size_t matched_feature = pending.size();
        bool matched_upper = false;
        for (std::size_t feature_index = 0; feature_index < pending.size(); ++feature_index) {
          bool belongs_upper = true;
          for (int vertex_index : face.loops.front()) {
            if (!upper_vertex_sets[feature_index].contains(vertex_index)) {
              belongs_upper = false;
              break;
            }
          }
          if (belongs_upper) {
            matched_feature = feature_index;
            matched_upper = true;
            break;
          }
          bool belongs_lower = true;
          for (int vertex_index : face.loops.front()) {
            if (!lower_vertex_sets[feature_index].contains(vertex_index)) {
              belongs_lower = false;
              break;
            }
          }
          if (belongs_lower) {
            matched_feature = feature_index;
            matched_upper = false;
            break;
          }
        }
        if (matched_feature == pending.size()) {
          fit_valid = false;
          break;
        }
        if (matched_upper) {
          ++upper_face_counts[matched_feature];
        } else {
          ++lower_face_counts[matched_feature];
        }
      }

      if (!fit_valid || outer_face_indices.size() < 4) {
        continue;
      }
      if (std::any_of(upper_face_counts.begin(), upper_face_counts.end(), [](int count) { return count < 8; }) ||
          std::any_of(lower_face_counts.begin(), lower_face_counts.end(), [](int count) { return count < 8; })) {
        continue;
      }

      std::vector<SimplePrismaticMultiCounterbore> counterbores;
      counterbores.reserve(pending.size());
      for (const PendingCounterbore& pending_counterbore : pending) {
        const Vec2 center {
            (pending_counterbore.top_circle.center.x + pending_counterbore.step_outer_circle.center.x +
             pending_counterbore.step_inner_circle.center.x + pending_counterbore.bottom_circle.center.x) * 0.25,
            (pending_counterbore.top_circle.center.y + pending_counterbore.step_outer_circle.center.y +
             pending_counterbore.step_inner_circle.center.y + pending_counterbore.bottom_circle.center.y) * 0.25,
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const Vec3 upper_base_center = axis_origin + axis * pending_counterbore.step_plane;
        const Vec3 upper_seam_vector =
            reconstruction.vertices[pending_counterbore.step_outer_loop.front()] - upper_base_center;
        const Vec3 upper_radial = upper_seam_vector - axis * dot(upper_seam_vector, axis);
        const Vec3 lower_base_center = axis_origin + axis * bottom_plane;
        const Vec3 lower_seam_vector =
            reconstruction.vertices[bottom_inner_loops[pending_counterbore.bottom_loop_index].front()] -
            lower_base_center;
        const Vec3 lower_radial = lower_seam_vector - axis * dot(lower_seam_vector, axis);
        if (length_squared(upper_radial) <= 1e-12 || length_squared(lower_radial) <= 1e-12) {
          fit_valid = false;
          break;
        }

        SimplePrismaticMultiCounterbore counterbore;
        counterbore.top_inner_area = top_inner_areas[pending_counterbore.top_loop_index];
        counterbore.bottom_inner_area = bottom_inner_areas[pending_counterbore.bottom_loop_index];
        counterbore.step_outer_area = pending_counterbore.step_outer_area;
        counterbore.step_inner_area = pending_counterbore.step_inner_area;
        counterbore.upper_bore = SimpleCylinderPrimitive {
            upper_base_center,
            axis,
            normalized(upper_radial),
            pending_counterbore.top_circle.radius,
            top_plane - pending_counterbore.step_plane,
        };
        counterbore.lower_bore = SimpleCylinderPrimitive {
            lower_base_center,
            axis,
            normalized(lower_radial),
            pending_counterbore.bottom_circle.radius,
            pending_counterbore.step_plane - bottom_plane,
        };
        counterbores.push_back(std::move(counterbore));
      }
      if (!fit_valid) {
        continue;
      }

      SimplePrismaticMultiCounterboreExport export_result;
      export_result.top_face_index = top_face_index;
      export_result.bottom_face_index = bottom_face_index;
      export_result.top_outer_loop = top_outer_loop;
      export_result.bottom_outer_loop = bottom_outer_loop;
      export_result.counterbores = std::move(counterbores);
      export_result.outer_face_indices = std::move(outer_face_indices);
      return export_result;
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

std::optional<GenericCylindricalThroughHoleExport> detect_generic_cylindrical_through_hole_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 8 || reconstruction.vertices.size() < 24) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() >= 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2) {
    return std::nullopt;
  }

  struct IndexedCircleLoop {
    std::size_t loop_index {};
    std::vector<int> loop;
    double area {};
    CircleFit2D circle {};
  };

  GenericCylindricalThroughHoleExport best_export;
  std::size_t best_hole_count = 0;
  std::size_t best_side_face_count = 0;

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.normal);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(face_a.fit.centroid - face_b.fit.centroid, axis) < 0.0) {
        axis = axis * -1.0;
      }

      const PlaneBasis axis_basis = basis_from_normal(axis);
      const double plane_a = dot(face_a.fit.centroid, axis);
      const double plane_b = dot(face_b.fit.centroid, axis);
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
      if (top_plane <= bottom_plane + 1e-4) {
        continue;
      }

      const bool a_is_top = plane_a >= plane_b;
      const int top_face_index = a_is_top ? cap_candidates[i] : cap_candidates[j];
      const int bottom_face_index = a_is_top ? cap_candidates[j] : cap_candidates[i];
      const ReconstructedFace& top_face = reconstruction.faces[top_face_index];
      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_face_index];

      const std::vector<double> top_areas =
          projected_face_loop_areas(top_face, reconstruction.vertices, axis_basis);
      const std::vector<double> bottom_areas =
          projected_face_loop_areas(bottom_face, reconstruction.vertices, axis_basis);
      const std::optional<std::size_t> top_outer_index = outer_loop_index_from_areas(top_areas);
      const std::optional<std::size_t> bottom_outer_index = outer_loop_index_from_areas(bottom_areas);
      if (!top_outer_index.has_value() || !bottom_outer_index.has_value()) {
        continue;
      }

      std::vector<IndexedCircleLoop> top_loops;
      std::vector<IndexedCircleLoop> bottom_loops;
      for (std::size_t loop_index = 0; loop_index < top_face.loops.size(); ++loop_index) {
        if (loop_index == *top_outer_index) {
          continue;
        }
        const std::optional<CircleFit2D> circle =
            fit_circle_2d(project_loop_to_basis(top_face.loops[loop_index], reconstruction.vertices, axis_basis));
        if (!circle.has_value() || circle->radius <= 1e-6) {
          continue;
        }
        const double tolerance = std::max(circle->radius * 0.03, 1e-4);
        if (circle->max_error > tolerance) {
          continue;
        }
        top_loops.push_back({loop_index, top_face.loops[loop_index], top_areas[loop_index], *circle});
      }
      for (std::size_t loop_index = 0; loop_index < bottom_face.loops.size(); ++loop_index) {
        if (loop_index == *bottom_outer_index) {
          continue;
        }
        const std::optional<CircleFit2D> circle = fit_circle_2d(
            project_loop_to_basis(bottom_face.loops[loop_index], reconstruction.vertices, axis_basis));
        if (!circle.has_value() || circle->radius <= 1e-6) {
          continue;
        }
        const double tolerance = std::max(circle->radius * 0.03, 1e-4);
        if (circle->max_error > tolerance) {
          continue;
        }
        bottom_loops.push_back(
            {loop_index, bottom_face.loops[loop_index], bottom_areas[loop_index], *circle});
      }
      if (top_loops.empty() || bottom_loops.empty()) {
        continue;
      }

      std::vector<bool> matched_bottom(bottom_loops.size(), false);
      std::unordered_set<int> used_side_faces;
      std::vector<GenericCylindricalThroughHole> holes;
      std::size_t total_side_faces = 0;

      for (const IndexedCircleLoop& top_loop : top_loops) {
        std::size_t best_bottom_index = bottom_loops.size();
        double best_score = std::numeric_limits<double>::infinity();
        double matched_tolerance = 0.0;

        for (std::size_t bottom_index = 0; bottom_index < bottom_loops.size(); ++bottom_index) {
          if (matched_bottom[bottom_index]) {
            continue;
          }
          const IndexedCircleLoop& bottom_loop = bottom_loops[bottom_index];
          const double tolerance =
              std::max((top_loop.circle.radius + bottom_loop.circle.radius) * 0.015, 1e-4);
          const Vec2 center_delta {top_loop.circle.center.x - bottom_loop.circle.center.x,
                                   top_loop.circle.center.y - bottom_loop.circle.center.y};
          const double center_distance =
              std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
          if (std::abs(top_loop.circle.radius - bottom_loop.circle.radius) > tolerance ||
              center_distance > tolerance) {
            continue;
          }

          const double score =
              center_distance + std::abs(top_loop.circle.radius - bottom_loop.circle.radius);
          if (score < best_score) {
            best_score = score;
            best_bottom_index = bottom_index;
            matched_tolerance = tolerance;
          }
        }

        if (best_bottom_index == bottom_loops.size()) {
          continue;
        }

        const IndexedCircleLoop& bottom_loop = bottom_loops[best_bottom_index];
        std::unordered_set<int> hole_vertices;
        hole_vertices.insert(top_loop.loop.begin(), top_loop.loop.end());
        hole_vertices.insert(bottom_loop.loop.begin(), bottom_loop.loop.end());

        std::vector<int> side_face_indices;
        side_face_indices.reserve(top_loop.loop.size());
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == top_face_index ||
              static_cast<int>(face_index) == bottom_face_index ||
              used_side_faces.contains(static_cast<int>(face_index))) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 4) {
            continue;
          }
          if (angle_degrees_unsigned(face.fit.normal, axis) < 20.0) {
            continue;
          }

          bool all_hole_vertices = true;
          for (int vertex_index : face.loops.front()) {
            if (!hole_vertices.contains(vertex_index)) {
              all_hole_vertices = false;
              break;
            }
          }
          if (!all_hole_vertices) {
            continue;
          }

          int top_vertices = 0;
          int bottom_vertices = 0;
          int intermediate_vertices = 0;
          for (int vertex_index : face.loops.front()) {
            const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
            if (std::abs(coordinate - top_plane) <= matched_tolerance) {
              ++top_vertices;
            } else if (std::abs(coordinate - bottom_plane) <= matched_tolerance) {
              ++bottom_vertices;
            } else {
              ++intermediate_vertices;
            }
          }
          if (top_vertices < 2 || bottom_vertices < 2 || intermediate_vertices != 0) {
            continue;
          }

          side_face_indices.push_back(static_cast<int>(face_index));
        }

        if (side_face_indices.size() < 6) {
          continue;
        }

        matched_bottom[best_bottom_index] = true;
        used_side_faces.insert(side_face_indices.begin(), side_face_indices.end());
        total_side_faces += side_face_indices.size();

        const double radius = 0.5 * (top_loop.circle.radius + bottom_loop.circle.radius);
        const Vec2 center {
            0.5 * (top_loop.circle.center.x + bottom_loop.circle.center.x),
            0.5 * (top_loop.circle.center.y + bottom_loop.circle.center.y),
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const Vec3 base_center = axis_origin + axis * bottom_plane;
        const Vec3 seam_vector =
            reconstruction.vertices[bottom_loop.loop.front()] - base_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          continue;
        }

        holes.push_back(GenericCylindricalThroughHole {
            top_face_index,
            bottom_face_index,
            top_loop.loop_index,
            bottom_loop.loop_index,
            top_loop.area,
            bottom_loop.area,
            SimpleCylinderPrimitive {base_center, axis, normalized(radial), radius, top_plane - bottom_plane},
            side_face_indices,
        });
      }

      if (holes.empty()) {
        continue;
      }

      if (holes.size() > best_hole_count ||
          (holes.size() == best_hole_count && total_side_faces > best_side_face_count)) {
        best_hole_count = holes.size();
        best_side_face_count = total_side_faces;
        best_export.holes = std::move(holes);
      }
    }
  }

  if (best_export.holes.empty()) {
    return std::nullopt;
  }
  return best_export;
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

struct StepExportContext {
  int product_shape {};
  int context {};
  int context_2d {};
  int global_axis {};
};

struct StepPlaneSurface {
  int plane_id {};
  Vec3 origin {};
  Vec3 normal {};
  PlaneBasis basis {};
};

struct StepCylinderBundle {
  int top_edge_curve {};
  int bottom_edge_curve {};
  int seam_edge_curve {};
  int lateral_face {};
};

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

int add_step_cylinder_solid(StepBuilder& step,
                            const StepExportContext& context_ids,
                            const SimpleCylinderPrimitive& cylinder) {
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

  const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                     step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface = step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                                        format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec =
      step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line = step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                                           step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(top_cyl_pcurve_repr) + ")");

  const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int top_circle_axis_2d = step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) +
                                          "," + step.ref(top_plane_ref_2d) + ")");
  const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                     format_double(cylinder.radius) + ")");
  const int top_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_plane_pcurve = step.add("PCURVE(''," + step.ref(top_plane) + "," +
                                        step.ref(top_circle_repr) + ")");
  const int top_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" +
                                         step.ref(top_cyl_pcurve) + "," + step.ref(top_plane_pcurve) +
                                         "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec =
      step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                 step.ref(seam_line_vec) + ")");

  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec =
      step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u0_repr) + ")");

  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u1_repr) + ")");
  const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                  step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                  "),.PCURVE_S1.)");
  const int seam_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(seam_curve) + ",.T.)");

  const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis) + "," +
                                     format_double(cylinder.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                         step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int bottom_circle_axis_2d =
      step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
               step.ref(bottom_plane_ref_2d) + ")");
  const int bottom_circle_2d = step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                                        format_double(cylinder.radius) + ")");
  const int bottom_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_plane_pcurve = step.add("PCURVE(''," + step.ref(bottom_plane) + "," +
                                           step.ref(bottom_circle_repr) + ")");
  const int bottom_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                                            step.ref(bottom_cyl_pcurve) + "," +
                                            step.ref(bottom_plane_pcurve) + "),.PCURVE_S1.)");
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
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                    step.ref(lateral_seam_forward) + "," +
                                    step.ref(lateral_bottom_oriented) + "," +
                                    step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  const int top_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.T.)");
  const int top_loop = step.add("EDGE_LOOP('',(" + step.ref(top_oriented_edge) + "))");
  const int top_bound = step.add("FACE_BOUND(''," + step.ref(top_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_bound) + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.F.)");
  const int bottom_loop = step.add("EDGE_LOOP('',(" + step.ref(bottom_oriented_edge) + "))");
  const int bottom_bound = step.add("FACE_BOUND(''," + step.ref(bottom_loop) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  const int closed_shell = step.add("CLOSED_SHELL('',(" + step.ref(lateral_face) + "," +
                                   step.ref(top_face) + "," + step.ref(bottom_face) + "))");
  return step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
}

bool write_step_file_simple_cylinders(const std::filesystem::path& path,
                                      const std::vector<SimpleCylinderPrimitive>& cylinders) {
  if (cylinders.empty()) {
    return false;
  }

  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);
  std::vector<int> solid_ids;
  solid_ids.reserve(cylinders.size());
  for (const SimpleCylinderPrimitive& cylinder : cylinders) {
    solid_ids.push_back(add_step_cylinder_solid(step, context_ids, cylinder));
  }

  std::ostringstream items;
  for (std::size_t index = 0; index < solid_ids.size(); ++index) {
    if (index != 0) {
      items << ",";
    }
    items << step.ref(solid_ids[index]);
  }
  items << "," << step.ref(context_ids.global_axis);

  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + items.str() + ")," +
               step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_multi_bore(const std::filesystem::path& path,
                                          const ReconstructionResult& reconstruction,
                                          const SimplePrismaticMultiBoreExport& bore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = bore_export.top_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(bore_export.bores.front().bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  std::vector<int> bottom_outer_loop = bore_export.bottom_outer_loop;
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 axis = bore_export.bores.front().bore.axis;
  const Vec3 top_center = bore_export.bores.front().bore.base_center + axis * bore_export.bores.front().bore.height;
  const Vec3 bottom_center = bore_export.bores.front().bore.base_center;
  const Vec3 ref_direction = bore_export.bores.front().bore.ref_direction;

  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");
    const int local_top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int local_bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(local_top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(local_bottom_axis) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                           step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  std::vector<CylinderBundle> bundles;
  bundles.reserve(bore_export.bores.size());
  for (const SimplePrismaticMultiBore& bore : bore_export.bores) {
    bundles.push_back(add_cylinder_bundle(bore.bore));
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  std::vector<int> top_bound_ids;
  top_bound_ids.push_back(top_outer_bound);
  for (std::size_t index = 0; index < bundles.size(); ++index) {
    const bool top_circle_forward = bore_export.bores[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  std::vector<int> bottom_bound_ids;
  bottom_bound_ids.push_back(bottom_outer_bound);
  for (std::size_t index = 0; index < bundles.size(); ++index) {
    const bool bottom_circle_forward = bore_export.bores[index].bottom_inner_area > 0.0;
    const int bottom_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].bottom_edge_curve) + "," +
                 std::string(bottom_circle_forward ? ".T." : ".F.") + ")");
    const int bottom_inner_loop =
        step.add("EDGE_LOOP('',(" + step.ref(bottom_inner_oriented) + "))");
    bottom_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop) + ",.T.)"));
  }
  std::ostringstream bottom_bound_refs;
  for (std::size_t index = 0; index < bottom_bound_ids.size(); ++index) {
    if (index > 0) {
      bottom_bound_refs << ",";
    }
    bottom_bound_refs << step.ref(bottom_bound_ids[index]);
  }
  const int bottom_face = step.add("ADVANCED_FACE('',(" + bottom_bound_refs.str() + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(bore_export.outer_face_indices.size());
  for (int face_index : bore_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (const CylinderBundle& bundle : bundles) {
    shell_faces << "," << step.ref(bundle.lateral_face);
  }
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_bore(const std::filesystem::path& path,
                                    const ReconstructionResult& reconstruction,
                                    const SimplePrismaticBoreExport& bore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = bore_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = bore_export.bottom_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(bore_export.bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const SimpleCylinderPrimitive& cylinder = bore_export.bore;
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

  const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                     step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface = step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                                        format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec =
      step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line = step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                                           step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(top_cyl_pcurve_repr) + ")");

  const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int top_circle_axis_2d = step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) +
                                          "," + step.ref(top_plane_ref_2d) + ")");
  const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                     format_double(cylinder.radius) + ")");
  const int top_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_plane_pcurve = step.add("PCURVE(''," + step.ref(top_plane) + "," +
                                        step.ref(top_circle_repr) + ")");
  const int top_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" +
                                         step.ref(top_cyl_pcurve) + "," + step.ref(top_plane_pcurve) +
                                         "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec =
      step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                 step.ref(seam_line_vec) + ")");

  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec =
      step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u0_repr) + ")");

  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u1_repr) + ")");
  const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                  step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                  "),.PCURVE_S1.)");
  const int seam_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(seam_curve) + ",.T.)");

  const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis) + "," +
                                     format_double(cylinder.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                         step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int bottom_circle_axis_2d =
      step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
               step.ref(bottom_plane_ref_2d) + ")");
  const int bottom_circle_2d = step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                                        format_double(cylinder.radius) + ")");
  const int bottom_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_plane_pcurve = step.add("PCURVE(''," + step.ref(bottom_plane) + "," +
                                           step.ref(bottom_circle_repr) + ")");
  const int bottom_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                                            step.ref(bottom_cyl_pcurve) + "," +
                                            step.ref(bottom_plane_pcurve) + "),.PCURVE_S1.)");
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
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                    step.ref(lateral_seam_forward) + "," +
                                    step.ref(lateral_bottom_oriented) + "," +
                                    step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = bore_export.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const bool bottom_circle_forward = bore_export.bottom_inner_area > 0.0;
  const int bottom_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + "," +
               std::string(bottom_circle_forward ? ".T." : ".F.") + ")");
  const int bottom_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(bottom_inner_oriented) + "))");
  const int bottom_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + "," +
                                   step.ref(bottom_inner_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(bore_export.outer_face_indices.size());
  for (int face_index : bore_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face) << "," <<
      step.ref(lateral_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_blind_counterbore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticBlindCounterboreExport& counterbore) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = counterbore.top_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(counterbore.upper_bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  const SimpleCylinderPrimitive& upper_cylinder = counterbore.upper_bore;
  const SimpleCylinderPrimitive& lower_cylinder = counterbore.lower_bore;
  const Vec3 top_center = upper_cylinder.base_center + upper_cylinder.axis * upper_cylinder.height;
  const Vec3 step_center = upper_cylinder.base_center;
  const Vec3 floor_center = lower_cylinder.base_center;

  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                      format_double(step_center.y) + "," +
                                      format_double(step_center.z) + "))");
  const int floor_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
               format_double(floor_center.y) + "," + format_double(floor_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(upper_cylinder.axis.x) + "," +
               format_double(upper_cylinder.axis.y) + "," + format_double(upper_cylinder.axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(upper_cylinder.ref_direction.x) + "," +
               format_double(upper_cylinder.ref_direction.y) + "," +
               format_double(upper_cylinder.ref_direction.z) + "))");

  const int top_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                      step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                      ")");
  const int step_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                                       step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                       ")");
  const int floor_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                                        step.ref(axis_direction) + "," +
                                        step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_plane_axis) + ")");
  const int step_plane = step.add("PLANE(''," + step.ref(step_plane_axis) + ")");
  const int floor_plane = step.add("PLANE(''," + step.ref(floor_plane_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
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
    const int top_center_point_local =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int axis_direction_local =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int ref_direction_local =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                       step.ref(axis_direction_local) + "," +
                                       step.ref(ref_direction_local) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");

    const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point_local) + "," +
                                  step.ref(axis_direction_local) + "," +
                                  step.ref(ref_direction_local) + ")");
    const int bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                 step.ref(axis_direction_local) + "," + step.ref(ref_direction_local) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
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
    const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                           step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  const CylinderBundle upper_bundle = add_cylinder_bundle(upper_cylinder, top_plane, step_plane);
  const CylinderBundle lower_bundle = add_cylinder_bundle(lower_cylinder, step_plane, floor_plane);

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = counterbore.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundle.top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const bool step_outer_circle_forward = counterbore.step_outer_area > 0.0;
  const int step_outer_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundle.bottom_edge_curve) + "," +
               std::string(step_outer_circle_forward ? ".T." : ".F.") + ")");
  const int step_outer_loop_id = step.add("EDGE_LOOP('',(" + step.ref(step_outer_oriented) + "))");
  const int step_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(step_outer_loop_id) + ",.T.)");
  const bool step_inner_circle_forward = counterbore.step_inner_area > 0.0;
  const int step_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundle.top_edge_curve) + "," +
               std::string(step_inner_circle_forward ? ".T." : ".F.") + ")");
  const int step_inner_loop_id = step.add("EDGE_LOOP('',(" + step.ref(step_inner_oriented) + "))");
  const int step_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(step_inner_loop_id) + ",.T.)");
  const int step_face = step.add("ADVANCED_FACE('',(" + step.ref(step_outer_bound) + "," +
                                 step.ref(step_inner_bound) + ")," + step.ref(step_plane) + ",.T.)");

  const bool floor_circle_forward = counterbore.floor_area > 0.0;
  const int floor_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundle.bottom_edge_curve) + "," +
               std::string(floor_circle_forward ? ".T." : ".F.") + ")");
  const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
  const int floor_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
  const int floor_face = step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                  step.ref(floor_plane) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(counterbore.outer_face_indices.size());
  for (int face_index : counterbore.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(step_face) << "," << step.ref(floor_face)
              << "," << step.ref(upper_bundle.lateral_face) << ","
              << step.ref(lower_bundle.lateral_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_blind_bore(const std::filesystem::path& path,
                                          const ReconstructionResult& reconstruction,
                                          const SimplePrismaticBlindBoreExport& bore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = bore_export.top_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(bore_export.bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  const SimpleCylinderPrimitive& cylinder = bore_export.bore;
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

  const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                     step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface = step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                                        format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec =
      step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line = step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                                           step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(top_cyl_pcurve_repr) + ")");

  const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int top_circle_axis_2d = step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) +
                                          "," + step.ref(top_plane_ref_2d) + ")");
  const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                     format_double(cylinder.radius) + ")");
  const int top_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_plane_pcurve = step.add("PCURVE(''," + step.ref(top_plane) + "," +
                                        step.ref(top_circle_repr) + ")");
  const int top_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" +
                                         step.ref(top_cyl_pcurve) + "," + step.ref(top_plane_pcurve) +
                                         "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                 step.ref(seam_line_vec) + ")");

  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u0_repr) + ")");

  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u1_repr) + ")");
  const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                  step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                  "),.PCURVE_S1.)");
  const int seam_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(seam_curve) + ",.T.)");

  const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis) + "," +
                                     format_double(cylinder.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                         step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int bottom_circle_axis_2d =
      step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
               step.ref(bottom_plane_ref_2d) + ")");
  const int bottom_circle_2d = step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                                        format_double(cylinder.radius) + ")");
  const int bottom_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_plane_pcurve = step.add("PCURVE(''," + step.ref(bottom_plane) + "," +
                                           step.ref(bottom_circle_repr) + ")");
  const int bottom_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                                            step.ref(bottom_cyl_pcurve) + "," +
                                            step.ref(bottom_plane_pcurve) + "),.PCURVE_S1.)");
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
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                    step.ref(lateral_seam_forward) + "," +
                                    step.ref(lateral_bottom_oriented) + "," +
                                    step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = bore_export.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const bool floor_circle_forward = bore_export.floor_area > 0.0;
  const int floor_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + "," +
               std::string(floor_circle_forward ? ".T." : ".F.") + ")");
  const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
  const int floor_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
  const int floor_face = step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                  step.ref(bottom_plane) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(bore_export.outer_face_indices.size());
  for (int face_index : bore_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(floor_face) << "," << step.ref(lateral_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_boss(const std::filesystem::path& path,
                                    const ReconstructionResult& reconstruction,
                                    const SimplePrismaticBossExport& boss_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> base_outer_loop = boss_export.base_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(boss_export.boss.axis);
  if (projected_loop_area(base_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(base_outer_loop.begin(), base_outer_loop.end());
  }

  const SimpleCylinderPrimitive& cylinder = boss_export.boss;
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

  const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                     step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface = step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                                        format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int base_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                 step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int base_plane = step.add("PLANE(''," + step.ref(base_axis) + ")");

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec =
      step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line = step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                                           step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(top_cyl_pcurve_repr) + ")");

  const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int top_circle_axis_2d = step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) +
                                          "," + step.ref(top_plane_ref_2d) + ")");
  const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                     format_double(cylinder.radius) + ")");
  const int top_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_plane_pcurve = step.add("PCURVE(''," + step.ref(top_plane) + "," +
                                        step.ref(top_circle_repr) + ")");
  const int top_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" +
                                         step.ref(top_cyl_pcurve) + "," + step.ref(top_plane_pcurve) +
                                         "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec =
      step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                 step.ref(seam_line_vec) + ")");
  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u0_repr) + ")");
  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u1_repr) + ")");
  const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                  step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                  "),.PCURVE_S1.)");
  const int seam_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(seam_curve) + ",.T.)");

  const int bottom_circle = step.add("CIRCLE(''," + step.ref(base_axis) + "," +
                                     format_double(cylinder.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                         step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int bottom_circle_axis_2d =
      step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
               step.ref(bottom_plane_ref_2d) + ")");
  const int bottom_circle_2d = step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                                        format_double(cylinder.radius) + ")");
  const int bottom_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_plane_pcurve = step.add("PCURVE(''," + step.ref(base_plane) + "," +
                                           step.ref(bottom_circle_repr) + ")");
  const int bottom_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                                            step.ref(bottom_cyl_pcurve) + "," +
                                            step.ref(bottom_plane_pcurve) + "),.PCURVE_S1.)");
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
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                    step.ref(lateral_seam_forward) + "," +
                                    step.ref(lateral_bottom_oriented) + "," +
                                    step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  const int base_outer_loop_id = add_line_loop(base_outer_loop);
  const int base_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(base_outer_loop_id) + ",.T.)");
  const bool base_circle_forward = boss_export.base_inner_area > 0.0;
  const int base_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + "," +
               std::string(base_circle_forward ? ".T." : ".F.") + ")");
  const int base_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(base_inner_oriented) + "))");
  const int base_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(base_inner_loop) + ",.T.)");
  const int base_face = step.add("ADVANCED_FACE('',(" + step.ref(base_outer_bound) + "," +
                                 step.ref(base_inner_bound) + ")," + step.ref(base_plane) + ",.T.)");

  const bool top_circle_forward = boss_export.top_area > 0.0;
  const int top_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_loop_id = step.add("EDGE_LOOP('',(" + step.ref(top_oriented) + "))");
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_loop_id) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + ")," +
                                step.ref(top_plane) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(boss_export.outer_face_indices.size());
  for (int face_index : boss_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(base_face) << "," << step.ref(top_face) << "," << step.ref(lateral_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_multi_boss(const std::filesystem::path& path,
                                          const ReconstructionResult& reconstruction,
                                          const SimplePrismaticMultiBossExport& boss_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> bottom_outer_loop = boss_export.bottom_outer_loop;
  std::vector<int> base_outer_loop = boss_export.base_outer_loop;
  const Vec3 axis = boss_export.bosses.front().boss.axis;
  const Vec3 ref_direction = boss_export.bosses.front().boss.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }
  if (projected_loop_area(base_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(base_outer_loop.begin(), base_outer_loop.end());
  }

  const Vec3 base_center = reconstruction.faces[boss_export.base_face_index].fit.centroid;
  const Vec3 bottom_center = reconstruction.faces[boss_export.bottom_face_index].fit.centroid;
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int base_center_point = step.add("CARTESIAN_POINT('',(" + format_double(base_center.x) + "," +
                                      format_double(base_center.y) + "," +
                                      format_double(base_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int base_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                 step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int base_plane = step.add("PLANE(''," + step.ref(base_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
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

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                 format_double(top_center.y) + "," + format_double(top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");

    const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                                  step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int bottom_axis_local =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
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
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis_local) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");
    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  struct BossFaceBundle {
    int top_plane {};
    CylinderBundle cylinder;
  };

  std::vector<BossFaceBundle> boss_bundles;
  boss_bundles.reserve(boss_export.bosses.size());
  for (const SimplePrismaticMultiBoss& boss : boss_export.bosses) {
    const Vec3 top_center = boss.boss.base_center + boss.boss.axis * boss.boss.height;
    const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                      format_double(top_center.y) + "," +
                                      format_double(top_center.z) + "))");
    const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                  step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
    boss_bundles.push_back(BossFaceBundle {top_plane, add_cylinder_bundle(boss.boss, top_plane, base_plane)});
  }

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  const int base_outer_loop_id = add_line_loop(base_outer_loop);
  std::vector<int> base_bound_ids;
  base_bound_ids.reserve(1 + boss_export.bosses.size());
  base_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(base_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < boss_export.bosses.size(); ++index) {
    const bool base_circle_forward = boss_export.bosses[index].base_inner_area > 0.0;
    const int base_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(boss_bundles[index].cylinder.bottom_edge_curve) + "," +
                 std::string(base_circle_forward ? ".T." : ".F.") + ")");
    const int base_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(base_inner_oriented) + "))");
    base_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(base_inner_loop) + ",.T.)"));
  }
  std::ostringstream base_bound_refs;
  for (std::size_t index = 0; index < base_bound_ids.size(); ++index) {
    if (index > 0) {
      base_bound_refs << ",";
    }
    base_bound_refs << step.ref(base_bound_ids[index]);
  }
  const int base_face = step.add("ADVANCED_FACE('',(" + base_bound_refs.str() + ")," +
                                 step.ref(base_plane) + ",.T.)");

  std::vector<int> top_face_ids;
  top_face_ids.reserve(boss_export.bosses.size());
  for (std::size_t index = 0; index < boss_export.bosses.size(); ++index) {
    const bool top_circle_forward = boss_export.bosses[index].top_area > 0.0;
    const int top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(boss_bundles[index].cylinder.top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_loop_id = step.add("EDGE_LOOP('',(" + step.ref(top_oriented) + "))");
    const int top_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(top_loop_id) + ",.T.)");
    top_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + ")," +
                                    step.ref(boss_bundles[index].top_plane) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(boss_export.outer_face_indices.size());
  for (int face_index : boss_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    const int plane_origin = step.add("CARTESIAN_POINT('',(" + format_double(face.fit.centroid.x) + "," +
                                   format_double(face.fit.centroid.y) + "," +
                                   format_double(face.fit.centroid.z) + "))");
    const int face_normal = step.add("DIRECTION('',(" + format_double(face.fit.normal.x) + "," +
                                   format_double(face.fit.normal.y) + "," +
                                   format_double(face.fit.normal.z) + "))");
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    const int face_ref = step.add("DIRECTION('',(" + format_double(basis.u.x) + "," +
                                format_double(basis.u.y) + "," + format_double(basis.u.z) + "))");
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(bottom_face) << "," << step.ref(base_face);
  for (std::size_t index = 0; index < boss_bundles.size(); ++index) {
    shell_faces << "," << step.ref(top_face_ids[index]) << ","
                << step.ref(boss_bundles[index].cylinder.lateral_face);
  }
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_multi_blind_bore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticMultiBlindBoreExport& bore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = bore_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = bore_export.bottom_outer_loop;
  const Vec3 axis = bore_export.bores.front().bore.axis;
  const Vec3 ref_direction = bore_export.bores.front().bore.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_center = reconstruction.faces[bore_export.top_face_index].fit.centroid;
  const Vec3 bottom_center = reconstruction.faces[bore_export.bottom_face_index].fit.centroid;
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");
    const int local_top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int local_bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(local_top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(local_bottom_axis) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");
    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  struct BoreFaceBundle {
    int floor_plane {};
    CylinderBundle cylinder;
  };

  std::vector<BoreFaceBundle> bore_bundles;
  bore_bundles.reserve(bore_export.bores.size());
  for (const SimplePrismaticMultiBlindBore& bore : bore_export.bores) {
    const Vec3 floor_center = bore.bore.base_center;
    const int floor_center_point = step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
                                        format_double(floor_center.y) + "," +
                                        format_double(floor_center.z) + "))");
    const int floor_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                                    step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int floor_plane = step.add("PLANE(''," + step.ref(floor_axis) + ")");
    bore_bundles.push_back(BoreFaceBundle {floor_plane, add_cylinder_bundle(bore.bore, top_plane, floor_plane)});
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  std::vector<int> top_bound_ids;
  top_bound_ids.reserve(1 + bore_export.bores.size());
  top_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < bore_export.bores.size(); ++index) {
    const bool top_circle_forward = bore_export.bores[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bore_bundles[index].cylinder.top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> floor_face_ids;
  floor_face_ids.reserve(bore_export.bores.size());
  for (std::size_t index = 0; index < bore_export.bores.size(); ++index) {
    const bool floor_circle_forward = bore_export.bores[index].floor_area > 0.0;
    const int floor_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bore_bundles[index].cylinder.bottom_edge_curve) + "," +
                 std::string(floor_circle_forward ? ".T." : ".F.") + ")");
    const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
    const int floor_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
    floor_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                      step.ref(bore_bundles[index].floor_plane) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(bore_export.outer_face_indices.size());
  for (int face_index : bore_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    const int plane_origin = step.add("CARTESIAN_POINT('',(" + format_double(face.fit.centroid.x) + "," +
                                   format_double(face.fit.centroid.y) + "," +
                                   format_double(face.fit.centroid.z) + "))");
    const int face_normal = step.add("DIRECTION('',(" + format_double(face.fit.normal.x) + "," +
                                   format_double(face.fit.normal.y) + "," +
                                   format_double(face.fit.normal.z) + "))");
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    const int face_ref = step.add("DIRECTION('',(" + format_double(basis.u.x) + "," +
                                format_double(basis.u.y) + "," + format_double(basis.u.z) + "))");
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (std::size_t index = 0; index < bore_export.bores.size(); ++index) {
    shell_faces << "," << step.ref(floor_face_ids[index]) << ","
                << step.ref(bore_bundles[index].cylinder.lateral_face);
  }
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_multi_blind_counterbore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticMultiBlindCounterboreExport& counterbore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = counterbore_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = counterbore_export.bottom_outer_loop;
  const Vec3 axis = counterbore_export.counterbores.front().upper_bore.axis;
  const Vec3 ref_direction = counterbore_export.counterbores.front().upper_bore.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_center = reconstruction.faces[counterbore_export.top_face_index].fit.centroid;
  const Vec3 bottom_center = reconstruction.faces[counterbore_export.bottom_face_index].fit.centroid;
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");
    const int local_top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int local_bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(local_top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(local_bottom_axis) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");
    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  struct CounterboreFaceBundle {
    int step_plane {};
    int floor_plane {};
    CylinderBundle upper_cylinder;
    CylinderBundle lower_cylinder;
  };

  std::vector<CounterboreFaceBundle> bundles;
  bundles.reserve(counterbore_export.counterbores.size());
  for (const SimplePrismaticMultiBlindCounterbore& counterbore : counterbore_export.counterbores) {
    const Vec3 step_center = counterbore.upper_bore.base_center;
    const Vec3 floor_center = counterbore.lower_bore.base_center;
    const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                       format_double(step_center.y) + "," +
                                       format_double(step_center.z) + "))");
    const int floor_center_point = step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
                                        format_double(floor_center.y) + "," +
                                        format_double(floor_center.z) + "))");
    const int step_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int floor_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                                    step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int step_plane = step.add("PLANE(''," + step.ref(step_axis) + ")");
    const int floor_plane = step.add("PLANE(''," + step.ref(floor_axis) + ")");
    bundles.push_back(CounterboreFaceBundle {
        step_plane,
        floor_plane,
        add_cylinder_bundle(counterbore.upper_bore, top_plane, step_plane),
        add_cylinder_bundle(counterbore.lower_bore, step_plane, floor_plane),
    });
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  std::vector<int> top_bound_ids;
  top_bound_ids.reserve(1 + counterbore_export.counterbores.size());
  top_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool top_circle_forward = counterbore_export.counterbores[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].upper_cylinder.top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> step_face_ids;
  std::vector<int> floor_face_ids;
  step_face_ids.reserve(counterbore_export.counterbores.size());
  floor_face_ids.reserve(counterbore_export.counterbores.size());
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool step_outer_circle_forward = counterbore_export.counterbores[index].step_outer_area > 0.0;
    const int step_outer_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].upper_cylinder.bottom_edge_curve) + "," +
                 std::string(step_outer_circle_forward ? ".T." : ".F.") + ")");
    const int step_outer_loop = step.add("EDGE_LOOP('',(" + step.ref(step_outer_oriented) + "))");
    const int step_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(step_outer_loop) + ",.T.)");

    const bool step_inner_circle_forward = counterbore_export.counterbores[index].step_inner_area > 0.0;
    const int step_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].lower_cylinder.top_edge_curve) + "," +
                 std::string(step_inner_circle_forward ? ".T." : ".F.") + ")");
    const int step_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(step_inner_oriented) + "))");
    const int step_inner_bound =
        step.add("FACE_BOUND(''," + step.ref(step_inner_loop) + ",.T.)");
    step_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(step_outer_bound) + "," +
                                     step.ref(step_inner_bound) + ")," +
                                     step.ref(bundles[index].step_plane) + ",.T.)"));

    const bool floor_circle_forward = counterbore_export.counterbores[index].floor_area > 0.0;
    const int floor_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].lower_cylinder.bottom_edge_curve) + "," +
                 std::string(floor_circle_forward ? ".T." : ".F.") + ")");
    const int floor_loop = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
    const int floor_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop) + ",.T.)");
    floor_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                      step.ref(bundles[index].floor_plane) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(counterbore_export.outer_face_indices.size());
  for (int face_index : counterbore_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    const int plane_origin = step.add("CARTESIAN_POINT('',(" + format_double(face.fit.centroid.x) + "," +
                                   format_double(face.fit.centroid.y) + "," +
                                   format_double(face.fit.centroid.z) + "))");
    const int face_normal = step.add("DIRECTION('',(" + format_double(face.fit.normal.x) + "," +
                                   format_double(face.fit.normal.y) + "," +
                                   format_double(face.fit.normal.z) + "))");
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    const int face_ref = step.add("DIRECTION('',(" + format_double(basis.u.x) + "," +
                                format_double(basis.u.y) + "," + format_double(basis.u.z) + "))");
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    shell_faces << "," << step.ref(step_face_ids[index]) << "," << step.ref(floor_face_ids[index]) << ","
                << step.ref(bundles[index].upper_cylinder.lateral_face) << ","
                << step.ref(bundles[index].lower_cylinder.lateral_face);
  }
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_standoff(const std::filesystem::path& path,
                                        const ReconstructionResult& reconstruction,
                                        const SimplePrismaticStandoffExport& standoff) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> base_outer_loop = standoff.base_outer_loop;
  std::vector<int> bottom_outer_loop = standoff.bottom_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(standoff.outer_cylinder.axis);
  if (projected_loop_area(base_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(base_outer_loop.begin(), base_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const SimpleCylinderPrimitive& outer_cylinder = standoff.outer_cylinder;
  const SimpleCylinderPrimitive& inner_cylinder = standoff.inner_cylinder;
  const Vec3 top_center = outer_cylinder.base_center + outer_cylinder.axis * outer_cylinder.height;
  const Vec3 base_center = outer_cylinder.base_center;
  const Vec3 bottom_center = inner_cylinder.base_center;

  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int base_center_point = step.add("CARTESIAN_POINT('',(" + format_double(base_center.x) + "," +
                                      format_double(base_center.y) + "," +
                                      format_double(base_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(outer_cylinder.axis.x) + "," +
               format_double(outer_cylinder.axis.y) + "," +
               format_double(outer_cylinder.axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(outer_cylinder.ref_direction.x) + "," +
               format_double(outer_cylinder.ref_direction.y) + "," +
               format_double(outer_cylinder.ref_direction.z) + "))");

  const int top_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                      step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                      ")");
  const int base_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                       step.ref(axis_direction) + "," +
                                       step.ref(plane_ref_direction) + ")");
  const int bottom_plane_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_plane_axis) + ")");
  const int base_plane = step.add("PLANE(''," + step.ref(base_plane_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_plane_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");

    const int top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line =
        step.add("LINE(''," + step.ref(bottom_seam_point) + "," + step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
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
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  const CylinderBundle outer_bundle = add_cylinder_bundle(outer_cylinder, top_plane, base_plane);
  const CylinderBundle inner_bundle = add_cylinder_bundle(inner_cylinder, top_plane, bottom_plane);

  const bool top_outer_circle_forward = standoff.top_outer_area > 0.0;
  const int top_outer_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(outer_bundle.top_edge_curve) + "," +
               std::string(top_outer_circle_forward ? ".T." : ".F.") + ")");
  const int top_outer_loop_id = step.add("EDGE_LOOP('',(" + step.ref(top_outer_oriented) + "))");
  const int top_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_inner_circle_forward = standoff.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(inner_bundle.top_edge_curve) + "," +
               std::string(top_inner_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop_id = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(top_inner_loop_id) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const int base_outer_loop_id = add_line_loop(base_outer_loop);
  const int base_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(base_outer_loop_id) + ",.T.)");
  const bool base_circle_forward = standoff.base_inner_area > 0.0;
  const int base_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(outer_bundle.bottom_edge_curve) + "," +
               std::string(base_circle_forward ? ".T." : ".F.") + ")");
  const int base_inner_loop_id = step.add("EDGE_LOOP('',(" + step.ref(base_inner_oriented) + "))");
  const int base_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(base_inner_loop_id) + ",.T.)");
  const int base_face = step.add("ADVANCED_FACE('',(" + step.ref(base_outer_bound) + "," +
                                 step.ref(base_inner_bound) + ")," + step.ref(base_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const bool bottom_circle_forward = standoff.bottom_inner_area > 0.0;
  const int bottom_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(inner_bundle.bottom_edge_curve) + "," +
               std::string(bottom_circle_forward ? ".T." : ".F.") + ")");
  const int bottom_inner_loop_id =
      step.add("EDGE_LOOP('',(" + step.ref(bottom_inner_oriented) + "))");
  const int bottom_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + "," +
                                   step.ref(bottom_inner_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(standoff.outer_face_indices.size());
  for (int face_index : standoff.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(base_face) << "," << step.ref(bottom_face)
              << "," << step.ref(outer_bundle.lateral_face) << ","
              << step.ref(inner_bundle.lateral_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_counterbore(const std::filesystem::path& path,
                                           const ReconstructionResult& reconstruction,
                                           const SimplePrismaticCounterboreExport& counterbore) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = counterbore.top_outer_loop;
  std::vector<int> bottom_outer_loop = counterbore.bottom_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(counterbore.upper_bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const SimpleCylinderPrimitive& upper_cylinder = counterbore.upper_bore;
  const SimpleCylinderPrimitive& lower_cylinder = counterbore.lower_bore;
  const Vec3 top_center = upper_cylinder.base_center + upper_cylinder.axis * upper_cylinder.height;
  const Vec3 step_center = upper_cylinder.base_center;
  const Vec3 bottom_center = lower_cylinder.base_center;

  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                      format_double(step_center.y) + "," +
                                      format_double(step_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(upper_cylinder.axis.x) + "," +
               format_double(upper_cylinder.axis.y) + "," + format_double(upper_cylinder.axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(upper_cylinder.ref_direction.x) + "," +
               format_double(upper_cylinder.ref_direction.y) + "," +
               format_double(upper_cylinder.ref_direction.z) + "))");

  const int top_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                      step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                      ")");
  const int step_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                                       step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                       ")");
  const int bottom_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
                                         step.ref(axis_direction) + "," +
                                         step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_plane_axis) + ")");
  const int step_plane = step.add("PLANE(''," + step.ref(step_plane_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_plane_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
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
    const int top_center_point_local =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int axis_direction_local =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int ref_direction_local =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                       step.ref(axis_direction_local) + "," +
                                       step.ref(ref_direction_local) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");

    const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point_local) + "," +
                                  step.ref(axis_direction_local) + "," +
                                  step.ref(ref_direction_local) + ")");
    const int bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                 step.ref(axis_direction_local) + "," + step.ref(ref_direction_local) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
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
    const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                           step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  const CylinderBundle upper_bundle = add_cylinder_bundle(upper_cylinder, top_plane, step_plane);
  const CylinderBundle lower_bundle = add_cylinder_bundle(lower_cylinder, step_plane, bottom_plane);

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = counterbore.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundle.top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const bool bottom_circle_forward = counterbore.bottom_inner_area > 0.0;
  const int bottom_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundle.bottom_edge_curve) + "," +
               std::string(bottom_circle_forward ? ".T." : ".F.") + ")");
  const int bottom_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(bottom_inner_oriented) + "))");
  const int bottom_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + "," +
                                   step.ref(bottom_inner_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  const bool step_outer_circle_forward = counterbore.step_outer_area > 0.0;
  const int step_outer_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundle.bottom_edge_curve) + "," +
               std::string(step_outer_circle_forward ? ".T." : ".F.") + ")");
  const int step_outer_loop_id = step.add("EDGE_LOOP('',(" + step.ref(step_outer_oriented) + "))");
  const int step_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(step_outer_loop_id) + ",.T.)");
  const bool step_inner_circle_forward = counterbore.step_inner_area > 0.0;
  const int step_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundle.top_edge_curve) + "," +
               std::string(step_inner_circle_forward ? ".T." : ".F.") + ")");
  const int step_inner_loop_id = step.add("EDGE_LOOP('',(" + step.ref(step_inner_oriented) + "))");
  const int step_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(step_inner_loop_id) + ",.T.)");
  const int step_face = step.add("ADVANCED_FACE('',(" + step.ref(step_outer_bound) + "," +
                                 step.ref(step_inner_bound) + ")," + step.ref(step_plane) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(counterbore.outer_face_indices.size());
  for (int face_index : counterbore.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face) << "," << step.ref(step_face)
              << "," << step.ref(upper_bundle.lateral_face) << ","
              << step.ref(lower_bundle.lateral_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_obround_slot(const std::filesystem::path& path,
                                            const ReconstructionResult& reconstruction,
                                            const SimplePrismaticObroundSlotExport& slot) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  struct StepVertexInfo {
    int point_id {};
    int vertex_id {};
    Vec3 position {};
  };

  std::vector<StepVertexInfo> vertices;
  vertices.reserve(reconstruction.vertices.size() + 8);
  for (const Vec3& vertex : reconstruction.vertices) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                  format_double(vertex.y) + "," + format_double(vertex.z) + "))");
    const int vertex_id = step.add("VERTEX_POINT(''," + step.ref(point_id) + ")");
    vertices.push_back({point_id, vertex_id, vertex});
  }

  auto add_vertex = [&](const Vec3& point) -> int {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    const int vertex_id = step.add("VERTEX_POINT(''," + step.ref(point_id) + ")");
    vertices.push_back({point_id, vertex_id, point});
    return static_cast<int>(vertices.size() - 1);
  };

  std::unordered_map<EdgeKey, int, EdgeKeyHash> line_edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = line_edge_curve_ids.find(key);
    if (found != line_edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = vertices[key.a].position;
    const Vec3 end = vertices[key.b].position;
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
    const int line_id =
        step.add("LINE(''," + step.ref(vertices[key.a].point_id) + "," + step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertices[key.a].vertex_id) + "," +
                 step.ref(vertices[key.b].vertex_id) + "," + step.ref(line_id) + ",.T.)");
    line_edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_oriented_edge = [&](int edge_curve_id, bool same_orientation) -> int {
    return step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                    std::string(same_orientation ? ".T." : ".F.") + ")");
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(add_oriented_edge(edge_curve_id, key.a == from && key.b == to));
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  auto add_edge_loop = [&](const std::vector<std::pair<int, bool>>& edges) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < edges.size(); ++index) {
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(add_oriented_edge(edges[index].first, edges[index].second));
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  auto wrap_angle = [](double angle) {
    while (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    while (angle >= 2.0 * kPi) {
      angle -= 2.0 * kPi;
    }
    return angle;
  };

  auto add_arc_edge_curve = [&](int from,
                                int to,
                                const Vec3& center,
                                const Vec3& axis,
                                const Vec3& ref_direction,
                                double radius,
                                const Vec3& midpoint) -> int {
    const Vec3 local_ref = normalized(ref_direction);
    const Vec3 local_v = normalized(cross(axis, local_ref));
    const auto angle_at = [&](const Vec3& point) {
      const Vec3 delta = point - center;
      return wrap_angle(std::atan2(dot(delta, local_v), dot(delta, local_ref)));
    };

    const double start_angle = angle_at(vertices[from].position);
    const double end_angle = angle_at(vertices[to].position);
    double midpoint_angle = angle_at(midpoint);
    double ccw_end = end_angle;
    if (ccw_end < start_angle) {
      ccw_end += 2.0 * kPi;
    }
    while (midpoint_angle < start_angle) {
      midpoint_angle += 2.0 * kPi;
    }
    const bool sense_agreement = midpoint_angle <= ccw_end + 1e-9;

    const int center_point = step.add("CARTESIAN_POINT('',(" + format_double(center.x) + "," +
                                    format_double(center.y) + "," + format_double(center.z) + "))");
    const int axis_direction = step.add("DIRECTION('',(" + format_double(axis.x) + "," +
                                        format_double(axis.y) + "," + format_double(axis.z) + "))");
    const int ref_dir_id = step.add("DIRECTION('',(" + format_double(local_ref.x) + "," +
                                      format_double(local_ref.y) + "," +
                                      format_double(local_ref.z) + "))");
    const int circle_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(center_point) + "," + step.ref(axis_direction) +
                 "," + step.ref(ref_dir_id) + ")");
    const int circle =
        step.add("CIRCLE(''," + step.ref(circle_axis) + "," + format_double(radius) + ")");
    const int trimmed =
        step.add("TRIMMED_CURVE(''," + step.ref(circle) + ",(" + step.ref(vertices[from].point_id) +
                 "),(" + step.ref(vertices[to].point_id) + ")," +
                 std::string(sense_agreement ? ".T." : ".F.") + ",.CARTESIAN.)");
    return step.add("EDGE_CURVE(''," + step.ref(vertices[from].vertex_id) + "," +
                    step.ref(vertices[to].vertex_id) + "," + step.ref(trimmed) + ",.T.)");
  };

  auto add_plane = [&](const Vec3& origin, const Vec3& normal, const Vec3& ref_direction) -> int {
    const int origin_id = step.add("CARTESIAN_POINT('',(" + format_double(origin.x) + "," +
                                 format_double(origin.y) + "," + format_double(origin.z) + "))");
    const int normal_id = step.add("DIRECTION('',(" + format_double(normal.x) + "," +
                                 format_double(normal.y) + "," + format_double(normal.z) + "))");
    const int ref_id = step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
                              format_double(ref_direction.y) + "," +
                              format_double(ref_direction.z) + "))");
    const int axis_id = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin_id) + "," +
                                 step.ref(normal_id) + "," + step.ref(ref_id) + ")");
    return step.add("PLANE(''," + step.ref(axis_id) + ")");
  };

  auto add_cylinder_surface = [&](const Vec3& base_center,
                                  const Vec3& axis,
                                  const Vec3& ref_direction,
                                  double radius) -> int {
    const int center_point = step.add("CARTESIAN_POINT('',(" + format_double(base_center.x) + "," +
                                    format_double(base_center.y) + "," +
                                    format_double(base_center.z) + "))");
    const int axis_direction = step.add("DIRECTION('',(" + format_double(axis.x) + "," +
                                        format_double(axis.y) + "," + format_double(axis.z) + "))");
    const int ref_dir_id = step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
                                      format_double(ref_direction.y) + "," +
                                      format_double(ref_direction.z) + "))");
    const int axis_id =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(center_point) + "," + step.ref(axis_direction) +
                 "," + step.ref(ref_dir_id) + ")");
    return step.add("CYLINDRICAL_SURFACE(''," + step.ref(axis_id) + "," +
                    format_double(radius) + ")");
  };

  std::vector<int> top_outer_loop = slot.top_outer_loop;
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, basis_from_normal(slot.axis)) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  std::vector<int> bottom_outer_loop = slot.bottom_outer_loop;
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, basis_from_normal(slot.axis)) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_center = slot.slot_origin + slot.axis * slot.top_plane;
  const Vec3 bottom_center = slot.slot_origin + slot.axis * slot.bottom_plane;
  const Vec3 top_right_center = top_center + slot.major_direction * slot.half_span;
  const Vec3 top_left_center = top_center - slot.major_direction * slot.half_span;
  const Vec3 bottom_right_center = bottom_center + slot.major_direction * slot.half_span;
  const Vec3 bottom_left_center = bottom_center - slot.major_direction * slot.half_span;

  const int top_tl = add_vertex(top_left_center + slot.minor_direction * slot.radius);
  const int top_tr = add_vertex(top_right_center + slot.minor_direction * slot.radius);
  const int top_br = add_vertex(top_right_center - slot.minor_direction * slot.radius);
  const int top_bl = add_vertex(top_left_center - slot.minor_direction * slot.radius);
  const int bottom_tl = add_vertex(bottom_left_center + slot.minor_direction * slot.radius);
  const int bottom_tr = add_vertex(bottom_right_center + slot.minor_direction * slot.radius);
  const int bottom_br = add_vertex(bottom_right_center - slot.minor_direction * slot.radius);
  const int bottom_bl = add_vertex(bottom_left_center - slot.minor_direction * slot.radius);

  const int top_upper_line = get_line_edge_curve_id(top_tl, top_tr);
  const int top_lower_line = get_line_edge_curve_id(top_br, top_bl);
  const int bottom_upper_line = get_line_edge_curve_id(bottom_tl, bottom_tr);
  const int bottom_lower_line = get_line_edge_curve_id(bottom_br, bottom_bl);
  const int seam_tl = get_line_edge_curve_id(bottom_tl, top_tl);
  const int seam_tr = get_line_edge_curve_id(bottom_tr, top_tr);
  const int seam_br = get_line_edge_curve_id(bottom_br, top_br);
  const int seam_bl = get_line_edge_curve_id(bottom_bl, top_bl);

  const int top_right_arc = add_arc_edge_curve(
      top_tr,
      top_br,
      top_right_center,
      slot.axis,
      slot.major_direction,
      slot.radius,
      top_right_center + slot.major_direction * slot.radius);
  const int top_left_arc = add_arc_edge_curve(
      top_bl,
      top_tl,
      top_left_center,
      slot.axis,
      slot.major_direction * -1.0,
      slot.radius,
      top_left_center - slot.major_direction * slot.radius);
  const int bottom_right_arc = add_arc_edge_curve(
      bottom_tr,
      bottom_br,
      bottom_right_center,
      slot.axis,
      slot.major_direction,
      slot.radius,
      bottom_right_center + slot.major_direction * slot.radius);
  const int bottom_left_arc = add_arc_edge_curve(
      bottom_bl,
      bottom_tl,
      bottom_left_center,
      slot.axis,
      slot.major_direction * -1.0,
      slot.radius,
      bottom_left_center - slot.major_direction * slot.radius);

  const int top_plane = add_plane(top_center, slot.axis, slot.major_direction);
  const int bottom_plane = add_plane(bottom_center, slot.axis, slot.major_direction);
  const int upper_slot_plane = add_plane(top_center + slot.minor_direction * slot.radius,
                                         slot.minor_direction,
                                         slot.major_direction);
  const int lower_slot_plane = add_plane(top_center - slot.minor_direction * slot.radius,
                                         slot.minor_direction * -1.0,
                                         slot.major_direction);
  const int right_cylinder =
      add_cylinder_surface(bottom_right_center, slot.axis, slot.major_direction, slot.radius);
  const int left_cylinder =
      add_cylinder_surface(bottom_left_center, slot.axis, slot.major_direction * -1.0, slot.radius);

  const std::vector<std::pair<int, bool>> top_hole_cw {
      {top_right_arc, true},
      {top_lower_line, true},
      {top_left_arc, true},
      {top_upper_line, true},
  };
  const std::vector<std::pair<int, bool>> top_hole_ccw {
      {top_upper_line, false},
      {top_left_arc, false},
      {top_lower_line, false},
      {top_right_arc, false},
  };
  const std::vector<std::pair<int, bool>> bottom_hole_cw {
      {bottom_right_arc, true},
      {bottom_lower_line, true},
      {bottom_left_arc, true},
      {bottom_upper_line, true},
  };
  const std::vector<std::pair<int, bool>> bottom_hole_ccw {
      {bottom_upper_line, false},
      {bottom_left_arc, false},
      {bottom_lower_line, false},
      {bottom_right_arc, false},
  };

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const int top_inner_loop_id =
      add_edge_loop(slot.top_inner_area < 0.0 ? top_hole_cw : top_hole_ccw);
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop_id) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_inner_loop_id =
      add_edge_loop(slot.bottom_inner_area < 0.0 ? bottom_hole_cw : bottom_hole_ccw);
  const int bottom_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + "," +
                                   step.ref(bottom_inner_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  const int upper_slot_loop = add_line_loop({bottom_tl, bottom_tr, top_tr, top_tl});
  const int upper_slot_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(upper_slot_loop) + ",.T.)");
  const int upper_slot_face = step.add("ADVANCED_FACE('',(" + step.ref(upper_slot_bound) + ")," +
                                       step.ref(upper_slot_plane) + ",.T.)");

  const int lower_slot_loop = add_line_loop({bottom_br, bottom_bl, top_bl, top_br});
  const int lower_slot_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(lower_slot_loop) + ",.T.)");
  const int lower_slot_face = step.add("ADVANCED_FACE('',(" + step.ref(lower_slot_bound) + ")," +
                                       step.ref(lower_slot_plane) + ",.T.)");

  const int right_cylinder_loop = add_edge_loop(
      {{top_right_arc, true}, {seam_br, false}, {bottom_right_arc, false}, {seam_tr, true}});
  const int right_cylinder_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(right_cylinder_loop) + ",.T.)");
  const int right_cylinder_face =
      step.add("ADVANCED_FACE('',(" + step.ref(right_cylinder_bound) + ")," +
               step.ref(right_cylinder) + ",.T.)");

  const int left_cylinder_loop = add_edge_loop(
      {{top_left_arc, true}, {seam_tl, false}, {bottom_left_arc, false}, {seam_bl, true}});
  const int left_cylinder_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(left_cylinder_loop) + ",.T.)");
  const int left_cylinder_face =
      step.add("ADVANCED_FACE('',(" + step.ref(left_cylinder_bound) + ")," +
               step.ref(left_cylinder) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(slot.outer_face_indices.size());
  for (int face_index : slot.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
    const int edge_loop = add_line_loop(face.loops.front());
    const int bound = step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)");
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + step.ref(bound) + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face) << ","
              << step.ref(upper_slot_face) << "," << step.ref(lower_slot_face) << ","
              << step.ref(right_cylinder_face) << "," << step.ref(left_cylinder_face);
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_prismatic_multi_counterbore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticMultiCounterboreExport& counterbore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = counterbore_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = counterbore_export.bottom_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(counterbore_export.counterbores.front().upper_bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 axis = counterbore_export.counterbores.front().upper_bore.axis;
  const Vec3 ref_direction = counterbore_export.counterbores.front().upper_bore.ref_direction;
  const Vec3 top_center =
      counterbore_export.counterbores.front().upper_bore.base_center +
      axis * counterbore_export.counterbores.front().upper_bore.height;
  const Vec3 bottom_center = counterbore_export.counterbores.front().lower_bore.base_center;

  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                      step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                      ")");
  const int bottom_plane_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_plane_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_plane_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");

    const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                                  step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
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
    const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                           step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
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
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  std::vector<int> step_plane_ids;
  std::vector<CylinderBundle> upper_bundles;
  std::vector<CylinderBundle> lower_bundles;
  step_plane_ids.reserve(counterbore_export.counterbores.size());
  upper_bundles.reserve(counterbore_export.counterbores.size());
  lower_bundles.reserve(counterbore_export.counterbores.size());
  for (const SimplePrismaticMultiCounterbore& counterbore : counterbore_export.counterbores) {
    const Vec3 step_center = counterbore.upper_bore.base_center;
    const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                          format_double(step_center.y) + "," +
                                          format_double(step_center.z) + "))");
    const int step_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int step_plane = step.add("PLANE(''," + step.ref(step_axis) + ")");
    step_plane_ids.push_back(step_plane);
    upper_bundles.push_back(add_cylinder_bundle(counterbore.upper_bore, top_plane, step_plane));
    lower_bundles.push_back(add_cylinder_bundle(counterbore.lower_bore, step_plane, bottom_plane));
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  std::vector<int> top_bound_ids;
  top_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool top_circle_forward = counterbore_export.counterbores[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundles[index].top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  std::vector<int> bottom_bound_ids;
  bottom_bound_ids.push_back(
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool bottom_circle_forward = counterbore_export.counterbores[index].bottom_inner_area > 0.0;
    const int bottom_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundles[index].bottom_edge_curve) + "," +
                 std::string(bottom_circle_forward ? ".T." : ".F.") + ")");
    const int bottom_inner_loop =
        step.add("EDGE_LOOP('',(" + step.ref(bottom_inner_oriented) + "))");
    bottom_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop) + ",.T.)"));
  }
  std::ostringstream bottom_bound_refs;
  for (std::size_t index = 0; index < bottom_bound_ids.size(); ++index) {
    if (index > 0) {
      bottom_bound_refs << ",";
    }
    bottom_bound_refs << step.ref(bottom_bound_ids[index]);
  }
  const int bottom_face = step.add("ADVANCED_FACE('',(" + bottom_bound_refs.str() + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> step_face_ids;
  step_face_ids.reserve(counterbore_export.counterbores.size());
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool step_outer_circle_forward = counterbore_export.counterbores[index].step_outer_area > 0.0;
    const int step_outer_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundles[index].bottom_edge_curve) + "," +
                 std::string(step_outer_circle_forward ? ".T." : ".F.") + ")");
    const int step_outer_loop = step.add("EDGE_LOOP('',(" + step.ref(step_outer_oriented) + "))");
    const int step_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(step_outer_loop) + ",.T.)");

    const bool step_inner_circle_forward = counterbore_export.counterbores[index].step_inner_area > 0.0;
    const int step_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundles[index].top_edge_curve) + "," +
                 std::string(step_inner_circle_forward ? ".T." : ".F.") + ")");
    const int step_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(step_inner_oriented) + "))");
    const int step_inner_bound =
        step.add("FACE_BOUND(''," + step.ref(step_inner_loop) + ",.T.)");
    step_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(step_outer_bound) + "," +
                                     step.ref(step_inner_bound) + ")," +
                                     step.ref(step_plane_ids[index]) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(counterbore_export.outer_face_indices.size());
  for (int face_index : counterbore_export.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
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
      const int edge_loop = add_line_loop(face.loops[loop_index]);
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
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    shell_faces << "," << step.ref(step_face_ids[index]) << ","
                << step.ref(upper_bundles[index].lateral_face) << ","
                << step.ref(lower_bundles[index].lateral_face);
  }
  for (int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }
  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

bool write_step_file_generic_cylindrical_holes(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const GenericCylindricalThroughHoleExport& hole_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

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
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
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
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::unordered_map<int, StepPlaneSurface> face_planes;
  auto get_face_plane = [&](int face_index) -> const StepPlaneSurface& {
    const auto found = face_planes.find(face_index);
    if (found != face_planes.end()) {
      return found->second;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    return face_planes.emplace(
        face_index,
        add_step_plane_surface(step, face.fit.centroid, face.fit.normal, basis.u)).first->second;
  };

  struct HoleLoopRef {
    std::size_t hole_index {};
    bool is_top {};
    double area {};
  };

  auto make_loop_key = [](int face_index, std::size_t loop_index) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(face_index)) << 32U) |
           static_cast<std::uint64_t>(loop_index);
  };

  std::unordered_set<int> omitted_side_faces;
  std::unordered_map<std::uint64_t, HoleLoopRef> hole_loop_refs;
  std::vector<StepCylinderBundle> hole_bundles;
  hole_bundles.reserve(hole_export.holes.size());
  for (std::size_t hole_index = 0; hole_index < hole_export.holes.size(); ++hole_index) {
    const GenericCylindricalThroughHole& hole = hole_export.holes[hole_index];
    const StepPlaneSurface& top_plane = get_face_plane(hole.top_face_index);
    const StepPlaneSurface& bottom_plane = get_face_plane(hole.bottom_face_index);
    hole_bundles.push_back(add_step_cylinder_bundle(step, context_ids, hole.cylinder, top_plane, bottom_plane));
    hole_loop_refs.emplace(
        make_loop_key(hole.top_face_index, hole.top_loop_index),
        HoleLoopRef {hole_index, true, hole.top_loop_area});
    hole_loop_refs.emplace(
        make_loop_key(hole.bottom_face_index, hole.bottom_loop_index),
        HoleLoopRef {hole_index, false, hole.bottom_loop_area});
    omitted_side_faces.insert(hole.side_face_indices.begin(), hole.side_face_indices.end());
  }

  std::unordered_set<int> special_cap_faces;
  for (const GenericCylindricalThroughHole& hole : hole_export.holes) {
    special_cap_faces.insert(hole.top_face_index);
    special_cap_faces.insert(hole.bottom_face_index);
  }

  std::vector<int> face_ids;
  face_ids.reserve(reconstruction.faces.size());
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (omitted_side_faces.contains(static_cast<int>(face_index))) {
      continue;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (!special_cap_faces.contains(static_cast<int>(face_index))) {
      const StepPlaneSurface& plane = get_face_plane(static_cast<int>(face_index));
      std::vector<int> bound_ids;
      bound_ids.reserve(face.loops.size());
      for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
        const int edge_loop = add_line_loop(face.loops[loop_index]);
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
      face_ids.push_back(
          step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane.plane_id) + ",.T.)"));
      continue;
    }

    const StepPlaneSurface& plane = get_face_plane(static_cast<int>(face_index));
    const std::vector<double> loop_areas =
        projected_face_loop_areas(face, reconstruction.vertices, plane.basis);
    const std::optional<std::size_t> outer_loop_index = outer_loop_index_from_areas(loop_areas);
    if (!outer_loop_index.has_value()) {
      return false;
    }

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const auto hole_ref = hole_loop_refs.find(make_loop_key(static_cast<int>(face_index), loop_index));
      if (hole_ref != hole_loop_refs.end()) {
        const StepCylinderBundle& bundle = hole_bundles[hole_ref->second.hole_index];
        const int edge_curve_id = hole_ref->second.is_top ? bundle.top_edge_curve : bundle.bottom_edge_curve;
        const bool same_orientation = hole_ref->second.area > 0.0;
        const int oriented_edge =
            step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                     std::string(same_orientation ? ".T." : ".F.") + ")");
        const int edge_loop = step.add("EDGE_LOOP('',(" + step.ref(oriented_edge) + "))");
        if (loop_index == *outer_loop_index) {
          bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
        } else {
          bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
        }
        continue;
      }

      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == *outer_loop_index) {
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
    face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane.plane_id) + ",.T.)"));
  }

  for (const StepCylinderBundle& bundle : hole_bundles) {
    face_ids.push_back(bundle.lateral_face);
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
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
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

  if (const std::optional<SimplePrismaticBlindBoreExport> blind_bore =
          detect_simple_prismatic_blind_bore_export(reconstruction);
      blind_bore.has_value()) {
    return write_step_file_prismatic_blind_bore(path, reconstruction, *blind_bore);
  }

  if (const std::optional<SimplePrismaticMultiBlindCounterboreExport> multi_blind_counterbore =
          detect_simple_prismatic_multi_blind_counterbore_export(reconstruction);
      multi_blind_counterbore.has_value()) {
    return write_step_file_prismatic_multi_blind_counterbore(path, reconstruction, *multi_blind_counterbore);
  }

  if (const std::optional<SimplePrismaticMultiBlindBoreExport> multi_blind_bore =
          detect_simple_prismatic_multi_blind_bore_export(reconstruction);
      multi_blind_bore.has_value()) {
    return write_step_file_prismatic_multi_blind_bore(path, reconstruction, *multi_blind_bore);
  }

  if (const std::optional<SimplePrismaticBlindCounterboreExport> blind_counterbore =
          detect_simple_prismatic_blind_counterbore_export(reconstruction);
      blind_counterbore.has_value()) {
    return write_step_file_prismatic_blind_counterbore(path, reconstruction, *blind_counterbore);
  }

  if (const std::optional<SimplePrismaticStandoffExport> standoff =
          detect_simple_prismatic_standoff_export(reconstruction);
      standoff.has_value()) {
    return write_step_file_prismatic_standoff(path, reconstruction, *standoff);
  }

  if (const std::optional<SimplePrismaticObroundSlotExport> obround_slot =
          detect_simple_prismatic_obround_slot_export(reconstruction);
      obround_slot.has_value()) {
    return write_step_file_prismatic_obround_slot(path, reconstruction, *obround_slot);
  }

  if (const std::optional<SimplePrismaticMultiBossExport> multi_boss =
          detect_simple_prismatic_multi_boss_export(reconstruction);
      multi_boss.has_value()) {
    return write_step_file_prismatic_multi_boss(path, reconstruction, *multi_boss);
  }

  if (const std::optional<SimplePrismaticBossExport> boss =
          detect_simple_prismatic_boss_export(reconstruction);
      boss.has_value()) {
    return write_step_file_prismatic_boss(path, reconstruction, *boss);
  }

  if (const std::optional<SimplePrismaticCounterboreExport> counterbore =
          detect_simple_prismatic_counterbore_export(reconstruction);
      counterbore.has_value()) {
    return write_step_file_prismatic_counterbore(path, reconstruction, *counterbore);
  }

  if (const std::optional<SimplePrismaticMultiCounterboreExport> multi_counterbore =
          detect_simple_prismatic_multi_counterbore_export(reconstruction);
      multi_counterbore.has_value()) {
    return write_step_file_prismatic_multi_counterbore(path, reconstruction, *multi_counterbore);
  }

  if (const std::optional<SimplePrismaticMultiBoreExport> multi_bore =
          detect_simple_prismatic_multi_bore_export(reconstruction);
      multi_bore.has_value()) {
    return write_step_file_prismatic_multi_bore(path, reconstruction, *multi_bore);
  }

  if (const std::optional<SimplePrismaticBoreExport> bore =
          detect_simple_prismatic_bore_export(reconstruction);
      bore.has_value()) {
    return write_step_file_prismatic_bore(path, reconstruction, *bore);
  }

  if (const std::optional<GenericCylindricalThroughHoleExport> generic_holes =
          detect_generic_cylindrical_through_hole_export(reconstruction);
      generic_holes.has_value()) {
    return write_step_file_generic_cylindrical_holes(path, reconstruction, *generic_holes);
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
  output << "  \"method\": \"" << escape_json(reconstruction_method_to_string(reconstruction.method))
         << "\",\n";
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
  output << "  \"plane_regularization\": {\n";
  output << "    \"method\": \"" << escape_json(report.plane_regularization.method) << "\",\n";
  output << "    \"applied\": " << bool_json(report.plane_regularization.applied) << ",\n";
  output << "    \"adjusted_planes\": " << report.plane_regularization.adjusted_planes << "\n";
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
  output << "    \"method\": \""
         << escape_json(reconstruction_method_to_string(report.reconstruction.method)) << "\",\n";
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
