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

struct SimplePrismaticBossExport {
  int base_face_index {};
  int top_face_index {};
  std::vector<int> base_outer_loop;
  double base_inner_area {};
  double top_area {};
  SimpleCylinderPrimitive boss {};
  std::vector<int> outer_face_indices;
};

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

  if (const std::optional<SimplePrismaticBoreExport> bore =
          detect_simple_prismatic_bore_export(reconstruction);
      bore.has_value()) {
    return write_step_file_prismatic_bore(path, reconstruction, *bore);
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
