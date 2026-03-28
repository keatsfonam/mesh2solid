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
