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

