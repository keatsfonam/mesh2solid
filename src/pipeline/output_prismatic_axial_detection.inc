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
