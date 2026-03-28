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

struct GenericCylindricalBlindFeature {
  int support_face_index {};
  int cap_face_index {};
  std::size_t support_loop_index {};
  double support_loop_area {};
  double cap_loop_area {};
  SimpleCylinderPrimitive cylinder {};
  std::vector<int> side_face_indices;
  bool support_is_top {};
};

struct GenericCylindricalBlindFeatureExport {
  std::vector<GenericCylindricalBlindFeature> features;
};

struct GenericCylindricalFeatureExport {
  std::vector<GenericCylindricalThroughHole> holes;
  std::vector<GenericCylindricalBlindFeature> blind_features;
  std::vector<GenericSmoothExtrudedBand> smooth_bands;
};

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

  std::vector<GenericCylindricalThroughHole> candidate_holes;

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
        const Vec2 center {
            0.5 * (top_loop.circle.center.x + bottom_loop.circle.center.x),
            0.5 * (top_loop.circle.center.y + bottom_loop.circle.center.y),
        };
        std::vector<int> side_face_indices;
        side_face_indices.reserve(top_loop.loop.size());
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        const double radius = 0.5 * (top_loop.circle.radius + bottom_loop.circle.radius);
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

          const std::optional<CylinderShellFaceMatch> shell_match = match_face_to_cylinder_shell(
              face,
              reconstruction.vertices,
              axis_origin,
              axis,
              radius,
              bottom_plane,
              top_plane,
              matched_tolerance);
          if (!shell_match.has_value()) {
            continue;
          }

          side_face_indices.push_back(static_cast<int>(face_index));
        }

        if (side_face_indices.size() < 6) {
          continue;
        }

        if (has_intermediate_matching_circular_section(reconstruction,
                                                       top_face_index,
                                                       bottom_face_index,
                                                       axis,
                                                       axis_basis,
                                                       top_plane,
                                                       bottom_plane,
                                                       center,
                                                       radius,
                                                       matched_tolerance)) {
          continue;
        }

        matched_bottom[best_bottom_index] = true;
        used_side_faces.insert(side_face_indices.begin(), side_face_indices.end());
        const Vec3 base_center = axis_origin + axis * bottom_plane;
        const Vec3 seam_vector =
            reconstruction.vertices[bottom_loop.loop.front()] - base_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          continue;
        }

        candidate_holes.push_back(GenericCylindricalThroughHole {
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
    }
  }

  if (candidate_holes.empty()) {
    return std::nullopt;
  }

  std::sort(candidate_holes.begin(),
            candidate_holes.end(),
            [](const GenericCylindricalThroughHole& lhs, const GenericCylindricalThroughHole& rhs) {
              if (lhs.side_face_indices.size() != rhs.side_face_indices.size()) {
                return lhs.side_face_indices.size() > rhs.side_face_indices.size();
              }
              if (std::abs(lhs.cylinder.radius - rhs.cylinder.radius) > 1e-9) {
                return lhs.cylinder.radius > rhs.cylinder.radius;
              }
              if (lhs.top_face_index != rhs.top_face_index) {
                return lhs.top_face_index < rhs.top_face_index;
              }
              return lhs.top_loop_index < rhs.top_loop_index;
            });

  auto make_loop_key = [](int face_index, std::size_t loop_index) -> std::uint64_t {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(face_index)) << 32U) |
           static_cast<std::uint64_t>(loop_index);
  };

  std::unordered_set<std::uint64_t> used_loops;
  std::unordered_set<int> used_side_faces;
  GenericCylindricalThroughHoleExport export_result;
  for (const GenericCylindricalThroughHole& hole : candidate_holes) {
    const std::uint64_t top_key = make_loop_key(hole.top_face_index, hole.top_loop_index);
    const std::uint64_t bottom_key = make_loop_key(hole.bottom_face_index, hole.bottom_loop_index);
    if (used_loops.contains(top_key) || used_loops.contains(bottom_key)) {
      continue;
    }

    bool side_conflict = false;
    for (int face_index : hole.side_face_indices) {
      if (used_side_faces.contains(face_index)) {
        side_conflict = true;
        break;
      }
    }
    if (side_conflict) {
      continue;
    }

    used_loops.insert(top_key);
    used_loops.insert(bottom_key);
    used_side_faces.insert(hole.side_face_indices.begin(), hole.side_face_indices.end());
    export_result.holes.push_back(hole);
  }

  if (export_result.holes.empty()) {
    return std::nullopt;
  }
  return export_result;
}

std::optional<GenericCylindricalBlindFeatureExport> detect_generic_cylindrical_blind_feature_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 8 || reconstruction.vertices.size() < 20) {
    return std::nullopt;
  }

  std::vector<int> support_candidates;
  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() >= 2) {
      support_candidates.push_back(static_cast<int>(face_index));
    } else if (face.loops.size() == 1 && face.loops.front().size() >= 8) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (support_candidates.empty() || cap_candidates.empty()) {
    return std::nullopt;
  }

  struct IndexedCircleLoop {
    std::size_t loop_index {};
    std::vector<int> loop;
    double area {};
    CircleFit2D circle {};
  };

  std::vector<GenericCylindricalBlindFeature> candidate_features;
  for (int support_face_index : support_candidates) {
    const ReconstructedFace& support_face = reconstruction.faces[support_face_index];
    Vec3 axis = normalized(support_face.fit.normal);
    if (length_squared(axis) <= 1e-12) {
      continue;
    }

    const PlaneBasis axis_basis = basis_from_normal(axis);
    const std::vector<double> support_areas =
        projected_face_loop_areas(support_face, reconstruction.vertices, axis_basis);
    const std::optional<std::size_t> outer_loop_index = outer_loop_index_from_areas(support_areas);
    if (!outer_loop_index.has_value()) {
      continue;
    }

    std::vector<IndexedCircleLoop> support_loops;
    for (std::size_t loop_index = 0; loop_index < support_face.loops.size(); ++loop_index) {
      if (loop_index == *outer_loop_index || support_face.loops[loop_index].size() < 8) {
        continue;
      }
      const std::optional<CircleFit2D> circle =
          fit_circle_2d(project_loop_to_basis(support_face.loops[loop_index], reconstruction.vertices, axis_basis));
      if (!circle.has_value() || circle->radius <= 1e-6) {
        continue;
      }
      const double tolerance = std::max(circle->radius * 0.03, 1e-4);
      if (circle->max_error > tolerance) {
        continue;
      }
      support_loops.push_back(
          {loop_index, support_face.loops[loop_index], support_areas[loop_index], *circle});
    }
    if (support_loops.empty()) {
      continue;
    }

    const double support_plane = dot(support_face.fit.centroid, axis);
    for (int cap_face_index : cap_candidates) {
      if (cap_face_index == support_face_index) {
        continue;
      }

      const ReconstructedFace& cap_face = reconstruction.faces[cap_face_index];
      if (angle_degrees_unsigned(support_face.fit.normal, cap_face.fit.normal) > 8.0) {
        continue;
      }

      const std::vector<int>& cap_loop = cap_face.loops.front();
      const std::optional<CircleFit2D> cap_circle =
          fit_circle_2d(project_loop_to_basis(cap_loop, reconstruction.vertices, axis_basis));
      if (!cap_circle.has_value() || cap_circle->radius <= 1e-6) {
        continue;
      }
      const double cap_tolerance = std::max(cap_circle->radius * 0.03, 1e-4);
      if (cap_circle->max_error > cap_tolerance) {
        continue;
      }

      const double cap_plane = dot(cap_face.fit.centroid, axis);
      for (const IndexedCircleLoop& support_loop : support_loops) {
        const double match_tolerance =
            std::max((support_loop.circle.radius + cap_circle->radius) * 0.015, 1e-4);
        const Vec2 center_delta {support_loop.circle.center.x - cap_circle->center.x,
                                 support_loop.circle.center.y - cap_circle->center.y};
        const double center_distance =
            std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y);
        if (std::abs(support_loop.circle.radius - cap_circle->radius) > match_tolerance ||
            center_distance > match_tolerance ||
            std::abs(cap_plane - support_plane) <= match_tolerance) {
          continue;
        }

        std::vector<int> side_face_indices;
        side_face_indices.reserve(support_loop.loop.size());
        const bool support_is_top = support_plane > cap_plane;
        const double top_plane = std::max(support_plane, cap_plane);
        const double bottom_plane = std::min(support_plane, cap_plane);
        const double radius = 0.5 * (support_loop.circle.radius + cap_circle->radius);
        const Vec2 center {
            0.5 * (support_loop.circle.center.x + cap_circle->center.x),
            0.5 * (support_loop.circle.center.y + cap_circle->center.y),
        };
        const Vec3 axis_origin = axis_basis.u * center.x + axis_basis.v * center.y;
        for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
          if (static_cast<int>(face_index) == support_face_index ||
              static_cast<int>(face_index) == cap_face_index) {
            continue;
          }

          const ReconstructedFace& face = reconstruction.faces[face_index];
          if (face.loops.size() != 1 || face.loops.front().size() < 4) {
            continue;
          }
          if (angle_degrees_unsigned(face.fit.normal, axis) < 20.0) {
            continue;
          }

          const std::optional<CylinderShellFaceMatch> shell_match = match_face_to_cylinder_shell(
              face,
              reconstruction.vertices,
              axis_origin,
              axis,
              radius,
              bottom_plane,
              top_plane,
              match_tolerance);
          if (!shell_match.has_value()) {
            continue;
          }

          side_face_indices.push_back(static_cast<int>(face_index));
        }

        if (side_face_indices.size() < 6) {
          continue;
        }

        const Vec3 base_center = axis_origin + axis * bottom_plane;
        const std::vector<int>& bottom_loop = support_is_top ? cap_loop : support_loop.loop;
        const Vec3 seam_vector = reconstruction.vertices[bottom_loop.front()] - base_center;
        const Vec3 radial = seam_vector - axis * dot(seam_vector, axis);
        if (length_squared(radial) <= 1e-12) {
          continue;
        }

        candidate_features.push_back(GenericCylindricalBlindFeature {
            support_face_index,
            cap_face_index,
            support_loop.loop_index,
            support_loop.area,
            projected_loop_area(cap_loop, reconstruction.vertices, axis_basis),
            SimpleCylinderPrimitive {base_center, axis, normalized(radial), radius, top_plane - bottom_plane},
            side_face_indices,
            support_is_top,
        });
      }
    }
  }

  if (candidate_features.empty()) {
    return std::nullopt;
  }

  std::sort(candidate_features.begin(),
            candidate_features.end(),
            [](const GenericCylindricalBlindFeature& lhs, const GenericCylindricalBlindFeature& rhs) {
              if (lhs.side_face_indices.size() != rhs.side_face_indices.size()) {
                return lhs.side_face_indices.size() > rhs.side_face_indices.size();
              }
              if (std::abs(lhs.cylinder.radius - rhs.cylinder.radius) > 1e-9) {
                return lhs.cylinder.radius > rhs.cylinder.radius;
              }
              if (lhs.support_face_index != rhs.support_face_index) {
                return lhs.support_face_index < rhs.support_face_index;
              }
              return lhs.support_loop_index < rhs.support_loop_index;
            });

  std::unordered_set<std::uint64_t> used_loops;
  std::unordered_set<int> used_side_faces;
  GenericCylindricalBlindFeatureExport export_result;
  for (const GenericCylindricalBlindFeature& feature : candidate_features) {
    const std::uint64_t support_key =
        make_face_loop_key(feature.support_face_index, feature.support_loop_index);
    const std::uint64_t cap_key = make_face_loop_key(feature.cap_face_index, 0);
    if (used_loops.contains(support_key) || used_loops.contains(cap_key)) {
      continue;
    }

    bool side_conflict = false;
    for (int face_index : feature.side_face_indices) {
      if (used_side_faces.contains(face_index)) {
        side_conflict = true;
        break;
      }
    }
    if (side_conflict) {
      continue;
    }

    used_loops.insert(support_key);
    used_loops.insert(cap_key);
    used_side_faces.insert(feature.side_face_indices.begin(), feature.side_face_indices.end());
    export_result.features.push_back(feature);
  }

  if (export_result.features.empty()) {
    return std::nullopt;
  }
  return export_result;
}

std::optional<GenericCylindricalFeatureExport> merge_generic_cylindrical_feature_exports(
    const std::optional<GenericCylindricalThroughHoleExport>& holes,
    const std::optional<GenericCylindricalBlindFeatureExport>& blind_features,
    const std::optional<GenericSmoothExtrudedBandExport>& smooth_bands) {
  GenericCylindricalFeatureExport merged;
  std::unordered_set<std::uint64_t> used_loops;
  std::unordered_set<int> used_side_faces;
  std::set<std::pair<std::uint64_t, std::uint64_t>> used_smooth_loop_pairs;
  std::set<std::tuple<std::uint64_t, std::uint64_t, std::size_t, std::size_t, std::size_t, std::size_t>>
      used_smooth_band_spans;

  if (holes.has_value()) {
    merged.holes = holes->holes;
    for (const GenericCylindricalThroughHole& hole : holes->holes) {
      used_loops.insert(make_face_loop_key(hole.top_face_index, hole.top_loop_index));
      used_loops.insert(make_face_loop_key(hole.bottom_face_index, hole.bottom_loop_index));
      used_side_faces.insert(hole.side_face_indices.begin(), hole.side_face_indices.end());
    }
  }

  if (blind_features.has_value()) {
    for (const GenericCylindricalBlindFeature& feature : blind_features->features) {
      const std::uint64_t support_key =
          make_face_loop_key(feature.support_face_index, feature.support_loop_index);
      const std::uint64_t cap_key = make_face_loop_key(feature.cap_face_index, 0);
      if (used_loops.contains(support_key) || used_loops.contains(cap_key)) {
        continue;
      }

      bool side_conflict = false;
      for (int face_index : feature.side_face_indices) {
        if (used_side_faces.contains(face_index)) {
          side_conflict = true;
          break;
        }
      }
      if (side_conflict) {
        continue;
      }

      used_loops.insert(support_key);
      used_loops.insert(cap_key);
      used_side_faces.insert(feature.side_face_indices.begin(), feature.side_face_indices.end());
      merged.blind_features.push_back(feature);
    }
  }

  if (smooth_bands.has_value()) {
    for (const GenericSmoothExtrudedBand& band : smooth_bands->bands) {
      const std::uint64_t top_key = make_face_loop_key(band.top_face_index, band.top_loop_index);
      const std::uint64_t bottom_key = make_face_loop_key(band.bottom_face_index, band.bottom_loop_index);
      const std::pair<std::uint64_t, std::uint64_t> loop_pair {top_key, bottom_key};
      const auto band_span_key = std::make_tuple(top_key,
                                                 bottom_key,
                                                 band.top_loop_start,
                                                 band.top_loop_span,
                                                 band.bottom_loop_start,
                                                 band.bottom_loop_span);
      if (used_smooth_band_spans.contains(band_span_key)) {
        continue;
      }
      const bool same_smooth_pair = used_smooth_loop_pairs.contains(loop_pair);
      if (!same_smooth_pair && (used_loops.contains(top_key) || used_loops.contains(bottom_key))) {
        continue;
      }

      used_loops.insert(top_key);
      used_loops.insert(bottom_key);
      used_smooth_loop_pairs.insert(loop_pair);
      used_smooth_band_spans.insert(band_span_key);
      used_side_faces.insert(band.side_face_indices.begin(), band.side_face_indices.end());
      merged.smooth_bands.push_back(band);
    }
  }

  if (merged.holes.empty() && merged.blind_features.empty() && merged.smooth_bands.empty()) {
    return std::nullopt;
  }
  return merged;
}
