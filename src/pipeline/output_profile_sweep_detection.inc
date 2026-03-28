struct GenericSmoothExtrudedBand {
  int top_face_index {};
  int bottom_face_index {};
  std::size_t top_loop_index {};
  std::size_t bottom_loop_index {};
  std::size_t top_loop_start {};
  std::size_t top_loop_span {};
  bool top_forward {};
  std::size_t bottom_loop_start {};
  std::size_t bottom_loop_span {};
  bool bottom_forward {};
  Vec3 axis {};
  PlaneBasis basis {};
  double top_plane {};
  double bottom_plane {};
  std::vector<Vec2> top_profile_points;
  std::vector<Vec2> bottom_profile_points;
  std::vector<int> side_face_indices;
  bool prefer_cylindrical_surface {};
};

struct GenericSmoothExtrudedBandExport {
  std::vector<GenericSmoothExtrudedBand> bands;
};

using PrismaticProfileSweepBand = GenericSmoothExtrudedBand;

struct PrismaticProfileSweepBandExport {
  std::vector<PrismaticProfileSweepBand> bands;
};

std::vector<Vec2> build_smooth_band_profile_points(const std::vector<Vec2>& points) {
  if (points.size() <= 3) {
    return points;
  }
  const std::size_t target_count =
      std::max<std::size_t>(points.size() <= 18 ? points.size() : 8,
                            std::min<std::size_t>(18, points.size()));
  return smooth_profile_points(resample_polyline_2d(points, target_count));
}

struct CircularBandSpanCandidate {
  std::size_t start_index {};
  std::size_t end_index {};
  double sweep_degrees {};
  double max_error {};
};

std::vector<CircularBandSpanCandidate> detect_circular_band_spans(const std::vector<Vec2>& top_profile,
                                                                  const std::vector<Vec2>& bottom_profile) {
  if (top_profile.size() != bottom_profile.size() || top_profile.size() < 9) {
    return {};
  }

  std::vector<CircularBandSpanCandidate> candidates;
  const std::size_t min_points = 9;
  const std::size_t max_points = 60;
  for (std::size_t start_index = 0; start_index + min_points <= top_profile.size(); ++start_index) {
    const std::size_t end_limit = std::min<std::size_t>(top_profile.size(), start_index + max_points);
    for (std::size_t end_index = start_index + min_points - 1; end_index < end_limit; ++end_index) {
      const std::vector<Vec2> top_span(top_profile.begin() + static_cast<std::ptrdiff_t>(start_index),
                                       top_profile.begin() + static_cast<std::ptrdiff_t>(end_index + 1));
      const std::vector<Vec2> bottom_span(
          bottom_profile.begin() + static_cast<std::ptrdiff_t>(start_index),
          bottom_profile.begin() + static_cast<std::ptrdiff_t>(end_index + 1));
      const std::optional<CircleFit2D> top_circle = fit_circle_2d(top_span);
      const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(bottom_span);
      if (!top_circle.has_value() || !bottom_circle.has_value()) {
        continue;
      }

      const double average_radius = 0.5 * (top_circle->radius + bottom_circle->radius);
      if (!std::isfinite(average_radius) || average_radius <= 1e-6) {
        continue;
      }

      const double tolerance = std::max(average_radius * 0.03, 0.08);
      if (std::abs(top_circle->radius - bottom_circle->radius) > tolerance ||
          !circle_fit_centers_coincident(*top_circle, *bottom_circle, tolerance) ||
          std::max(top_circle->max_error, bottom_circle->max_error) > tolerance) {
        continue;
      }

      const Vec2 start_radial {top_span.front().x - top_circle->center.x,
                               top_span.front().y - top_circle->center.y};
      const Vec2 end_radial {top_span.back().x - top_circle->center.x,
                             top_span.back().y - top_circle->center.y};
      const double start_length = std::sqrt(start_radial.x * start_radial.x +
                                            start_radial.y * start_radial.y);
      const double end_length = std::sqrt(end_radial.x * end_radial.x +
                                          end_radial.y * end_radial.y);
      if (start_length <= 1e-9 || end_length <= 1e-9) {
        continue;
      }

      const double sweep_radians =
          std::acos(clamp((start_radial.x * end_radial.x + start_radial.y * end_radial.y) /
                              (start_length * end_length),
                          -1.0,
                          1.0));
      const double sweep_degrees = sweep_radians * 180.0 / kPi;
      if (!std::isfinite(sweep_degrees) || sweep_degrees < 20.0 || sweep_degrees > 170.0) {
        continue;
      }

      const double line_error =
          std::max(polyline_line_fit_error_2d(top_span), polyline_line_fit_error_2d(bottom_span));
      const double circle_error = std::max(top_circle->max_error, bottom_circle->max_error);
      if (line_error < circle_error * 1.2) {
        continue;
      }

      candidates.push_back(
          CircularBandSpanCandidate {start_index, end_index, sweep_degrees, circle_error});
    }
  }

  std::sort(candidates.begin(),
            candidates.end(),
            [](const CircularBandSpanCandidate& lhs, const CircularBandSpanCandidate& rhs) {
              const std::size_t lhs_span = lhs.end_index - lhs.start_index;
              const std::size_t rhs_span = rhs.end_index - rhs.start_index;
              if (lhs_span != rhs_span) {
                return lhs_span > rhs_span;
              }
              if (lhs.sweep_degrees != rhs.sweep_degrees) {
                return lhs.sweep_degrees > rhs.sweep_degrees;
              }
              return lhs.max_error < rhs.max_error;
            });

  std::vector<CircularBandSpanCandidate> selected;
  for (const CircularBandSpanCandidate& candidate : candidates) {
    bool overlaps = false;
    for (const CircularBandSpanCandidate& existing : selected) {
      if (!(candidate.end_index < existing.start_index || candidate.start_index > existing.end_index)) {
        overlaps = true;
        break;
      }
    }
    if (!overlaps) {
      selected.push_back(candidate);
    }
  }

  std::sort(selected.begin(),
            selected.end(),
            [](const CircularBandSpanCandidate& lhs, const CircularBandSpanCandidate& rhs) {
              return lhs.start_index < rhs.start_index;
            });
  return selected;
}

std::optional<GenericSmoothExtrudedBand> make_generic_smooth_band_from_chains(
    const ReconstructionResult& reconstruction,
    int top_face_index,
    std::size_t top_loop_index,
    int bottom_face_index,
    std::size_t bottom_loop_index,
    const std::vector<int>& top_chain,
    const std::vector<int>& bottom_chain,
    const Vec3& axis,
    const PlaneBasis& axis_basis,
    double top_plane,
    double bottom_plane,
    const std::vector<int>& side_face_indices,
    bool preserve_profile_points = false,
    bool prefer_cylindrical_surface = false) {
  if (top_chain.size() < 2 || bottom_chain.size() < 2) {
    return std::nullopt;
  }

  const std::vector<Vec2> top_profile = project_chain_to_basis(top_chain, reconstruction.vertices, axis_basis);
  const std::vector<Vec2> bottom_profile =
      project_chain_to_basis(bottom_chain, reconstruction.vertices, axis_basis);
  if (top_profile.size() != bottom_profile.size()) {
    return std::nullopt;
  }

  const std::optional<LoopSubsequenceMatch> top_match =
      find_loop_subsequence(reconstruction.faces[top_face_index].loops[top_loop_index], top_chain);
  const std::optional<LoopSubsequenceMatch> bottom_match =
      find_loop_subsequence(reconstruction.faces[bottom_face_index].loops[bottom_loop_index], bottom_chain);
  if (!top_match.has_value() || !bottom_match.has_value()) {
    return std::nullopt;
  }

  GenericSmoothExtrudedBand band;
  band.top_face_index = top_face_index;
  band.bottom_face_index = bottom_face_index;
  band.top_loop_index = top_loop_index;
  band.bottom_loop_index = bottom_loop_index;
  band.top_loop_start = top_match->start;
  band.top_loop_span = top_match->span;
  band.top_forward = top_match->forward;
  band.bottom_loop_start = bottom_match->start;
  band.bottom_loop_span = bottom_match->span;
  band.bottom_forward = bottom_match->forward;
  band.axis = axis;
  band.basis = axis_basis;
  band.top_plane = top_plane;
  band.bottom_plane = bottom_plane;
  band.top_profile_points =
      preserve_profile_points ? top_profile : build_smooth_band_profile_points(top_profile);
  band.bottom_profile_points =
      preserve_profile_points ? bottom_profile : build_smooth_band_profile_points(bottom_profile);
  band.side_face_indices = side_face_indices;
  band.prefer_cylindrical_surface = prefer_cylindrical_surface;
  std::sort(band.side_face_indices.begin(), band.side_face_indices.end());
  band.side_face_indices.erase(std::unique(band.side_face_indices.begin(), band.side_face_indices.end()),
                               band.side_face_indices.end());
  return band;
}

std::vector<GenericSmoothExtrudedBand> subdivide_generic_smooth_band_candidate(
    const ReconstructionResult& reconstruction,
    int top_face_index,
    std::size_t top_loop_index,
    int bottom_face_index,
    std::size_t bottom_loop_index,
    const std::vector<int>& top_chain,
    const std::vector<int>& bottom_chain,
    const Vec3& axis,
    const PlaneBasis& axis_basis,
    double top_plane,
    double bottom_plane,
    const std::vector<int>& side_face_indices) {
  const std::vector<Vec2> top_profile = project_chain_to_basis(top_chain, reconstruction.vertices, axis_basis);
  const std::vector<Vec2> bottom_profile =
      project_chain_to_basis(bottom_chain, reconstruction.vertices, axis_basis);
  if (top_profile.size() != bottom_profile.size() || top_profile.size() < 2) {
    return {};
  }

  const std::vector<CircularBandSpanCandidate> circular_spans =
      detect_circular_band_spans(top_profile, bottom_profile);
  if (circular_spans.empty()) {
    if (const std::optional<GenericSmoothExtrudedBand> band = make_generic_smooth_band_from_chains(
            reconstruction,
            top_face_index,
            top_loop_index,
            bottom_face_index,
            bottom_loop_index,
            top_chain,
            bottom_chain,
            axis,
            axis_basis,
            top_plane,
            bottom_plane,
            side_face_indices);
        band.has_value()) {
      return {*band};
    }
    return {};
  }

  std::vector<std::pair<std::size_t, std::size_t>> ranges;
  std::size_t cursor = 0;
  for (const CircularBandSpanCandidate& span : circular_spans) {
    if (span.start_index > cursor) {
      ranges.emplace_back(cursor, span.start_index);
    }
    ranges.emplace_back(span.start_index, span.end_index);
    cursor = span.end_index;
  }
  if (cursor + 1 < top_chain.size()) {
    ranges.emplace_back(cursor, top_chain.size() - 1);
  }

  std::vector<GenericSmoothExtrudedBand> bands;
  bands.reserve(ranges.size());
  for (const auto& [start_index, end_index] : ranges) {
    if (end_index <= start_index || end_index >= top_chain.size() || end_index >= bottom_chain.size()) {
      continue;
    }
    const std::vector<int> top_subchain(top_chain.begin() + static_cast<std::ptrdiff_t>(start_index),
                                        top_chain.begin() + static_cast<std::ptrdiff_t>(end_index + 1));
    const std::vector<int> bottom_subchain(
        bottom_chain.begin() + static_cast<std::ptrdiff_t>(start_index),
        bottom_chain.begin() + static_cast<std::ptrdiff_t>(end_index + 1));
    const bool preserve_profile_points = std::any_of(
        circular_spans.begin(),
        circular_spans.end(),
        [&](const CircularBandSpanCandidate& span) {
          return span.start_index == start_index && span.end_index == end_index;
        });
    if (const std::optional<GenericSmoothExtrudedBand> band = make_generic_smooth_band_from_chains(
            reconstruction,
            top_face_index,
            top_loop_index,
            bottom_face_index,
            bottom_loop_index,
            top_subchain,
            bottom_subchain,
            axis,
            axis_basis,
            top_plane,
            bottom_plane,
            side_face_indices,
            preserve_profile_points,
            preserve_profile_points);
        band.has_value()) {
      bands.push_back(*band);
    }
  }

  if (bands.empty()) {
    if (const std::optional<GenericSmoothExtrudedBand> band = make_generic_smooth_band_from_chains(
            reconstruction,
            top_face_index,
            top_loop_index,
            bottom_face_index,
            bottom_loop_index,
            top_chain,
            bottom_chain,
            axis,
            axis_basis,
            top_plane,
            bottom_plane,
            side_face_indices);
        band.has_value()) {
      return {*band};
    }
  }
  return bands;
}

std::optional<GenericSmoothExtrudedBandExport> detect_generic_smooth_extruded_band_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 12 || reconstruction.vertices.size() < 40) {
    return std::nullopt;
  }

  struct EdgeRef {
    int face_index {};
    std::size_t loop_index {};
    std::size_t edge_index {};
  };

  std::unordered_map<EdgeKey, std::vector<EdgeRef>, EdgeKeyHash> edge_faces;
  edge_faces.reserve(reconstruction.faces.size() * 4);
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const std::vector<int>& loop = face.loops[loop_index];
      for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
        edge_faces[make_edge_key(loop[edge_index], loop[(edge_index + 1) % loop.size()])].push_back(
            EdgeRef {static_cast<int>(face_index), loop_index, edge_index});
      }
    }
  }

  auto quantize_plane = [](double value) {
    return static_cast<int>(std::llround(value * 1000.0));
  };

  std::unordered_map<std::uint64_t, std::vector<int>> grouped_faces;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() != 1) {
      continue;
    }
    if (std::abs(face.fit.normal.z) >= 0.25) {
      continue;
    }

    std::vector<int> plane_keys;
    plane_keys.reserve(face.loops.front().size());
    for (int vertex_index : face.loops.front()) {
      plane_keys.push_back(quantize_plane(reconstruction.vertices[vertex_index].z));
    }
    std::sort(plane_keys.begin(), plane_keys.end());
    plane_keys.erase(std::unique(plane_keys.begin(), plane_keys.end()), plane_keys.end());
    if (plane_keys.size() != 2 || plane_keys[1] <= plane_keys[0]) {
      continue;
    }

    const std::uint64_t group_key =
        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(plane_keys[0])) << 32U) |
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(plane_keys[1]));
    grouped_faces[group_key].push_back(static_cast<int>(face_index));
  }

  auto decode_group_key = [](std::uint64_t key) {
    return std::pair<int, int> {
        static_cast<int>(static_cast<std::uint32_t>(key >> 32U)),
        static_cast<int>(static_cast<std::uint32_t>(key & 0xffffffffU)),
    };
  };

  auto rms_profile_delta = [](const std::vector<Vec2>& lhs, const std::vector<Vec2>& rhs) {
    if (lhs.size() != rhs.size() || lhs.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    double error_sum = 0.0;
    for (std::size_t index = 0; index < lhs.size(); ++index) {
      const double dx = lhs[index].x - rhs[index].x;
      const double dy = lhs[index].y - rhs[index].y;
      error_sum += dx * dx + dy * dy;
    }
    return std::sqrt(error_sum / static_cast<double>(lhs.size()));
  };

  std::vector<GenericSmoothExtrudedBand> candidates;
  for (const auto& grouped_entry : grouped_faces) {
    const auto [bottom_plane_key, top_plane_key] = decode_group_key(grouped_entry.first);
    const std::vector<int>& grouped = grouped_entry.second;
    if (grouped.size() < 8) {
      continue;
    }

    const std::unordered_set<int> grouped_set(grouped.begin(), grouped.end());
    std::unordered_map<int, std::vector<int>> adjacency;
    for (int face_index : grouped) {
      const std::vector<int>& loop = reconstruction.faces[face_index].loops.front();
      for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
        const EdgeKey key = make_edge_key(loop[edge_index], loop[(edge_index + 1) % loop.size()]);
        const auto found = edge_faces.find(key);
        if (found == edge_faces.end()) {
          continue;
        }
        for (const EdgeRef& ref : found->second) {
          if (ref.face_index != face_index && grouped_set.contains(ref.face_index)) {
            adjacency[face_index].push_back(ref.face_index);
          }
        }
      }
    }

    std::unordered_set<int> visited_faces;
    for (int seed_face : grouped) {
      if (visited_faces.contains(seed_face)) {
        continue;
      }

      std::vector<int> component;
      std::vector<int> stack {seed_face};
      visited_faces.insert(seed_face);
      while (!stack.empty()) {
        const int face_index = stack.back();
        stack.pop_back();
        component.push_back(face_index);
        for (int neighbor : adjacency[face_index]) {
          if (!visited_faces.contains(neighbor)) {
            visited_faces.insert(neighbor);
            stack.push_back(neighbor);
          }
        }
      }
      if (component.size() < 10) {
        continue;
      }

      const std::unordered_set<int> component_set(component.begin(), component.end());
      std::vector<std::pair<int, int>> top_edges;
      std::vector<std::pair<int, int>> bottom_edges;
      std::unordered_map<std::uint64_t, int> top_loop_counts;
      std::unordered_map<std::uint64_t, int> bottom_loop_counts;
      std::unordered_map<std::uint64_t, std::pair<int, std::size_t>> loop_lookup;

      for (int face_index : component) {
        const std::vector<int>& loop = reconstruction.faces[face_index].loops.front();
        for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
          const int from = loop[edge_index];
          const int to = loop[(edge_index + 1) % loop.size()];
          const double from_z = reconstruction.vertices[from].z;
          const double to_z = reconstruction.vertices[to].z;
          const EdgeKey key = make_edge_key(from, to);
          const auto found = edge_faces.find(key);
          if (found == edge_faces.end()) {
            continue;
          }

          std::vector<EdgeRef> external_refs;
          external_refs.reserve(found->second.size());
          for (const EdgeRef& ref : found->second) {
            if (!component_set.contains(ref.face_index)) {
              external_refs.push_back(ref);
            }
          }
          if (external_refs.empty()) {
            continue;
          }

          if (quantize_plane(from_z) == top_plane_key && quantize_plane(to_z) == top_plane_key) {
            top_edges.emplace_back(from, to);
            for (const EdgeRef& ref : external_refs) {
              const std::uint64_t loop_key = make_face_loop_key(ref.face_index, ref.loop_index);
              top_loop_counts[loop_key] += 1;
              loop_lookup.emplace(loop_key, std::pair<int, std::size_t> {ref.face_index, ref.loop_index});
            }
          } else if (quantize_plane(from_z) == bottom_plane_key && quantize_plane(to_z) == bottom_plane_key) {
            bottom_edges.emplace_back(from, to);
            for (const EdgeRef& ref : external_refs) {
              const std::uint64_t loop_key = make_face_loop_key(ref.face_index, ref.loop_index);
              bottom_loop_counts[loop_key] += 1;
              loop_lookup.emplace(loop_key, std::pair<int, std::size_t> {ref.face_index, ref.loop_index});
            }
          }
        }
      }

      if (top_edges.size() < 8 || bottom_edges.size() < 8) {
        continue;
      }

      const auto dominant_loop = [&](const std::unordered_map<std::uint64_t, int>& counts)
          -> std::optional<std::pair<int, std::size_t>> {
        if (counts.empty()) {
          return std::nullopt;
        }
        auto best = std::max_element(
            counts.begin(),
            counts.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.second != rhs.second) {
                return lhs.second < rhs.second;
              }
              return lhs.first > rhs.first;
            });
        if (best == counts.end() || best->second < 6) {
          return std::nullopt;
        }
        return loop_lookup[best->first];
      };

      const std::optional<std::pair<int, std::size_t>> top_loop_ref = dominant_loop(top_loop_counts);
      const std::optional<std::pair<int, std::size_t>> bottom_loop_ref = dominant_loop(bottom_loop_counts);
      if (!top_loop_ref.has_value() || !bottom_loop_ref.has_value()) {
        continue;
      }

      const ReconstructedFace& top_face = reconstruction.faces[top_loop_ref->first];
      const ReconstructedFace& bottom_face = reconstruction.faces[bottom_loop_ref->first];
      if (angle_degrees_unsigned(top_face.fit.normal, bottom_face.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(top_face.fit.normal);
      if (length_squared(axis) <= 1e-12 || std::abs(axis.z) < 0.95) {
        continue;
      }
      if (dot(top_face.fit.centroid - bottom_face.fit.centroid, axis) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      auto filter_edges_for_loop = [&](const std::vector<std::pair<int, int>>& edges,
                                       int face_index,
                                       std::size_t loop_index) {
        std::vector<std::pair<int, int>> filtered;
        filtered.reserve(edges.size());
        for (const auto& edge : edges) {
          const auto found = edge_faces.find(make_edge_key(edge.first, edge.second));
          if (found == edge_faces.end()) {
            continue;
          }
          bool matches = false;
          for (const EdgeRef& ref : found->second) {
            if (ref.face_index == face_index && ref.loop_index == loop_index) {
              matches = true;
              break;
            }
          }
          if (matches) {
            filtered.push_back(edge);
          }
        }
        return filtered;
      };

      std::vector<int> top_chain = ordered_open_chain_from_directed_edges(
          filter_edges_for_loop(top_edges, top_loop_ref->first, top_loop_ref->second));
      std::vector<int> bottom_chain = ordered_open_chain_from_directed_edges(
          filter_edges_for_loop(bottom_edges, bottom_loop_ref->first, bottom_loop_ref->second));
      if (top_chain.size() < 6 || bottom_chain.size() < 6) {
        continue;
      }

      std::vector<Vec2> top_profile = project_chain_to_basis(top_chain, reconstruction.vertices, axis_basis);
      std::vector<Vec2> bottom_profile =
          project_chain_to_basis(bottom_chain, reconstruction.vertices, axis_basis);
      const std::size_t sample_count =
          std::max<std::size_t>(8, std::min<std::size_t>(18, std::max(top_profile.size(), bottom_profile.size())));
      const std::vector<Vec2> sampled_top = resample_polyline_2d(top_profile, sample_count);
      const std::vector<Vec2> sampled_bottom = resample_polyline_2d(bottom_profile, sample_count);
      std::vector<Vec2> reversed_bottom = sampled_bottom;
      std::reverse(reversed_bottom.begin(), reversed_bottom.end());
      if (rms_profile_delta(sampled_top, reversed_bottom) < rms_profile_delta(sampled_top, sampled_bottom)) {
        std::reverse(bottom_chain.begin(), bottom_chain.end());
        bottom_profile = project_chain_to_basis(bottom_chain, reconstruction.vertices, axis_basis);
      }

      const std::vector<Vec2> normalized_top = resample_polyline_2d(top_profile, sample_count);
      const std::vector<Vec2> normalized_bottom = resample_polyline_2d(bottom_profile, sample_count);
      const double profile_delta = rms_profile_delta(normalized_top, normalized_bottom);
      const double profile_length =
          std::max(polyline_length_2d(normalized_top), polyline_length_2d(normalized_bottom));
      if (!std::isfinite(profile_delta) ||
          profile_delta > std::max(profile_length * 0.04, 0.18)) {
        continue;
      }

      std::vector<GenericSmoothExtrudedBand> bands = subdivide_generic_smooth_band_candidate(
          reconstruction,
          top_loop_ref->first,
          top_loop_ref->second,
          bottom_loop_ref->first,
          bottom_loop_ref->second,
          top_chain,
          bottom_chain,
          axis,
          axis_basis,
          static_cast<double>(top_plane_key) / 1000.0,
          static_cast<double>(bottom_plane_key) / 1000.0,
          component);
      candidates.insert(candidates.end(), bands.begin(), bands.end());
    }
  }

  std::vector<int> global_side_faces;
  global_side_faces.reserve(reconstruction.faces.size());
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() == 1 && std::abs(face.fit.normal.z) < 0.7) {
      global_side_faces.push_back(static_cast<int>(face_index));
    }
  }

  const std::unordered_set<int> global_side_set(global_side_faces.begin(), global_side_faces.end());
  std::unordered_map<int, std::vector<int>> global_adjacency;
  for (int face_index : global_side_faces) {
    const std::vector<int>& loop = reconstruction.faces[face_index].loops.front();
    for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
      const EdgeKey key = make_edge_key(loop[edge_index], loop[(edge_index + 1) % loop.size()]);
      const auto found = edge_faces.find(key);
      if (found == edge_faces.end()) {
        continue;
      }
      for (const EdgeRef& ref : found->second) {
        if (ref.face_index != face_index && global_side_set.contains(ref.face_index)) {
          global_adjacency[face_index].push_back(ref.face_index);
        }
      }
    }
  }

  std::unordered_set<int> visited_global_faces;
  for (int seed_face : global_side_faces) {
    if (visited_global_faces.contains(seed_face)) {
      continue;
    }

    std::vector<int> component;
    std::vector<int> stack {seed_face};
    visited_global_faces.insert(seed_face);
    while (!stack.empty()) {
      const int face_index = stack.back();
      stack.pop_back();
      component.push_back(face_index);
      for (int neighbor : global_adjacency[face_index]) {
        if (!visited_global_faces.contains(neighbor)) {
          visited_global_faces.insert(neighbor);
          stack.push_back(neighbor);
        }
      }
    }
    if (component.size() < 8 || component.size() > 96) {
      continue;
    }

    const std::unordered_set<int> component_set(component.begin(), component.end());
    std::unordered_map<std::uint64_t, int> loop_counts;
    std::unordered_map<std::uint64_t, std::pair<int, std::size_t>> loop_lookup;
    std::unordered_map<std::uint64_t, std::vector<std::pair<int, int>>> boundary_edges_by_loop;

    for (int face_index : component) {
      const std::vector<int>& loop = reconstruction.faces[face_index].loops.front();
      for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
        const int from = loop[edge_index];
        const int to = loop[(edge_index + 1) % loop.size()];
        const auto found = edge_faces.find(make_edge_key(from, to));
        if (found == edge_faces.end()) {
          continue;
        }
        for (const EdgeRef& ref : found->second) {
          if (component_set.contains(ref.face_index)) {
            continue;
          }
          const std::uint64_t loop_key = make_face_loop_key(ref.face_index, ref.loop_index);
          loop_counts[loop_key] += 1;
          loop_lookup.emplace(loop_key, std::pair<int, std::size_t> {ref.face_index, ref.loop_index});
          boundary_edges_by_loop[loop_key].push_back({from, to});
        }
      }
    }

    if (loop_counts.size() < 2) {
      continue;
    }

    std::vector<std::pair<std::uint64_t, int>> ranked_loops(loop_counts.begin(), loop_counts.end());
    std::sort(ranked_loops.begin(),
              ranked_loops.end(),
              [](const auto& lhs, const auto& rhs) {
                if (lhs.second != rhs.second) {
                  return lhs.second > rhs.second;
                }
                return lhs.first < rhs.first;
              });
    if (ranked_loops[0].second < 6 || ranked_loops[1].second < 6) {
      continue;
    }

    const auto top_loop_ref = loop_lookup[ranked_loops[0].first];
    const auto bottom_loop_ref = loop_lookup[ranked_loops[1].first];
    const ReconstructedFace& top_face = reconstruction.faces[top_loop_ref.first];
    const ReconstructedFace& bottom_face = reconstruction.faces[bottom_loop_ref.first];
    if (angle_degrees_unsigned(top_face.fit.normal, bottom_face.fit.normal) > 8.0) {
      continue;
    }

    Vec3 axis = normalized(top_face.fit.normal);
    if (length_squared(axis) <= 1e-12) {
      continue;
    }
    if (dot(top_face.fit.centroid - bottom_face.fit.centroid, axis) < 0.0) {
      axis = axis * -1.0;
    }
    const PlaneBasis axis_basis = basis_from_normal(axis);

    const auto top_edges = boundary_edges_by_loop[ranked_loops[0].first];
    const auto bottom_edges = boundary_edges_by_loop[ranked_loops[1].first];
    std::vector<int> top_chain = ordered_open_chain_from_directed_edges(top_edges);
    std::vector<int> bottom_chain = ordered_open_chain_from_directed_edges(bottom_edges);
    if (top_chain.size() < 6 || bottom_chain.size() < 6) {
      continue;
    }

    std::vector<Vec2> top_profile = project_chain_to_basis(top_chain, reconstruction.vertices, axis_basis);
    std::vector<Vec2> bottom_profile = project_chain_to_basis(bottom_chain, reconstruction.vertices, axis_basis);
    const std::size_t sample_count =
        std::max<std::size_t>(8, std::min<std::size_t>(18, std::max(top_profile.size(), bottom_profile.size())));
    const std::vector<Vec2> sampled_top = resample_polyline_2d(top_profile, sample_count);
    const std::vector<Vec2> sampled_bottom = resample_polyline_2d(bottom_profile, sample_count);
    std::vector<Vec2> reversed_bottom = sampled_bottom;
    std::reverse(reversed_bottom.begin(), reversed_bottom.end());
    if (rms_profile_delta(sampled_top, reversed_bottom) < rms_profile_delta(sampled_top, sampled_bottom)) {
      std::reverse(bottom_chain.begin(), bottom_chain.end());
      bottom_profile = project_chain_to_basis(bottom_chain, reconstruction.vertices, axis_basis);
    }

    const std::vector<Vec2> normalized_top = resample_polyline_2d(top_profile, sample_count);
    const std::vector<Vec2> normalized_bottom = resample_polyline_2d(bottom_profile, sample_count);
    const double profile_delta = rms_profile_delta(normalized_top, normalized_bottom);
    const double profile_length =
        std::max(polyline_length_2d(normalized_top), polyline_length_2d(normalized_bottom));
    if (!std::isfinite(profile_delta) ||
        profile_delta > std::max(profile_length * 0.04, 0.18)) {
      continue;
    }

    std::vector<GenericSmoothExtrudedBand> bands = subdivide_generic_smooth_band_candidate(
        reconstruction,
        top_loop_ref.first,
        top_loop_ref.second,
        bottom_loop_ref.first,
        bottom_loop_ref.second,
        top_chain,
        bottom_chain,
        axis,
        axis_basis,
        dot(top_face.fit.centroid, axis),
        dot(bottom_face.fit.centroid, axis),
        component);
    candidates.insert(candidates.end(), bands.begin(), bands.end());
  }

  if (candidates.empty()) {
    return std::nullopt;
  }

  std::sort(candidates.begin(),
            candidates.end(),
            [](const GenericSmoothExtrudedBand& lhs, const GenericSmoothExtrudedBand& rhs) {
              if (lhs.side_face_indices.size() != rhs.side_face_indices.size()) {
                return lhs.side_face_indices.size() > rhs.side_face_indices.size();
              }
              if (lhs.top_face_index != rhs.top_face_index) {
                return lhs.top_face_index < rhs.top_face_index;
              }
              return lhs.bottom_face_index < rhs.bottom_face_index;
            });

  std::unordered_set<int> used_side_faces;
  std::unordered_set<std::uint64_t> used_loops;
  GenericSmoothExtrudedBandExport export_result;
  for (const GenericSmoothExtrudedBand& band : candidates) {
    if (used_loops.contains(make_face_loop_key(band.top_face_index, band.top_loop_index)) ||
        used_loops.contains(make_face_loop_key(band.bottom_face_index, band.bottom_loop_index))) {
      continue;
    }

    bool side_conflict = false;
    for (int face_index : band.side_face_indices) {
      if (used_side_faces.contains(face_index)) {
        side_conflict = true;
        break;
      }
    }
    if (side_conflict) {
      continue;
    }

    used_loops.insert(make_face_loop_key(band.top_face_index, band.top_loop_index));
    used_loops.insert(make_face_loop_key(band.bottom_face_index, band.bottom_loop_index));
    used_side_faces.insert(band.side_face_indices.begin(), band.side_face_indices.end());
    export_result.bands.push_back(band);
  }

  if (export_result.bands.empty()) {
    return std::nullopt;
  }
  return export_result;
}
