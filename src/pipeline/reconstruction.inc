bool solve_linear_system_3x3(std::array<std::array<double, 3>, 3> matrix, Vec3 rhs, Vec3& solution) {
  std::array<double, 3> b {rhs.x, rhs.y, rhs.z};

  for (int pivot = 0; pivot < 3; ++pivot) {
    int best = pivot;
    for (int row = pivot + 1; row < 3; ++row) {
      if (std::abs(matrix[row][pivot]) > std::abs(matrix[best][pivot])) {
        best = row;
      }
    }
    if (std::abs(matrix[best][pivot]) <= 1e-12) {
      return false;
    }
    if (best != pivot) {
      std::swap(matrix[best], matrix[pivot]);
      std::swap(b[best], b[pivot]);
    }

    const double diagonal = matrix[pivot][pivot];
    for (int column = pivot; column < 3; ++column) {
      matrix[pivot][column] /= diagonal;
    }
    b[pivot] /= diagonal;

    for (int row = 0; row < 3; ++row) {
      if (row == pivot) {
        continue;
      }
      const double factor = matrix[row][pivot];
      for (int column = pivot; column < 3; ++column) {
        matrix[row][column] -= factor * matrix[pivot][column];
      }
      b[row] -= factor * b[pivot];
    }
  }

  solution = {b[0], b[1], b[2]};
  return true;
}

std::vector<int> collect_unique_vertices(const MeshModel& mesh, const std::vector<int>& triangles) {
  std::vector<char> seen(mesh.vertices.size(), false);
  std::vector<int> vertices;
  for (int triangle_index : triangles) {
    for (int vertex : mesh.triangles[triangle_index].vertices) {
      if (!seen[vertex]) {
        seen[vertex] = true;
        vertices.push_back(vertex);
      }
    }
  }
  return vertices;
}

PlaneFit fit_plane_region(const MeshModel& mesh,
                         const std::vector<int>& triangle_indices,
                         double total_area,
                         const Tolerances& tolerances) {
  const std::vector<int> vertex_indices = collect_unique_vertices(mesh, triangle_indices);
  std::vector<Vec3> points;
  points.reserve(vertex_indices.size());

  Vec3 centroid {};
  for (int vertex_index : vertex_indices) {
    points.push_back(mesh.vertices[vertex_index]);
    centroid += mesh.vertices[vertex_index];
  }

  if (points.empty()) {
    throw std::runtime_error("Attempted to fit plane to empty region");
  }

  centroid = centroid / static_cast<double>(points.size());
  const auto covariance = covariance_matrix(points, centroid);
  const auto [eigenvalues, eigenvectors] = jacobi_eigenvectors(covariance);
  int smallest = 0;
  for (int index = 1; index < 3; ++index) {
    if (eigenvalues[index] < eigenvalues[smallest]) {
      smallest = index;
    }
  }

  Vec3 normal = eigenvectors[smallest];
  Vec3 average_normal {};
  double area = 0.0;
  for (int triangle_index : triangle_indices) {
    average_normal += mesh.triangles[triangle_index].normal * mesh.triangles[triangle_index].area;
    area += mesh.triangles[triangle_index].area;
  }

  if (length_squared(normal) <= 1e-12) {
    normal = normalized(average_normal);
  }
  if (dot(normal, average_normal) < 0.0) {
    normal = normal * -1.0;
  }
  if (length_squared(normal) <= 1e-12) {
    normal = {0.0, 0.0, 1.0};
  }

  PlaneFit fit;
  fit.normal = normalized(normal);
  fit.centroid = centroid;
  fit.d = -dot(fit.normal, centroid);

  double sum_squared_error = 0.0;
  double max_error = 0.0;
  double sum_angle = 0.0;
  double weighted_angle = 0.0;
  double area_weight_sum = 0.0;
  for (const Vec3& point : points) {
    const double distance = std::abs(plane_distance(fit, point));
    sum_squared_error += distance * distance;
    max_error = std::max(max_error, distance);
  }
  for (int triangle_index : triangle_indices) {
    const double angle = angle_degrees_unsigned(mesh.triangles[triangle_index].normal, fit.normal);
    sum_angle += angle;
    weighted_angle += angle * std::max(mesh.triangles[triangle_index].area, 1e-12);
    area_weight_sum += std::max(mesh.triangles[triangle_index].area, 1e-12);
  }

  fit.rms_error = std::sqrt(sum_squared_error / static_cast<double>(points.size()));
  fit.max_error = max_error;

  const double average_angle =
      area_weight_sum > 0.0 ? weighted_angle / area_weight_sum
                            : (triangle_indices.empty() ? 90.0
                                                        : sum_angle / triangle_indices.size());
  const double residual_score = clamp(1.0 - fit.rms_error / (tolerances.plane_distance * 1.5), 0.0, 1.0);
  const double max_error_score =
      clamp(1.0 - fit.max_error / (tolerances.plane_distance * 3.0), 0.0, 1.0);
  const double angle_score =
      clamp(1.0 - average_angle / (tolerances.normal_angle_degrees * 1.25), 0.0, 1.0);
  const double support_score =
      total_area > 0.0 ? clamp((area / total_area) * 8.0, 0.0, 1.0) : 0.0;
  fit.confidence =
      0.45 * residual_score + 0.25 * max_error_score + 0.20 * angle_score + 0.10 * support_score;
  return fit;
}

PlaneRegion build_region(int id,
                        const MeshModel& mesh,
                        const std::vector<int>& triangle_indices,
                        double total_area,
                        const Tolerances& tolerances) {
  PlaneRegion region;
  region.id = id;
  region.triangle_indices = triangle_indices;
  region.vertex_indices = collect_unique_vertices(mesh, triangle_indices);
  region.fit = fit_plane_region(mesh, triangle_indices, total_area, tolerances);
  region.area = 0.0;
  for (int triangle_index : triangle_indices) {
    region.area += mesh.triangles[triangle_index].area;
  }
  region.unresolved = triangle_indices.size() < 2 || region.fit.confidence < tolerances.min_region_confidence;
  return region;
}

bool triangle_is_compatible(const MeshModel& mesh,
                            int triangle_index,
                            const PlaneFit& fit,
                            const Tolerances& tolerances) {
  const Triangle& triangle = mesh.triangles[triangle_index];
  if (triangle.area <= 1e-12) {
    return false;
  }
  if (angle_degrees_unsigned(triangle.normal, fit.normal) > tolerances.normal_angle_degrees) {
    return false;
  }
  for (int vertex_index : triangle.vertices) {
    if (std::abs(plane_distance(fit, mesh.vertices[vertex_index])) >
        tolerances.plane_distance * 1.5) {
      return false;
    }
  }
  return true;
}

std::vector<PlaneRegion> segment_planar_regions(MeshModel& mesh, const Tolerances& tolerances) {
  const AdjacencyData adjacency = build_adjacency(mesh);
  const double total_area = std::accumulate(
      mesh.triangles.begin(), mesh.triangles.end(), 0.0,
      [](double sum, const Triangle& triangle) { return sum + triangle.area; });

  std::vector<int> order(mesh.triangles.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(order.begin(), order.end(), [&mesh](int lhs, int rhs) {
    return mesh.triangles[lhs].area > mesh.triangles[rhs].area;
  });

  std::vector<int> triangle_to_region(mesh.triangles.size(), -1);
  std::vector<PlaneRegion> regions;

  for (int seed : order) {
    if (triangle_to_region[seed] != -1) {
      continue;
    }
    std::vector<int> triangles {seed};
    std::queue<int> frontier;
    frontier.push(seed);
    triangle_to_region[seed] = static_cast<int>(regions.size());
    PlaneRegion working = build_region(static_cast<int>(regions.size()), mesh, triangles, total_area, tolerances);

    while (!frontier.empty()) {
      const int current = frontier.front();
      frontier.pop();
      for (int neighbor : adjacency.triangle_graph[current]) {
        if (triangle_to_region[neighbor] != -1) {
          continue;
        }
        if (!triangle_is_compatible(mesh, neighbor, working.fit, tolerances)) {
          continue;
        }
        std::vector<int> candidate_triangles = triangles;
        candidate_triangles.push_back(neighbor);
        PlaneRegion candidate =
            build_region(static_cast<int>(regions.size()), mesh, candidate_triangles, total_area, tolerances);
        if (candidate.fit.rms_error > tolerances.plane_distance * 1.5 ||
            angle_degrees_unsigned(mesh.triangles[neighbor].normal, candidate.fit.normal) >
                tolerances.normal_angle_degrees * 1.25) {
          continue;
        }
        triangles = std::move(candidate_triangles);
        working = std::move(candidate);
        triangle_to_region[neighbor] = static_cast<int>(regions.size());
        frontier.push(neighbor);
      }
    }

    regions.push_back(working);
  }

  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    mesh.triangles[triangle_index].region_id = triangle_to_region[triangle_index];
  }

  std::vector<std::set<int>> neighbor_sets(regions.size());
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const int region_id = mesh.triangles[triangle_index].region_id;
    for (int neighbor : adjacency.triangle_graph[triangle_index]) {
      const int neighbor_region = mesh.triangles[neighbor].region_id;
      if (neighbor_region >= 0 && neighbor_region != region_id) {
        neighbor_sets[region_id].insert(neighbor_region);
      }
    }
  }
  for (std::size_t index = 0; index < regions.size(); ++index) {
    regions[index].neighbor_region_ids.assign(neighbor_sets[index].begin(), neighbor_sets[index].end());
  }
  return regions;
}

double coplanar_offset(const PlaneFit& lhs, const PlaneFit& rhs) {
  const double sign = dot(lhs.normal, rhs.normal) >= 0.0 ? 1.0 : -1.0;
  return std::abs(lhs.d - rhs.d * sign);
}

bool snap_parallel(PlaneRegion& anchor, PlaneRegion& moving) {
  Vec3 target = anchor.fit.normal;
  if (dot(target, moving.fit.normal) < 0.0) {
    target = target * -1.0;
  }
  if (length_squared(target) <= 1e-12) {
    return false;
  }
  moving.fit.normal = normalized(target);
  moving.fit.d = -dot(moving.fit.normal, moving.fit.centroid);
  return true;
}

bool snap_perpendicular(PlaneRegion& anchor, PlaneRegion& moving) {
  Vec3 candidate = moving.fit.normal - anchor.fit.normal * dot(moving.fit.normal, anchor.fit.normal);
  if (length_squared(candidate) <= 1e-12) {
    return false;
  }
  candidate = normalized(candidate);
  if (dot(candidate, moving.fit.normal) < 0.0) {
    candidate = candidate * -1.0;
  }
  moving.fit.normal = candidate;
  moving.fit.d = -dot(moving.fit.normal, moving.fit.centroid);
  return true;
}

std::vector<std::pair<int, int>> collect_region_pairs(const MeshModel& mesh) {
  const AdjacencyData adjacency = build_adjacency(mesh);
  std::set<std::pair<int, int>> pairs;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const int region_id = mesh.triangles[triangle_index].region_id;
    for (int neighbor : adjacency.triangle_graph[triangle_index]) {
      const int neighbor_region = mesh.triangles[neighbor].region_id;
      if (region_id >= 0 && neighbor_region >= 0 && region_id != neighbor_region) {
        pairs.insert({std::min(region_id, neighbor_region), std::max(region_id, neighbor_region)});
      }
    }
  }
  return {pairs.begin(), pairs.end()};
}

std::pair<std::vector<PlaneRegion>, ConstraintGraph> apply_constraints(
    MeshModel& mesh,
    const std::vector<PlaneRegion>& segmented_regions,
    const Tolerances& tolerances) {
  ConstraintGraph graph;
  UnionFind merge_groups(segmented_regions.size());

  for (const auto& [a, b] : collect_region_pairs(mesh)) {
    const PlaneRegion& lhs = segmented_regions[a];
    const PlaneRegion& rhs = segmented_regions[b];
    const double angle = angle_degrees_unsigned(lhs.fit.normal, rhs.fit.normal);
    const double offset = coplanar_offset(lhs.fit, rhs.fit);
    if (angle <= tolerances.merge_angle_degrees && offset <= tolerances.plane_merge_distance) {
      merge_groups.unite(a, b);
      const double score =
          clamp((lhs.fit.confidence + rhs.fit.confidence) * 0.5, 0.0, 1.0);
      graph.constraints.push_back(
          {a, b, ConstraintType::CoplanarMerge, true, score,
           "Merged adjacent regions after coplanar snap within angle and offset tolerances"});
    }
  }

  const double total_area = std::accumulate(
      mesh.triangles.begin(), mesh.triangles.end(), 0.0,
      [](double sum, const Triangle& triangle) { return sum + triangle.area; });

  std::unordered_map<int, std::vector<int>> root_to_triangles;
  for (const PlaneRegion& region : segmented_regions) {
    const int root = merge_groups.find(region.id);
    auto& triangles = root_to_triangles[root];
    triangles.insert(triangles.end(), region.triangle_indices.begin(), region.triangle_indices.end());
  }

  std::vector<PlaneRegion> merged_regions;
  std::vector<int> sorted_roots;
  sorted_roots.reserve(root_to_triangles.size());
  for (const auto& entry : root_to_triangles) {
    sorted_roots.push_back(entry.first);
  }
  std::sort(sorted_roots.begin(), sorted_roots.end());

  for (int root : sorted_roots) {
    auto& triangles = root_to_triangles[root];
    std::sort(triangles.begin(), triangles.end());
    triangles.erase(std::unique(triangles.begin(), triangles.end()), triangles.end());
    const int new_id = static_cast<int>(merged_regions.size());
    PlaneRegion region = build_region(new_id, mesh, triangles, total_area, tolerances);
    region.unresolved = region.fit.confidence < tolerances.min_region_confidence;
    merged_regions.push_back(std::move(region));
  }

  for (PlaneRegion& region : merged_regions) {
    for (int triangle_index : region.triangle_indices) {
      mesh.triangles[triangle_index].region_id = region.id;
    }
  }

  for (const auto& [a, b] : collect_region_pairs(mesh)) {
    PlaneRegion& lhs = merged_regions[a];
    PlaneRegion& rhs = merged_regions[b];
    const double angle = angle_degrees_unsigned(lhs.fit.normal, rhs.fit.normal);
    const double offset = coplanar_offset(lhs.fit, rhs.fit);

    if (angle <= tolerances.parallel_angle_degrees && offset > tolerances.plane_merge_distance) {
      PlaneRegion* anchor = lhs.fit.confidence >= rhs.fit.confidence ? &lhs : &rhs;
      PlaneRegion* moving = anchor == &lhs ? &rhs : &lhs;
      const bool applied = std::min(lhs.fit.confidence, rhs.fit.confidence) >= 0.55 &&
                           snap_parallel(*anchor, *moving);
      graph.constraints.push_back(
          {lhs.id, rhs.id, ConstraintType::Parallel, applied,
           clamp((lhs.fit.confidence + rhs.fit.confidence) * 0.5, 0.0, 1.0),
           applied ? "Snapped lower-confidence plane normal to parallel anchor"
                   : "Detected near-parallel planes but confidence was too low to enforce"});
    } else if (std::abs(90.0 - angle) <= tolerances.perpendicular_angle_degrees) {
      PlaneRegion* anchor = lhs.fit.confidence >= rhs.fit.confidence ? &lhs : &rhs;
      PlaneRegion* moving = anchor == &lhs ? &rhs : &lhs;
      const bool applied = std::min(lhs.fit.confidence, rhs.fit.confidence) >= 0.55 &&
                           snap_perpendicular(*anchor, *moving);
      graph.constraints.push_back(
          {lhs.id, rhs.id, ConstraintType::Perpendicular, applied,
           clamp((lhs.fit.confidence + rhs.fit.confidence) * 0.5, 0.0, 1.0),
           applied ? "Snapped lower-confidence plane normal to perpendicular anchor"
                   : "Detected near-perpendicular planes but confidence was too low to enforce"});
    }
  }

  std::vector<std::set<int>> neighbor_sets(merged_regions.size());
  for (const auto& [a, b] : collect_region_pairs(mesh)) {
    neighbor_sets[a].insert(b);
    neighbor_sets[b].insert(a);
  }
  for (std::size_t index = 0; index < merged_regions.size(); ++index) {
    merged_regions[index].neighbor_region_ids.assign(neighbor_sets[index].begin(),
                                                     neighbor_sets[index].end());
  }

  std::sort(merged_regions.begin(), merged_regions.end(), [](const PlaneRegion& lhs, const PlaneRegion& rhs) {
    return lhs.id < rhs.id;
  });
  return {merged_regions, graph};
}

Vec3 snap_vertex_to_incident_planes(const Vec3& original,
                                    const std::vector<int>& incident_regions,
                                    const std::vector<PlaneRegion>& regions,
                                    const Tolerances& tolerances) {
  struct CandidatePlane {
    int region_id {};
    double distance {};
  };

  std::vector<CandidatePlane> candidates;
  candidates.reserve(incident_regions.size());
  for (int region_id : incident_regions) {
    candidates.push_back({region_id, std::abs(plane_distance(regions[region_id].fit, original))});
  }
  std::sort(candidates.begin(), candidates.end(), [&regions](const CandidatePlane& lhs,
                                                             const CandidatePlane& rhs) {
    if (std::abs(lhs.distance - rhs.distance) > 1e-9) {
      return lhs.distance < rhs.distance;
    }
    if (std::abs(regions[lhs.region_id].fit.confidence - regions[rhs.region_id].fit.confidence) > 1e-9) {
      return regions[lhs.region_id].fit.confidence > regions[rhs.region_id].fit.confidence;
    }
    if (std::abs(regions[lhs.region_id].area - regions[rhs.region_id].area) > 1e-9) {
      return regions[lhs.region_id].area > regions[rhs.region_id].area;
    }
    return lhs.region_id < rhs.region_id;
  });

  const double distinct_angle = std::max(tolerances.parallel_angle_degrees * 2.0, 5.0);
  std::vector<int> selected;
  selected.reserve(3);
  for (const CandidatePlane& candidate : candidates) {
    bool distinct = true;
    for (int selected_region : selected) {
      if (angle_degrees_unsigned(regions[selected_region].fit.normal, regions[candidate.region_id].fit.normal) <
          distinct_angle) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      selected.push_back(candidate.region_id);
      if (selected.size() == 3) {
        break;
      }
    }
  }

  if (selected.empty() && !candidates.empty()) {
    selected.push_back(candidates.front().region_id);
  }

  Vec3 snapped = original;
  if (selected.size() == 1) {
    const PlaneFit& fit = regions[selected.front()].fit;
    snapped = original - fit.normal * plane_distance(fit, original);
  } else {
    bool solved_exact = false;
    if (selected.size() >= 3) {
      const PlaneFit& a = regions[selected[0]].fit;
      const PlaneFit& b = regions[selected[1]].fit;
      const PlaneFit& c = regions[selected[2]].fit;
      std::array<std::array<double, 3>, 3> exact_matrix {{
          {a.normal.x, a.normal.y, a.normal.z},
          {b.normal.x, b.normal.y, b.normal.z},
          {c.normal.x, c.normal.y, c.normal.z},
      }};
      Vec3 exact_rhs {-a.d, -b.d, -c.d};
      solved_exact = solve_linear_system_3x3(exact_matrix, exact_rhs, snapped);
    }

    if (!solved_exact) {
      std::array<std::array<double, 3>, 3> matrix {{
          {1.0, 0.0, 0.0},
          {0.0, 1.0, 0.0},
          {0.0, 0.0, 1.0},
      }};
      Vec3 rhs = original;
      for (int region_id : selected) {
        const Vec3& n = regions[region_id].fit.normal;
        matrix[0][0] += n.x * n.x;
        matrix[0][1] += n.x * n.y;
        matrix[0][2] += n.x * n.z;
        matrix[1][0] += n.y * n.x;
        matrix[1][1] += n.y * n.y;
        matrix[1][2] += n.y * n.z;
        matrix[2][0] += n.z * n.x;
        matrix[2][1] += n.z * n.y;
        matrix[2][2] += n.z * n.z;
        rhs = rhs - n * regions[region_id].fit.d;
      }

      if (!solve_linear_system_3x3(matrix, rhs, snapped)) {
        return original;
      }
    }
  }

  const double max_snap_distance =
      std::max(std::max(tolerances.plane_merge_distance, tolerances.vertex_weld * 4.0), 1e-6);
  if (length(snapped - original) > max_snap_distance) {
    return original;
  }
  return snapped;
}

std::vector<int> simplify_loop(const std::vector<int>& loop,
                               const std::vector<Vec3>& vertices,
                               const Vec3& normal,
                               double tolerance) {
  if (loop.size() < 3) {
    return {};
  }

  std::vector<int> simplified = loop;
  bool changed = true;
  const PlaneBasis basis = basis_from_normal(normal);

  while (changed && simplified.size() >= 3) {
    changed = false;
    for (std::size_t index = 0; index < simplified.size(); ++index) {
      const int previous = simplified[(index + simplified.size() - 1) % simplified.size()];
      const int current = simplified[index];
      const int next = simplified[(index + 1) % simplified.size()];
      if (previous == current || current == next || previous == next) {
        simplified.erase(simplified.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
      const Vec2 a = project_to_basis(vertices[previous], basis);
      const Vec2 b = project_to_basis(vertices[current], basis);
      const Vec2 c = project_to_basis(vertices[next], basis);
      const double previous_length = std::hypot(b.x - a.x, b.y - a.y);
      const double next_length = std::hypot(c.x - b.x, c.y - b.y);
      const double local_tolerance =
          std::min(tolerance, std::min(previous_length, next_length) * 0.2);
      const Vec2 ab {a.x - b.x, a.y - b.y};
      const Vec2 cb {c.x - b.x, c.y - b.y};
      const double straightness =
          (ab.x * cb.x + ab.y * cb.y) / std::max(previous_length * next_length, 1e-12);
      if (local_tolerance > 1e-9 &&
          point_line_distance_2d(b, a, c) <= local_tolerance &&
          straightness <= -0.995) {
        simplified.erase(simplified.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
    }
  }

  return simplified.size() >= 3 ? simplified : std::vector<int> {};
}

std::vector<int> fallback_convex_hull_loop(const PlaneRegion& region,
                                           const std::vector<int>& original_to_snapped,
                                           const std::vector<Vec3>& vertices,
                                           const Vec3& normal,
                                           double tolerance) {
  struct ProjectedVertex {
    Vec2 point;
    int vertex_id {};
  };

  std::vector<ProjectedVertex> projected_vertices;
  projected_vertices.reserve(region.vertex_indices.size());
  std::set<int> unique_vertices;
  const PlaneBasis basis = basis_from_normal(normal);

  for (int original_vertex : region.vertex_indices) {
    const int snapped_vertex = original_to_snapped[original_vertex];
    if (snapped_vertex < 0 || !unique_vertices.insert(snapped_vertex).second) {
      continue;
    }
    projected_vertices.push_back({project_to_basis(vertices[snapped_vertex], basis), snapped_vertex});
  }

  if (projected_vertices.size() < 3) {
    return {};
  }

  std::sort(projected_vertices.begin(), projected_vertices.end(), [](const ProjectedVertex& lhs,
                                                                     const ProjectedVertex& rhs) {
    if (std::abs(lhs.point.x - rhs.point.x) > 1e-9) {
      return lhs.point.x < rhs.point.x;
    }
    if (std::abs(lhs.point.y - rhs.point.y) > 1e-9) {
      return lhs.point.y < rhs.point.y;
    }
    return lhs.vertex_id < rhs.vertex_id;
  });

  projected_vertices.erase(
      std::unique(projected_vertices.begin(), projected_vertices.end(),
                  [](const ProjectedVertex& lhs, const ProjectedVertex& rhs) {
                    return std::abs(lhs.point.x - rhs.point.x) <= 1e-9 &&
                           std::abs(lhs.point.y - rhs.point.y) <= 1e-9;
                  }),
      projected_vertices.end());

  if (projected_vertices.size() < 3) {
    return {};
  }

  std::vector<ProjectedVertex> hull;
  hull.reserve(projected_vertices.size() * 2);
  for (const ProjectedVertex& vertex : projected_vertices) {
    while (hull.size() >= 2 &&
           cross_2d(hull[hull.size() - 2].point, hull.back().point, vertex.point) <= 1e-9) {
      hull.pop_back();
    }
    hull.push_back(vertex);
  }

  const std::size_t lower_size = hull.size();
  for (std::size_t index = projected_vertices.size(); index-- > 0;) {
    const ProjectedVertex& vertex = projected_vertices[index];
    while (hull.size() > lower_size &&
           cross_2d(hull[hull.size() - 2].point, hull.back().point, vertex.point) <= 1e-9) {
      hull.pop_back();
    }
    hull.push_back(vertex);
  }

  if (!hull.empty()) {
    hull.pop_back();
  }
  if (hull.size() < 3) {
    return {};
  }

  std::vector<int> loop;
  loop.reserve(hull.size());
  for (const ProjectedVertex& vertex : hull) {
    loop.push_back(vertex.vertex_id);
  }
  return simplify_loop(loop, vertices, normal, tolerance);
}

void add_or_cancel_oriented_edge(
    std::unordered_map<DirectedEdgeKey, int, DirectedEdgeKeyHash>& oriented_edges,
    int from,
    int to) {
  if (from == to) {
    return;
  }

  const DirectedEdgeKey reverse {to, from};
  const auto reverse_it = oriented_edges.find(reverse);
  if (reverse_it != oriented_edges.end()) {
    if (--reverse_it->second <= 0) {
      oriented_edges.erase(reverse_it);
    }
    return;
  }

  ++oriented_edges[DirectedEdgeKey {from, to}];
}

std::vector<std::pair<int, int>> build_region_boundary_edges_from_triangles(
    const PlaneRegion& region,
    const MeshModel& mesh,
    const std::vector<int>& original_to_snapped) {
  std::unordered_map<DirectedEdgeKey, int, DirectedEdgeKeyHash> oriented_edges;

  for (int triangle_index : region.triangle_indices) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    std::array<int, 3> snapped_vertices {
        original_to_snapped[triangle.vertices[0]],
        original_to_snapped[triangle.vertices[1]],
        original_to_snapped[triangle.vertices[2]],
    };
    if (snapped_vertices[0] < 0 || snapped_vertices[1] < 0 || snapped_vertices[2] < 0) {
      continue;
    }
    if (snapped_vertices[0] == snapped_vertices[1] || snapped_vertices[1] == snapped_vertices[2] ||
        snapped_vertices[0] == snapped_vertices[2]) {
      continue;
    }
    if (dot(triangle.normal, region.fit.normal) < 0.0) {
      std::swap(snapped_vertices[1], snapped_vertices[2]);
    }

    add_or_cancel_oriented_edge(oriented_edges, snapped_vertices[0], snapped_vertices[1]);
    add_or_cancel_oriented_edge(oriented_edges, snapped_vertices[1], snapped_vertices[2]);
    add_or_cancel_oriented_edge(oriented_edges, snapped_vertices[2], snapped_vertices[0]);
  }

  std::vector<std::pair<int, int>> edges;
  edges.reserve(oriented_edges.size());
  for (const auto& [edge, count] : oriented_edges) {
    if (count > 0) {
      edges.push_back({edge.from, edge.to});
    }
  }
  std::sort(edges.begin(), edges.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.first != rhs.first) {
      return lhs.first < rhs.first;
    }
    return lhs.second < rhs.second;
  });
  edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
  return edges;
}

int choose_next_edge(const std::vector<std::pair<int, int>>& edges,
                     const std::vector<bool>& used,
                     const std::vector<int>& candidates,
                     int previous,
                     int current,
                     const std::vector<Vec3>& vertices,
                     const PlaneBasis& basis) {
  if (candidates.empty()) {
    return -1;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  const Vec2 previous_point = project_to_basis(vertices[previous], basis);
  const Vec2 current_point = project_to_basis(vertices[current], basis);
  const Vec2 previous_direction {current_point.x - previous_point.x, current_point.y - previous_point.y};
  const double previous_length = std::hypot(previous_direction.x, previous_direction.y);
  if (previous_length <= 1e-12) {
    return candidates.front();
  }
  const Vec2 normalized_previous {previous_direction.x / previous_length,
                                  previous_direction.y / previous_length};

  int best_index = candidates.front();
  double best_turn = std::numeric_limits<double>::max();
  for (int candidate : candidates) {
    if (used[candidate]) {
      continue;
    }
    const Vec2 next_point = project_to_basis(vertices[edges[candidate].second], basis);
    Vec2 direction {next_point.x - current_point.x, next_point.y - current_point.y};
    const double direction_length = std::hypot(direction.x, direction.y);
    if (direction_length <= 1e-12) {
      continue;
    }
    direction.x /= direction_length;
    direction.y /= direction_length;

    double turn = std::atan2(normalized_previous.x * direction.y - normalized_previous.y * direction.x,
                             normalized_previous.x * direction.x + normalized_previous.y * direction.y);
    if (turn <= 1e-9) {
      turn += 2.0 * kPi;
    }
    if (turn < best_turn) {
      best_turn = turn;
      best_index = candidate;
    }
  }
  return best_index;
}

int choose_next_neighbor(const std::vector<int>& candidates,
                         int previous,
                         int current,
                         const std::vector<Vec3>& vertices,
                         const PlaneBasis& basis) {
  if (candidates.empty()) {
    return -1;
  }
  if (candidates.size() == 1) {
    return candidates.front();
  }

  const Vec2 previous_point = project_to_basis(vertices[previous], basis);
  const Vec2 current_point = project_to_basis(vertices[current], basis);
  const Vec2 previous_direction {current_point.x - previous_point.x, current_point.y - previous_point.y};
  const double previous_length = std::hypot(previous_direction.x, previous_direction.y);
  if (previous_length <= 1e-12) {
    return candidates.front();
  }
  const Vec2 normalized_previous {previous_direction.x / previous_length,
                                  previous_direction.y / previous_length};

  int best_vertex = candidates.front();
  double best_turn = std::numeric_limits<double>::max();
  for (int candidate : candidates) {
    const Vec2 next_point = project_to_basis(vertices[candidate], basis);
    Vec2 direction {next_point.x - current_point.x, next_point.y - current_point.y};
    const double direction_length = std::hypot(direction.x, direction.y);
    if (direction_length <= 1e-12) {
      continue;
    }
    direction.x /= direction_length;
    direction.y /= direction_length;

    double turn = std::atan2(normalized_previous.x * direction.y - normalized_previous.y * direction.x,
                             normalized_previous.x * direction.x + normalized_previous.y * direction.y);
    if (turn <= 1e-9) {
      turn += 2.0 * kPi;
    }
    if (turn < best_turn) {
      best_turn = turn;
      best_vertex = candidate;
    }
  }
  return best_vertex;
}

std::size_t total_loop_edges(const std::vector<std::vector<int>>& loops) {
  std::size_t total = 0;
  for (const auto& loop : loops) {
    total += loop.size();
  }
  return total;
}

double polygon_area(const std::vector<int>& loop, const std::vector<Vec3>& vertices, const Vec3& normal);

double loops_face_area_estimate(const std::vector<std::vector<int>>& loops,
                                const std::vector<Vec3>& vertices,
                                const Vec3& normal) {
  if (loops.empty()) {
    return 0.0;
  }

  std::vector<double> areas;
  areas.reserve(loops.size());
  for (const auto& loop : loops) {
    if (loop.size() >= 3) {
      areas.push_back(polygon_area(loop, vertices, normal));
    }
  }
  if (areas.empty()) {
    return 0.0;
  }

  std::sort(areas.begin(), areas.end(), std::greater<double>());
  double total = areas.front();
  for (std::size_t index = 1; index < areas.size(); ++index) {
    total -= areas[index];
  }
  return std::max(total, 0.0);
}

struct LoopCandidate {
  std::vector<std::vector<int>> loops;
  std::size_t edge_coverage {};
  double area {};
  double area_error {std::numeric_limits<double>::infinity()};
  bool plausible {};
};

LoopCandidate score_loop_candidate(std::vector<std::vector<int>> loops,
                                   const std::vector<Vec3>& vertices,
                                   const Vec3& normal,
                                   double expected_area) {
  LoopCandidate candidate;
  candidate.edge_coverage = total_loop_edges(loops);
  candidate.area = loops_face_area_estimate(loops, vertices, normal);
  if (expected_area > 1e-9) {
    candidate.area_error = std::abs(candidate.area - expected_area);
    const double area_ratio = candidate.area / expected_area;
    candidate.plausible = area_ratio >= 0.45 && area_ratio <= 1.35;
  } else {
    candidate.area_error = candidate.area;
    candidate.plausible = candidate.area > 0.0;
  }
  candidate.loops = std::move(loops);
  return candidate;
}

bool is_better_loop_candidate(const LoopCandidate& candidate, const LoopCandidate& best) {
  if (candidate.loops.empty()) {
    return false;
  }
  if (best.loops.empty()) {
    return true;
  }
  if (candidate.plausible != best.plausible) {
    return candidate.plausible;
  }
  if (candidate.edge_coverage != best.edge_coverage) {
    return candidate.edge_coverage > best.edge_coverage;
  }
  if (std::abs(candidate.area_error - best.area_error) > 1e-9) {
    return candidate.area_error < best.area_error;
  }
  return candidate.loops.size() < best.loops.size();
}

std::vector<std::vector<int>> extract_loops_for_region(const std::vector<std::pair<int, int>>& edges,
                                                       const std::vector<Vec3>& vertices,
                                                       const Vec3& normal,
                                                       double tolerance) {
  std::vector<std::vector<int>> loops;
  if (edges.empty()) {
    return loops;
  }

  std::unordered_map<int, std::vector<int>> outgoing;
  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    outgoing[edges[edge_index].first].push_back(static_cast<int>(edge_index));
  }

  std::vector<bool> used(edges.size(), false);
  const PlaneBasis basis = basis_from_normal(normal);

  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    if (used[edge_index]) {
      continue;
    }
    std::vector<int> loop;
    int start = edges[edge_index].first;
    int previous = start;
    int current = edges[edge_index].second;
    used[edge_index] = true;
    loop.push_back(start);

    int safety = 0;
    while (true) {
      if (++safety > static_cast<int>(edges.size()) + 4) {
        loop.clear();
        break;
      }
      loop.push_back(current);
      if (current == start) {
        loop.pop_back();
        break;
      }
      const auto candidates_it = outgoing.find(current);
      if (candidates_it == outgoing.end()) {
        loop.clear();
        break;
      }
      const int next_edge =
          choose_next_edge(edges, used, candidates_it->second, previous, current, vertices, basis);
      if (next_edge < 0 || used[next_edge]) {
        loop.clear();
        break;
      }
      used[next_edge] = true;
      previous = current;
      current = edges[next_edge].second;
    }

    if (loop.size() >= 3) {
      std::vector<int> simplified = simplify_loop(loop, vertices, normal, tolerance);
      if (simplified.size() >= 3) {
        loops.push_back(std::move(simplified));
      }
    }
  }

  return loops;
}

std::vector<std::vector<int>> extract_loops_for_region_undirected(
    const std::vector<std::pair<int, int>>& edges,
    const std::vector<Vec3>& vertices,
    const Vec3& normal,
    double tolerance) {
  std::vector<std::vector<int>> loops;
  if (edges.empty()) {
    return loops;
  }

  std::unordered_map<int, std::vector<int>> adjacency;
  std::vector<EdgeKey> edge_keys;
  edge_keys.reserve(edges.size());
  for (const auto& [from, to] : edges) {
    adjacency[from].push_back(to);
    adjacency[to].push_back(from);
    edge_keys.push_back(make_edge_key(from, to));
  }

  std::vector<bool> used(edge_keys.size(), false);
  const PlaneBasis basis = basis_from_normal(normal);

  auto try_walk = [&](int start_index, bool reverse_start) -> std::vector<int> {
    std::vector<bool> local_used = used;
    const int start = reverse_start ? edges[start_index].second : edges[start_index].first;
    int previous = start;
    int current = reverse_start ? edges[start_index].first : edges[start_index].second;
    local_used[start_index] = true;

    std::vector<int> loop;
    loop.push_back(start);
    int safety = 0;
    while (true) {
      if (++safety > static_cast<int>(edges.size()) + 4) {
        return {};
      }
      loop.push_back(current);
      if (current == start) {
        loop.pop_back();
        used = std::move(local_used);
        return loop;
      }

      const auto adjacency_it = adjacency.find(current);
      if (adjacency_it == adjacency.end()) {
        return {};
      }

      std::vector<int> candidates;
      for (int candidate : adjacency_it->second) {
        if (candidate == current) {
          continue;
        }
        const EdgeKey key = make_edge_key(current, candidate);
        bool already_used = false;
        for (std::size_t edge_index = 0; edge_index < edge_keys.size(); ++edge_index) {
          if (edge_keys[edge_index] == key && local_used[edge_index]) {
            already_used = true;
            break;
          }
        }
        if (!already_used) {
          candidates.push_back(candidate);
        }
      }

      const int next = choose_next_neighbor(candidates, previous, current, vertices, basis);
      if (next < 0) {
        return {};
      }

      bool marked = false;
      const EdgeKey key = make_edge_key(current, next);
      for (std::size_t edge_index = 0; edge_index < edge_keys.size(); ++edge_index) {
        if (edge_keys[edge_index] == key && !local_used[edge_index]) {
          local_used[edge_index] = true;
          marked = true;
          break;
        }
      }
      if (!marked) {
        return {};
      }

      previous = current;
      current = next;
    }
  };

  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    if (used[edge_index]) {
      continue;
    }

    std::vector<int> loop = try_walk(static_cast<int>(edge_index), false);
    if (loop.size() < 3) {
      loop = try_walk(static_cast<int>(edge_index), true);
    }
    if (loop.size() < 3) {
      continue;
    }

    std::vector<int> simplified = simplify_loop(loop, vertices, normal, tolerance);
    if (simplified.size() >= 3) {
      loops.push_back(std::move(simplified));
    }
  }

  return loops;
}

double polygon_area(const std::vector<int>& loop, const std::vector<Vec3>& vertices, const Vec3& normal) {
  const PlaneBasis basis = basis_from_normal(normal);
  std::vector<Vec2> projected;
  projected.reserve(loop.size());
  for (int vertex : loop) {
    projected.push_back(project_to_basis(vertices[vertex], basis));
  }
  return std::abs(signed_area_2d(projected));
}

double signed_polygon_area(const std::vector<int>& loop,
                           const std::vector<Vec3>& vertices,
                           const Vec3& normal) {
  const PlaneBasis basis = basis_from_normal(normal);
  std::vector<Vec2> projected;
  projected.reserve(loop.size());
  for (int vertex : loop) {
    projected.push_back(project_to_basis(vertices[vertex], basis));
  }
  return signed_area_2d(projected);
}

bool point_in_polygon_2d(const Vec2& point, const std::vector<Vec2>& polygon) {
  bool inside = false;
  for (std::size_t index = 0, previous = polygon.size() - 1; index < polygon.size(); previous = index++) {
    const Vec2& a = polygon[index];
    const Vec2& b = polygon[previous];
    const bool crosses = ((a.y > point.y) != (b.y > point.y)) &&
                         (point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) + 1e-12) + a.x);
    if (crosses) {
      inside = !inside;
    }
  }
  return inside;
}

void orient_loop_to_sign(std::vector<int>& loop,
                         const std::vector<Vec3>& vertices,
                         const Vec3& normal,
                         double target_sign) {
  if (loop.size() < 3) {
    return;
  }
  const double area = signed_polygon_area(loop, vertices, normal);
  if ((area < 0.0 && target_sign > 0.0) || (area > 0.0 && target_sign < 0.0)) {
    std::reverse(loop.begin(), loop.end());
  }
}

double face_total_area(const ReconstructedFace& face,
                       const std::vector<Vec3>& vertices,
                       const Vec3& normal) {
  if (face.loops.empty()) {
    return 0.0;
  }
  double total = polygon_area(face.loops.front(), vertices, normal);
  for (std::size_t index = 1; index < face.loops.size(); ++index) {
    total -= polygon_area(face.loops[index], vertices, normal);
  }
  return std::max(total, 0.0);
}

void suppress_duplicate_coplanar_faces(std::vector<ReconstructedFace>& faces,
                                       const Tolerances& tolerances) {
  std::vector<bool> keep(faces.size(), true);

  for (std::size_t lhs_index = 0; lhs_index < faces.size(); ++lhs_index) {
    if (!keep[lhs_index]) {
      continue;
    }
    for (std::size_t rhs_index = lhs_index + 1; rhs_index < faces.size(); ++rhs_index) {
      if (!keep[rhs_index]) {
        continue;
      }

      const ReconstructedFace& lhs = faces[lhs_index];
      const ReconstructedFace& rhs = faces[rhs_index];
      const double angle = angle_degrees_unsigned(lhs.fit.normal, rhs.fit.normal);
      const double offset = coplanar_offset(lhs.fit, rhs.fit);
      const double centroid_distance = length(lhs.fit.centroid - rhs.fit.centroid);
      const double area_scale = std::max(std::max(lhs.area, rhs.area), 1e-9);
      const double area_ratio = std::min(lhs.area, rhs.area) / area_scale;

      if (angle > 0.5 || offset > tolerances.plane_merge_distance * 0.25 ||
          centroid_distance > tolerances.vertex_weld * 2.0 || area_ratio < 0.95) {
        continue;
      }

      const bool keep_lhs = lhs.confidence > rhs.confidence + 1e-9 ||
                            (std::abs(lhs.confidence - rhs.confidence) <= 1e-9 &&
                             lhs.region_id <= rhs.region_id);
      if (keep_lhs) {
        keep[rhs_index] = false;
      } else {
        keep[lhs_index] = false;
        break;
      }
    }
  }

  std::vector<ReconstructedFace> filtered;
  filtered.reserve(faces.size());
  for (std::size_t index = 0; index < faces.size(); ++index) {
    if (keep[index]) {
      filtered.push_back(std::move(faces[index]));
    }
  }
  faces = std::move(filtered);
}

std::size_t face_edge_count(const ReconstructedFace& face) {
  std::size_t count = 0;
  for (const auto& loop : face.loops) {
    count += loop.size();
  }
  return count;
}

void prune_dangling_faces(std::vector<ReconstructedFace>& faces, const Tolerances& tolerances) {
  for (int iteration = 0; iteration < 4; ++iteration) {
    std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
    std::vector<std::vector<EdgeKey>> face_edges(faces.size());
    for (std::size_t face_index = 0; face_index < faces.size(); ++face_index) {
      for (const auto& loop : faces[face_index].loops) {
        for (std::size_t edge_index = 0; edge_index < loop.size(); ++edge_index) {
          const EdgeKey key = make_edge_key(loop[edge_index], loop[(edge_index + 1) % loop.size()]);
          face_edges[face_index].push_back(key);
          ++edge_use_count[key];
        }
      }
    }

    const auto mismatch_score = [&]() -> std::size_t {
      std::size_t score = 0;
      for (const auto& [edge, count] : edge_use_count) {
        (void)edge;
        if (count == 1 || count > 2) {
          ++score;
        }
      }
      return score;
    };

    const std::size_t baseline_score = mismatch_score();
    std::optional<std::size_t> best_face_index;
    std::size_t best_score = baseline_score;

    for (std::size_t face_index = 0; face_index < faces.size(); ++face_index) {
      const std::size_t total_edges = face_edge_count(faces[face_index]);
      if (total_edges == 0) {
        continue;
      }

      std::size_t unmatched_edges = 0;
      for (const EdgeKey& edge : face_edges[face_index]) {
        const int count = edge_use_count[edge];
        if (count == 1 || count > 2) {
          ++unmatched_edges;
        }
      }

      if (unmatched_edges + 1 < total_edges) {
        continue;
      }
      if (faces[face_index].area > tolerances.plane_distance * tolerances.plane_distance * 4.0) {
        continue;
      }

      for (const EdgeKey& edge : face_edges[face_index]) {
        auto it = edge_use_count.find(edge);
        if (it == edge_use_count.end()) {
          continue;
        }
        if (--it->second <= 0) {
          edge_use_count.erase(it);
        }
      }

      const std::size_t candidate_score = mismatch_score();
      if (candidate_score < best_score) {
        best_score = candidate_score;
        best_face_index = face_index;
      }

      for (const EdgeKey& edge : face_edges[face_index]) {
        ++edge_use_count[edge];
      }
    }

    if (!best_face_index.has_value()) {
      break;
    }

    faces.erase(faces.begin() + static_cast<long>(*best_face_index));
  }
}

Vec3 newell_loop_normal(const std::vector<int>& loop, const std::vector<Vec3>& vertices) {
  Vec3 normal {};
  for (std::size_t index = 0; index < loop.size(); ++index) {
    const Vec3& current = vertices[loop[index]];
    const Vec3& next = vertices[loop[(index + 1) % loop.size()]];
    normal.x += (current.y - next.y) * (current.z + next.z);
    normal.y += (current.z - next.z) * (current.x + next.x);
    normal.z += (current.x - next.x) * (current.y + next.y);
  }
  return normalized(normal);
}

std::vector<int> sanitize_loop_indices(std::vector<int> loop);

void add_small_planar_gap_caps(std::vector<ReconstructedFace>& faces,
                               const std::vector<Vec3>& vertices,
                               const Tolerances& tolerances,
                               double max_cap_area) {
  if (faces.size() < 20) {
    return;
  }

  auto loop_perimeter = [&vertices](const std::vector<int>& loop) {
    if (loop.size() < 2) {
      return 0.0;
    }

    double perimeter = 0.0;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int current = loop[index];
      const int next = loop[(index + 1) % loop.size()];
      perimeter += length(vertices[next] - vertices[current]);
    }
    return perimeter;
  };

  auto canonicalize_cycle = [](const std::vector<int>& loop) {
    if (loop.empty()) {
      return std::vector<int> {};
    }

    auto canonical_rotation = [](const std::vector<int>& sequence) {
      std::vector<int> best = sequence;
      for (std::size_t offset = 1; offset < sequence.size(); ++offset) {
        std::vector<int> rotated;
        rotated.reserve(sequence.size());
        for (std::size_t index = 0; index < sequence.size(); ++index) {
          rotated.push_back(sequence[(offset + index) % sequence.size()]);
        }
        if (rotated < best) {
          best = std::move(rotated);
        }
      }
      return best;
    };

    std::vector<int> forward = canonical_rotation(loop);
    std::vector<int> reversed(loop.rbegin(), loop.rend());
    reversed = canonical_rotation(reversed);
    return reversed < forward ? reversed : forward;
  };

  auto append_gap_cap_from_loops =
      [&](const std::vector<int>& component_vertices, std::vector<std::vector<int>> loops,
          Vec3 seed_normal) {
        if (loops.empty()) {
          return false;
        }

        Vec3 normal = seed_normal;
        for (const auto& loop : loops) {
          const Vec3 candidate = newell_loop_normal(loop, vertices);
          if (length_squared(candidate) > 1e-12) {
            normal = candidate;
            break;
          }
        }
        if (length_squared(normal) <= 1e-12) {
          return false;
        }

        std::sort(loops.begin(), loops.end(),
                  [&vertices, &normal](const auto& lhs, const auto& rhs) {
                    return polygon_area(lhs, vertices, normal) > polygon_area(rhs, vertices, normal);
                  });

        std::set<int> component_vertex_set(component_vertices.begin(), component_vertices.end());
        Vec3 centroid {};
        for (int vertex_index : component_vertex_set) {
          centroid += vertices[vertex_index];
        }
        centroid =
            centroid / static_cast<double>(std::max<std::size_t>(component_vertex_set.size(), 1));

        double max_distance = 0.0;
        double sum_squared = 0.0;
        for (int vertex_index : component_vertex_set) {
          const double distance = std::abs(dot(normal, vertices[vertex_index] - centroid));
          max_distance = std::max(max_distance, distance);
          sum_squared += distance * distance;
        }

        ReconstructedFace face;
        face.region_id = -1;
        face.fit.normal = normal;
        face.fit.centroid = centroid;
        face.fit.d = -dot(face.fit.normal, face.fit.centroid);
        face.fit.max_error = max_distance;
        face.fit.rms_error =
            std::sqrt(sum_squared /
                      static_cast<double>(std::max<std::size_t>(component_vertex_set.size(), 1)));
        face.fit.confidence = 0.55;
        face.confidence = face.fit.confidence;
        face.loops = std::move(loops);
        orient_loop_to_sign(face.loops.front(), vertices, face.fit.normal, 1.0);
        for (std::size_t loop_index = 1; loop_index < face.loops.size(); ++loop_index) {
          orient_loop_to_sign(face.loops[loop_index], vertices, face.fit.normal, -1.0);
        }
        face.area = face_total_area(face, vertices, face.fit.normal);
        if (face.area <= 1e-9) {
          return false;
        }

        double total_perimeter = 0.0;
        for (const auto& loop : face.loops) {
          total_perimeter += loop_perimeter(loop);
        }
        const double thin_strip_width_limit =
            std::max(tolerances.plane_distance * 0.35, tolerances.vertex_weld * 3.0);
        const bool thin_strip =
            total_perimeter > 1e-9 && (face.area / total_perimeter) <= thin_strip_width_limit;
        if (face.area > max_cap_area &&
            !thin_strip) {
          return false;
        }

        if (max_distance > tolerances.plane_distance * 0.25) {
          const std::size_t max_triangulated_vertices = thin_strip ? 8 : 6;
          const double max_triangulated_distance =
              tolerances.plane_distance * (thin_strip ? 2.5 : 1.25);
          if (face.loops.size() != 1 || face.loops.front().size() < 4 ||
              face.loops.front().size() > max_triangulated_vertices ||
              max_distance > max_triangulated_distance) {
            return false;
          }

          const std::vector<int>& loop = face.loops.front();
          bool emitted = false;
          for (std::size_t index = 1; index + 1 < loop.size(); ++index) {
            ReconstructedFace triangle_face;
            triangle_face.region_id = -1;
            triangle_face.confidence = 0.45;
            triangle_face.loops.push_back({loop[0], loop[index], loop[index + 1]});
            Vec3 triangle_normal = newell_loop_normal(triangle_face.loops.front(), vertices);
            if (length_squared(triangle_normal) <= 1e-12) {
              continue;
            }
            if (dot(triangle_normal, face.fit.normal) < 0.0) {
              std::reverse(triangle_face.loops.front().begin(), triangle_face.loops.front().end());
              triangle_normal = triangle_normal * -1.0;
            }
            Vec3 triangle_centroid {};
            for (int vertex_index : triangle_face.loops.front()) {
              triangle_centroid += vertices[vertex_index];
            }
            triangle_centroid = triangle_centroid / 3.0;
            triangle_face.fit.normal = triangle_normal;
            triangle_face.fit.centroid = triangle_centroid;
            triangle_face.fit.d = -dot(triangle_face.fit.normal, triangle_face.fit.centroid);
            triangle_face.fit.max_error = 0.0;
            triangle_face.fit.rms_error = 0.0;
            triangle_face.fit.confidence = triangle_face.confidence;
            triangle_face.area = face_total_area(triangle_face, vertices, triangle_face.fit.normal);
            if (triangle_face.area > 1e-9) {
              faces.push_back(std::move(triangle_face));
              emitted = true;
            }
          }
          return emitted;
        }

        faces.push_back(std::move(face));
        return true;
      };

  std::unordered_map<EdgeKey, std::vector<std::pair<int, int>>, EdgeKeyHash> edge_orientations;
  for (const ReconstructedFace& face : faces) {
    for (const auto& loop : face.loops) {
      for (std::size_t index = 0; index < loop.size(); ++index) {
        const int from = loop[index];
        const int to = loop[(index + 1) % loop.size()];
        edge_orientations[make_edge_key(from, to)].push_back({from, to});
      }
    }
  }

  std::unordered_map<int, std::vector<int>> adjacency;
  std::unordered_map<EdgeKey, std::pair<int, int>, EdgeKeyHash> open_orientations;
  std::vector<EdgeKey> open_edges;
  for (const auto& [edge, orientations] : edge_orientations) {
    if (orientations.size() != 1) {
      continue;
    }
    open_edges.push_back(edge);
    open_orientations[edge] = orientations.front();
    adjacency[edge.a].push_back(edge.b);
    adjacency[edge.b].push_back(edge.a);
  }

  if (open_edges.empty()) {
    return;
  }

  std::unordered_set<int> visited_vertices;
  auto estimate_component_normal = [&](const std::vector<int>& component_vertices) -> Vec3 {
    for (std::size_t i = 0; i < component_vertices.size(); ++i) {
      const Vec3& a = vertices[component_vertices[i]];
      for (std::size_t j = i + 1; j < component_vertices.size(); ++j) {
        const Vec3& b = vertices[component_vertices[j]];
        for (std::size_t k = j + 1; k < component_vertices.size(); ++k) {
          const Vec3& c = vertices[component_vertices[k]];
          const Vec3 normal = cross(b - a, c - a);
          if (length_squared(normal) > 1e-12) {
            return normalized(normal);
          }
        }
      }
    }
    return Vec3 {0.0, 0.0, 1.0};
  };

  for (const auto& [seed_vertex, neighbors] : adjacency) {
    (void)neighbors;
    if (visited_vertices.count(seed_vertex) > 0) {
      continue;
    }

    std::vector<int> component_vertices;
    std::vector<int> frontier {seed_vertex};
    visited_vertices.insert(seed_vertex);
    while (!frontier.empty()) {
      const int current = frontier.back();
      frontier.pop_back();
      component_vertices.push_back(current);
      for (int neighbor : adjacency[current]) {
        if (visited_vertices.insert(neighbor).second) {
          frontier.push_back(neighbor);
        }
      }
    }

    std::set<int> component_vertex_set(component_vertices.begin(), component_vertices.end());
    std::vector<std::pair<int, int>> component_edges;
    component_edges.reserve(open_edges.size());
    for (const EdgeKey& edge : open_edges) {
      if (component_vertex_set.count(edge.a) != 0 && component_vertex_set.count(edge.b) != 0) {
        const auto orientation_it = open_orientations.find(edge);
        if (orientation_it != open_orientations.end()) {
          component_edges.push_back(orientation_it->second);
        }
      }
    }

    if (component_edges.size() < 3 || component_edges.size() > 24) {
      continue;
    }

    const Vec3 seed_normal = estimate_component_normal(component_vertices);
    std::vector<std::vector<int>> loops;
    std::unordered_map<int, std::vector<int>> component_adjacency;
    for (const auto& [from, to] : component_edges) {
      component_adjacency[from].push_back(to);
      component_adjacency[to].push_back(from);
    }
    bool degree_two_cycle = true;
    for (int vertex_index : component_vertices) {
      if (adjacency[vertex_index].size() != 2) {
        degree_two_cycle = false;
        break;
      }
    }
    if (degree_two_cycle) {
      const int start = component_vertices.front();
      int previous = -1;
      int current = start;
      std::vector<int> loop;
      bool closed = false;
      for (int safety = 0; safety < static_cast<int>(component_vertices.size()) + 2; ++safety) {
        loop.push_back(current);
        const std::vector<int>& neighbors_for_vertex = adjacency[current];
        int next = neighbors_for_vertex.front();
        if (next == previous && neighbors_for_vertex.size() > 1) {
          next = neighbors_for_vertex[1];
        }
        previous = current;
        current = next;
        if (current == start) {
          closed = true;
          break;
        }
      }
      if (closed && loop.size() >= 3) {
        loops.push_back(std::move(loop));
      }
    }
    if (loops.empty()) {
      const double gap_loop_tolerance = std::max(tolerances.vertex_weld * 0.1, 1e-6);
      loops = extract_loops_for_region_undirected(component_edges, vertices, seed_normal,
                                                  gap_loop_tolerance);
    }
    if (!degree_two_cycle && component_edges.size() <= 24) {
      const double gap_loop_tolerance = std::max(tolerances.vertex_weld * 0.1, 1e-6);
      std::vector<std::vector<int>> simple_cycles;
      std::set<std::vector<int>> seen_cycles;
      std::vector<int> starts;
      starts.reserve(component_adjacency.size());
      for (const auto& [vertex, _] : component_adjacency) {
        starts.push_back(vertex);
      }
      std::sort(starts.begin(), starts.end());

      for (int start : starts) {
        std::vector<int> path {start};
        std::unordered_set<int> in_path {start};
        std::function<void(int, int)> dfs = [&](int current, int previous) {
          if (path.size() > 12) {
            return;
          }
          const auto adjacency_it = component_adjacency.find(current);
          if (adjacency_it == component_adjacency.end()) {
            return;
          }
          for (int neighbor : adjacency_it->second) {
            if (neighbor == previous) {
              continue;
            }
            if (neighbor == start) {
              if (path.size() >= 3) {
                std::vector<int> candidate =
                    simplify_loop(path, vertices, seed_normal, gap_loop_tolerance);
                candidate = sanitize_loop_indices(std::move(candidate));
                if (candidate.size() >= 3) {
                  std::vector<int> canonical = canonicalize_cycle(candidate);
                  if (seen_cycles.insert(canonical).second) {
                    simple_cycles.push_back(std::move(candidate));
                  }
                }
              }
              continue;
            }
            if (in_path.count(neighbor) > 0) {
              continue;
            }
            path.push_back(neighbor);
            in_path.insert(neighbor);
            dfs(neighbor, current);
            in_path.erase(neighbor);
            path.pop_back();
          }
        };
        dfs(start, -1);
      }

      bool emitted_cycle_caps = false;
      for (std::vector<int>& cycle : simple_cycles) {
        emitted_cycle_caps |= append_gap_cap_from_loops(cycle, {cycle}, seed_normal);
      }
      if (emitted_cycle_caps) {
        continue;
      }
    }
    if (loops.empty()) {
      continue;
    }

    append_gap_cap_from_loops(component_vertices, std::move(loops), seed_normal);
  }
}

std::vector<ReconstructedFace> build_faces_from_region_loops(const PlaneRegion& region,
                                                             const std::vector<std::vector<int>>& loops,
                                                             const std::vector<Vec3>& vertices) {
  struct ProjectedLoop {
    std::size_t index {};
    double area {};
    std::vector<Vec2> projected;
    Vec2 sample_point {};
  };

  std::vector<ProjectedLoop> projected_loops;
  projected_loops.reserve(loops.size());
  const PlaneBasis basis = basis_from_normal(region.fit.normal);

  for (std::size_t loop_index = 0; loop_index < loops.size(); ++loop_index) {
    if (loops[loop_index].size() < 3) {
      continue;
    }
    std::vector<Vec2> projected;
    projected.reserve(loops[loop_index].size());
    Vec2 sample_point {};
    for (int vertex_index : loops[loop_index]) {
      const Vec2 point = project_to_basis(vertices[vertex_index], basis);
      projected.push_back(point);
      sample_point.x += point.x;
      sample_point.y += point.y;
    }
    sample_point.x /= static_cast<double>(projected.size());
    sample_point.y /= static_cast<double>(projected.size());
    projected_loops.push_back(
        {loop_index, std::abs(signed_area_2d(projected)), std::move(projected), sample_point});
  }

  std::sort(projected_loops.begin(), projected_loops.end(),
            [](const ProjectedLoop& lhs, const ProjectedLoop& rhs) {
              if (std::abs(lhs.area - rhs.area) > 1e-9) {
                return lhs.area > rhs.area;
              }
              return lhs.index < rhs.index;
            });

  std::vector<int> parent(loops.size(), -1);
  std::vector<int> depth(loops.size(), 0);
  for (std::size_t candidate_index = 0; candidate_index < projected_loops.size(); ++candidate_index) {
    const ProjectedLoop& candidate = projected_loops[candidate_index];
    for (std::size_t container_index = 0; container_index < candidate_index; ++container_index) {
      const ProjectedLoop& container = projected_loops[container_index];
      if (point_in_polygon_2d(candidate.sample_point, container.projected)) {
        parent[candidate.index] = static_cast<int>(container.index);
        depth[candidate.index] = depth[container.index] + 1;
        break;
      }
    }
  }

  std::unordered_map<int, std::vector<std::size_t>> outer_to_holes;
  std::vector<std::size_t> outer_indices;
  for (const ProjectedLoop& projected_loop : projected_loops) {
    const std::size_t loop_index = projected_loop.index;
    if (depth[loop_index] % 2 == 0) {
      outer_indices.push_back(loop_index);
      continue;
    }

    int outer_index = parent[loop_index];
    while (outer_index >= 0 && depth[static_cast<std::size_t>(outer_index)] % 2 == 1) {
      outer_index = parent[static_cast<std::size_t>(outer_index)];
    }
    if (outer_index >= 0) {
      outer_to_holes[outer_index].push_back(loop_index);
    }
  }

  std::vector<ReconstructedFace> faces;
  faces.reserve(outer_indices.size());
  for (std::size_t outer_index : outer_indices) {
    ReconstructedFace face;
    face.region_id = region.id;
    face.fit = region.fit;
    face.confidence = region.fit.confidence;
    face.loops.push_back(loops[outer_index]);
    orient_loop_to_sign(face.loops.front(), vertices, region.fit.normal, 1.0);

    auto holes_it = outer_to_holes.find(static_cast<int>(outer_index));
    if (holes_it != outer_to_holes.end()) {
      std::sort(holes_it->second.begin(), holes_it->second.end());
      for (std::size_t hole_index : holes_it->second) {
        std::vector<int> hole = loops[hole_index];
        orient_loop_to_sign(hole, vertices, region.fit.normal, -1.0);
        face.loops.push_back(std::move(hole));
      }
    }

    face.area = face_total_area(face, vertices, region.fit.normal);
    faces.push_back(std::move(face));
  }

  return faces;
}

template <typename Fn>
void for_each_face_edge(const ReconstructedFace& face, Fn&& fn) {
  for (const auto& loop : face.loops) {
    for (std::size_t index = 0; index < loop.size(); ++index) {
      fn(loop[index], loop[(index + 1) % loop.size()]);
    }
  }
}

std::vector<int> sanitize_loop_indices(std::vector<int> loop) {
  if (loop.size() < 3) {
    return {};
  }
  if (loop.size() >= 2 && loop.front() == loop.back()) {
    loop.pop_back();
  }

  bool changed = true;
  while (changed && loop.size() >= 3) {
    changed = false;

    std::vector<int> compacted;
    compacted.reserve(loop.size());
    for (int vertex : loop) {
      if (!compacted.empty() && compacted.back() == vertex) {
        changed = true;
        continue;
      }
      compacted.push_back(vertex);
    }
    if (compacted.size() >= 2 && compacted.front() == compacted.back()) {
      compacted.pop_back();
      changed = true;
    }
    loop = std::move(compacted);
    if (loop.size() < 3) {
      break;
    }

    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int previous = loop[(index + loop.size() - 1) % loop.size()];
      const int next = loop[(index + 1) % loop.size()];
      if (previous == next) {
        loop.erase(loop.begin() + static_cast<long>(index));
        changed = true;
        break;
      }
    }
  }

  return loop.size() >= 3 ? loop : std::vector<int> {};
}

std::vector<int> refine_loop_with_shared_vertices(const std::vector<int>& loop,
                                                  const std::vector<Vec3>& vertices,
                                                  const std::vector<int>& candidate_vertices,
                                                  double tolerance,
                                                  std::size_t& insertion_count) {
  if (loop.size() < 3) {
    return loop;
  }

  std::vector<int> refined;
  refined.reserve(loop.size() * 2);
  const double endpoint_epsilon = std::max(tolerance * 0.5, 1e-8);

  for (std::size_t index = 0; index < loop.size(); ++index) {
    const int from = loop[index];
    const int to = loop[(index + 1) % loop.size()];
    refined.push_back(from);

    std::vector<std::pair<double, int>> inserts;
    inserts.reserve(4);
    for (int candidate : candidate_vertices) {
      if (candidate == from || candidate == to) {
        continue;
      }
      double t = 0.0;
      const double distance =
          point_segment_distance_3d(vertices[candidate], vertices[from], vertices[to], t);
      if (distance > tolerance || t <= endpoint_epsilon || t >= 1.0 - endpoint_epsilon) {
        continue;
      }
      inserts.push_back({t, candidate});
    }

    std::sort(inserts.begin(), inserts.end(), [](const auto& lhs, const auto& rhs) {
      if (std::abs(lhs.first - rhs.first) > 1e-9) {
        return lhs.first < rhs.first;
      }
      return lhs.second < rhs.second;
    });

    int last_added = from;
    double last_t = -1.0;
    for (const auto& [t, candidate] : inserts) {
      if (candidate == last_added || std::abs(t - last_t) <= 1e-9) {
        continue;
      }
      refined.push_back(candidate);
      ++insertion_count;
      last_added = candidate;
      last_t = t;
    }
  }

  std::vector<int> compacted;
  compacted.reserve(refined.size());
  for (int vertex : refined) {
    if (!compacted.empty() && compacted.back() == vertex) {
      continue;
    }
    compacted.push_back(vertex);
  }
  if (compacted.size() >= 2 && compacted.front() == compacted.back()) {
    compacted.pop_back();
  }

  compacted = sanitize_loop_indices(std::move(compacted));
  return compacted.size() >= 3 ? compacted : loop;
}

void refine_face_loops_with_shared_vertices(std::vector<ReconstructedFace>& faces,
                                            const std::vector<Vec3>& vertices,
                                            double tolerance,
                                            std::size_t& insertion_count) {
  std::set<int> boundary_vertex_set;
  for (const ReconstructedFace& face : faces) {
    for (const auto& loop : face.loops) {
      boundary_vertex_set.insert(loop.begin(), loop.end());
    }
  }
  const std::vector<int> candidate_vertices {boundary_vertex_set.begin(), boundary_vertex_set.end()};

  for (ReconstructedFace& face : faces) {
    for (std::vector<int>& loop : face.loops) {
      loop = refine_loop_with_shared_vertices(loop, vertices, candidate_vertices, tolerance,
                                              insertion_count);
    }
  }
}

void sanitize_face_loops(std::vector<ReconstructedFace>& faces) {
  for (ReconstructedFace& face : faces) {
    std::vector<std::vector<int>> sanitized;
    sanitized.reserve(face.loops.size());
    for (std::vector<int>& loop : face.loops) {
      std::vector<int> cleaned = sanitize_loop_indices(std::move(loop));
      if (cleaned.size() >= 3) {
        sanitized.push_back(std::move(cleaned));
      }
    }
    face.loops = std::move(sanitized);
  }
}

double signed_loop_volume(const std::vector<int>& loop, const std::vector<Vec3>& vertices) {
  if (loop.size() < 3) {
    return 0.0;
  }
  double volume = 0.0;
  const Vec3& a = vertices[loop[0]];
  for (std::size_t index = 1; index + 1 < loop.size(); ++index) {
    const Vec3& b = vertices[loop[index]];
    const Vec3& c = vertices[loop[index + 1]];
    volume += dot(a, cross(b, c)) / 6.0;
  }
  return volume;
}

double signed_polyhedron_volume(const std::vector<ReconstructedFace>& faces,
                                const std::vector<Vec3>& vertices) {
  double volume = 0.0;
  for (const ReconstructedFace& face : faces) {
    if (face.loops.empty()) {
      continue;
    }
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const double contribution = signed_loop_volume(face.loops[loop_index], vertices);
      volume += contribution;
    }
  }
  return volume;
}

void append_loop_candidate(std::vector<LoopCandidate>& candidates,
                           std::vector<std::vector<int>> loops,
                           const std::vector<Vec3>& vertices,
                           const Vec3& normal,
                           double expected_area) {
  LoopCandidate candidate =
      score_loop_candidate(std::move(loops), vertices, normal, expected_area);
  if (!candidate.loops.empty()) {
    candidates.push_back(std::move(candidate));
  }
}

std::vector<LoopCandidate> collect_region_loop_candidates(
    const PlaneRegion& region,
    const std::unordered_map<int, std::vector<std::pair<int, int>>>& boundary_edges,
    const MeshModel& mesh,
    const std::vector<int>& original_to_snapped,
    const std::vector<Vec3>& vertices,
    const Tolerances& tolerances) {
  std::vector<LoopCandidate> candidates;

  const auto topological_edges_it = boundary_edges.find(region.id);
  if (topological_edges_it != boundary_edges.end()) {
    append_loop_candidate(candidates,
                          extract_loops_for_region(topological_edges_it->second, vertices,
                                                   region.fit.normal,
                                                   tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
    append_loop_candidate(candidates,
                          extract_loops_for_region_undirected(topological_edges_it->second, vertices,
                                                              region.fit.normal,
                                                              tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
  }

  const std::vector<std::pair<int, int>> triangle_boundary_edges =
      build_region_boundary_edges_from_triangles(region, mesh, original_to_snapped);
  if (!triangle_boundary_edges.empty()) {
    append_loop_candidate(candidates,
                          extract_loops_for_region(triangle_boundary_edges, vertices,
                                                   region.fit.normal,
                                                   tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
    append_loop_candidate(candidates,
                          extract_loops_for_region_undirected(triangle_boundary_edges, vertices,
                                                              region.fit.normal,
                                                              tolerances.collinear_distance),
                          vertices, region.fit.normal, region.area);
  }

  if (candidates.empty()) {
    std::vector<int> hull =
        fallback_convex_hull_loop(region, original_to_snapped, vertices, region.fit.normal,
                                  tolerances.collinear_distance);
    const double hull_area =
        hull.size() >= 3 ? polygon_area(hull, vertices, region.fit.normal) : 0.0;
    const double hull_ratio = region.area > 1e-9 ? hull_area / region.area : 0.0;
    if (hull.size() >= 3 && hull_ratio >= 0.7 && hull_ratio <= 1.15) {
      append_loop_candidate(candidates, {std::move(hull)}, vertices, region.fit.normal, region.area);
    }
  }

  return candidates;
}

std::size_t choose_initial_candidate_index(const std::vector<LoopCandidate>& candidates) {
  if (candidates.empty()) {
    return std::numeric_limits<std::size_t>::max();
  }

  std::size_t best_index = 0;
  LoopCandidate best_candidate = candidates.front();
  for (std::size_t index = 1; index < candidates.size(); ++index) {
    if (is_better_loop_candidate(candidates[index], best_candidate)) {
      best_candidate = candidates[index];
      best_index = index;
    }
  }
  return best_index;
}

void build_faces_from_selected_candidates(const std::vector<PlaneRegion>& regions,
                                          const std::vector<std::vector<LoopCandidate>>& candidates,
                                          const std::vector<std::size_t>& selected_indices,
                                          const std::vector<Vec3>& vertices,
                                          ReconstructionResult& result) {
  result.faces.clear();
  result.omitted_region_ids.clear();
  result.edge_split_insertions = 0;

  for (std::size_t region_index = 0; region_index < regions.size(); ++region_index) {
    const PlaneRegion& region = regions[region_index];
    if (region_index >= selected_indices.size() ||
        selected_indices[region_index] >= candidates[region_index].size()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }

    std::vector<ReconstructedFace> faces =
        build_faces_from_region_loops(region, candidates[region_index][selected_indices[region_index]].loops,
                                      vertices);
    if (faces.empty()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }

    bool emitted_face = false;
    for (ReconstructedFace& face : faces) {
      if (!face.loops.empty() && face.loops.front().size() >= 3) {
        result.faces.push_back(std::move(face));
        emitted_face = true;
      }
    }
    if (!emitted_face) {
      result.omitted_region_ids.push_back(region.id);
    }
  }
}

void assess_shell_topology(ReconstructionResult& result,
                           const MeshModel& mesh,
                           double solid_threshold) {
  result.failure_reasons.clear();
  result.problematic_regions.clear();
  result.open_edge_count = 0;
  result.non_manifold_edge_count = 0;
  result.shell_gap_score = 1.0;
  result.confidence = 0.0;

  if (result.faces.empty()) {
    result.failure_reasons.push_back("Unable to reconstruct any planar shell faces");
    result.outcome = ReconstructionOutcome::AnalysisOnly;
    return;
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
  std::unordered_map<EdgeKey, std::vector<int>, EdgeKeyHash> edge_regions;
  for (const ReconstructedFace& face : result.faces) {
    for_each_face_edge(face, [&](int from, int to) {
      const EdgeKey key = make_edge_key(from, to);
      ++edge_use_count[key];
      edge_regions[key].push_back(face.region_id);
    });
  }

  std::size_t open_shell_edges = 0;
  std::size_t non_manifold_shell_edges = 0;
  std::unordered_map<int, std::size_t> problematic_region_counts;
  for (const auto& [edge_key, count] : edge_use_count) {
    if (count == 1) {
      ++open_shell_edges;
      for (int region_id : edge_regions[edge_key]) {
        ++problematic_region_counts[region_id];
      }
    } else if (count != 2) {
      ++non_manifold_shell_edges;
      for (int region_id : edge_regions[edge_key]) {
        ++problematic_region_counts[region_id];
      }
    }
  }

  result.open_edge_count = open_shell_edges;
  result.non_manifold_edge_count = non_manifold_shell_edges;
  result.problematic_regions.reserve(problematic_region_counts.size());
  for (const auto& [region_id, edge_count] : problematic_region_counts) {
    result.problematic_regions.push_back({region_id, edge_count});
  }
  std::sort(result.problematic_regions.begin(), result.problematic_regions.end(),
            [](const RegionTopologyIssue& lhs, const RegionTopologyIssue& rhs) {
              if (lhs.edge_count != rhs.edge_count) {
                return lhs.edge_count > rhs.edge_count;
              }
              return lhs.region_id < rhs.region_id;
            });

  result.shell_gap_score =
      edge_use_count.empty()
          ? 1.0
          : static_cast<double>(open_shell_edges + non_manifold_shell_edges) /
                static_cast<double>(edge_use_count.size());

  for (const ReconstructedFace& face : result.faces) {
    result.confidence += face.confidence;
  }
  result.confidence /= static_cast<double>(result.faces.size());

  if (open_shell_edges > 0 || non_manifold_shell_edges > 0) {
    result.failure_reasons.push_back("Reconstructed shell is open or topologically inconsistent");
  }
  if (result.faces.size() < 4) {
    result.failure_reasons.push_back("Too few reconstructed faces for a closed solid");
  }

  const double volume = std::abs(signed_polyhedron_volume(result.faces, result.vertices));
  const double min_volume = std::pow(std::max(bounds_diagonal(mesh.bounds), 1e-6), 3.0) * 1e-6;
  if (volume <= min_volume) {
    result.failure_reasons.push_back("Reconstructed shell volume is too small or inconsistent");
  }

  if (open_shell_edges == 0 && non_manifold_shell_edges == 0 && result.faces.size() >= 4 &&
      volume > min_volume && result.confidence >= solid_threshold) {
    result.outcome = ReconstructionOutcome::SolidCreated;
  } else if (!result.faces.empty()) {
    result.outcome = ReconstructionOutcome::ShellOnly;
    if (result.confidence < solid_threshold) {
      result.failure_reasons.push_back("Solid export was gated by low reconstruction confidence");
    }
  } else {
    result.outcome = ReconstructionOutcome::AnalysisOnly;
  }
}

void finalize_shell_reconstruction(ReconstructionResult& result,
                                   const MeshModel& mesh,
                                   const Tolerances& tolerances,
                                   double solid_threshold) {
  if (result.faces.empty()) {
    assess_shell_topology(result, mesh, solid_threshold);
    return;
  }

  refine_face_loops_with_shared_vertices(result.faces, result.vertices, tolerances.collinear_distance,
                                         result.edge_split_insertions);
  for (ReconstructedFace& face : result.faces) {
    if (face.loops.empty()) {
      continue;
    }
    orient_loop_to_sign(face.loops.front(), result.vertices, face.fit.normal, 1.0);
    for (std::size_t loop_index = 1; loop_index < face.loops.size(); ++loop_index) {
      orient_loop_to_sign(face.loops[loop_index], result.vertices, face.fit.normal, -1.0);
    }
    face.area = face_total_area(face, result.vertices, face.fit.normal);
  }
  suppress_duplicate_coplanar_faces(result.faces, tolerances);
  prune_dangling_faces(result.faces, tolerances);
  const double shell_cap_area_limit =
      std::min(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.015,
               tolerances.plane_distance * tolerances.plane_distance * 40.0);
  add_small_planar_gap_caps(result.faces, result.vertices, tolerances, shell_cap_area_limit);
  sanitize_face_loops(result.faces);
  assess_shell_topology(result, mesh, solid_threshold);
}

bool is_better_reconstruction_result(const ReconstructionResult& candidate,
                                     const ReconstructionResult& current) {
  const auto outcome_rank = [](ReconstructionOutcome outcome) {
    switch (outcome) {
      case ReconstructionOutcome::SolidCreated:
        return 3;
      case ReconstructionOutcome::ShellOnly:
        return 2;
      case ReconstructionOutcome::AnalysisOnly:
      default:
        return 1;
    }
  };

  if (outcome_rank(candidate.outcome) != outcome_rank(current.outcome)) {
    return outcome_rank(candidate.outcome) > outcome_rank(current.outcome);
  }
  if (candidate.non_manifold_edge_count != current.non_manifold_edge_count) {
    return candidate.non_manifold_edge_count < current.non_manifold_edge_count;
  }
  if (candidate.open_edge_count != current.open_edge_count) {
    return candidate.open_edge_count < current.open_edge_count;
  }
  if (candidate.omitted_region_ids.size() != current.omitted_region_ids.size()) {
    return candidate.omitted_region_ids.size() < current.omitted_region_ids.size();
  }
  if (std::abs(candidate.shell_gap_score - current.shell_gap_score) > 1e-9) {
    return candidate.shell_gap_score < current.shell_gap_score;
  }
  if (std::abs(candidate.confidence - current.confidence) > 1e-9) {
    return candidate.confidence > current.confidence;
  }
  return candidate.faces.size() > current.faces.size();
}

std::vector<std::size_t> collect_problem_region_indices(
    const ReconstructionResult& result,
    const std::unordered_map<int, std::size_t>& region_id_to_index,
    std::size_t max_regions) {
  std::vector<std::size_t> indices;
  std::unordered_set<std::size_t> seen;

  for (const RegionTopologyIssue& issue : result.problematic_regions) {
    const auto it = region_id_to_index.find(issue.region_id);
    if (it != region_id_to_index.end() && seen.insert(it->second).second) {
      indices.push_back(it->second);
      if (indices.size() >= max_regions) {
        return indices;
      }
    }
  }
  for (int region_id : result.omitted_region_ids) {
    const auto it = region_id_to_index.find(region_id);
    if (it != region_id_to_index.end() && seen.insert(it->second).second) {
      indices.push_back(it->second);
      if (indices.size() >= max_regions) {
        return indices;
      }
    }
  }
  return indices;
}

void optimize_region_loop_choices(ReconstructionResult& result,
                                  const MeshModel& mesh,
                                  const std::vector<PlaneRegion>& regions,
                                  const std::vector<std::vector<LoopCandidate>>& candidates,
                                  std::vector<std::size_t>& selected_indices,
                                  const Tolerances& tolerances,
                                  double solid_threshold) {
  std::unordered_map<int, std::size_t> region_id_to_index;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    region_id_to_index[regions[index].id] = index;
  }

  for (int pass = 0; pass < 2; ++pass) {
    bool improved = false;
    const std::vector<std::size_t> priority =
        collect_problem_region_indices(result, region_id_to_index, 24);
    for (std::size_t region_index : priority) {
      if (region_index >= candidates.size() || candidates[region_index].size() <= 1) {
        continue;
      }

      ReconstructionResult best_result = result;
      std::size_t best_choice = selected_indices[region_index];
      for (std::size_t candidate_index = 0; candidate_index < candidates[region_index].size();
           ++candidate_index) {
        if (candidate_index == best_choice) {
          continue;
        }

        std::vector<std::size_t> trial_indices = selected_indices;
        trial_indices[region_index] = candidate_index;

        ReconstructionResult trial;
        trial.vertices = result.vertices;
        build_faces_from_selected_candidates(regions, candidates, trial_indices, trial.vertices, trial);
        finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
        if (is_better_reconstruction_result(trial, best_result)) {
          best_result = std::move(trial);
          best_choice = candidate_index;
        }
      }

      if (best_choice != selected_indices[region_index]) {
        selected_indices[region_index] = best_choice;
        result = std::move(best_result);
        improved = true;
      }
    }

    if (result.outcome != ReconstructionOutcome::SolidCreated) {
      std::vector<std::size_t> pair_priority;
      pair_priority.reserve(priority.size());
      for (std::size_t region_index : priority) {
        if (region_index < candidates.size() && candidates[region_index].size() > 1) {
          pair_priority.push_back(region_index);
        }
      }

      const std::size_t pair_limit = std::min<std::size_t>(6, pair_priority.size());
      ReconstructionResult best_pair_result = result;
      std::vector<std::size_t> best_pair_indices = selected_indices;

      for (std::size_t left = 0; left < pair_limit; ++left) {
        const std::size_t left_region = pair_priority[left];
        const std::size_t left_current = selected_indices[left_region];
        for (std::size_t right = left + 1; right < pair_limit; ++right) {
          const std::size_t right_region = pair_priority[right];
          const std::size_t right_current = selected_indices[right_region];
          for (std::size_t left_candidate = 0; left_candidate < candidates[left_region].size();
               ++left_candidate) {
            if (left_candidate == left_current) {
              continue;
            }
            for (std::size_t right_candidate = 0;
                 right_candidate < candidates[right_region].size();
                 ++right_candidate) {
              if (right_candidate == right_current) {
                continue;
              }

              std::vector<std::size_t> trial_indices = selected_indices;
              trial_indices[left_region] = left_candidate;
              trial_indices[right_region] = right_candidate;

              ReconstructionResult trial;
              trial.vertices = result.vertices;
              build_faces_from_selected_candidates(regions, candidates, trial_indices, trial.vertices,
                                                  trial);
              finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
              if (is_better_reconstruction_result(trial, best_pair_result)) {
                best_pair_result = std::move(trial);
                best_pair_indices = std::move(trial_indices);
              }
            }
          }
        }
      }

      if (is_better_reconstruction_result(best_pair_result, result)) {
        selected_indices = std::move(best_pair_indices);
        result = std::move(best_pair_result);
        improved = true;
      }
    }

    if (!improved || result.outcome == ReconstructionOutcome::SolidCreated) {
      break;
    }
  }

  if (result.outcome == ReconstructionOutcome::SolidCreated) {
    return;
  }

  const std::vector<std::size_t> priority =
      collect_problem_region_indices(result, region_id_to_index, 10);
  for (std::size_t lhs_offset = 0; lhs_offset < priority.size(); ++lhs_offset) {
    const std::size_t lhs_region = priority[lhs_offset];
    if (lhs_region >= candidates.size() || candidates[lhs_region].size() <= 1) {
      continue;
    }
    for (std::size_t rhs_offset = lhs_offset + 1; rhs_offset < priority.size(); ++rhs_offset) {
      const std::size_t rhs_region = priority[rhs_offset];
      if (rhs_region >= candidates.size() || candidates[rhs_region].size() <= 1) {
        continue;
      }

      ReconstructionResult best_result = result;
      std::size_t best_lhs_choice = selected_indices[lhs_region];
      std::size_t best_rhs_choice = selected_indices[rhs_region];

      for (std::size_t lhs_choice = 0; lhs_choice < candidates[lhs_region].size(); ++lhs_choice) {
        for (std::size_t rhs_choice = 0; rhs_choice < candidates[rhs_region].size(); ++rhs_choice) {
          if (lhs_choice == selected_indices[lhs_region] &&
              rhs_choice == selected_indices[rhs_region]) {
            continue;
          }

          std::vector<std::size_t> trial_indices = selected_indices;
          trial_indices[lhs_region] = lhs_choice;
          trial_indices[rhs_region] = rhs_choice;

          ReconstructionResult trial;
          trial.vertices = result.vertices;
          build_faces_from_selected_candidates(regions, candidates, trial_indices, trial.vertices, trial);
          finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
          if (is_better_reconstruction_result(trial, best_result)) {
            best_result = std::move(trial);
            best_lhs_choice = lhs_choice;
            best_rhs_choice = rhs_choice;
          }
        }
      }

      if (best_lhs_choice != selected_indices[lhs_region] ||
          best_rhs_choice != selected_indices[rhs_region]) {
        selected_indices[lhs_region] = best_lhs_choice;
        selected_indices[rhs_region] = best_rhs_choice;
        result = std::move(best_result);
        if (result.outcome == ReconstructionOutcome::SolidCreated) {
          return;
        }
      }
    }
  }

}

void resolve_tiny_conflict_faces(ReconstructionResult& result,
                                 const MeshModel& mesh,
                                 const Tolerances& tolerances,
                                 double solid_threshold) {
  if (result.non_manifold_edge_count == 0 || result.faces.empty()) {
    return;
  }

  const double shell_cap_area_limit =
      std::min(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.015,
               tolerances.plane_distance * tolerances.plane_distance * 40.0);

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_use_count;
  std::vector<std::unordered_set<EdgeKey, EdgeKeyHash>> face_edges(result.faces.size());
  for (std::size_t face_index = 0; face_index < result.faces.size(); ++face_index) {
    for_each_face_edge(result.faces[face_index], [&](int from, int to) {
      const EdgeKey key = make_edge_key(from, to);
      ++edge_use_count[key];
      face_edges[face_index].insert(key);
    });
  }

  std::vector<std::size_t> candidate_faces;
  candidate_faces.reserve(result.faces.size());
  for (std::size_t face_index = 0; face_index < result.faces.size(); ++face_index) {
    const ReconstructedFace& face = result.faces[face_index];
    if (face.region_id < 0 || face.area > shell_cap_area_limit || face.loops.size() != 1 ||
        face.loops.front().size() > 8) {
      continue;
    }
    bool touches_non_manifold = false;
    for (const EdgeKey& edge : face_edges[face_index]) {
      const auto edge_it = edge_use_count.find(edge);
      if (edge_it != edge_use_count.end() && edge_it->second > 2) {
        touches_non_manifold = true;
        break;
      }
    }
    if (touches_non_manifold) {
      candidate_faces.push_back(face_index);
    }
  }

  if (candidate_faces.empty() || candidate_faces.size() > 6) {
    return;
  }

  ReconstructionResult best_result = result;
  const std::size_t subset_limit = std::min<std::size_t>(3, candidate_faces.size());
  const std::size_t total_masks = std::size_t {1} << candidate_faces.size();
  for (std::size_t mask = 1; mask < total_masks; ++mask) {
    if (static_cast<std::size_t>(__builtin_popcountll(static_cast<unsigned long long>(mask))) >
        subset_limit) {
      continue;
    }

    ReconstructionResult trial = result;
    std::vector<ReconstructedFace> kept_faces;
    kept_faces.reserve(result.faces.size());
    std::unordered_set<int> removed_region_ids;
    for (std::size_t face_index = 0; face_index < result.faces.size(); ++face_index) {
      bool remove_face = false;
      for (std::size_t candidate_index = 0; candidate_index < candidate_faces.size(); ++candidate_index) {
        if (((mask >> candidate_index) & 1U) != 0U && candidate_faces[candidate_index] == face_index) {
          remove_face = true;
          removed_region_ids.insert(result.faces[face_index].region_id);
          break;
        }
      }
      if (!remove_face) {
        kept_faces.push_back(result.faces[face_index]);
      }
    }
    trial.faces = std::move(kept_faces);
    for (int region_id : removed_region_ids) {
      if (std::find(trial.omitted_region_ids.begin(), trial.omitted_region_ids.end(), region_id) ==
          trial.omitted_region_ids.end()) {
        trial.omitted_region_ids.push_back(region_id);
      }
    }
    finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
    if (is_better_reconstruction_result(trial, best_result)) {
      best_result = std::move(trial);
    }
  }

  if (is_better_reconstruction_result(best_result, result)) {
    result = std::move(best_result);
  }
}

void resolve_tiny_triangle_gaps(ReconstructionResult& result,
                                const MeshModel& mesh,
                                const Tolerances& tolerances,
                                double solid_threshold) {
  if (result.open_edge_count < 3 || result.non_manifold_edge_count != 0 || result.faces.empty()) {
    return;
  }

  std::unordered_map<EdgeKey, std::vector<std::pair<int, int>>, EdgeKeyHash> edge_orientations;
  for (const ReconstructedFace& face : result.faces) {
    for_each_face_edge(face, [&](int from, int to) {
      edge_orientations[make_edge_key(from, to)].push_back({from, to});
    });
  }

  std::unordered_map<int, std::vector<int>> adjacency;
  std::vector<std::pair<int, int>> open_edges;
  for (const auto& [edge, orientations] : edge_orientations) {
    if (orientations.size() != 1) {
      continue;
    }
    open_edges.push_back(orientations.front());
    adjacency[edge.a].push_back(edge.b);
    adjacency[edge.b].push_back(edge.a);
  }
  if (open_edges.size() < 3) {
    return;
  }

  const double triangle_cap_area_limit =
      std::max(tolerances.plane_distance * tolerances.plane_distance * 180.0,
               bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.0002);

  std::unordered_set<int> visited_vertices;
  ReconstructionResult trial = result;
  bool added_face = false;

  for (const auto& [seed_vertex, neighbors] : adjacency) {
    (void)neighbors;
    if (visited_vertices.count(seed_vertex) > 0) {
      continue;
    }

    std::vector<int> component_vertices;
    std::vector<int> frontier {seed_vertex};
    visited_vertices.insert(seed_vertex);
    while (!frontier.empty()) {
      const int current = frontier.back();
      frontier.pop_back();
      component_vertices.push_back(current);
      for (int neighbor : adjacency[current]) {
        if (visited_vertices.insert(neighbor).second) {
          frontier.push_back(neighbor);
        }
      }
    }

    if (component_vertices.size() != 3) {
      continue;
    }
    bool degree_two_cycle = true;
    for (int vertex_index : component_vertices) {
      if (adjacency[vertex_index].size() != 2) {
        degree_two_cycle = false;
        break;
      }
    }
    if (!degree_two_cycle) {
      continue;
    }

    const int start = component_vertices.front();
    int previous = -1;
    int current = start;
    std::vector<int> loop;
    bool closed = false;
    for (int safety = 0; safety < 5; ++safety) {
      loop.push_back(current);
      const std::vector<int>& neighbors_for_vertex = adjacency[current];
      int next = neighbors_for_vertex.front();
      if (next == previous && neighbors_for_vertex.size() > 1) {
        next = neighbors_for_vertex[1];
      }
      previous = current;
      current = next;
      if (current == start) {
        closed = true;
        break;
      }
    }
    if (!closed || loop.size() != 3) {
      continue;
    }

    ReconstructedFace face;
    face.region_id = -1;
    face.confidence = 0.4;
    face.loops.push_back(loop);
    Vec3 normal = newell_loop_normal(loop, trial.vertices);
    if (length_squared(normal) <= 1e-12) {
      continue;
    }
    face.fit.normal = normal;
    Vec3 centroid {};
    for (int vertex_index : loop) {
      centroid += trial.vertices[vertex_index];
    }
    centroid = centroid / 3.0;
    face.fit.centroid = centroid;
    face.fit.d = -dot(face.fit.normal, face.fit.centroid);
    face.fit.max_error = 0.0;
    face.fit.rms_error = 0.0;
    face.fit.confidence = face.confidence;
    orient_loop_to_sign(face.loops.front(), trial.vertices, face.fit.normal, 1.0);
    face.area = face_total_area(face, trial.vertices, face.fit.normal);
    if (face.area <= 1e-9 || face.area > triangle_cap_area_limit) {
      continue;
    }

    trial.faces.push_back(std::move(face));
    added_face = true;
  }

  if (!added_face) {
    return;
  }

  finalize_shell_reconstruction(trial, mesh, tolerances, solid_threshold);
  if (is_better_reconstruction_result(trial, result)) {
    result = std::move(trial);
  }
}

ReconstructionResult reconstruct_shell(const MeshModel& mesh,
                                       const std::vector<PlaneRegion>& regions,
                                       const Tolerances& tolerances,
                                       double solid_threshold) {
  ReconstructionResult result;
  result.method = ReconstructionMethod::AnalyticPlanar;
  if (regions.empty()) {
    result.failure_reasons.push_back("No planar regions were detected");
    return result;
  }

  const AdjacencyData adjacency = build_adjacency(mesh);
  std::vector<std::vector<int>> incident_regions(mesh.vertices.size());
  for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index) {
    std::set<int> ids;
    for (int triangle_index : adjacency.vertex_to_triangles[vertex_index]) {
      if (mesh.triangles[triangle_index].region_id >= 0) {
        ids.insert(mesh.triangles[triangle_index].region_id);
      }
    }
    incident_regions[vertex_index] = {ids.begin(), ids.end()};
  }

  std::vector<int> original_to_snapped(mesh.vertices.size(), -1);
  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> snapped_lookup;
  const double snap_weld_tolerance = std::max(tolerances.vertex_weld * 0.10, 1e-7);
  auto quantize = [snap_weld_tolerance](double value) -> long long {
    return static_cast<long long>(std::llround(value / snap_weld_tolerance));
  };

  for (std::size_t vertex_index = 0; vertex_index < mesh.vertices.size(); ++vertex_index) {
    Vec3 snapped = mesh.vertices[vertex_index];
    if (!incident_regions[vertex_index].empty()) {
      snapped = snap_vertex_to_incident_planes(mesh.vertices[vertex_index], incident_regions[vertex_index],
                                               regions, tolerances);
    }
    const GridKey key {quantize(snapped.x), quantize(snapped.y), quantize(snapped.z)};
    const std::optional<int> found =
        find_nearby_grid_vertex(snapped_lookup, result.vertices, key, snapped, snap_weld_tolerance);
    if (found.has_value()) {
      original_to_snapped[vertex_index] = *found;
    } else {
      const int snapped_index = static_cast<int>(result.vertices.size());
      result.vertices.push_back(snapped);
      snapped_lookup[key].push_back(snapped_index);
      original_to_snapped[vertex_index] = snapped_index;
    }
  }

  std::unordered_map<int, std::vector<std::pair<int, int>>> boundary_edges;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    for (int edge_index = 0; edge_index < 3; ++edge_index) {
      const int original_from = triangle.vertices[edge_index];
      const int original_to = triangle.vertices[(edge_index + 1) % 3];
      const EdgeKey edge_key = make_edge_key(original_from, original_to);
      const auto occurrences_it = adjacency.edge_occurrences.find(edge_key);
      bool shared_with_same_region = false;
      if (occurrences_it != adjacency.edge_occurrences.end()) {
        for (const TriangleEdgeOccurrence& occurrence : occurrences_it->second) {
          if (occurrence.triangle == static_cast<int>(triangle_index)) {
            continue;
          }
          if (mesh.triangles[occurrence.triangle].region_id == triangle.region_id) {
            shared_with_same_region = true;
            break;
          }
        }
      }
      if (shared_with_same_region) {
        continue;
      }
      const int snapped_from = original_to_snapped[original_from];
      const int snapped_to = original_to_snapped[original_to];
      if (snapped_from == snapped_to) {
        continue;
      }
      boundary_edges[triangle.region_id].push_back({snapped_from, snapped_to});
    }
  }

  for (const PlaneRegion& region : regions) {
    LoopCandidate best_candidate;
    const auto topological_edges_it = boundary_edges.find(region.id);
    if (topological_edges_it != boundary_edges.end()) {
      LoopCandidate directed_candidate =
          score_loop_candidate(extract_loops_for_region(topological_edges_it->second, result.vertices,
                                                        region.fit.normal,
                                                        tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(directed_candidate, best_candidate)) {
        best_candidate = std::move(directed_candidate);
      }

      LoopCandidate undirected_candidate =
          score_loop_candidate(extract_loops_for_region_undirected(topological_edges_it->second,
                                                                   result.vertices, region.fit.normal,
                                                                   tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(undirected_candidate, best_candidate)) {
        best_candidate = std::move(undirected_candidate);
      }
    }

    const std::vector<std::pair<int, int>> triangle_boundary_edges =
        build_region_boundary_edges_from_triangles(region, mesh, original_to_snapped);
    if (!triangle_boundary_edges.empty()) {
      LoopCandidate triangle_directed =
          score_loop_candidate(extract_loops_for_region(triangle_boundary_edges, result.vertices,
                                                        region.fit.normal,
                                                        tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(triangle_directed, best_candidate)) {
        best_candidate = std::move(triangle_directed);
      }

      LoopCandidate triangle_undirected =
          score_loop_candidate(extract_loops_for_region_undirected(triangle_boundary_edges,
                                                                   result.vertices, region.fit.normal,
                                                                   tolerances.collinear_distance),
                               result.vertices, region.fit.normal, region.area);
      if (is_better_loop_candidate(triangle_undirected, best_candidate)) {
        best_candidate = std::move(triangle_undirected);
      }
    }

    std::vector<std::vector<int>> loops = std::move(best_candidate.loops);
    if (loops.empty()) {
      std::vector<int> hull =
          fallback_convex_hull_loop(region, original_to_snapped, result.vertices, region.fit.normal,
                                    tolerances.collinear_distance);
      const double hull_area =
          hull.size() >= 3 ? polygon_area(hull, result.vertices, region.fit.normal) : 0.0;
      const double hull_ratio = region.area > 1e-9 ? hull_area / region.area : 0.0;
      if (hull.size() >= 3 && hull_ratio >= 0.7 && hull_ratio <= 1.15) {
        loops.push_back(std::move(hull));
      }
    }
    if (loops.empty()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }

    std::vector<ReconstructedFace> faces = build_faces_from_region_loops(region, loops, result.vertices);
    if (faces.empty()) {
      result.omitted_region_ids.push_back(region.id);
      continue;
    }
    for (ReconstructedFace& face : faces) {
      if (!face.loops.empty() && face.loops.front().size() >= 3) {
        result.faces.push_back(std::move(face));
      } else {
        result.omitted_region_ids.push_back(region.id);
      }
    }
  }

  if (result.faces.empty()) {
    result.failure_reasons.push_back("Unable to reconstruct any planar shell faces");
    result.outcome = ReconstructionOutcome::AnalysisOnly;
    return result;
  }

  refine_face_loops_with_shared_vertices(result.faces, result.vertices, tolerances.collinear_distance,
                                         result.edge_split_insertions);
  for (ReconstructedFace& face : result.faces) {
    if (face.loops.empty()) {
      continue;
    }
    orient_loop_to_sign(face.loops.front(), result.vertices, face.fit.normal, 1.0);
    for (std::size_t loop_index = 1; loop_index < face.loops.size(); ++loop_index) {
      orient_loop_to_sign(face.loops[loop_index], result.vertices, face.fit.normal, -1.0);
    }
    face.area = face_total_area(face, result.vertices, face.fit.normal);
  }
  suppress_duplicate_coplanar_faces(result.faces, tolerances);
  prune_dangling_faces(result.faces, tolerances);
  const double shell_cap_area_limit =
      std::min(bounds_diagonal(mesh.bounds) * bounds_diagonal(mesh.bounds) * 0.015,
               tolerances.plane_distance * tolerances.plane_distance * 40.0);
  add_small_planar_gap_caps(result.faces, result.vertices, tolerances, shell_cap_area_limit);
  sanitize_face_loops(result.faces);
  assess_shell_topology(result, mesh, solid_threshold);

  if (result.outcome != ReconstructionOutcome::SolidCreated &&
      result.non_manifold_edge_count > 0 &&
      result.open_edge_count <= 12) {
    resolve_tiny_conflict_faces(result, mesh, tolerances, solid_threshold);
  }

  if (result.outcome != ReconstructionOutcome::SolidCreated &&
      result.non_manifold_edge_count == 0 &&
      result.open_edge_count > 0 &&
      result.faces.size() >= 32 &&
      result.open_edge_count <= 64) {
    resolve_tiny_triangle_gaps(result, mesh, tolerances, solid_threshold);
  }

  if (result.outcome != ReconstructionOutcome::SolidCreated &&
      (result.open_edge_count + result.non_manifold_edge_count) > 0 &&
      result.faces.size() >= 32 &&
      (result.open_edge_count + result.non_manifold_edge_count) <= 64) {
    std::vector<std::vector<LoopCandidate>> candidates;
    candidates.reserve(regions.size());
    std::vector<std::size_t> selected_indices;
    selected_indices.reserve(regions.size());
    for (const PlaneRegion& region : regions) {
      candidates.push_back(collect_region_loop_candidates(region, boundary_edges, mesh,
                                                          original_to_snapped, result.vertices,
                                                          tolerances));
      selected_indices.push_back(choose_initial_candidate_index(candidates.back()));
    }

    optimize_region_loop_choices(result, mesh, regions, candidates, selected_indices, tolerances,
                                 solid_threshold);
    ReconstructionResult candidate_result;
    candidate_result.vertices = result.vertices;
    build_faces_from_selected_candidates(regions, candidates, selected_indices, candidate_result.vertices,
                                         candidate_result);
    finalize_shell_reconstruction(candidate_result, mesh, tolerances, solid_threshold);
    if (is_better_reconstruction_result(candidate_result, result)) {
      result = std::move(candidate_result);
    }
  }
  return result;
}
