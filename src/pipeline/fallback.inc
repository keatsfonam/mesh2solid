void weld_reconstructed_vertices(ReconstructionResult& result,
                                 const MeshModel& mesh,
                                 const Tolerances& tolerances) {
  if (result.vertices.empty()) {
    return;
  }

  const double diagonal = std::max(bounds_diagonal(mesh.bounds), 1e-6);
  const double weld_tolerance =
      std::max(diagonal * 1e-9, std::min(tolerances.vertex_weld * 1e-4, 1e-7));
  auto quantize = [weld_tolerance](double value) -> long long {
    return static_cast<long long>(std::llround(value / weld_tolerance));
  };

  std::vector<Vec3> welded_vertices;
  welded_vertices.reserve(result.vertices.size());
  std::vector<int> remap(result.vertices.size(), -1);
  std::unordered_map<GridKey, std::vector<int>, GridKeyHash> buckets;

  for (std::size_t vertex_index = 0; vertex_index < result.vertices.size(); ++vertex_index) {
    const Vec3& point = result.vertices[vertex_index];
    const GridKey key {quantize(point.x), quantize(point.y), quantize(point.z)};
    const std::optional<int> found =
        find_nearby_grid_vertex(buckets, welded_vertices, key, point, weld_tolerance);
    if (found.has_value()) {
      remap[vertex_index] = *found;
      continue;
    }

    const int welded_index = static_cast<int>(welded_vertices.size());
    welded_vertices.push_back(point);
    buckets[key].push_back(welded_index);
    remap[vertex_index] = welded_index;
  }

  if (welded_vertices.size() == result.vertices.size()) {
    return;
  }

  for (ReconstructedFace& face : result.faces) {
    std::vector<std::vector<int>> welded_loops;
    welded_loops.reserve(face.loops.size());
    for (std::vector<int>& loop : face.loops) {
      for (int& vertex_index : loop) {
        vertex_index = remap[vertex_index];
      }
      std::vector<int> cleaned = sanitize_loop_indices(std::move(loop));
      if (cleaned.size() >= 3) {
        welded_loops.push_back(std::move(cleaned));
      }
    }
    face.loops = std::move(welded_loops);
  }

  result.vertices = std::move(welded_vertices);
}

struct FacetedTriangleGeometry {
  Vec3 a {};
  Vec3 b {};
  Vec3 c {};
  Vec3 normal {};
  double area {};
};

std::optional<FacetedTriangleGeometry> compute_faceted_triangle_geometry(const MeshModel& mesh,
                                                                         const Triangle& triangle) {
  if (triangle.vertices[0] == triangle.vertices[1] || triangle.vertices[1] == triangle.vertices[2] ||
      triangle.vertices[2] == triangle.vertices[0]) {
    return std::nullopt;
  }

  const Vec3& a = mesh.vertices[triangle.vertices[0]];
  const Vec3& b = mesh.vertices[triangle.vertices[1]];
  const Vec3& c = mesh.vertices[triangle.vertices[2]];
  const double area = triangle_area(a, b, c);
  if (area <= 1e-12) {
    return std::nullopt;
  }

  const Vec3 normal = triangle_normal(a, b, c);
  if (length_squared(normal) <= 1e-12) {
    return std::nullopt;
  }

  return FacetedTriangleGeometry {a, b, c, normal, area};
}

ReconstructionResult reconstruct_internal_faceted_mesh_fallback(const MeshModel& mesh,
                                                                const Tolerances& tolerances,
                                                                double solid_threshold) {
  ReconstructionResult result;
  result.method = ReconstructionMethod::FacetedMeshFallback;
  result.vertices = mesh.vertices;
  result.faces.reserve(mesh.triangles.size());

  for (const Triangle& triangle : mesh.triangles) {
    const std::optional<FacetedTriangleGeometry> geometry =
        compute_faceted_triangle_geometry(mesh, triangle);
    if (!geometry.has_value()) {
      continue;
    }

    ReconstructedFace face;
    face.region_id = triangle.region_id;
    face.loops.push_back({triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]});

    Vec3 centroid {};
    for (int vertex_index : triangle.vertices) {
      centroid += mesh.vertices[vertex_index];
    }
    centroid = centroid / 3.0;

    face.fit.normal = geometry->normal;
    face.fit.centroid = centroid;
    face.fit.d = -dot(face.fit.normal, face.fit.centroid);
    face.fit.rms_error = 0.0;
    face.fit.max_error = 0.0;
    face.fit.confidence = 1.0;
    face.confidence = 1.0;
    face.area = geometry->area;
    result.faces.push_back(std::move(face));
  }

  weld_reconstructed_vertices(result, mesh, tolerances);
  sanitize_face_loops(result.faces);
  assess_shell_topology(result, mesh, solid_threshold);
  return result;
}

std::vector<std::vector<int>> collect_triangle_components(const MeshModel& mesh) {
  std::vector<std::vector<int>> components;
  if (mesh.triangles.empty()) {
    return components;
  }

  const AdjacencyData adjacency = build_adjacency(mesh);
  std::vector<bool> visited(mesh.triangles.size(), false);

  for (std::size_t seed = 0; seed < mesh.triangles.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }

    components.push_back({});
    std::queue<int> frontier;
    frontier.push(static_cast<int>(seed));
    visited[seed] = true;

    while (!frontier.empty()) {
      const int triangle_index = frontier.front();
      frontier.pop();
      components.back().push_back(triangle_index);

      for (int neighbor : adjacency.triangle_graph[triangle_index]) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          frontier.push(neighbor);
        }
      }
    }
  }

  return components;
}

MeshModel extract_triangle_component_mesh(const MeshModel& mesh,
                                         const std::vector<int>& triangle_indices) {
  MeshModel component;
  component.source_format = mesh.source_format;
  component.vertices.reserve(triangle_indices.size());
  component.triangles.reserve(triangle_indices.size());

  std::unordered_map<int, int> remap;
  remap.reserve(triangle_indices.size() * 3);

  for (int triangle_index : triangle_indices) {
    Triangle triangle = mesh.triangles[triangle_index];
    for (int& vertex_index : triangle.vertices) {
      const auto [it, inserted] =
          remap.emplace(vertex_index, static_cast<int>(component.vertices.size()));
      if (inserted) {
        component.vertices.push_back(mesh.vertices[vertex_index]);
      }
      vertex_index = it->second;
    }
    component.triangles.push_back(std::move(triangle));
  }

  recompute_mesh_geometry(component);
  return component;
}

struct CircleFit2D {
  Vec2 center {};
  double radius {};
  double rms_error {};
  double max_error {};
};

struct SimpleCylinderPrimitive {
  Vec3 base_center {};
  Vec3 axis {};
  Vec3 ref_direction {};
  double radius {};
  double height {};
};

struct SimpleConicalFrustumPrimitive {
  Vec3 base_center {};
  Vec3 axis {};
  Vec3 ref_direction {};
  double base_radius {};
  double top_radius {};
  double height {};
};

bool solve_linear_3x3(double matrix[3][4], std::array<double, 3>& solution) {
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    double pivot_value = std::abs(matrix[pivot][column]);
    for (int row = column + 1; row < 3; ++row) {
      const double candidate = std::abs(matrix[row][column]);
      if (candidate > pivot_value) {
        pivot = row;
        pivot_value = candidate;
      }
    }
    if (pivot_value <= 1e-12) {
      return false;
    }
    if (pivot != column) {
      for (int entry = column; entry < 4; ++entry) {
        std::swap(matrix[column][entry], matrix[pivot][entry]);
      }
    }

    const double divisor = matrix[column][column];
    for (int entry = column; entry < 4; ++entry) {
      matrix[column][entry] /= divisor;
    }

    for (int row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      if (std::abs(factor) <= 1e-12) {
        continue;
      }
      for (int entry = column; entry < 4; ++entry) {
        matrix[row][entry] -= factor * matrix[column][entry];
      }
    }
  }

  solution = {matrix[0][3], matrix[1][3], matrix[2][3]};
  return true;
}

std::optional<CircleFit2D> fit_circle_2d(const std::vector<Vec2>& points) {
  if (points.size() < 3) {
    return std::nullopt;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_yy = 0.0;
  double sum_xy = 0.0;
  double sum_xxx = 0.0;
  double sum_yyy = 0.0;
  double sum_xxy = 0.0;
  double sum_xyy = 0.0;

  for (const Vec2& point : points) {
    const double x = point.x;
    const double y = point.y;
    const double xx = x * x;
    const double yy = y * y;
    sum_x += x;
    sum_y += y;
    sum_xx += xx;
    sum_yy += yy;
    sum_xy += x * y;
    sum_xxx += xx * x;
    sum_yyy += yy * y;
    sum_xxy += xx * y;
    sum_xyy += x * yy;
  }

  double system[3][4] = {
      {sum_xx, sum_xy, sum_x, -(sum_xxx + sum_xyy)},
      {sum_xy, sum_yy, sum_y, -(sum_xxy + sum_yyy)},
      {sum_x, sum_y, static_cast<double>(points.size()), -(sum_xx + sum_yy)},
  };
  std::array<double, 3> solution {};
  if (!solve_linear_3x3(system, solution)) {
    return std::nullopt;
  }

  const double a = solution[0];
  const double b = solution[1];
  const double c = solution[2];
  const Vec2 center {-0.5 * a, -0.5 * b};
  const double radius_squared = center.x * center.x + center.y * center.y - c;
  if (radius_squared <= 1e-12) {
    return std::nullopt;
  }

  CircleFit2D fit;
  fit.center = center;
  fit.radius = std::sqrt(radius_squared);
  double error_sum = 0.0;
  for (const Vec2& point : points) {
    const double dx = point.x - fit.center.x;
    const double dy = point.y - fit.center.y;
    const double error = std::abs(std::sqrt(dx * dx + dy * dy) - fit.radius);
    error_sum += error * error;
    fit.max_error = std::max(fit.max_error, error);
  }
  fit.rms_error = std::sqrt(error_sum / static_cast<double>(points.size()));
  return fit;
}

std::optional<SimpleCylinderPrimitive> detect_simple_cylinder_component(
    const MeshModel& mesh,
    const Tolerances& tolerances) {
  if (mesh.triangles.size() < 16 || mesh.vertices.size() < 10 || !mesh.bounds.valid) {
    return std::nullopt;
  }

  const double total_area = std::accumulate(
      mesh.triangles.begin(), mesh.triangles.end(), 0.0,
      [](double sum, const Triangle& triangle) { return sum + triangle.area; });
  if (total_area <= 1e-9) {
    return std::nullopt;
  }

  const double cap_angle_degrees = std::max(8.0, tolerances.parallel_angle_degrees * 1.5);
  const double cap_alignment = std::cos(cap_angle_degrees * kPi / 180.0);
  const double side_alignment_limit = std::sin(cap_angle_degrees * kPi / 180.0);

  Vec3 axis {};
  double best_score = -1.0;
  for (const Triangle& seed : mesh.triangles) {
    const Vec3 seed_normal = normalized(seed.normal);
    if (length_squared(seed_normal) <= 1e-12) {
      continue;
    }

    double positive_cap_area = 0.0;
    double negative_cap_area = 0.0;
    double side_area = 0.0;
    for (const Triangle& triangle : mesh.triangles) {
      const double alignment = dot(seed_normal, normalized(triangle.normal));
      if (alignment >= cap_alignment) {
        positive_cap_area += triangle.area;
      } else if (alignment <= -cap_alignment) {
        negative_cap_area += triangle.area;
      }
      if (std::abs(alignment) <= side_alignment_limit) {
        side_area += triangle.area;
      }
    }

    if (positive_cap_area <= 0.0 || negative_cap_area <= 0.0 || side_area <= 0.0) {
      continue;
    }

    const double score = std::min(positive_cap_area, negative_cap_area) + side_area * 0.25;
    if (score > best_score) {
      best_score = score;
      axis = seed_normal;
    }
  }

  if (best_score <= 0.0) {
    return std::nullopt;
  }

  std::vector<int> positive_caps;
  std::vector<int> negative_caps;
  std::vector<int> side_triangles;
  positive_caps.reserve(mesh.triangles.size());
  negative_caps.reserve(mesh.triangles.size());
  side_triangles.reserve(mesh.triangles.size());

  double positive_cap_area = 0.0;
  double negative_cap_area = 0.0;
  double side_area = 0.0;
  for (std::size_t triangle_index = 0; triangle_index < mesh.triangles.size(); ++triangle_index) {
    const Triangle& triangle = mesh.triangles[triangle_index];
    const double alignment = dot(axis, normalized(triangle.normal));
    if (alignment >= cap_alignment) {
      positive_caps.push_back(static_cast<int>(triangle_index));
      positive_cap_area += triangle.area;
    } else if (alignment <= -cap_alignment) {
      negative_caps.push_back(static_cast<int>(triangle_index));
      negative_cap_area += triangle.area;
    } else if (std::abs(alignment) <= side_alignment_limit) {
      side_triangles.push_back(static_cast<int>(triangle_index));
      side_area += triangle.area;
    }
  }

  if (positive_caps.empty() || negative_caps.empty() || side_triangles.size() < 16) {
    return std::nullopt;
  }
  if (positive_cap_area < total_area * 0.08 || negative_cap_area < total_area * 0.08 ||
      side_area < total_area * 0.30) {
    return std::nullopt;
  }

  auto weighted_plane_coordinate = [&mesh, &axis](const std::vector<int>& triangle_indices) {
    double coordinate_sum = 0.0;
    double weight_sum = 0.0;
    for (int triangle_index : triangle_indices) {
      const Triangle& triangle = mesh.triangles[triangle_index];
      Vec3 centroid {};
      for (int vertex_index : triangle.vertices) {
        centroid += mesh.vertices[vertex_index];
      }
      centroid = centroid / 3.0;
      coordinate_sum += dot(centroid, axis) * triangle.area;
      weight_sum += triangle.area;
    }
    return weight_sum <= 1e-12 ? 0.0 : coordinate_sum / weight_sum;
  };

  double positive_plane = weighted_plane_coordinate(positive_caps);
  double negative_plane = weighted_plane_coordinate(negative_caps);
  if (positive_plane < negative_plane) {
    axis = axis * -1.0;
    positive_plane *= -1.0;
    negative_plane *= -1.0;
  }

  const PlaneBasis basis = basis_from_normal(axis);
  std::set<int> side_vertex_indices;
  for (int triangle_index : side_triangles) {
    for (int vertex_index : mesh.triangles[triangle_index].vertices) {
      side_vertex_indices.insert(vertex_index);
    }
  }
  if (side_vertex_indices.size() < 12) {
    return std::nullopt;
  }

  std::vector<Vec2> radial_points;
  radial_points.reserve(side_vertex_indices.size());
  for (int vertex_index : side_vertex_indices) {
    const Vec3& vertex = mesh.vertices[vertex_index];
    radial_points.push_back({dot(vertex, basis.u), dot(vertex, basis.v)});
  }

  const std::optional<CircleFit2D> circle_fit = fit_circle_2d(radial_points);
  if (!circle_fit.has_value() || circle_fit->radius <= 1e-6) {
    return std::nullopt;
  }

  const double diagonal = bounds_diagonal(mesh.bounds);
  const double radial_tolerance =
      std::max({diagonal * 0.01, circle_fit->radius * 0.03, tolerances.plane_distance * 2.0});
  if (circle_fit->rms_error > radial_tolerance * 0.45 ||
      circle_fit->max_error > radial_tolerance) {
    return std::nullopt;
  }

  const double height = positive_plane - negative_plane;
  if (height <= radial_tolerance) {
    return std::nullopt;
  }

  const Vec3 axis_origin = basis.u * circle_fit->center.x + basis.v * circle_fit->center.y;
  const Vec3 base_center = axis_origin + axis * negative_plane;

  auto radial_distance_from_axis = [&](const Vec3& point) {
    const Vec3 relative = point - axis_origin;
    const double axial = dot(relative, axis);
    const Vec3 radial = relative - axis * axial;
    return length(radial);
  };

  Vec3 cap_centroid_sum {};
  std::size_t cap_centroid_count = 0;
  for (int triangle_index : positive_caps) {
    for (int vertex_index : mesh.triangles[triangle_index].vertices) {
      cap_centroid_sum += mesh.vertices[vertex_index];
      ++cap_centroid_count;
    }
  }
  for (int triangle_index : negative_caps) {
    for (int vertex_index : mesh.triangles[triangle_index].vertices) {
      cap_centroid_sum += mesh.vertices[vertex_index];
      ++cap_centroid_count;
    }
  }
  if (cap_centroid_count == 0) {
    return std::nullopt;
  }
  const Vec3 cap_centroid = cap_centroid_sum / static_cast<double>(cap_centroid_count);
  const double centroid_axis_offset = radial_distance_from_axis(cap_centroid);
  if (centroid_axis_offset > radial_tolerance * 2.0) {
    return std::nullopt;
  }

  return SimpleCylinderPrimitive {base_center, axis, basis.u, circle_fit->radius, height};
}

#if defined(MESH2SOLID_WITH_OCCT)
struct OcctFallbackBuild {
  TopoDS_Shape result_shape;
  std::size_t faces_added {};
  std::size_t free_edges {};
  std::size_t multiple_edges {};
  bool valid {};
  std::vector<std::string> notes;
};

double occt_sewing_tolerance(const MeshModel& mesh, const Tolerances& tolerances) {
  const double diagonal = mesh.bounds.valid ? length(mesh.bounds.max - mesh.bounds.min) : 0.0;
  const double scale_floor = diagonal > 1e-6 ? diagonal * 1e-6 : 1e-6;
  return std::max({scale_floor, tolerances.vertex_weld * 2.0, tolerances.plane_merge_distance});
}

std::string occt_failure_message(const Standard_Failure& failure) {
  const char* message = failure.GetMessageString();
  if (message != nullptr && message[0] != '\0') {
    return message;
  }
  return "Standard_Failure";
}

TopoDS_Shape make_occt_compound(const std::vector<TopoDS_Shape>& shapes) {
  if (shapes.empty()) {
    return {};
  }
  if (shapes.size() == 1) {
    return shapes.front();
  }

  TopoDS_Compound compound;
  BRep_Builder builder;
  builder.MakeCompound(compound);
  for (const TopoDS_Shape& shape : shapes) {
    builder.Add(compound, shape);
  }
  return compound;
}

TopoDS_Shape build_occt_solid_from_shell(const TopoDS_Shell& shell, double sewing_tolerance) {
  ShapeFix_Solid solid_fixer;
  solid_fixer.SetPrecision(sewing_tolerance);
  solid_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
  solid_fixer.SetMaxTolerance(sewing_tolerance * 4.0);

  TopoDS_Solid solid = solid_fixer.SolidFromShell(shell);
  if (solid.IsNull()) {
    BRepBuilderAPI_MakeSolid solid_builder(shell);
    if (!solid_builder.IsDone()) {
      return {};
    }
    solid = TopoDS::Solid(solid_builder.Solid());
  }

  BRepLib::OrientClosedSolid(solid);
  solid_fixer.Init(solid);
  solid_fixer.Perform();

  TopoDS_Shape fixed = solid_fixer.Shape();
  if (fixed.IsNull()) {
    return solid;
  }
  if (fixed.ShapeType() == TopAbs_SOLID) {
    TopoDS_Solid oriented = TopoDS::Solid(fixed);
    BRepLib::OrientClosedSolid(oriented);
    return oriented;
  }
  return fixed;
}

TopoDS_Shape promote_occt_shape_to_solid_shape(const TopoDS_Shape& shape,
                                               double sewing_tolerance) {
  if (shape.IsNull()) {
    return {};
  }
  if (shape.ShapeType() == TopAbs_SOLID) {
    return shape;
  }

  std::vector<TopoDS_Shape> solids;
  for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
    solids.push_back(explorer.Current());
  }
  if (!solids.empty()) {
    if (solids.size() == 1) {
      return solids.front();
    }
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const TopoDS_Shape& solid : solids) {
      builder.Add(compound, solid);
    }
    return compound;
  }

  std::vector<TopoDS_Shell> shells;
  for (TopExp_Explorer explorer(shape, TopAbs_SHELL); explorer.More(); explorer.Next()) {
    shells.push_back(TopoDS::Shell(explorer.Current()));
  }
  if (shells.empty()) {
    return {};
  }
  if (shells.size() == 1) {
    return build_occt_solid_from_shell(shells.front(), sewing_tolerance);
  }

  TopoDS_Compound compound;
  BRep_Builder builder;
  builder.MakeCompound(compound);
  for (const TopoDS_Shell& shell : shells) {
    TopoDS_Shape solid = build_occt_solid_from_shell(shell, sewing_tolerance);
    if (solid.IsNull()) {
      return {};
    }
    builder.Add(compound, solid);
  }
  return compound;
}

bool occt_shape_is_valid_solid(const TopoDS_Shape& shape) {
  if (shape.IsNull()) {
    return false;
  }
  if (shape.ShapeType() == TopAbs_SOLID) {
    BRepCheck_Analyzer analyzer(shape);
    return analyzer.IsValid();
  }
  if (shape.ShapeType() != TopAbs_COMPOUND) {
    return false;
  }

  bool saw_solid = false;
  for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
    saw_solid = true;
    BRepCheck_Analyzer analyzer(explorer.Current());
    if (!analyzer.IsValid()) {
      return false;
    }
  }
  return saw_solid;
}

std::string occt_status_name(BRepCheck_Status status) {
  switch (status) {
    case BRepCheck_NoError:
      return "NoError";
    case BRepCheck_InvalidPointOnCurve:
      return "InvalidPointOnCurve";
    case BRepCheck_InvalidPointOnCurveOnSurface:
      return "InvalidPointOnCurveOnSurface";
    case BRepCheck_InvalidPointOnSurface:
      return "InvalidPointOnSurface";
    case BRepCheck_No3DCurve:
      return "No3DCurve";
    case BRepCheck_Multiple3DCurve:
      return "Multiple3DCurve";
    case BRepCheck_Invalid3DCurve:
      return "Invalid3DCurve";
    case BRepCheck_NoCurveOnSurface:
      return "NoCurveOnSurface";
    case BRepCheck_InvalidCurveOnSurface:
      return "InvalidCurveOnSurface";
    case BRepCheck_InvalidCurveOnClosedSurface:
      return "InvalidCurveOnClosedSurface";
    case BRepCheck_InvalidSameRangeFlag:
      return "InvalidSameRangeFlag";
    case BRepCheck_InvalidSameParameterFlag:
      return "InvalidSameParameterFlag";
    case BRepCheck_InvalidDegeneratedFlag:
      return "InvalidDegeneratedFlag";
    case BRepCheck_FreeEdge:
      return "FreeEdge";
    case BRepCheck_InvalidMultiConnexity:
      return "InvalidMultiConnexity";
    case BRepCheck_InvalidRange:
      return "InvalidRange";
    case BRepCheck_EmptyWire:
      return "EmptyWire";
    case BRepCheck_RedundantEdge:
      return "RedundantEdge";
    case BRepCheck_SelfIntersectingWire:
      return "SelfIntersectingWire";
    case BRepCheck_NoSurface:
      return "NoSurface";
    case BRepCheck_InvalidWire:
      return "InvalidWire";
    case BRepCheck_RedundantWire:
      return "RedundantWire";
    case BRepCheck_IntersectingWires:
      return "IntersectingWires";
    case BRepCheck_InvalidImbricationOfWires:
      return "InvalidImbricationOfWires";
    case BRepCheck_EmptyShell:
      return "EmptyShell";
    case BRepCheck_RedundantFace:
      return "RedundantFace";
    case BRepCheck_InvalidImbricationOfShells:
      return "InvalidImbricationOfShells";
    case BRepCheck_UnorientableShape:
      return "UnorientableShape";
    case BRepCheck_NotClosed:
      return "NotClosed";
    case BRepCheck_NotConnected:
      return "NotConnected";
    case BRepCheck_SubshapeNotInShape:
      return "SubshapeNotInShape";
    case BRepCheck_BadOrientation:
      return "BadOrientation";
    case BRepCheck_BadOrientationOfSubshape:
      return "BadOrientationOfSubshape";
    case BRepCheck_InvalidPolygonOnTriangulation:
      return "InvalidPolygonOnTriangulation";
    case BRepCheck_InvalidToleranceValue:
      return "InvalidToleranceValue";
    case BRepCheck_EnclosedRegion:
      return "EnclosedRegion";
    case BRepCheck_CheckFail:
      return "CheckFail";
  }
  return "UnknownStatus";
}

std::unordered_map<std::string, int> collect_occt_validation_counts(const TopoDS_Shape& shape) {
  std::unordered_map<std::string, int> counts;
  if (shape.IsNull()) {
    return counts;
  }

  auto accumulate_statuses = [&counts](const TopoDS_Shape& subject) {
    BRepCheck_Analyzer analyzer(subject);
    if (analyzer.IsValid()) {
      return;
    }

    auto add_result_statuses = [&counts](const Handle(BRepCheck_Result)& result) {
      if (result.IsNull()) {
        return;
      }
      for (const BRepCheck_Status status : result->Status()) {
        if (status != BRepCheck_NoError) {
          ++counts[occt_status_name(status)];
        }
      }
      result->InitContextIterator();
      while (result->MoreShapeInContext()) {
        for (const BRepCheck_Status status : result->StatusOnShape()) {
          if (status != BRepCheck_NoError) {
            ++counts[occt_status_name(status)];
          }
        }
        result->NextShapeInContext();
      }
    };

    add_result_statuses(analyzer.Result(subject));
    for (TopExp_Explorer explorer(subject, TopAbs_FACE); explorer.More(); explorer.Next()) {
      add_result_statuses(analyzer.Result(explorer.Current()));
    }
    for (TopExp_Explorer explorer(subject, TopAbs_WIRE); explorer.More(); explorer.Next()) {
      add_result_statuses(analyzer.Result(explorer.Current()));
    }
    for (TopExp_Explorer explorer(subject, TopAbs_EDGE); explorer.More(); explorer.Next()) {
      add_result_statuses(analyzer.Result(explorer.Current()));
    }
  };

  if (shape.ShapeType() == TopAbs_SOLID) {
    accumulate_statuses(shape);
  } else {
    for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
      accumulate_statuses(explorer.Current());
    }
  }
  return counts;
}

std::string occt_validation_summary(const TopoDS_Shape& shape) {
  if (shape.IsNull()) {
    return "null_shape";
  }

  const std::unordered_map<std::string, int> counts = collect_occt_validation_counts(shape);
  if (counts.empty()) {
    return "no_reported_statuses";
  }

  std::vector<std::pair<std::string, int>> ordered(counts.begin(), counts.end());
  std::sort(ordered.begin(), ordered.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.second != rhs.second) {
      return lhs.second > rhs.second;
    }
    return lhs.first < rhs.first;
  });

  std::ostringstream output;
  for (std::size_t index = 0; index < ordered.size() && index < 4; ++index) {
    if (index > 0) {
      output << ", ";
    }
    output << ordered[index].first << "=" << ordered[index].second;
  }
  return output.str();
}

std::size_t occt_validation_issue_count(const TopoDS_Shape& shape) {
  const std::unordered_map<std::string, int> counts = collect_occt_validation_counts(shape);
  std::size_t total = 0;
  for (const auto& [_, count] : counts) {
    total += static_cast<std::size_t>(std::max(count, 0));
  }
  return total;
}

TopoDS_Shape heal_occt_sewed_shells(const TopoDS_Shape& shape, double sewing_tolerance) {
  if (shape.IsNull()) {
    return {};
  }

  std::vector<TopoDS_Shape> healed_shells;
  for (TopExp_Explorer explorer(shape, TopAbs_SHELL); explorer.More(); explorer.Next()) {
    ShapeFix_Shell shell_fixer(TopoDS::Shell(explorer.Current()));
    shell_fixer.SetPrecision(sewing_tolerance);
    shell_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
    shell_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
    shell_fixer.SetNonManifoldFlag(Standard_False);
    shell_fixer.Perform();

    TopoDS_Shape fixed_shell = shell_fixer.Shape();
    if (fixed_shell.IsNull()) {
      fixed_shell = shell_fixer.Shell();
    }
    if (fixed_shell.IsNull()) {
      fixed_shell = explorer.Current();
    }
    healed_shells.push_back(fixed_shell);
  }

  if (healed_shells.empty()) {
    return shape;
  }

  TopoDS_Shape healed = make_occt_compound(healed_shells);
  ShapeFix_Shape shape_fixer(healed);
  shape_fixer.SetPrecision(sewing_tolerance);
  shape_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
  shape_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
  shape_fixer.Perform();

  TopoDS_Shape fixed = shape_fixer.Shape();
  if (!fixed.IsNull()) {
    healed = fixed;
  }

  BRepLib::BuildCurves3d(healed, std::max(1e-7, sewing_tolerance));
  return healed;
}

TopoDS_Shape fix_occt_promoted_solids(const TopoDS_Shape& shape, double sewing_tolerance) {
  if (shape.IsNull()) {
    return {};
  }

  std::vector<TopoDS_Shape> solids;
  if (shape.ShapeType() == TopAbs_SOLID) {
    solids.push_back(shape);
  } else {
    for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
      solids.push_back(explorer.Current());
    }
  }
  if (solids.empty()) {
    return shape;
  }

  std::vector<TopoDS_Shape> fixed_solids;
  fixed_solids.reserve(solids.size());
  for (const TopoDS_Shape& solid_shape : solids) {
    ShapeFix_Solid solid_fixer;
    solid_fixer.SetPrecision(sewing_tolerance);
    solid_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
    solid_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
    solid_fixer.Init(TopoDS::Solid(solid_shape));
    solid_fixer.Perform();

    TopoDS_Shape fixed = solid_fixer.Shape();
    if (fixed.IsNull()) {
      fixed = solid_shape;
    }
    if (fixed.ShapeType() == TopAbs_SOLID) {
      TopoDS_Solid oriented = TopoDS::Solid(fixed);
      BRepLib::OrientClosedSolid(oriented);
      fixed = oriented;
    }
    fixed_solids.push_back(fixed);
  }

  return make_occt_compound(fixed_solids);
}

TopoDS_Shape finalize_occt_solid_candidate(const TopoDS_Shape& shape, double sewing_tolerance) {
  if (shape.IsNull()) {
    return {};
  }

  TopoDS_Shape finalized = fix_occt_promoted_solids(shape, sewing_tolerance);
  if (finalized.IsNull()) {
    return {};
  }

  ShapeFix_Shape shape_fixer(finalized);
  shape_fixer.SetPrecision(sewing_tolerance);
  shape_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
  shape_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
  shape_fixer.Perform();

  TopoDS_Shape fixed = shape_fixer.Shape();
  if (!fixed.IsNull()) {
    finalized = fixed;
  }

  ShapeFix_ShapeTolerance tolerance_fixer;
  tolerance_fixer.LimitTolerance(finalized, std::max(1e-7, sewing_tolerance * 0.25),
                                 sewing_tolerance * 4.0, TopAbs_SHAPE);
  BRepLib::BuildCurves3d(finalized, std::max(1e-7, sewing_tolerance));
  BRepLib::SameParameter(finalized, std::max(1e-7, sewing_tolerance), Standard_True);
  BRepLib::UpdateTolerances(finalized);
  return finalized;
}

bool should_unify_same_domain(const MeshModel& mesh) {
  return mesh.triangles.size() <= 5000;
}

OcctFallbackBuild build_occt_faceted_shape_single(const MeshModel& mesh,
                                                  const Tolerances& tolerances) {
  OcctFallbackBuild build;
  try {
    const double sewing_tolerance = occt_sewing_tolerance(mesh, tolerances);
    BRepBuilderAPI_Sewing sewing(sewing_tolerance, Standard_True, Standard_True, Standard_True,
                                 Standard_False);

    for (const Triangle& triangle : mesh.triangles) {
      if (!compute_faceted_triangle_geometry(mesh, triangle).has_value()) {
        continue;
      }

      BRepBuilderAPI_MakePolygon polygon;
      for (int vertex_index : triangle.vertices) {
        const Vec3& vertex = mesh.vertices[vertex_index];
        polygon.Add(gp_Pnt(vertex.x, vertex.y, vertex.z));
      }
      polygon.Close();
      if (!polygon.IsDone()) {
        build.notes.push_back("Open CASCADE failed to build a triangle boundary wire");
        continue;
      }

      BRepBuilderAPI_MakeFace face_builder(polygon.Wire(), Standard_True);
      if (!face_builder.IsDone()) {
        build.notes.push_back("Open CASCADE failed to build a planar triangle face");
        continue;
      }

      sewing.Add(face_builder.Face());
      ++build.faces_added;
    }

    if (build.faces_added == 0) {
      build.notes.push_back("Open CASCADE could not construct any fallback faces");
      return build;
    }

    sewing.Perform();
    build.free_edges = static_cast<std::size_t>(sewing.NbFreeEdges());
    build.multiple_edges = static_cast<std::size_t>(sewing.NbMultipleEdges());

    TopoDS_Shape repaired = sewing.SewedShape();
    if (repaired.IsNull()) {
      build.notes.push_back("Open CASCADE sewing produced no output shape");
      return build;
    }

    repaired = heal_occt_sewed_shells(repaired, sewing_tolerance);

    TopoDS_Shape solid_candidate = promote_occt_shape_to_solid_shape(repaired, sewing_tolerance);
    if (solid_candidate.IsNull()) {
      build.notes.push_back("Open CASCADE could not promote the sewed shell(s) to solid bodies");
      return build;
    }

    solid_candidate = finalize_occt_solid_candidate(solid_candidate, sewing_tolerance);
    if (solid_candidate.IsNull()) {
      build.notes.push_back("Open CASCADE solid normalization failed after shell promotion");
      return build;
    }

    TopoDS_Shape best_candidate = solid_candidate;
    bool best_valid = occt_shape_is_valid_solid(best_candidate);
    std::size_t best_issues = best_valid ? 0 : occt_validation_issue_count(best_candidate);

    if (should_unify_same_domain(mesh)) {
      ShapeUpgrade_UnifySameDomain unify(solid_candidate, Standard_True, Standard_True,
                                         Standard_False);
      unify.SetLinearTolerance(sewing_tolerance);
      unify.SetAngularTolerance(std::max(1.0, tolerances.merge_angle_degrees) * kPi / 180.0);
      unify.Build();
      TopoDS_Shape unified_candidate =
          promote_occt_shape_to_solid_shape(unify.Shape(), sewing_tolerance);
      if (unified_candidate.IsNull()) {
        build.notes.push_back("Open CASCADE same-domain unification broke solid-body promotion");
        return build;
      }
      unified_candidate = finalize_occt_solid_candidate(unified_candidate, sewing_tolerance);
      if (unified_candidate.IsNull()) {
        build.notes.push_back("Open CASCADE same-domain unification could not normalize the solid");
        return build;
      }

      const bool unified_valid = occt_shape_is_valid_solid(unified_candidate);
      const std::size_t unified_issues =
          unified_valid ? 0 : occt_validation_issue_count(unified_candidate);
      if (unified_valid || (!best_valid && unified_issues < best_issues)) {
        best_candidate = unified_candidate;
        best_valid = unified_valid;
        best_issues = unified_issues;
      }
    }

    build.result_shape = best_candidate;
    build.valid = best_valid;
    if (!build.valid) {
      build.notes.push_back("Open CASCADE produced a solid candidate that failed validation: " +
                            occt_validation_summary(best_candidate));
      return build;
    }
  } catch (const Standard_Failure& failure) {
    build.notes.push_back(std::string("Open CASCADE exception: ") + occt_failure_message(failure));
    return build;
  } catch (const std::exception& exception) {
    build.notes.push_back(std::string("Open CASCADE exception: ") + exception.what());
    return build;
  }
  return build;
}

OcctFallbackBuild build_occt_faceted_shape(const MeshModel& mesh, const Tolerances& tolerances) {
  const std::vector<std::vector<int>> components = collect_triangle_components(mesh);
  if (components.size() <= 1) {
    return build_occt_faceted_shape_single(mesh, tolerances);
  }

  OcctFallbackBuild build;
  std::vector<TopoDS_Shape> solids;
  solids.reserve(components.size());

  for (std::size_t index = 0; index < components.size(); ++index) {
    const MeshModel component_mesh = extract_triangle_component_mesh(mesh, components[index]);
    OcctFallbackBuild component_build = build_occt_faceted_shape_single(component_mesh, tolerances);
    build.faces_added += component_build.faces_added;
    build.free_edges += component_build.free_edges;
    build.multiple_edges += component_build.multiple_edges;

    if (component_build.valid && !component_build.result_shape.IsNull()) {
      solids.push_back(component_build.result_shape);
      continue;
    }

    if (component_build.notes.empty()) {
      build.notes.push_back("Open CASCADE did not reconstruct mesh component " +
                            std::to_string(index + 1));
      continue;
    }

    for (const std::string& note : component_build.notes) {
      build.notes.push_back("Component " + std::to_string(index + 1) + ": " + note);
    }
  }

  if (solids.size() != components.size()) {
    if (build.notes.empty()) {
      build.notes.push_back("Open CASCADE did not reconstruct all disconnected mesh components");
    }
    return build;
  }

  build.result_shape = make_occt_compound(solids);
  if (build.result_shape.IsNull()) {
    build.notes.push_back("Open CASCADE could not combine component solids");
    return build;
  }

  build.valid = occt_shape_is_valid_solid(build.result_shape);
  if (!build.valid) {
    build.notes.push_back("Open CASCADE combined component solids failed validation: " +
                          occt_validation_summary(build.result_shape));
  }
  return build;
}

ReconstructionResult reconstruct_occt_faceted_mesh_fallback(const MeshModel& mesh,
                                                            const Tolerances& tolerances,
                                                            double solid_threshold,
                                                            ReconstructionResult result) {
  if (result.faces.empty()) {
    result = reconstruct_internal_faceted_mesh_fallback(mesh, tolerances, solid_threshold);
  }
  result.method = ReconstructionMethod::OcctFacetedMeshFallback;

  const OcctFallbackBuild build = build_occt_faceted_shape(mesh, tolerances);
  result.open_edge_count = build.free_edges;
  result.non_manifold_edge_count = build.multiple_edges;
  result.shell_gap_score =
      build.faces_added == 0
          ? 1.0
          : static_cast<double>(build.free_edges + build.multiple_edges * 2) /
                static_cast<double>(build.faces_added);

  if (build.valid) {
    result.outcome = ReconstructionOutcome::SolidCreated;
    result.open_edge_count = 0;
    result.non_manifold_edge_count = 0;
    result.shell_gap_score = 0.0;
    result.failure_reasons.clear();
    result.confidence = std::max(result.confidence, std::max(solid_threshold, 0.92));
    return result;
  }

  result.failure_reasons = build.notes;
  if (result.failure_reasons.empty()) {
    result.failure_reasons.push_back(
        "Open CASCADE faceted fallback did not produce a valid solid");
  }
  if (build.free_edges == 0 && build.multiple_edges == 0) {
    result.outcome = ReconstructionOutcome::ShellOnly;
    result.confidence = std::max(result.confidence, solid_threshold * 0.7);
  }
  return result;
}

bool write_step_file_occt(const std::filesystem::path& path,
                          const MeshModel& mesh,
                          const Tolerances& tolerances) {
  const OcctFallbackBuild build = build_occt_faceted_shape(mesh, tolerances);
  if (!build.valid || build.result_shape.IsNull()) {
    return false;
  }

  try {
    STEPControl_Writer writer;
    writer.SetTolerance(occt_sewing_tolerance(mesh, tolerances));
    const STEPControl_StepModelType mode =
        build.result_shape.ShapeType() == TopAbs_SOLID ? STEPControl_ManifoldSolidBrep
                                                       : STEPControl_AsIs;
    if (writer.Transfer(build.result_shape, mode) != IFSelect_RetDone) {
      return false;
    }

    const std::string path_string = path.string();
    return writer.Write(path_string.c_str()) == IFSelect_RetDone;
  } catch (const Standard_Failure&) {
    return false;
  }
}
#else
bool write_step_file_occt(const std::filesystem::path&,
                          const MeshModel&,
                          const Tolerances&) {
  return false;
}
#endif

ReconstructionResult reconstruct_faceted_mesh_fallback(const MeshModel& mesh,
                                                       const Tolerances& tolerances,
                                                       double solid_threshold,
                                                       bool allow_internal_fallback) {
  ReconstructionResult result;
  if (allow_internal_fallback) {
    result = reconstruct_internal_faceted_mesh_fallback(mesh, tolerances, solid_threshold);
  }
#if defined(MESH2SOLID_WITH_OCCT)
  if (result.outcome == ReconstructionOutcome::SolidCreated) {
    return result;
  }
  ReconstructionResult occt_result =
      reconstruct_occt_faceted_mesh_fallback(mesh, tolerances, solid_threshold, result);
  if (is_better_reconstruction_result(occt_result, result)) {
    result = std::move(occt_result);
  }
#else
  (void)tolerances;
#endif
  return result;
}
