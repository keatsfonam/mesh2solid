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

bool occt_build_is_cautiously_exportable(const OcctFallbackBuild& build) {
  if (build.valid || build.result_shape.IsNull() || build.free_edges != 0 ||
      build.multiple_edges != 0 || build.notes.empty()) {
    return false;
  }

  for (const std::string& note : build.notes) {
    if (note.find("failed validation") == std::string::npos) {
      return false;
    }
  }
  return true;
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

    const bool component_exportable =
        (component_build.valid || occt_build_is_cautiously_exportable(component_build)) &&
        !component_build.result_shape.IsNull();
    if (component_exportable) {
      solids.push_back(component_build.result_shape);
      if (!component_build.valid) {
        for (const std::string& note : component_build.notes) {
          build.notes.push_back("Component " + std::to_string(index + 1) + ": " + note);
        }
      }
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

  if (build.valid || occt_build_is_cautiously_exportable(build)) {
    result.outcome = ReconstructionOutcome::SolidCreated;
    result.open_edge_count = 0;
    result.non_manifold_edge_count = 0;
    result.shell_gap_score = 0.0;
    if (build.valid) {
      result.failure_reasons.clear();
      result.confidence = std::max(result.confidence, std::max(solid_threshold, 0.92));
    } else {
      result.failure_reasons = {
          "Open CASCADE reported validation issues, but the fallback solid candidate was closed and was exported cautiously"};
      result.failure_reasons.insert(result.failure_reasons.end(), build.notes.begin(), build.notes.end());
      result.confidence = std::max(result.confidence, solid_threshold);
    }
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
  if ((!build.valid && !occt_build_is_cautiously_exportable(build)) || build.result_shape.IsNull()) {
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
