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

    ShapeFix_Shape shape_fixer(repaired);
    shape_fixer.SetPrecision(sewing_tolerance);
    shape_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
    shape_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
    shape_fixer.Perform();
    repaired = shape_fixer.Shape();

    TopoDS_Shape solid_candidate = promote_occt_shape_to_solid_shape(repaired, sewing_tolerance);
    if (solid_candidate.IsNull()) {
      build.notes.push_back("Open CASCADE could not promote the sewed shell(s) to solid bodies");
      return build;
    }

    ShapeFix_Shape solid_fixer(solid_candidate);
    solid_fixer.SetPrecision(sewing_tolerance);
    solid_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
    solid_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
    solid_fixer.Perform();
    solid_candidate = solid_fixer.Shape();
    BRepLib::SameParameter(solid_candidate, std::max(1e-7, sewing_tolerance), Standard_True);
    BRepLib::UpdateTolerances(solid_candidate);

    if (should_unify_same_domain(mesh)) {
      ShapeUpgrade_UnifySameDomain unify(solid_candidate, Standard_True, Standard_True,
                                         Standard_False);
      unify.SetLinearTolerance(sewing_tolerance);
      unify.SetAngularTolerance(std::max(1.0, tolerances.merge_angle_degrees) * kPi / 180.0);
      unify.Build();
      solid_candidate = promote_occt_shape_to_solid_shape(unify.Shape(), sewing_tolerance);
      if (solid_candidate.IsNull()) {
        build.notes.push_back("Open CASCADE same-domain unification broke solid-body promotion");
        return build;
      }
      BRepLib::SameParameter(solid_candidate, std::max(1e-7, sewing_tolerance), Standard_True);
      BRepLib::UpdateTolerances(solid_candidate);
    }

    build.result_shape = solid_candidate;
    build.valid = occt_shape_is_valid_solid(solid_candidate);
    if (!build.valid) {
      build.notes.push_back("Open CASCADE produced a solid candidate that failed validation");
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
    build.notes.push_back("Open CASCADE combined component solids failed validation");
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
