ReconstructionResult reconstruct_internal_faceted_mesh_fallback(const MeshModel& mesh,
                                                                double solid_threshold) {
  ReconstructionResult result;
  result.method = ReconstructionMethod::FacetedMeshFallback;
  result.vertices = mesh.vertices;
  result.faces.reserve(mesh.triangles.size());

  for (const Triangle& triangle : mesh.triangles) {
    if (triangle.area <= 1e-12) {
      continue;
    }

    ReconstructedFace face;
    face.region_id = triangle.region_id;
    face.loops.push_back({triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]});

    Vec3 normal = triangle.normal;
    if (length_squared(normal) <= 1e-12) {
      const Vec3& a = mesh.vertices[triangle.vertices[0]];
      const Vec3& b = mesh.vertices[triangle.vertices[1]];
      const Vec3& c = mesh.vertices[triangle.vertices[2]];
      normal = normalized(cross(b - a, c - a));
    }
    if (length_squared(normal) <= 1e-12) {
      continue;
    }

    Vec3 centroid {};
    for (int vertex_index : triangle.vertices) {
      centroid += mesh.vertices[vertex_index];
    }
    centroid = centroid / 3.0;

    face.fit.normal = normalized(normal);
    face.fit.centroid = centroid;
    face.fit.d = -dot(face.fit.normal, face.fit.centroid);
    face.fit.rms_error = 0.0;
    face.fit.max_error = 0.0;
    face.fit.confidence = 1.0;
    face.confidence = 1.0;
    face.area = triangle.area;
    result.faces.push_back(std::move(face));
  }

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

TopoDS_Shape build_occt_solid_from_shell(const TopoDS_Shell& shell) {
  BRepBuilderAPI_MakeSolid solid_builder(shell);
  if (!solid_builder.IsDone()) {
    return {};
  }
  return solid_builder.Shape();
}

TopoDS_Shape promote_occt_shape_to_solid_shape(const TopoDS_Shape& shape) {
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
    return build_occt_solid_from_shell(shells.front());
  }

  TopoDS_Compound compound;
  BRep_Builder builder;
  builder.MakeCompound(compound);
  for (const TopoDS_Shell& shell : shells) {
    TopoDS_Shape solid = build_occt_solid_from_shell(shell);
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
      if (triangle.area <= 1e-12) {
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

    TopoDS_Shape solid_candidate = promote_occt_shape_to_solid_shape(repaired);
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

    if (should_unify_same_domain(mesh)) {
      ShapeUpgrade_UnifySameDomain unify(solid_candidate, Standard_True, Standard_True,
                                         Standard_False);
      unify.SetLinearTolerance(sewing_tolerance);
      unify.SetAngularTolerance(std::max(1.0, tolerances.merge_angle_degrees) * kPi / 180.0);
      unify.Build();
      solid_candidate = promote_occt_shape_to_solid_shape(unify.Shape());
      if (solid_candidate.IsNull()) {
        build.notes.push_back("Open CASCADE same-domain unification broke solid-body promotion");
        return build;
      }
    }

    build.valid = occt_shape_is_valid_solid(solid_candidate);
    if (!build.valid) {
      build.notes.push_back("Open CASCADE produced a solid candidate that failed validation");
      return build;
    }

    build.result_shape = solid_candidate;
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
    build.notes.push_back("Open CASCADE combined component solids failed validation");
  }
  return build;
}

ReconstructionResult reconstruct_occt_faceted_mesh_fallback(const MeshModel& mesh,
                                                            const Tolerances& tolerances,
                                                            double solid_threshold) {
  ReconstructionResult result = reconstruct_internal_faceted_mesh_fallback(mesh, solid_threshold);
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
    result = reconstruct_internal_faceted_mesh_fallback(mesh, solid_threshold);
  }
#if defined(MESH2SOLID_WITH_OCCT)
  ReconstructionResult occt_result =
      reconstruct_occt_faceted_mesh_fallback(mesh, tolerances, solid_threshold);
  if (is_better_reconstruction_result(occt_result, result)) {
    result = std::move(occt_result);
  }
#else
  (void)tolerances;
#endif
  return result;
}
