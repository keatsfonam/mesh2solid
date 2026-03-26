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
  TopoDS_Shape solid_shape;
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

TopoDS_Shape promote_occt_shape_to_solid(const TopoDS_Shape& shape) {
  if (shape.IsNull()) {
    return {};
  }
  if (shape.ShapeType() == TopAbs_SOLID) {
    return shape;
  }

  TopoDS_Shape first_solid;
  for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
    first_solid = explorer.Current();
    break;
  }
  if (!first_solid.IsNull()) {
    return first_solid;
  }

  TopoDS_Shape first_shell_shape;
  std::size_t shell_count = 0;
  for (TopExp_Explorer explorer(shape, TopAbs_SHELL); explorer.More(); explorer.Next()) {
    if (shell_count == 0) {
      first_shell_shape = explorer.Current();
    }
    ++shell_count;
  }
  if (shell_count != 1 || first_shell_shape.IsNull()) {
    return {};
  }

  BRepBuilderAPI_MakeSolid solid_builder(TopoDS::Shell(first_shell_shape));
  if (!solid_builder.IsDone()) {
    return {};
  }
  return solid_builder.Shape();
}

bool occt_shape_is_valid_solid(const TopoDS_Shape& shape) {
  if (shape.IsNull() || shape.ShapeType() != TopAbs_SOLID) {
    return false;
  }
  BRepCheck_Analyzer analyzer(shape);
  return analyzer.IsValid();
}

OcctFallbackBuild build_occt_faceted_shape(const MeshModel& mesh, const Tolerances& tolerances) {
  OcctFallbackBuild build;
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

  TopoDS_Shape solid_candidate = promote_occt_shape_to_solid(repaired);
  if (solid_candidate.IsNull()) {
    build.notes.push_back("Open CASCADE could not promote the sewed shell to a solid");
    return build;
  }

  ShapeFix_Shape solid_fixer(solid_candidate);
  solid_fixer.SetPrecision(sewing_tolerance);
  solid_fixer.SetMinTolerance(std::max(1e-7, sewing_tolerance * 0.25));
  solid_fixer.SetMaxTolerance(sewing_tolerance * 4.0);
  solid_fixer.Perform();
  solid_candidate = solid_fixer.Shape();

  ShapeUpgrade_UnifySameDomain unify(solid_candidate, Standard_True, Standard_True,
                                     Standard_False);
  unify.SetLinearTolerance(sewing_tolerance);
  unify.SetAngularTolerance(std::max(1.0, tolerances.merge_angle_degrees) * kPi / 180.0);
  unify.Build();
  solid_candidate = promote_occt_shape_to_solid(unify.Shape());
  if (solid_candidate.IsNull()) {
    build.notes.push_back("Open CASCADE same-domain unification broke solid promotion");
    return build;
  }

  build.valid = occt_shape_is_valid_solid(solid_candidate);
  if (!build.valid) {
    build.notes.push_back("Open CASCADE produced a solid candidate that failed validation");
    return build;
  }

  build.solid_shape = solid_candidate;
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
  if (!build.valid || build.solid_shape.IsNull()) {
    return false;
  }

  STEPControl_Writer writer;
  writer.SetTolerance(occt_sewing_tolerance(mesh, tolerances));
  if (writer.Transfer(build.solid_shape, STEPControl_ManifoldSolidBrep) != IFSelect_RetDone) {
    return false;
  }

  const std::string path_string = path.string();
  return writer.Write(path_string.c_str()) == IFSelect_RetDone;
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
                                                       double solid_threshold) {
  ReconstructionResult result = reconstruct_internal_faceted_mesh_fallback(mesh, solid_threshold);
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
