PrismaticAxialSectionColumn make_prismatic_blind_section_column(
    const SimplePrismaticBlindBoreExport& bore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = bore_export.top_inner_area;
  column.bottom_inner_area = bore_export.floor_area;
  column.kind = PrismaticAxialThroughFeatureKind::bore;
  column.bore = bore_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_blind_section_column(
    const SimplePrismaticMultiBlindBore& bore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = bore_export.top_inner_area;
  column.bottom_inner_area = bore_export.floor_area;
  column.kind = PrismaticAxialThroughFeatureKind::bore;
  column.bore = bore_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_blind_section_column(
    const SimplePrismaticBlindCountersinkExport& countersink_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = countersink_export.top_inner_area;
  column.bottom_inner_area = countersink_export.floor_area;
  column.kind = PrismaticAxialThroughFeatureKind::countersink;
  column.countersink = countersink_export.countersink;
  column.bore = countersink_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_blind_section_column(
    const SimplePrismaticMultiBlindCountersink& countersink_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = countersink_export.top_inner_area;
  column.bottom_inner_area = countersink_export.floor_area;
  column.kind = PrismaticAxialThroughFeatureKind::countersink;
  column.countersink = countersink_export.countersink;
  column.bore = countersink_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_blind_section_column(
    const SimplePrismaticBlindCounterboreExport& counterbore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = counterbore_export.top_inner_area;
  column.bottom_inner_area = counterbore_export.floor_area;
  column.step_outer_area = counterbore_export.step_outer_area;
  column.step_inner_area = counterbore_export.step_inner_area;
  column.kind = PrismaticAxialThroughFeatureKind::counterbore;
  column.upper_bore = counterbore_export.upper_bore;
  column.lower_bore = counterbore_export.lower_bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_blind_section_column(
    const SimplePrismaticMultiBlindCounterbore& counterbore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = counterbore_export.top_inner_area;
  column.bottom_inner_area = counterbore_export.floor_area;
  column.step_outer_area = counterbore_export.step_outer_area;
  column.step_inner_area = counterbore_export.step_inner_area;
  column.kind = PrismaticAxialThroughFeatureKind::counterbore;
  column.upper_bore = counterbore_export.upper_bore;
  column.lower_bore = counterbore_export.lower_bore;
  return column;
}

PrismaticBlindSectionModel make_prismatic_blind_section_model(
    const SimplePrismaticBlindBoreExport& bore_export) {
  PrismaticBlindSectionModel model;
  model.top_face_index = bore_export.top_face_index;
  model.top_outer_loop = bore_export.top_outer_loop;
  model.features.push_back(make_prismatic_blind_section_column(bore_export));
  model.outer_face_indices = bore_export.outer_face_indices;
  return model;
}

PrismaticBlindSectionModel make_prismatic_blind_section_model(
    const SimplePrismaticBlindCountersinkExport& countersink_export) {
  PrismaticBlindSectionModel model;
  model.top_face_index = countersink_export.top_face_index;
  model.top_outer_loop = countersink_export.top_outer_loop;
  model.features.push_back(make_prismatic_blind_section_column(countersink_export));
  model.outer_face_indices = countersink_export.outer_face_indices;
  return model;
}

PrismaticBlindSectionModel make_prismatic_blind_section_model(
    const SimplePrismaticBlindCounterboreExport& counterbore_export) {
  PrismaticBlindSectionModel model;
  model.top_face_index = counterbore_export.top_face_index;
  model.top_outer_loop = counterbore_export.top_outer_loop;
  model.features.push_back(make_prismatic_blind_section_column(counterbore_export));
  model.outer_face_indices = counterbore_export.outer_face_indices;
  return model;
}

PrismaticBlindSectionModel make_prismatic_blind_section_model(
    const SimplePrismaticMultiBlindBoreExport& bore_export) {
  PrismaticBlindSectionModel model;
  model.top_face_index = bore_export.top_face_index;
  model.support_bottom_face_index = bore_export.bottom_face_index;
  model.top_outer_loop = bore_export.top_outer_loop;
  model.support_bottom_outer_loop = bore_export.bottom_outer_loop;
  model.features.reserve(bore_export.bores.size());
  for (const SimplePrismaticMultiBlindBore& bore : bore_export.bores) {
    model.features.push_back(make_prismatic_blind_section_column(bore));
  }
  model.outer_face_indices = bore_export.outer_face_indices;
  return model;
}

PrismaticBlindSectionModel make_prismatic_blind_section_model(
    const SimplePrismaticMultiBlindCountersinkExport& countersink_export) {
  PrismaticBlindSectionModel model;
  model.top_face_index = countersink_export.top_face_index;
  model.support_bottom_face_index = countersink_export.bottom_face_index;
  model.top_outer_loop = countersink_export.top_outer_loop;
  model.support_bottom_outer_loop = countersink_export.bottom_outer_loop;
  model.features.reserve(countersink_export.countersinks.size());
  for (const SimplePrismaticMultiBlindCountersink& countersink : countersink_export.countersinks) {
    model.features.push_back(make_prismatic_blind_section_column(countersink));
  }
  model.outer_face_indices = countersink_export.outer_face_indices;
  return model;
}

PrismaticBlindSectionModel make_prismatic_blind_section_model(
    const SimplePrismaticMultiBlindCounterboreExport& counterbore_export) {
  PrismaticBlindSectionModel model;
  model.top_face_index = counterbore_export.top_face_index;
  model.support_bottom_face_index = counterbore_export.bottom_face_index;
  model.top_outer_loop = counterbore_export.top_outer_loop;
  model.support_bottom_outer_loop = counterbore_export.bottom_outer_loop;
  model.features.reserve(counterbore_export.counterbores.size());
  for (const SimplePrismaticMultiBlindCounterbore& counterbore : counterbore_export.counterbores) {
    model.features.push_back(make_prismatic_blind_section_column(counterbore));
  }
  model.outer_face_indices = counterbore_export.outer_face_indices;
  return model;
}

std::optional<PrismaticBlindSectionModel> detect_prismatic_blind_section_model(
    const ReconstructionResult& reconstruction) {
  if (const std::optional<SimplePrismaticMultiBlindCounterboreExport> counterbore =
          detect_simple_prismatic_multi_blind_counterbore_export(reconstruction);
      counterbore.has_value()) {
    return make_prismatic_blind_section_model(*counterbore);
  }

  if (const std::optional<SimplePrismaticMultiBlindCountersinkExport> countersink =
          detect_simple_prismatic_multi_blind_countersink_export(reconstruction);
      countersink.has_value()) {
    return make_prismatic_blind_section_model(*countersink);
  }

  if (const std::optional<SimplePrismaticMultiBlindBoreExport> bore =
          detect_simple_prismatic_multi_blind_bore_export(reconstruction);
      bore.has_value()) {
    return make_prismatic_blind_section_model(*bore);
  }

  if (const std::optional<SimplePrismaticBlindCounterboreExport> counterbore =
          detect_simple_prismatic_blind_counterbore_export(reconstruction);
      counterbore.has_value()) {
    return make_prismatic_blind_section_model(*counterbore);
  }

  if (const std::optional<SimplePrismaticBlindCountersinkExport> countersink =
          detect_simple_prismatic_blind_countersink_export(reconstruction);
      countersink.has_value()) {
    return make_prismatic_blind_section_model(*countersink);
  }

  if (const std::optional<SimplePrismaticBlindBoreExport> bore =
          detect_simple_prismatic_blind_bore_export(reconstruction);
      bore.has_value()) {
    return make_prismatic_blind_section_model(*bore);
  }

  return std::nullopt;
}

Vec3 prismatic_blind_section_top_origin(const PrismaticAxialSectionColumn& feature) {
  if (feature.kind == PrismaticAxialThroughFeatureKind::counterbore) {
    return feature.upper_bore.base_center + feature.upper_bore.axis * feature.upper_bore.height;
  }
  if (feature.kind == PrismaticAxialThroughFeatureKind::countersink) {
    return feature.countersink.base_center + feature.countersink.axis * feature.countersink.height;
  }
  return feature.bore.base_center + feature.bore.axis * feature.bore.height;
}

Vec3 prismatic_blind_section_floor_origin(const PrismaticAxialSectionColumn& feature) {
  if (feature.kind == PrismaticAxialThroughFeatureKind::counterbore) {
    return feature.lower_bore.base_center;
  }
  return feature.bore.base_center;
}

Vec3 prismatic_blind_section_axis(const PrismaticAxialSectionColumn& feature) {
  if (feature.kind == PrismaticAxialThroughFeatureKind::counterbore) {
    return feature.upper_bore.axis;
  }
  if (feature.kind == PrismaticAxialThroughFeatureKind::countersink) {
    return feature.countersink.axis;
  }
  return feature.bore.axis;
}

Vec3 prismatic_blind_section_ref_direction(const PrismaticAxialSectionColumn& feature) {
  if (feature.kind == PrismaticAxialThroughFeatureKind::counterbore) {
    return feature.upper_bore.ref_direction;
  }
  if (feature.kind == PrismaticAxialThroughFeatureKind::countersink) {
    return feature.countersink.ref_direction;
  }
  return feature.bore.ref_direction;
}

int add_step_face_for_loops_and_fit(StepBuilder& step,
                                    StepLineLoopWriter& writer,
                                    const PlaneFit& fit,
                                    const std::vector<std::vector<int>>& loops) {
  const int plane_origin =
      step.add("CARTESIAN_POINT('',(" + format_double(fit.centroid.x) + "," +
               format_double(fit.centroid.y) + "," + format_double(fit.centroid.z) + "))");
  const int face_normal =
      step.add("DIRECTION('',(" + format_double(fit.normal.x) + "," +
               format_double(fit.normal.y) + "," + format_double(fit.normal.z) + "))");
  const PlaneBasis basis = basis_from_normal(fit.normal);
  const int face_ref =
      step.add("DIRECTION('',(" + format_double(basis.u.x) + "," + format_double(basis.u.y) + "," +
               format_double(basis.u.z) + "))");
  const int face_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                 step.ref(face_normal) + "," + step.ref(face_ref) + ")");
  const int plane = step.add("PLANE(''," + step.ref(face_axis) + ")");
  return add_step_planar_face_from_loops(step, writer, plane, loops);
}

bool write_step_file_prismatic_blind_section_model(const std::filesystem::path& path,
                                                   const ReconstructionResult& reconstruction,
                                                   const PrismaticBlindSectionModel& section_model) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);
  StepLineLoopWriter writer(step, context_ids, reconstruction);

  if (section_model.features.empty()) {
    return false;
  }

  std::vector<int> top_outer_loop = section_model.top_outer_loop;
  const PrismaticAxialSectionColumn& reference_feature = section_model.features.front();
  const Vec3 axis = prismatic_blind_section_axis(reference_feature);
  const Vec3 ref_direction = prismatic_blind_section_ref_direction(reference_feature);
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  const StepPlaneSurface top_plane =
      add_step_plane_surface(step, prismatic_blind_section_top_origin(reference_feature), axis, ref_direction);

  std::vector<AxialThroughFeatureBundle> bundles;
  std::vector<int> floor_faces;
  bundles.reserve(section_model.features.size());
  floor_faces.reserve(section_model.features.size());
  for (const PrismaticAxialSectionColumn& feature : section_model.features) {
    const StepPlaneSurface floor_plane =
        add_step_plane_surface(step, prismatic_blind_section_floor_origin(feature), axis, ref_direction);
    bundles.push_back(
        add_prismatic_axial_section_column_bundle(writer, top_plane, floor_plane, axis, ref_direction, feature));

    const bool floor_circle_forward = feature.bottom_inner_area > 0.0;
    const int floor_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles.back().bottom_edge_curve) + "," +
                 std::string(floor_circle_forward ? ".T." : ".F.") + ")");
    const int floor_loop = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
    const int floor_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop) + ",.T.)");
    floor_faces.push_back(step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                   step.ref(floor_plane.plane_id) + ",.T.)"));
  }

  const int top_outer_loop_id = writer.add_line_loop(top_outer_loop);
  std::vector<int> top_edge_loops;
  top_edge_loops.push_back(top_outer_loop_id);
  for (std::size_t index = 0; index < section_model.features.size(); ++index) {
    const bool top_circle_forward = section_model.features[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    top_edge_loops.push_back(step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))"));
  }
  const int top_face = add_step_planar_face_from_edge_loops(step, top_plane.plane_id, top_edge_loops);

  int support_bottom_face = 0;
  if (section_model.support_bottom_face_index >= 0 && !section_model.support_bottom_outer_loop.empty()) {
    std::vector<int> support_bottom_loop = section_model.support_bottom_outer_loop;
    const ReconstructedFace& support_face = reconstruction.faces[section_model.support_bottom_face_index];
    const PlaneBasis support_basis = basis_from_normal(support_face.fit.normal);
    if (projected_loop_area(support_bottom_loop, reconstruction.vertices, support_basis) < 0.0) {
      std::reverse(support_bottom_loop.begin(), support_bottom_loop.end());
    }
    support_bottom_face = add_step_face_for_loops_and_fit(
        step, writer, support_face.fit, std::vector<std::vector<int>> {support_bottom_loop});
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(section_model.outer_face_indices.size());
  for (int face_index : section_model.outer_face_indices) {
    side_face_ids.push_back(add_step_face_for_loops_and_fit(
        step, writer, reconstruction.faces[face_index].fit, reconstruction.faces[face_index].loops));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face);
  if (support_bottom_face != 0) {
    shell_faces << "," << step.ref(support_bottom_face);
  }
  for (const int floor_face : floor_faces) {
    shell_faces << "," << step.ref(floor_face);
  }
  for (const AxialThroughFeatureBundle& bundle : bundles) {
    for (const int face_id : bundle.shell_faces) {
      shell_faces << "," << step.ref(face_id);
    }
  }
  for (const int face_id : side_face_ids) {
    shell_faces << "," << step.ref(face_id);
  }

  const int closed_shell = step.add("CLOSED_SHELL('',(" + shell_faces.str() + "))");
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(context_ids.global_axis) + ")," + step.ref(context_ids.context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(context_ids.product_shape) + "," +
           step.ref(shape_representation) + ")");

  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write STEP file: " + path.string());
  }
  output << step_header();
  output << step.emit();
  output << "ENDSEC;\n";
  output << "END-ISO-10303-21;\n";
  return true;
}
