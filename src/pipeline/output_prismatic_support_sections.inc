PrismaticSupportSectionColumn make_prismatic_support_section_column(
    const SimplePrismaticBossExport& boss_export) {
  PrismaticSupportSectionColumn column;
  column.base_inner_area = boss_export.base_inner_area;
  column.top_outer_area = boss_export.top_area;
  column.outer_cylinder = boss_export.boss;
  column.has_inner_cylinder = false;
  return column;
}

PrismaticSupportSectionColumn make_prismatic_support_section_column(
    const SimplePrismaticMultiBoss& boss_export) {
  PrismaticSupportSectionColumn column;
  column.base_inner_area = boss_export.base_inner_area;
  column.top_outer_area = boss_export.top_area;
  column.outer_cylinder = boss_export.boss;
  column.has_inner_cylinder = false;
  return column;
}

PrismaticSupportSectionColumn make_prismatic_support_section_column(
    const SimplePrismaticStandoffExport& standoff_export) {
  PrismaticSupportSectionColumn column;
  column.base_inner_area = standoff_export.base_inner_area;
  column.top_outer_area = standoff_export.top_outer_area;
  column.top_inner_area = standoff_export.top_inner_area;
  column.bottom_inner_area = standoff_export.bottom_inner_area;
  column.outer_cylinder = standoff_export.outer_cylinder;
  column.has_inner_cylinder = true;
  column.inner_cylinder = standoff_export.inner_cylinder;
  return column;
}

PrismaticSupportSectionModel make_prismatic_support_section_model(
    const SimplePrismaticBossExport& boss_export) {
  PrismaticSupportSectionModel model;
  model.base_face_index = boss_export.base_face_index;
  model.base_outer_loop = boss_export.base_outer_loop;
  model.features.push_back(make_prismatic_support_section_column(boss_export));
  model.outer_face_indices = boss_export.outer_face_indices;
  return model;
}

PrismaticSupportSectionModel make_prismatic_support_section_model(
    const SimplePrismaticMultiBossExport& boss_export) {
  PrismaticSupportSectionModel model;
  model.base_face_index = boss_export.base_face_index;
  model.support_bottom_face_index = boss_export.bottom_face_index;
  model.base_outer_loop = boss_export.base_outer_loop;
  model.support_bottom_outer_loop = boss_export.bottom_outer_loop;
  model.features.reserve(boss_export.bosses.size());
  for (const SimplePrismaticMultiBoss& boss : boss_export.bosses) {
    model.features.push_back(make_prismatic_support_section_column(boss));
  }
  model.outer_face_indices = boss_export.outer_face_indices;
  return model;
}

PrismaticSupportSectionModel make_prismatic_support_section_model(
    const SimplePrismaticStandoffExport& standoff_export) {
  PrismaticSupportSectionModel model;
  model.base_face_index = standoff_export.base_face_index;
  model.support_bottom_face_index = standoff_export.bottom_face_index;
  model.base_outer_loop = standoff_export.base_outer_loop;
  model.support_bottom_outer_loop = standoff_export.bottom_outer_loop;
  model.features.push_back(make_prismatic_support_section_column(standoff_export));
  model.outer_face_indices = standoff_export.outer_face_indices;
  return model;
}

std::optional<PrismaticSupportSectionModel> detect_prismatic_support_section_model(
    const ReconstructionResult& reconstruction) {
  if (const std::optional<SimplePrismaticStandoffExport> standoff =
          detect_simple_prismatic_standoff_export(reconstruction);
      standoff.has_value()) {
    return make_prismatic_support_section_model(*standoff);
  }

  if (const std::optional<SimplePrismaticMultiBossExport> multi_boss =
          detect_simple_prismatic_multi_boss_export(reconstruction);
      multi_boss.has_value()) {
    return make_prismatic_support_section_model(*multi_boss);
  }

  if (const std::optional<SimplePrismaticBossExport> boss =
          detect_simple_prismatic_boss_export(reconstruction);
      boss.has_value()) {
    return make_prismatic_support_section_model(*boss);
  }

  return std::nullopt;
}

Vec3 prismatic_support_section_axis(const PrismaticSupportSectionColumn& feature) {
  return feature.outer_cylinder.axis;
}

Vec3 prismatic_support_section_ref_direction(const PrismaticSupportSectionColumn& feature) {
  return feature.outer_cylinder.ref_direction;
}

int add_step_face_for_support_loops(StepBuilder& step,
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

bool write_step_file_prismatic_support_section_model(const std::filesystem::path& path,
                                                     const ReconstructionResult& reconstruction,
                                                     const PrismaticSupportSectionModel& section_model) {
  if (section_model.features.empty()) {
    return false;
  }

  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);
  StepLineLoopWriter writer(step, context_ids, reconstruction);

  const PrismaticSupportSectionColumn& reference_feature = section_model.features.front();
  const Vec3 axis = prismatic_support_section_axis(reference_feature);
  const Vec3 ref_direction = prismatic_support_section_ref_direction(reference_feature);
  const PlaneBasis axis_basis = basis_from_normal(axis);

  std::vector<int> base_outer_loop = section_model.base_outer_loop;
  if (projected_loop_area(base_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(base_outer_loop.begin(), base_outer_loop.end());
  }

  const StepPlaneSurface base_plane =
      add_step_plane_surface(step, reference_feature.outer_cylinder.base_center, axis, ref_direction);
  std::optional<StepPlaneSurface> support_bottom_plane;
  if (section_model.support_bottom_face_index >= 0 && !section_model.support_bottom_outer_loop.empty()) {
    const ReconstructedFace& support_face = reconstruction.faces[section_model.support_bottom_face_index];
    support_bottom_plane = add_step_plane_surface(step, support_face.fit.centroid, axis, ref_direction);
  }

  struct SupportSectionBundle {
    StepCylinderBundle outer_bundle {};
    std::optional<StepCylinderBundle> inner_bundle;
    int top_face {};
  };

  std::vector<SupportSectionBundle> bundles;
  bundles.reserve(section_model.features.size());
  for (const PrismaticSupportSectionColumn& feature : section_model.features) {
    const Vec3 top_center =
        feature.outer_cylinder.base_center + feature.outer_cylinder.axis * feature.outer_cylinder.height;
    const StepPlaneSurface top_plane = add_step_plane_surface(step, top_center, axis, ref_direction);
    SupportSectionBundle bundle;
    bundle.outer_bundle =
        add_step_cylinder_bundle(step, context_ids, feature.outer_cylinder, top_plane, base_plane);

    std::vector<int> top_edge_loops;
    const bool outer_top_forward = feature.top_outer_area > 0.0;
    const int outer_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundle.outer_bundle.top_edge_curve) + "," +
                 std::string(outer_top_forward ? ".T." : ".F.") + ")");
    top_edge_loops.push_back(step.add("EDGE_LOOP('',(" + step.ref(outer_top_oriented) + "))"));

    if (feature.has_inner_cylinder) {
      if (!support_bottom_plane.has_value()) {
        return false;
      }
      bundle.inner_bundle = add_step_cylinder_bundle(
          step, context_ids, feature.inner_cylinder, top_plane, *support_bottom_plane);
      const bool inner_top_forward = feature.top_inner_area > 0.0;
      const int inner_top_oriented =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(bundle.inner_bundle->top_edge_curve) + "," +
                   std::string(inner_top_forward ? ".T." : ".F.") + ")");
      top_edge_loops.push_back(step.add("EDGE_LOOP('',(" + step.ref(inner_top_oriented) + "))"));
    }

    bundle.top_face = add_step_planar_face_from_edge_loops(step, top_plane.plane_id, top_edge_loops);
    bundles.push_back(std::move(bundle));
  }

  const int base_outer_loop_id = writer.add_line_loop(base_outer_loop);
  std::vector<int> base_edge_loops;
  base_edge_loops.push_back(base_outer_loop_id);
  for (std::size_t index = 0; index < section_model.features.size(); ++index) {
    const bool base_inner_forward = section_model.features[index].base_inner_area > 0.0;
    const int base_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].outer_bundle.bottom_edge_curve) + "," +
                 std::string(base_inner_forward ? ".T." : ".F.") + ")");
    base_edge_loops.push_back(step.add("EDGE_LOOP('',(" + step.ref(base_inner_oriented) + "))"));
  }
  const int base_face = add_step_planar_face_from_edge_loops(step, base_plane.plane_id, base_edge_loops);

  int support_bottom_face = 0;
  if (section_model.support_bottom_face_index >= 0 && !section_model.support_bottom_outer_loop.empty()) {
    std::vector<int> support_bottom_loop = section_model.support_bottom_outer_loop;
    const ReconstructedFace& support_face = reconstruction.faces[section_model.support_bottom_face_index];
    const PlaneBasis support_basis = basis_from_normal(support_face.fit.normal);
    if (projected_loop_area(support_bottom_loop, reconstruction.vertices, support_basis) < 0.0) {
      std::reverse(support_bottom_loop.begin(), support_bottom_loop.end());
    }
    std::vector<int> edge_loops;
    edge_loops.push_back(writer.add_line_loop(support_bottom_loop));
    for (std::size_t index = 0; index < section_model.features.size(); ++index) {
      if (!section_model.features[index].has_inner_cylinder || !bundles[index].inner_bundle.has_value()) {
        continue;
      }
      const bool bottom_inner_forward = section_model.features[index].bottom_inner_area > 0.0;
      const int inner_bottom_oriented =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].inner_bundle->bottom_edge_curve) +
                   "," + std::string(bottom_inner_forward ? ".T." : ".F.") + ")");
      edge_loops.push_back(step.add("EDGE_LOOP('',(" + step.ref(inner_bottom_oriented) + "))"));
    }
    support_bottom_face = add_step_planar_face_from_edge_loops(
        step, support_bottom_plane->plane_id, edge_loops, 0, false);
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(section_model.outer_face_indices.size());
  for (int face_index : section_model.outer_face_indices) {
    side_face_ids.push_back(add_step_face_for_support_loops(
        step, writer, reconstruction.faces[face_index].fit, reconstruction.faces[face_index].loops));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(base_face);
  if (support_bottom_face != 0) {
    shell_faces << "," << step.ref(support_bottom_face);
  }
  for (const SupportSectionBundle& bundle : bundles) {
    shell_faces << "," << step.ref(bundle.top_face) << "," << step.ref(bundle.outer_bundle.lateral_face);
    if (bundle.inner_bundle.has_value()) {
      shell_faces << "," << step.ref(bundle.inner_bundle->lateral_face);
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
