PrismaticAxialSectionColumn make_prismatic_axial_section_column(
    const SimplePrismaticBoreExport& bore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = bore_export.top_inner_area;
  column.bottom_inner_area = bore_export.bottom_inner_area;
  column.kind = PrismaticAxialThroughFeatureKind::bore;
  column.bore = bore_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_axial_section_column(
    const SimplePrismaticMultiBore& bore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = bore_export.top_inner_area;
  column.bottom_inner_area = bore_export.bottom_inner_area;
  column.kind = PrismaticAxialThroughFeatureKind::bore;
  column.bore = bore_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_axial_section_column(
    const SimplePrismaticCountersinkExport& countersink_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = countersink_export.top_inner_area;
  column.bottom_inner_area = countersink_export.bottom_inner_area;
  column.kind = PrismaticAxialThroughFeatureKind::countersink;
  column.countersink = countersink_export.countersink;
  column.bore = countersink_export.bore;
  return column;
}

PrismaticAxialSectionColumn make_prismatic_axial_section_column(
    const SimplePrismaticCounterboreExport& counterbore_export) {
  PrismaticAxialSectionColumn column;
  column.top_inner_area = counterbore_export.top_inner_area;
  column.bottom_inner_area = counterbore_export.bottom_inner_area;
  column.step_outer_area = counterbore_export.step_outer_area;
  column.step_inner_area = counterbore_export.step_inner_area;
  column.kind = PrismaticAxialThroughFeatureKind::counterbore;
  column.upper_bore = counterbore_export.upper_bore;
  column.lower_bore = counterbore_export.lower_bore;
  return column;
}

PrismaticAxialSectionModel make_prismatic_axial_section_model(
    const SimplePrismaticBoreExport& bore_export) {
  PrismaticAxialSectionModel model;
  model.top_face_index = bore_export.top_face_index;
  model.bottom_face_index = bore_export.bottom_face_index;
  model.top_outer_loop = bore_export.top_outer_loop;
  model.bottom_outer_loop = bore_export.bottom_outer_loop;
  model.features.push_back(make_prismatic_axial_section_column(bore_export));
  model.outer_face_indices = bore_export.outer_face_indices;
  return model;
}

PrismaticAxialSectionModel make_prismatic_axial_section_model(
    const SimplePrismaticMultiBoreExport& bore_export) {
  PrismaticAxialSectionModel model;
  model.top_face_index = bore_export.top_face_index;
  model.bottom_face_index = bore_export.bottom_face_index;
  model.top_outer_loop = bore_export.top_outer_loop;
  model.bottom_outer_loop = bore_export.bottom_outer_loop;
  model.features.reserve(bore_export.bores.size());
  for (const SimplePrismaticMultiBore& bore : bore_export.bores) {
    model.features.push_back(make_prismatic_axial_section_column(bore));
  }
  model.outer_face_indices = bore_export.outer_face_indices;
  return model;
}

PrismaticAxialSectionModel make_prismatic_axial_section_model(
    const SimplePrismaticCountersinkExport& countersink_export) {
  PrismaticAxialSectionModel model;
  model.top_face_index = countersink_export.top_face_index;
  model.bottom_face_index = countersink_export.bottom_face_index;
  model.top_outer_loop = countersink_export.top_outer_loop;
  model.bottom_outer_loop = countersink_export.bottom_outer_loop;
  model.features.push_back(make_prismatic_axial_section_column(countersink_export));
  model.outer_face_indices = countersink_export.outer_face_indices;
  return model;
}

PrismaticAxialSectionModel make_prismatic_axial_section_model(
    const SimplePrismaticCounterboreExport& counterbore_export) {
  PrismaticAxialSectionModel model;
  model.top_face_index = counterbore_export.top_face_index;
  model.bottom_face_index = counterbore_export.bottom_face_index;
  model.top_outer_loop = counterbore_export.top_outer_loop;
  model.bottom_outer_loop = counterbore_export.bottom_outer_loop;
  model.features.push_back(make_prismatic_axial_section_column(counterbore_export));
  model.outer_face_indices = counterbore_export.outer_face_indices;
  return model;
}

std::optional<PrismaticAxialSectionModel> detect_prismatic_axial_section_model(
    const ReconstructionResult& reconstruction) {
  if (const std::optional<SimplePrismaticAxialThroughFeatureExport> axial =
          detect_simple_prismatic_axial_through_feature_export(reconstruction);
      axial.has_value()) {
    return *axial;
  }

  if (const std::optional<SimplePrismaticCountersinkExport> countersink =
          detect_simple_prismatic_countersink_export(reconstruction);
      countersink.has_value()) {
    return make_prismatic_axial_section_model(*countersink);
  }

  if (const std::optional<SimplePrismaticCounterboreExport> counterbore =
          detect_simple_prismatic_counterbore_export(reconstruction);
      counterbore.has_value()) {
    return make_prismatic_axial_section_model(*counterbore);
  }

  if (const std::optional<SimplePrismaticMultiBoreExport> multi_bore =
          detect_simple_prismatic_multi_bore_export(reconstruction);
      multi_bore.has_value()) {
    return make_prismatic_axial_section_model(*multi_bore);
  }

  if (const std::optional<SimplePrismaticBoreExport> bore =
          detect_simple_prismatic_bore_export(reconstruction);
      bore.has_value()) {
    return make_prismatic_axial_section_model(*bore);
  }

  return std::nullopt;
}
