enum class PrismaticProfileSweepModelKind {
  SmoothBands,
  ObroundSlot,
};

struct PrismaticProfileSweepModel {
  PrismaticProfileSweepModelKind kind {PrismaticProfileSweepModelKind::SmoothBands};
  PrismaticProfileSweepBandExport band_export;
  PrismaticProfileSweepObroundSlotModel obround_slot;
};

std::optional<PrismaticProfileSweepBandExport> detect_prismatic_profile_sweep_band_export(
    const std::optional<GenericSmoothExtrudedBandExport>& smooth_bands) {
  if (!smooth_bands.has_value() || smooth_bands->bands.empty()) {
    return std::nullopt;
  }

  PrismaticProfileSweepBandExport export_result;
  export_result.bands = smooth_bands->bands;
  return export_result;
}

std::optional<PrismaticProfileSweepModel> detect_prismatic_profile_sweep_model(
    const ReconstructionResult& reconstruction,
    const std::optional<GenericSmoothExtrudedBandExport>& smooth_bands,
    bool allow_smooth_bands) {
  if (const std::optional<PrismaticProfileSweepObroundSlotModel> obround_slot =
          detect_prismatic_profile_sweep_obround_slot_model(reconstruction);
      obround_slot.has_value()) {
    PrismaticProfileSweepModel model;
    model.kind = PrismaticProfileSweepModelKind::ObroundSlot;
    model.obround_slot = *obround_slot;
    return model;
  }

  if (!allow_smooth_bands) {
    return std::nullopt;
  }

  if (const std::optional<PrismaticProfileSweepBandExport> band_export =
          detect_prismatic_profile_sweep_band_export(smooth_bands);
      band_export.has_value()) {
    PrismaticProfileSweepModel model;
    model.kind = PrismaticProfileSweepModelKind::SmoothBands;
    model.band_export = *band_export;
    return model;
  }

  return std::nullopt;
}

bool write_step_file_prismatic_profile_sweep_bands(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const PrismaticProfileSweepBandExport& band_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);
  StepLineLoopWriter writer(step, context_ids, reconstruction);

  std::unordered_map<int, StepPlaneSurface> face_planes;
  auto get_face_plane = [&](int face_index) -> const StepPlaneSurface& {
    const auto found = face_planes.find(face_index);
    if (found != face_planes.end()) {
      return found->second;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    return face_planes
        .emplace(face_index,
                 add_step_plane_surface(step, face.fit.centroid, face.fit.normal, basis.u))
        .first->second;
  };

  struct SmoothBandLoopRef {
    std::size_t band_index {};
    bool is_top {};
  };

  std::unordered_set<int> omitted_side_faces;
  std::unordered_map<std::uint64_t, std::vector<SmoothBandLoopRef>> smooth_band_loop_refs;
  std::vector<StepSmoothBandBundle> smooth_band_bundles;
  smooth_band_bundles.reserve(band_export.bands.size());
  for (std::size_t band_index = 0; band_index < band_export.bands.size(); ++band_index) {
    const PrismaticProfileSweepBand& band = band_export.bands[band_index];
    smooth_band_bundles.push_back(add_step_smooth_band_bundle(step, band));
    smooth_band_loop_refs[make_face_loop_key(band.top_face_index, band.top_loop_index)].push_back(
        SmoothBandLoopRef {band_index, true});
    smooth_band_loop_refs[make_face_loop_key(band.bottom_face_index, band.bottom_loop_index)]
        .push_back(SmoothBandLoopRef {band_index, false});
    omitted_side_faces.insert(band.side_face_indices.begin(), band.side_face_indices.end());
  }

  std::unordered_set<int> special_loop_faces;
  for (const PrismaticProfileSweepBand& band : band_export.bands) {
    special_loop_faces.insert(band.top_face_index);
    special_loop_faces.insert(band.bottom_face_index);
  }

  std::vector<int> face_ids;
  face_ids.reserve(reconstruction.faces.size());
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (omitted_side_faces.contains(static_cast<int>(face_index))) {
      continue;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    const int face_index_int = static_cast<int>(face_index);
    const StepPlaneSurface& plane = get_face_plane(face_index_int);
    if (!special_loop_faces.contains(face_index_int)) {
      face_ids.push_back(
          add_step_planar_face_from_loops(step, writer, plane.plane_id, face.loops));
      continue;
    }

    const std::vector<double> loop_areas =
        projected_face_loop_areas(face, reconstruction.vertices, plane.basis);
    const std::optional<std::size_t> outer_loop_index = outer_loop_index_from_areas(loop_areas);
    if (!outer_loop_index.has_value()) {
      return false;
    }

    std::vector<int> edge_loop_ids;
    edge_loop_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const auto smooth_band_ref =
          smooth_band_loop_refs.find(make_face_loop_key(face_index_int, loop_index));
      if (smooth_band_ref == smooth_band_loop_refs.end()) {
        edge_loop_ids.push_back(writer.add_line_loop(face.loops[loop_index]));
        continue;
      }

      std::vector<StepLoopCurveReplacement> replacements;
      replacements.reserve(smooth_band_ref->second.size());
      for (const SmoothBandLoopRef& loop_ref : smooth_band_ref->second) {
        const PrismaticProfileSweepBand& band = band_export.bands[loop_ref.band_index];
        const StepSmoothBandBundle& bundle = smooth_band_bundles[loop_ref.band_index];
        replacements.push_back(StepLoopCurveReplacement {
            LoopSubsequenceMatch {
                loop_ref.is_top ? band.top_loop_start : band.bottom_loop_start,
                loop_ref.is_top ? band.top_loop_span : band.bottom_loop_span,
                loop_ref.is_top ? band.top_forward : band.bottom_forward,
            },
            loop_ref.is_top ? bundle.top_edge_curve : bundle.bottom_edge_curve,
        });
      }
      edge_loop_ids.push_back(add_step_loop_with_curve_replacements(
          step, writer, face.loops[loop_index], replacements));
    }

    face_ids.push_back(add_step_planar_face_from_edge_loops(
        step, plane.plane_id, edge_loop_ids, *outer_loop_index));
  }

  for (const StepSmoothBandBundle& bundle : smooth_band_bundles) {
    face_ids.push_back(bundle.lateral_face);
  }

  std::ostringstream shell_faces;
  for (std::size_t index = 0; index < face_ids.size(); ++index) {
    if (index > 0) {
      shell_faces << ",";
    }
    shell_faces << step.ref(face_ids[index]);
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

bool write_step_file_prismatic_profile_sweep_model(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const PrismaticProfileSweepModel& model) {
  switch (model.kind) {
    case PrismaticProfileSweepModelKind::SmoothBands:
      return write_step_file_prismatic_profile_sweep_bands(path, reconstruction, model.band_export);
    case PrismaticProfileSweepModelKind::ObroundSlot:
      return write_step_file_prismatic_profile_sweep_obround_slot_model(
          path, reconstruction, model.obround_slot);
  }

  return false;
}
