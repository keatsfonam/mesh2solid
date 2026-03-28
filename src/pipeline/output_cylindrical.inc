bool write_step_file_generic_cylindrical_features(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const GenericCylindricalFeatureExport& feature_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);
  StepLineLoopWriter writer(step, context_ids, reconstruction);
  StepPlaneSurfaceCache plane_cache(step, reconstruction);

  struct HoleLoopRef {
    std::size_t hole_index {};
    bool is_top {};
    double area {};
  };
  struct BlindSupportLoopRef {
    std::size_t feature_index {};
    bool use_top_edge {};
    double area {};
  };
  struct BlindCapFaceRef {
    std::size_t feature_index {};
    bool use_top_edge {};
    double area {};
  };
  struct SmoothBandLoopRef {
    std::size_t band_index {};
    bool is_top {};
  };

  std::unordered_set<int> omitted_side_faces;
  std::unordered_map<std::uint64_t, HoleLoopRef> hole_loop_refs;
  std::unordered_map<std::uint64_t, BlindSupportLoopRef> blind_support_loop_refs;
  std::unordered_map<int, BlindCapFaceRef> blind_cap_face_refs;
  std::unordered_map<std::uint64_t, std::vector<SmoothBandLoopRef>> smooth_band_loop_refs;
  std::vector<StepCylinderBundle> hole_bundles;
  hole_bundles.reserve(feature_export.holes.size());
  for (std::size_t hole_index = 0; hole_index < feature_export.holes.size(); ++hole_index) {
    const GenericCylindricalThroughHole& hole = feature_export.holes[hole_index];
    const StepPlaneSurface& top_plane = plane_cache.get(hole.top_face_index);
    const StepPlaneSurface& bottom_plane = plane_cache.get(hole.bottom_face_index);
    hole_bundles.push_back(
        add_step_cylinder_bundle(step, context_ids, hole.cylinder, top_plane, bottom_plane));
    hole_loop_refs.emplace(make_face_loop_key(hole.top_face_index, hole.top_loop_index),
                           HoleLoopRef {hole_index, true, hole.top_loop_area});
    hole_loop_refs.emplace(make_face_loop_key(hole.bottom_face_index, hole.bottom_loop_index),
                           HoleLoopRef {hole_index, false, hole.bottom_loop_area});
    omitted_side_faces.insert(hole.side_face_indices.begin(), hole.side_face_indices.end());
  }

  std::vector<StepCylinderBundle> blind_bundles;
  blind_bundles.reserve(feature_export.blind_features.size());
  for (std::size_t feature_index = 0; feature_index < feature_export.blind_features.size();
       ++feature_index) {
    const GenericCylindricalBlindFeature& feature = feature_export.blind_features[feature_index];
    const StepPlaneSurface& support_plane = plane_cache.get(feature.support_face_index);
    const StepPlaneSurface& cap_plane = plane_cache.get(feature.cap_face_index);
    const StepPlaneSurface& top_plane = feature.support_is_top ? support_plane : cap_plane;
    const StepPlaneSurface& bottom_plane = feature.support_is_top ? cap_plane : support_plane;
    blind_bundles.push_back(
        add_step_cylinder_bundle(step, context_ids, feature.cylinder, top_plane, bottom_plane));
    blind_support_loop_refs.emplace(
        make_face_loop_key(feature.support_face_index, feature.support_loop_index),
        BlindSupportLoopRef {feature_index, feature.support_is_top, feature.support_loop_area});
    blind_cap_face_refs.emplace(
        feature.cap_face_index,
        BlindCapFaceRef {feature_index, !feature.support_is_top, feature.cap_loop_area});
    omitted_side_faces.insert(feature.side_face_indices.begin(), feature.side_face_indices.end());
  }

  std::vector<StepSmoothBandBundle> smooth_band_bundles;
  smooth_band_bundles.reserve(feature_export.smooth_bands.size());
  for (std::size_t band_index = 0; band_index < feature_export.smooth_bands.size(); ++band_index) {
    const GenericSmoothExtrudedBand& band = feature_export.smooth_bands[band_index];
    smooth_band_bundles.push_back(add_step_smooth_band_bundle(step, band));
    smooth_band_loop_refs[make_face_loop_key(band.top_face_index, band.top_loop_index)].push_back(
        SmoothBandLoopRef {band_index, true});
    smooth_band_loop_refs[make_face_loop_key(band.bottom_face_index, band.bottom_loop_index)]
        .push_back(SmoothBandLoopRef {band_index, false});
    omitted_side_faces.insert(band.side_face_indices.begin(), band.side_face_indices.end());
  }

  std::unordered_set<int> special_loop_faces;
  for (const GenericCylindricalThroughHole& hole : feature_export.holes) {
    special_loop_faces.insert(hole.top_face_index);
    special_loop_faces.insert(hole.bottom_face_index);
  }
  for (const GenericCylindricalBlindFeature& feature : feature_export.blind_features) {
    special_loop_faces.insert(feature.support_face_index);
  }
  for (const GenericSmoothExtrudedBand& band : feature_export.smooth_bands) {
    special_loop_faces.insert(band.top_face_index);
    special_loop_faces.insert(band.bottom_face_index);
  }

  std::unordered_set<int> special_blind_cap_faces;
  for (const GenericCylindricalBlindFeature& feature : feature_export.blind_features) {
    special_blind_cap_faces.insert(feature.cap_face_index);
  }

  std::vector<int> face_ids;
  face_ids.reserve(reconstruction.faces.size());
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (omitted_side_faces.contains(static_cast<int>(face_index))) {
      continue;
    }

    const ReconstructedFace& face = reconstruction.faces[face_index];
    const int face_index_int = static_cast<int>(face_index);
    const StepPlaneSurface& plane = plane_cache.get(face_index_int);
    if (special_blind_cap_faces.contains(face_index_int)) {
      const auto cap_ref = blind_cap_face_refs.find(face_index_int);
      if (cap_ref == blind_cap_face_refs.end()) {
        return false;
      }
      const StepCylinderBundle& bundle = blind_bundles[cap_ref->second.feature_index];
      const int edge_curve_id =
          cap_ref->second.use_top_edge ? bundle.top_edge_curve : bundle.bottom_edge_curve;
      const bool same_orientation = cap_ref->second.area > 0.0;
      const int oriented_edge =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      const int edge_loop = step.add("EDGE_LOOP('',(" + step.ref(oriented_edge) + "))");
      face_ids.push_back(add_step_planar_face_from_edge_loops(
          step, plane.plane_id, std::vector<int> {edge_loop}));
      continue;
    }

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
      if (smooth_band_ref != smooth_band_loop_refs.end()) {
        std::vector<StepLoopCurveReplacement> replacements;
        replacements.reserve(smooth_band_ref->second.size());
        for (const SmoothBandLoopRef& loop_ref : smooth_band_ref->second) {
          const GenericSmoothExtrudedBand& band = feature_export.smooth_bands[loop_ref.band_index];
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
        const int edge_loop =
            add_step_loop_with_curve_replacements(step, writer, face.loops[loop_index], replacements);
        edge_loop_ids.push_back(edge_loop);
        continue;
      }

      const auto hole_ref = hole_loop_refs.find(make_face_loop_key(face_index_int, loop_index));
      if (hole_ref != hole_loop_refs.end()) {
        const StepCylinderBundle& bundle = hole_bundles[hole_ref->second.hole_index];
        const int edge_curve_id =
            hole_ref->second.is_top ? bundle.top_edge_curve : bundle.bottom_edge_curve;
        const bool same_orientation = hole_ref->second.area > 0.0;
        const int oriented_edge =
            step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                     std::string(same_orientation ? ".T." : ".F.") + ")");
        const int edge_loop = step.add("EDGE_LOOP('',(" + step.ref(oriented_edge) + "))");
        edge_loop_ids.push_back(edge_loop);
        continue;
      }

      const auto blind_ref =
          blind_support_loop_refs.find(make_face_loop_key(face_index_int, loop_index));
      if (blind_ref != blind_support_loop_refs.end()) {
        const StepCylinderBundle& bundle = blind_bundles[blind_ref->second.feature_index];
        const int edge_curve_id =
            blind_ref->second.use_top_edge ? bundle.top_edge_curve : bundle.bottom_edge_curve;
        const bool same_orientation = blind_ref->second.area > 0.0;
        const int oriented_edge =
            step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                     std::string(same_orientation ? ".T." : ".F.") + ")");
        const int edge_loop = step.add("EDGE_LOOP('',(" + step.ref(oriented_edge) + "))");
        edge_loop_ids.push_back(edge_loop);
        continue;
      }

      const int edge_loop = writer.add_line_loop(face.loops[loop_index]);
      edge_loop_ids.push_back(edge_loop);
    }
    face_ids.push_back(add_step_planar_face_from_edge_loops(
        step, plane.plane_id, edge_loop_ids, *outer_loop_index));
  }

  for (const StepCylinderBundle& bundle : hole_bundles) {
    face_ids.push_back(bundle.lateral_face);
  }
  for (const StepCylinderBundle& bundle : blind_bundles) {
    face_ids.push_back(bundle.lateral_face);
  }
  for (const StepSmoothBandBundle& bundle : smooth_band_bundles) {
    face_ids.push_back(bundle.lateral_face);
  }

  add_step_manifold_solid_representation(step, context_ids, face_ids);
  write_step_builder_to_file(path, step);
  return true;
}
