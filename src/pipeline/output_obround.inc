struct PrismaticProfileSweepObroundSlotModel {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  Vec3 slot_origin {};
  Vec3 axis {};
  Vec3 major_direction {};
  Vec3 minor_direction {};
  double top_plane {};
  double bottom_plane {};
  double radius {};
  double half_span {};
  std::vector<int> outer_face_indices;
};

std::optional<PrismaticProfileSweepObroundSlotModel> detect_prismatic_profile_sweep_obround_slot_model(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 16) {
    return std::nullopt;
  }

  std::vector<int> cap_candidates;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    if (reconstruction.faces[face_index].loops.size() == 2) {
      cap_candidates.push_back(static_cast<int>(face_index));
    }
  }
  if (cap_candidates.size() < 2) {
    return std::nullopt;
  }

  for (std::size_t i = 0; i < cap_candidates.size(); ++i) {
    for (std::size_t j = i + 1; j < cap_candidates.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[cap_candidates[i]];
      const ReconstructedFace& face_b = reconstruction.faces[cap_candidates[j]];
      if (angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal) > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.centroid - face_b.fit.centroid);
      if (length_squared(axis) <= 1e-12) {
        continue;
      }
      if (dot(axis, face_a.fit.normal) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis axis_basis = basis_from_normal(axis);

      const std::optional<ClassifiedFaceLoops> loops_a =
          classify_face_loops_by_area(face_a, reconstruction.vertices, axis_basis);
      const std::optional<ClassifiedFaceLoops> loops_b =
          classify_face_loops_by_area(face_b, reconstruction.vertices, axis_basis);
      if (!loops_a.has_value() || !loops_b.has_value()) {
        continue;
      }

      const auto& [outer_a, inner_a, outer_area_a, inner_area_a] = *loops_a;
      const auto& [outer_b, inner_b, outer_area_b, inner_area_b] = *loops_b;
      if (outer_a.size() != 4 || outer_b.size() != 4 || inner_a.size() < 6 || inner_b.size() < 6) {
        continue;
      }

      const std::optional<ObroundFit2D> fit_a =
          fit_obround_2d(project_loop_to_basis(inner_a, reconstruction.vertices, axis_basis));
      const std::optional<ObroundFit2D> fit_b =
          fit_obround_2d(project_loop_to_basis(inner_b, reconstruction.vertices, axis_basis));
      if (!fit_a.has_value() || !fit_b.has_value()) {
        continue;
      }

      const double radius = 0.5 * (fit_a->radius + fit_b->radius);
      const double span = 0.5 * (fit_a->half_span + fit_b->half_span);
      const double radius_tolerance = std::max(radius * 0.08, 0.2);
      const double span_tolerance = std::max(radius * 0.12, 0.25);
      const double center_tolerance = std::max(radius * 0.12, 0.25);
      if (std::abs(fit_a->radius - fit_b->radius) > radius_tolerance ||
          std::abs(fit_a->half_span - fit_b->half_span) > span_tolerance) {
        continue;
      }

      const Vec2 center_delta {fit_a->center.x - fit_b->center.x, fit_a->center.y - fit_b->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          center_tolerance) {
        continue;
      }

      const double basis_alignment =
          std::abs(fit_a->major.x * fit_b->major.x + fit_a->major.y * fit_b->major.y);
      if (basis_alignment < std::cos(6.0 * kPi / 180.0)) {
        continue;
      }

      const double plane_a = dot(face_a.fit.centroid, axis);
      const double plane_b = dot(face_b.fit.centroid, axis);
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
      if (top_plane - bottom_plane <= radius_tolerance) {
        continue;
      }

      const bool a_is_top = plane_a >= plane_b;
      const int top_face_index = a_is_top ? cap_candidates[i] : cap_candidates[j];
      const int bottom_face_index = a_is_top ? cap_candidates[j] : cap_candidates[i];
      const std::vector<int>& top_outer_loop = a_is_top ? outer_a : outer_b;
      const std::vector<int>& bottom_outer_loop = a_is_top ? outer_b : outer_a;
      const std::vector<int>& top_inner_loop = a_is_top ? inner_a : inner_b;
      const std::vector<int>& bottom_inner_loop = a_is_top ? inner_b : inner_a;
      const double top_inner_area = a_is_top ? inner_area_a : inner_area_b;
      const double bottom_inner_area = a_is_top ? inner_area_b : inner_area_a;

      std::unordered_set<int> inner_vertices;
      inner_vertices.insert(top_inner_loop.begin(), top_inner_loop.end());
      inner_vertices.insert(bottom_inner_loop.begin(), bottom_inner_loop.end());

      std::vector<int> outer_face_indices;
      int inner_face_count = 0;
      bool valid = true;
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == top_face_index ||
            static_cast<int>(face_index) == bottom_face_index) {
          continue;
        }
        const ReconstructedFace& face = reconstruction.faces[face_index];
        if (face.loops.size() != 1 || face.loops.front().size() < 4) {
          valid = false;
          break;
        }

        bool all_inner = true;
        for (int vertex_index : face.loops.front()) {
          if (!inner_vertices.contains(vertex_index)) {
            all_inner = false;
            break;
          }
        }

        if (all_inner) {
          ++inner_face_count;
        } else {
          if (angle_degrees_unsigned(face.fit.normal, axis) < 2.0) {
            valid = false;
            break;
          }
          outer_face_indices.push_back(static_cast<int>(face_index));
        }
      }

      if (!valid || inner_face_count < 4 || outer_face_indices.size() != 4) {
        continue;
      }

      Vec2 averaged_major = fit_a->major;
      if (fit_a->major.x * fit_b->major.x + fit_a->major.y * fit_b->major.y < 0.0) {
        averaged_major.x += -fit_b->major.x;
        averaged_major.y += -fit_b->major.y;
      } else {
        averaged_major.x += fit_b->major.x;
        averaged_major.y += fit_b->major.y;
      }
      const double major_length =
          std::sqrt(averaged_major.x * averaged_major.x + averaged_major.y * averaged_major.y);
      if (major_length <= 1e-12) {
        continue;
      }
      averaged_major.x /= major_length;
      averaged_major.y /= major_length;
      const Vec2 averaged_minor {-averaged_major.y, averaged_major.x};

      Vec3 major_direction =
          normalized(axis_basis.u * averaged_major.x + axis_basis.v * averaged_major.y);
      Vec3 minor_direction =
          normalized(axis_basis.u * averaged_minor.x + axis_basis.v * averaged_minor.y);
      if (dot(cross(major_direction, minor_direction), axis) < 0.0) {
        minor_direction = minor_direction * -1.0;
      }

      const Vec2 averaged_center {
          0.5 * (fit_a->center.x + fit_b->center.x),
          0.5 * (fit_a->center.y + fit_b->center.y),
      };

      PrismaticProfileSweepObroundSlotModel slot;
      slot.top_face_index = top_face_index;
      slot.bottom_face_index = bottom_face_index;
      slot.top_outer_loop = top_outer_loop;
      slot.bottom_outer_loop = bottom_outer_loop;
      slot.top_inner_area = top_inner_area;
      slot.bottom_inner_area = bottom_inner_area;
      slot.slot_origin = axis_basis.u * averaged_center.x + axis_basis.v * averaged_center.y;
      slot.axis = axis;
      slot.major_direction = major_direction;
      slot.minor_direction = minor_direction;
      slot.top_plane = top_plane;
      slot.bottom_plane = bottom_plane;
      slot.radius = radius;
      slot.half_span = span;
      slot.outer_face_indices = std::move(outer_face_indices);
      return slot;
    }
  }

  return std::nullopt;
}

bool write_step_file_prismatic_profile_sweep_obround_slot_model(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const PrismaticProfileSweepObroundSlotModel& slot) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  StepSyntheticLoopWriter writer(step, reconstruction.vertices);

  auto wrap_angle = [](double angle) {
    while (angle < 0.0) {
      angle += 2.0 * kPi;
    }
    while (angle >= 2.0 * kPi) {
      angle -= 2.0 * kPi;
    }
    return angle;
  };

  auto add_arc_edge_curve = [&](int from,
                                int to,
                                const Vec3& center,
                                const Vec3& axis,
                                const Vec3& ref_direction,
                                double radius,
                                const Vec3& midpoint) -> int {
    const Vec3 local_ref = normalized(ref_direction);
    const Vec3 local_v = normalized(cross(axis, local_ref));
    const auto angle_at = [&](const Vec3& point) {
      const Vec3 delta = point - center;
      return wrap_angle(std::atan2(dot(delta, local_v), dot(delta, local_ref)));
    };

    const double start_angle = angle_at(writer.position(from));
    const double end_angle = angle_at(writer.position(to));
    double midpoint_angle = angle_at(midpoint);
    double ccw_end = end_angle;
    if (ccw_end < start_angle) {
      ccw_end += 2.0 * kPi;
    }
    while (midpoint_angle < start_angle) {
      midpoint_angle += 2.0 * kPi;
    }
    const bool sense_agreement = midpoint_angle <= ccw_end + 1e-9;

    const int center_point = step.add("CARTESIAN_POINT('',(" + format_double(center.x) + "," +
                                    format_double(center.y) + "," + format_double(center.z) + "))");
    const int axis_direction = step.add("DIRECTION('',(" + format_double(axis.x) + "," +
                                        format_double(axis.y) + "," + format_double(axis.z) + "))");
    const int ref_dir_id = step.add("DIRECTION('',(" + format_double(local_ref.x) + "," +
                                      format_double(local_ref.y) + "," +
                                      format_double(local_ref.z) + "))");
    const int circle_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(center_point) + "," + step.ref(axis_direction) +
                 "," + step.ref(ref_dir_id) + ")");
    const int circle =
        step.add("CIRCLE(''," + step.ref(circle_axis) + "," + format_double(radius) + ")");
    const int trimmed =
        step.add("TRIMMED_CURVE(''," + step.ref(circle) + ",(" + step.ref(writer.point_id(from)) +
                 "),(" + step.ref(writer.point_id(to)) + ")," +
                 std::string(sense_agreement ? ".T." : ".F.") + ",.CARTESIAN.)");
    return step.add("EDGE_CURVE(''," + step.ref(writer.vertex_id(from)) + "," +
                    step.ref(writer.vertex_id(to)) + "," + step.ref(trimmed) + ",.T.)");
  };

  auto add_plane = [&](const Vec3& origin, const Vec3& normal, const Vec3& ref_direction) -> int {
    const int origin_id = step.add("CARTESIAN_POINT('',(" + format_double(origin.x) + "," +
                                 format_double(origin.y) + "," + format_double(origin.z) + "))");
    const int normal_id = step.add("DIRECTION('',(" + format_double(normal.x) + "," +
                                 format_double(normal.y) + "," + format_double(normal.z) + "))");
    const int ref_id = step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
                              format_double(ref_direction.y) + "," +
                              format_double(ref_direction.z) + "))");
    const int axis_id = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin_id) + "," +
                                 step.ref(normal_id) + "," + step.ref(ref_id) + ")");
    return step.add("PLANE(''," + step.ref(axis_id) + ")");
  };

  auto add_cylinder_surface = [&](const Vec3& base_center,
                                  const Vec3& axis,
                                  const Vec3& ref_direction,
                                  double radius) -> int {
    const int center_point = step.add("CARTESIAN_POINT('',(" + format_double(base_center.x) + "," +
                                    format_double(base_center.y) + "," +
                                    format_double(base_center.z) + "))");
    const int axis_direction = step.add("DIRECTION('',(" + format_double(axis.x) + "," +
                                        format_double(axis.y) + "," + format_double(axis.z) + "))");
    const int ref_dir_id = step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
                                      format_double(ref_direction.y) + "," +
                                      format_double(ref_direction.z) + "))");
    const int axis_id =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(center_point) + "," + step.ref(axis_direction) +
                 "," + step.ref(ref_dir_id) + ")");
    return step.add("CYLINDRICAL_SURFACE(''," + step.ref(axis_id) + "," +
                    format_double(radius) + ")");
  };

  std::vector<int> top_outer_loop = slot.top_outer_loop;
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, basis_from_normal(slot.axis)) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  std::vector<int> bottom_outer_loop = slot.bottom_outer_loop;
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, basis_from_normal(slot.axis)) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_center = slot.slot_origin + slot.axis * slot.top_plane;
  const Vec3 bottom_center = slot.slot_origin + slot.axis * slot.bottom_plane;
  const Vec3 top_right_center = top_center + slot.major_direction * slot.half_span;
  const Vec3 top_left_center = top_center - slot.major_direction * slot.half_span;
  const Vec3 bottom_right_center = bottom_center + slot.major_direction * slot.half_span;
  const Vec3 bottom_left_center = bottom_center - slot.major_direction * slot.half_span;

  const int top_tl = writer.add_vertex(top_left_center + slot.minor_direction * slot.radius);
  const int top_tr = writer.add_vertex(top_right_center + slot.minor_direction * slot.radius);
  const int top_br = writer.add_vertex(top_right_center - slot.minor_direction * slot.radius);
  const int top_bl = writer.add_vertex(top_left_center - slot.minor_direction * slot.radius);
  const int bottom_tl = writer.add_vertex(bottom_left_center + slot.minor_direction * slot.radius);
  const int bottom_tr = writer.add_vertex(bottom_right_center + slot.minor_direction * slot.radius);
  const int bottom_br = writer.add_vertex(bottom_right_center - slot.minor_direction * slot.radius);
  const int bottom_bl = writer.add_vertex(bottom_left_center - slot.minor_direction * slot.radius);

  const int top_upper_line = writer.add_line_edge_curve_id(top_tl, top_tr);
  const int top_lower_line = writer.add_line_edge_curve_id(top_br, top_bl);
  const int bottom_upper_line = writer.add_line_edge_curve_id(bottom_tl, bottom_tr);
  const int bottom_lower_line = writer.add_line_edge_curve_id(bottom_br, bottom_bl);
  const int seam_tl = writer.add_line_edge_curve_id(bottom_tl, top_tl);
  const int seam_tr = writer.add_line_edge_curve_id(bottom_tr, top_tr);
  const int seam_br = writer.add_line_edge_curve_id(bottom_br, top_br);
  const int seam_bl = writer.add_line_edge_curve_id(bottom_bl, top_bl);

  const int top_right_arc = add_arc_edge_curve(
      top_tr,
      top_br,
      top_right_center,
      slot.axis,
      slot.major_direction,
      slot.radius,
      top_right_center + slot.major_direction * slot.radius);
  const int top_left_arc = add_arc_edge_curve(
      top_bl,
      top_tl,
      top_left_center,
      slot.axis,
      slot.major_direction * -1.0,
      slot.radius,
      top_left_center - slot.major_direction * slot.radius);
  const int bottom_right_arc = add_arc_edge_curve(
      bottom_tr,
      bottom_br,
      bottom_right_center,
      slot.axis,
      slot.major_direction,
      slot.radius,
      bottom_right_center + slot.major_direction * slot.radius);
  const int bottom_left_arc = add_arc_edge_curve(
      bottom_bl,
      bottom_tl,
      bottom_left_center,
      slot.axis,
      slot.major_direction * -1.0,
      slot.radius,
      bottom_left_center - slot.major_direction * slot.radius);

  const int top_plane = add_plane(top_center, slot.axis, slot.major_direction);
  const int bottom_plane = add_plane(bottom_center, slot.axis, slot.major_direction);
  const int upper_slot_plane = add_plane(top_center + slot.minor_direction * slot.radius,
                                         slot.minor_direction,
                                         slot.major_direction);
  const int lower_slot_plane = add_plane(top_center - slot.minor_direction * slot.radius,
                                         slot.minor_direction * -1.0,
                                         slot.major_direction);
  const int right_cylinder =
      add_cylinder_surface(bottom_right_center, slot.axis, slot.major_direction, slot.radius);
  const int left_cylinder =
      add_cylinder_surface(bottom_left_center, slot.axis, slot.major_direction * -1.0, slot.radius);

  const std::vector<std::pair<int, bool>> top_hole_cw {
      {top_right_arc, true},
      {top_lower_line, true},
      {top_left_arc, true},
      {top_upper_line, true},
  };
  const std::vector<std::pair<int, bool>> top_hole_ccw {
      {top_upper_line, false},
      {top_left_arc, false},
      {top_lower_line, false},
      {top_right_arc, false},
  };
  const std::vector<std::pair<int, bool>> bottom_hole_cw {
      {bottom_right_arc, true},
      {bottom_lower_line, true},
      {bottom_left_arc, true},
      {bottom_upper_line, true},
  };
  const std::vector<std::pair<int, bool>> bottom_hole_ccw {
      {bottom_upper_line, false},
      {bottom_left_arc, false},
      {bottom_lower_line, false},
      {bottom_right_arc, false},
  };

  const int top_outer_loop_id = writer.add_line_loop(top_outer_loop);
  const int top_inner_loop_id =
      writer.add_edge_loop(slot.top_inner_area < 0.0 ? top_hole_cw : top_hole_ccw);
  const int top_face = add_step_planar_face_from_edge_loops(
      step, top_plane, std::vector<int> {top_outer_loop_id, top_inner_loop_id});

  const int bottom_outer_loop_id = writer.add_line_loop(bottom_outer_loop);
  const int bottom_inner_loop_id =
      writer.add_edge_loop(slot.bottom_inner_area < 0.0 ? bottom_hole_cw : bottom_hole_ccw);
  const int bottom_face = add_step_planar_face_from_edge_loops(
      step, bottom_plane, std::vector<int> {bottom_outer_loop_id, bottom_inner_loop_id}, 0, false);

  const int upper_slot_loop = writer.add_line_loop({bottom_tl, bottom_tr, top_tr, top_tl});
  const int upper_slot_face =
      add_step_face_from_single_outer_loop(step, upper_slot_plane, upper_slot_loop);

  const int lower_slot_loop = writer.add_line_loop({bottom_br, bottom_bl, top_bl, top_br});
  const int lower_slot_face =
      add_step_face_from_single_outer_loop(step, lower_slot_plane, lower_slot_loop);

  const int right_cylinder_loop = writer.add_edge_loop(
      {{top_right_arc, true}, {seam_br, false}, {bottom_right_arc, false}, {seam_tr, true}});
  const int right_cylinder_face =
      add_step_face_from_single_outer_loop(step, right_cylinder, right_cylinder_loop);

  const int left_cylinder_loop = writer.add_edge_loop(
      {{top_left_arc, true}, {seam_tl, false}, {bottom_left_arc, false}, {seam_bl, true}});
  const int left_cylinder_face =
      add_step_face_from_single_outer_loop(step, left_cylinder, left_cylinder_loop);

  std::vector<int> side_face_ids;
  side_face_ids.reserve(slot.outer_face_indices.size());
  for (int face_index : slot.outer_face_indices) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    const int plane_origin = step.add("CARTESIAN_POINT('',(" + format_double(face.fit.centroid.x) + "," +
                                   format_double(face.fit.centroid.y) + "," +
                                   format_double(face.fit.centroid.z) + "))");
    const int face_normal = step.add("DIRECTION('',(" + format_double(face.fit.normal.x) + "," +
                                   format_double(face.fit.normal.y) + "," +
                                   format_double(face.fit.normal.z) + "))");
    const PlaneBasis basis = basis_from_normal(face.fit.normal);
    const int face_ref = step.add("DIRECTION('',(" + format_double(basis.u.x) + "," +
                                format_double(basis.u.y) + "," + format_double(basis.u.z) + "))");
    const int axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                              step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis) + ")");
    const int edge_loop = writer.add_line_loop(face.loops.front());
    side_face_ids.push_back(add_step_face_from_single_outer_loop(step, plane, edge_loop));
  }

  std::vector<int> face_ids {top_face,    bottom_face,   upper_slot_face,
                             lower_slot_face, right_cylinder_face, left_cylinder_face};
  face_ids.insert(face_ids.end(), side_face_ids.begin(), side_face_ids.end());
  add_step_manifold_solid_representation(step, context_ids, face_ids);
  write_step_builder_to_file(path, step);
  return true;
}
