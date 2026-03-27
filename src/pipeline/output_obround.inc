bool write_step_file_prismatic_obround_slot(const std::filesystem::path& path,
                                            const ReconstructionResult& reconstruction,
                                            const SimplePrismaticObroundSlotExport& slot) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  struct StepVertexInfo {
    int point_id {};
    int vertex_id {};
    Vec3 position {};
  };

  std::vector<StepVertexInfo> vertices;
  vertices.reserve(reconstruction.vertices.size() + 8);
  for (const Vec3& vertex : reconstruction.vertices) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                  format_double(vertex.y) + "," + format_double(vertex.z) + "))");
    const int vertex_id = step.add("VERTEX_POINT(''," + step.ref(point_id) + ")");
    vertices.push_back({point_id, vertex_id, vertex});
  }

  auto add_vertex = [&](const Vec3& point) -> int {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    const int vertex_id = step.add("VERTEX_POINT(''," + step.ref(point_id) + ")");
    vertices.push_back({point_id, vertex_id, point});
    return static_cast<int>(vertices.size() - 1);
  };

  std::unordered_map<EdgeKey, int, EdgeKeyHash> line_edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = line_edge_curve_ids.find(key);
    if (found != line_edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = vertices[key.a].position;
    const Vec3 end = vertices[key.b].position;
    const Vec3 delta = end - start;
    const double edge_length = length(delta);
    if (edge_length <= 1e-12) {
      throw std::runtime_error("Cannot export zero-length STEP edge");
    }

    const Vec3 direction = normalized(delta);
    const int direction_id =
        step.add("DIRECTION('',(" + format_double(direction.x) + "," + format_double(direction.y) +
                 "," + format_double(direction.z) + "))");
    const int vector_id = step.add("VECTOR(''," + step.ref(direction_id) + "," +
                                   format_double(edge_length) + ")");
    const int line_id =
        step.add("LINE(''," + step.ref(vertices[key.a].point_id) + "," + step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertices[key.a].vertex_id) + "," +
                 step.ref(vertices[key.b].vertex_id) + "," + step.ref(line_id) + ",.T.)");
    line_edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_oriented_edge = [&](int edge_curve_id, bool same_orientation) -> int {
    return step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                    std::string(same_orientation ? ".T." : ".F.") + ")");
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(add_oriented_edge(edge_curve_id, key.a == from && key.b == to));
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  auto add_edge_loop = [&](const std::vector<std::pair<int, bool>>& edges) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < edges.size(); ++index) {
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(add_oriented_edge(edges[index].first, edges[index].second));
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

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

    const double start_angle = angle_at(vertices[from].position);
    const double end_angle = angle_at(vertices[to].position);
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
        step.add("TRIMMED_CURVE(''," + step.ref(circle) + ",(" + step.ref(vertices[from].point_id) +
                 "),(" + step.ref(vertices[to].point_id) + ")," +
                 std::string(sense_agreement ? ".T." : ".F.") + ",.CARTESIAN.)");
    return step.add("EDGE_CURVE(''," + step.ref(vertices[from].vertex_id) + "," +
                    step.ref(vertices[to].vertex_id) + "," + step.ref(trimmed) + ",.T.)");
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

  const int top_tl = add_vertex(top_left_center + slot.minor_direction * slot.radius);
  const int top_tr = add_vertex(top_right_center + slot.minor_direction * slot.radius);
  const int top_br = add_vertex(top_right_center - slot.minor_direction * slot.radius);
  const int top_bl = add_vertex(top_left_center - slot.minor_direction * slot.radius);
  const int bottom_tl = add_vertex(bottom_left_center + slot.minor_direction * slot.radius);
  const int bottom_tr = add_vertex(bottom_right_center + slot.minor_direction * slot.radius);
  const int bottom_br = add_vertex(bottom_right_center - slot.minor_direction * slot.radius);
  const int bottom_bl = add_vertex(bottom_left_center - slot.minor_direction * slot.radius);

  const int top_upper_line = get_line_edge_curve_id(top_tl, top_tr);
  const int top_lower_line = get_line_edge_curve_id(top_br, top_bl);
  const int bottom_upper_line = get_line_edge_curve_id(bottom_tl, bottom_tr);
  const int bottom_lower_line = get_line_edge_curve_id(bottom_br, bottom_bl);
  const int seam_tl = get_line_edge_curve_id(bottom_tl, top_tl);
  const int seam_tr = get_line_edge_curve_id(bottom_tr, top_tr);
  const int seam_br = get_line_edge_curve_id(bottom_br, top_br);
  const int seam_bl = get_line_edge_curve_id(bottom_bl, top_bl);

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

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const int top_inner_loop_id =
      add_edge_loop(slot.top_inner_area < 0.0 ? top_hole_cw : top_hole_ccw);
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop_id) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_inner_loop_id =
      add_edge_loop(slot.bottom_inner_area < 0.0 ? bottom_hole_cw : bottom_hole_ccw);
  const int bottom_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + "," +
                                   step.ref(bottom_inner_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  const int upper_slot_loop = add_line_loop({bottom_tl, bottom_tr, top_tr, top_tl});
  const int upper_slot_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(upper_slot_loop) + ",.T.)");
  const int upper_slot_face = step.add("ADVANCED_FACE('',(" + step.ref(upper_slot_bound) + ")," +
                                       step.ref(upper_slot_plane) + ",.T.)");

  const int lower_slot_loop = add_line_loop({bottom_br, bottom_bl, top_bl, top_br});
  const int lower_slot_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(lower_slot_loop) + ",.T.)");
  const int lower_slot_face = step.add("ADVANCED_FACE('',(" + step.ref(lower_slot_bound) + ")," +
                                       step.ref(lower_slot_plane) + ",.T.)");

  const int right_cylinder_loop = add_edge_loop(
      {{top_right_arc, true}, {seam_br, false}, {bottom_right_arc, false}, {seam_tr, true}});
  const int right_cylinder_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(right_cylinder_loop) + ",.T.)");
  const int right_cylinder_face =
      step.add("ADVANCED_FACE('',(" + step.ref(right_cylinder_bound) + ")," +
               step.ref(right_cylinder) + ",.T.)");

  const int left_cylinder_loop = add_edge_loop(
      {{top_left_arc, true}, {seam_tl, false}, {bottom_left_arc, false}, {seam_bl, true}});
  const int left_cylinder_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(left_cylinder_loop) + ",.T.)");
  const int left_cylinder_face =
      step.add("ADVANCED_FACE('',(" + step.ref(left_cylinder_bound) + ")," +
               step.ref(left_cylinder) + ",.T.)");

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
    const int edge_loop = add_line_loop(face.loops.front());
    const int bound = step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)");
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + step.ref(bound) + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face) << ","
              << step.ref(upper_slot_face) << "," << step.ref(lower_slot_face) << ","
              << step.ref(right_cylinder_face) << "," << step.ref(left_cylinder_face);
  for (int face_id : side_face_ids) {
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
