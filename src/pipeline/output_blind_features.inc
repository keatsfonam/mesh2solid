bool write_step_file_prismatic_blind_counterbore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticBlindCounterboreExport& counterbore) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      const bool same_orientation = key.a == from && key.b == to;
      const int oriented_edge_id =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(oriented_edge_id);
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = counterbore.top_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(counterbore.upper_bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  const SimpleCylinderPrimitive& upper_cylinder = counterbore.upper_bore;
  const SimpleCylinderPrimitive& lower_cylinder = counterbore.lower_bore;
  const Vec3 top_center = upper_cylinder.base_center + upper_cylinder.axis * upper_cylinder.height;
  const Vec3 step_center = upper_cylinder.base_center;
  const Vec3 floor_center = lower_cylinder.base_center;

  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                      format_double(step_center.y) + "," +
                                      format_double(step_center.z) + "))");
  const int floor_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
               format_double(floor_center.y) + "," + format_double(floor_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(upper_cylinder.axis.x) + "," +
               format_double(upper_cylinder.axis.y) + "," + format_double(upper_cylinder.axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(upper_cylinder.ref_direction.x) + "," +
               format_double(upper_cylinder.ref_direction.y) + "," +
               format_double(upper_cylinder.ref_direction.z) + "))");

  const int top_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                      step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                      ")");
  const int step_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                                       step.ref(axis_direction) + "," + step.ref(plane_ref_direction) +
                                       ")");
  const int floor_plane_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                                        step.ref(axis_direction) + "," +
                                        step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_plane_axis) + ")");
  const int step_plane = step.add("PLANE(''," + step.ref(step_plane_axis) + ")");
  const int floor_plane = step.add("PLANE(''," + step.ref(floor_plane_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int top_center_point_local =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int axis_direction_local =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int ref_direction_local =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                       step.ref(axis_direction_local) + "," +
                                       step.ref(ref_direction_local) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");

    const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point_local) + "," +
                                  step.ref(axis_direction_local) + "," +
                                  step.ref(ref_direction_local) + ")");
    const int bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                 step.ref(axis_direction_local) + "," + step.ref(ref_direction_local) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                        step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                           step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
    const int bottom_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                 step.ref(bottom_cyl_pcurve) + "," + step.ref(bottom_plane_pcurve) +
                 "),.PCURVE_S1.)");
    const int bottom_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(bottom_seam_vertex) + "," + step.ref(bottom_surface_curve) + ",.T.)");

    const int lateral_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
    const int lateral_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.F.)");
    const int lateral_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.T.)");
    const int lateral_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.T.)");
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  const CylinderBundle upper_bundle = add_cylinder_bundle(upper_cylinder, top_plane, step_plane);
  const CylinderBundle lower_bundle = add_cylinder_bundle(lower_cylinder, step_plane, floor_plane);

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = counterbore.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundle.top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const bool step_outer_circle_forward = counterbore.step_outer_area > 0.0;
  const int step_outer_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(upper_bundle.bottom_edge_curve) + "," +
               std::string(step_outer_circle_forward ? ".T." : ".F.") + ")");
  const int step_outer_loop_id = step.add("EDGE_LOOP('',(" + step.ref(step_outer_oriented) + "))");
  const int step_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(step_outer_loop_id) + ",.T.)");
  const bool step_inner_circle_forward = counterbore.step_inner_area > 0.0;
  const int step_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundle.top_edge_curve) + "," +
               std::string(step_inner_circle_forward ? ".T." : ".F.") + ")");
  const int step_inner_loop_id = step.add("EDGE_LOOP('',(" + step.ref(step_inner_oriented) + "))");
  const int step_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(step_inner_loop_id) + ",.T.)");
  const int step_face = step.add("ADVANCED_FACE('',(" + step.ref(step_outer_bound) + "," +
                                 step.ref(step_inner_bound) + ")," + step.ref(step_plane) + ",.T.)");

  const bool floor_circle_forward = counterbore.floor_area > 0.0;
  const int floor_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(lower_bundle.bottom_edge_curve) + "," +
               std::string(floor_circle_forward ? ".T." : ".F.") + ")");
  const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
  const int floor_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
  const int floor_face = step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                  step.ref(floor_plane) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(counterbore.outer_face_indices.size());
  for (int face_index : counterbore.outer_face_indices) {
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

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(step_face) << "," << step.ref(floor_face)
              << "," << step.ref(upper_bundle.lateral_face) << ","
              << step.ref(lower_bundle.lateral_face);
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

bool write_step_file_prismatic_blind_bore(const std::filesystem::path& path,
                                          const ReconstructionResult& reconstruction,
                                          const SimplePrismaticBlindBoreExport& bore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      const bool same_orientation = key.a == from && key.b == to;
      const int oriented_edge_id =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(oriented_edge_id);
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = bore_export.top_outer_loop;
  const PlaneBasis axis_basis = basis_from_normal(bore_export.bore.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  const SimpleCylinderPrimitive& cylinder = bore_export.bore;
  const Vec3 top_center = cylinder.base_center + cylinder.axis * cylinder.height;
  const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
  const Vec3 top_seam = top_center + cylinder.ref_direction * cylinder.radius;
  const double circumference = 2.0 * kPi * cylinder.radius;

  const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                     format_double(top_seam.y) + "," +
                                     format_double(top_seam.z) + "))");
  const int bottom_seam_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
               format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
  const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
  const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

  const int base_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
               format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
  const int top_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
               format_double(top_center.y) + "," + format_double(top_center.z) + "))");
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
               format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
  const int ref_direction =
      step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
               format_double(cylinder.ref_direction.y) + "," +
               format_double(cylinder.ref_direction.z) + "))");

  const int cylinder_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                     step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface = step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                                        format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec =
      step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line = step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                                           step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(top_cyl_pcurve_repr) + ")");

  const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int top_circle_axis_2d = step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) +
                                          "," + step.ref(top_plane_ref_2d) + ")");
  const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                     format_double(cylinder.radius) + ")");
  const int top_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_plane_pcurve = step.add("PCURVE(''," + step.ref(top_plane) + "," +
                                        step.ref(top_circle_repr) + ")");
  const int top_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" +
                                         step.ref(top_cyl_pcurve) + "," + step.ref(top_plane_pcurve) +
                                         "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                 step.ref(seam_line_vec) + ")");

  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u0_repr) + ")");

  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u1_repr) + ")");
  const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                  step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                  "),.PCURVE_S1.)");
  const int seam_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(seam_curve) + ",.T.)");

  const int bottom_circle = step.add("CIRCLE(''," + step.ref(bottom_axis) + "," +
                                     format_double(cylinder.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                         step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
  const int bottom_circle_axis_2d =
      step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
               step.ref(bottom_plane_ref_2d) + ")");
  const int bottom_circle_2d = step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                                        format_double(cylinder.radius) + ")");
  const int bottom_circle_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_plane_pcurve = step.add("PCURVE(''," + step.ref(bottom_plane) + "," +
                                           step.ref(bottom_circle_repr) + ")");
  const int bottom_surface_curve = step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                                            step.ref(bottom_cyl_pcurve) + "," +
                                            step.ref(bottom_plane_pcurve) + "),.PCURVE_S1.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
               step.ref(bottom_seam_vertex) + "," + step.ref(bottom_surface_curve) + ",.T.)");

  const int lateral_top_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
  const int lateral_seam_forward =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.F.)");
  const int lateral_bottom_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.T.)");
  const int lateral_seam_backward =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.T.)");
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                    step.ref(lateral_seam_forward) + "," +
                                    step.ref(lateral_bottom_oriented) + "," +
                                    step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = bore_export.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," + step.ref(top_plane) + ",.T.)");

  const bool floor_circle_forward = bore_export.floor_area > 0.0;
  const int floor_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + "," +
               std::string(floor_circle_forward ? ".T." : ".F.") + ")");
  const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
  const int floor_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
  const int floor_face = step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                  step.ref(bottom_plane) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(bore_export.outer_face_indices.size());
  for (int face_index : bore_export.outer_face_indices) {
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

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(floor_face) << "," << step.ref(lateral_face);
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

bool write_step_file_prismatic_blind_countersink(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticBlindCountersinkExport& countersink_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      const bool same_orientation = key.a == from && key.b == to;
      const int oriented_edge_id =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(oriented_edge_id);
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = countersink_export.top_outer_loop;
  const Vec3 axis = countersink_export.countersink.axis;
  const Vec3 ref_direction = countersink_export.countersink.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }

  const SimpleConicalFrustumPrimitive& countersink = countersink_export.countersink;
  const SimpleCylinderPrimitive& bore = countersink_export.bore;
  const Vec3 top_center = countersink.base_center + countersink.axis * countersink.height;
  const Vec3 step_center = countersink.base_center;
  const Vec3 floor_center = bore.base_center;

  const StepPlaneSurface top_plane =
      add_step_plane_surface(step, top_center, axis, ref_direction);
  const StepPlaneSurface floor_plane =
      add_step_plane_surface(step, floor_center, axis, ref_direction);

  auto add_plane_circle_pcurve = [&](const StepPlaneSurface& plane,
                                     const Vec3& circle_center,
                                     double radius,
                                     const Vec3& circle_ref_direction) -> int {
    const Vec3 delta = circle_center - plane.origin;
    const double center_u = dot(delta, plane.basis.u);
    const double center_v = dot(delta, plane.basis.v);
    const double ref_u_raw = dot(circle_ref_direction, plane.basis.u);
    const double ref_v_raw = dot(circle_ref_direction, plane.basis.v);
    const double ref_scale = std::sqrt(ref_u_raw * ref_u_raw + ref_v_raw * ref_v_raw);
    const double ref_u = ref_scale <= 1e-12 ? 1.0 : ref_u_raw / ref_scale;
    const double ref_v = ref_scale <= 1e-12 ? 0.0 : ref_v_raw / ref_scale;

    const int center_2d =
        step.add("CARTESIAN_POINT('',(" + format_double(center_u) + "," +
                 format_double(center_v) + "))");
    const int ref_2d = step.add("DIRECTION('',(" + format_double(ref_u) + "," +
                                format_double(ref_v) + "))");
    const int circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(center_2d) + "," + step.ref(ref_2d) + ")");
    const int circle_2d =
        step.add("CIRCLE(''," + step.ref(circle_axis_2d) + "," + format_double(radius) + ")");
    const int circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    return step.add("PCURVE(''," + step.ref(plane.plane_id) + "," + step.ref(circle_repr) + ")");
  };

  struct CountersinkBundle {
    int top_edge_curve {};
    int floor_edge_curve {};
    int cone_lateral_face {};
    int cylinder_lateral_face {};
  };

  auto add_countersink_bundle = [&]() -> CountersinkBundle {
    const double cone_height = countersink.height;
    const double radius_delta = countersink.top_radius - countersink.base_radius;
    const double cone_angle = std::atan2(radius_delta, cone_height);
    const double slant_height = std::sqrt(cone_height * cone_height + radius_delta * radius_delta);
    if (slant_height <= 1e-12) {
      throw std::runtime_error("Cannot export zero-height blind countersink");
    }

    const Vec3 top_seam = top_center + countersink.ref_direction * countersink.top_radius;
    const Vec3 step_seam = step_center + countersink.ref_direction * countersink.base_radius;
    const Vec3 floor_seam = floor_center + bore.ref_direction * bore.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int step_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(step_seam.x) + "," +
                 format_double(step_seam.y) + "," + format_double(step_seam.z) + "))");
    const int floor_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(floor_seam.x) + "," +
                 format_double(floor_seam.y) + "," + format_double(floor_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int step_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(step_seam_point) + ")");
    const int floor_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(floor_seam_point) + ")");

    const int axis_direction_id =
        step.add("DIRECTION('',(" + format_double(countersink.axis.x) + "," +
                 format_double(countersink.axis.y) + "," + format_double(countersink.axis.z) + "))");
    const int ref_direction_id =
        step.add("DIRECTION('',(" + format_double(countersink.ref_direction.x) + "," +
                 format_double(countersink.ref_direction.y) + "," +
                 format_double(countersink.ref_direction.z) + "))");
    const int top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                 format_double(top_center.y) + "," + format_double(top_center.z) + "))");
    const int step_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                 format_double(step_center.y) + "," + format_double(step_center.z) + "))");
    const int floor_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
                 format_double(floor_center.y) + "," + format_double(floor_center.z) + "))");

    const int cone_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int cone_surface =
        step.add("CONICAL_SURFACE(''," + step.ref(cone_axis) + "," +
                 format_double(countersink.base_radius) + "," + format_double(cone_angle) + ")");
    const int top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int step_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(bore.radius) + ")");
    const int floor_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");

    const int top_circle =
        step.add("CIRCLE(''," + step.ref(top_axis) + "," + format_double(countersink.top_radius) + ")");
    const int top_cone_pcurve_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(slant_height) + "))");
    const int u_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int u_vec = step.add("VECTOR(''," + step.ref(u_dir) + ",1.0)");
    const int top_cone_pcurve_line =
        step.add("LINE(''," + step.ref(top_cone_pcurve_point) + "," + step.ref(u_vec) + ")");
    const int top_cone_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cone_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cone_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(top_cone_pcurve_repr) + ")");
    const int top_plane_pcurve =
        add_plane_circle_pcurve(top_plane, top_center, countersink.top_radius, countersink.ref_direction);
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cone_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int transition_circle =
        step.add("CIRCLE(''," + step.ref(step_axis) + "," + format_double(countersink.base_radius) + ")");
    const int cone_transition_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int cone_transition_line =
        step.add("LINE(''," + step.ref(cone_transition_point) + "," + step.ref(u_vec) + ")");
    const int cone_transition_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cone_transition_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cone_transition_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(cone_transition_repr) + ")");
    const int cylinder_top_pcurve_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(bore.height) + "))");
    const int cylinder_top_pcurve_line =
        step.add("LINE(''," + step.ref(cylinder_top_pcurve_point) + "," + step.ref(u_vec) + ")");
    const int cylinder_top_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cylinder_top_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cylinder_top_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(cylinder_top_pcurve_repr) + ")");
    const int transition_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(transition_circle) + ",(" +
                 step.ref(cone_transition_pcurve) + "," + step.ref(cylinder_top_pcurve) +
                 "),.PCURVE_S1.)");
    const int transition_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(step_seam_vertex) + "," + step.ref(step_seam_vertex) + "," +
                 step.ref(transition_surface_curve) + ",.T.)");

    const int floor_circle =
        step.add("CIRCLE(''," + step.ref(floor_axis) + "," + format_double(bore.radius) + ")");
    const int floor_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int floor_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(floor_cyl_pcurve_geom_point) + "," + step.ref(u_vec) + ")");
    const int floor_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(floor_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int floor_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(floor_cyl_pcurve_repr) + ")");
    const int floor_plane_pcurve =
        add_plane_circle_pcurve(floor_plane, floor_center, bore.radius, bore.ref_direction);
    const int floor_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(floor_circle) + ",(" +
                 step.ref(floor_cyl_pcurve) + "," + step.ref(floor_plane_pcurve) +
                 "),.PCURVE_S1.)");
    const int floor_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(floor_seam_vertex) + "," +
                 step.ref(floor_seam_vertex) + "," + step.ref(floor_surface_curve) + ",.T.)");

    const int cone_seam_line_dir =
        step.add("DIRECTION('',(" + format_double(countersink.axis.x) + "," +
                 format_double(countersink.axis.y) + "," + format_double(countersink.axis.z) + "))");
    const int cone_seam_line_vec = step.add("VECTOR(''," + step.ref(cone_seam_line_dir) + ",1.0)");
    const int cone_seam_line =
        step.add("LINE(''," + step.ref(step_seam_point) + "," + step.ref(cone_seam_line_vec) + ")");
    const int cone_seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int cone_seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int cone_seam_v_vec = step.add("VECTOR(''," + step.ref(cone_seam_v_dir) + ",1.0)");
    const int cone_seam_u0_line =
        step.add("LINE(''," + step.ref(cone_seam_u0_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cone_seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cone_seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cone_seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(cone_seam_u0_repr) + ")");
    const int cone_seam_u1_point = step.add("CARTESIAN_POINT('',(" + format_double(2.0 * kPi) + ",0.0))");
    const int cone_seam_u1_line =
        step.add("LINE(''," + step.ref(cone_seam_u1_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cone_seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cone_seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cone_seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(cone_seam_u1_repr) + ")");
    const int cone_seam_curve =
        step.add("SEAM_CURVE(''," + step.ref(cone_seam_line) + ",(" + step.ref(cone_seam_u1_pcurve) +
                 "," + step.ref(cone_seam_u0_pcurve) + "),.PCURVE_S1.)");
    const int cone_seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(step_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(cone_seam_curve) + ",.T.)");

    const double circumference = 2.0 * kPi * bore.radius;
    const int cylinder_seam_line =
        step.add("LINE(''," + step.ref(floor_seam_point) + "," + step.ref(cone_seam_line_vec) + ")");
    const int cylinder_seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int cylinder_seam_u0_line =
        step.add("LINE(''," + step.ref(cylinder_seam_u0_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cylinder_seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cylinder_seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cylinder_seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(cylinder_seam_u0_repr) + ")");
    const int cylinder_seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int cylinder_seam_u1_line =
        step.add("LINE(''," + step.ref(cylinder_seam_u1_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cylinder_seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cylinder_seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cylinder_seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(cylinder_seam_u1_repr) + ")");
    const int cylinder_seam_curve =
        step.add("SEAM_CURVE(''," + step.ref(cylinder_seam_line) + ",(" +
                 step.ref(cylinder_seam_u1_pcurve) + "," + step.ref(cylinder_seam_u0_pcurve) +
                 "),.PCURVE_S1.)");
    const int cylinder_seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(floor_seam_vertex) + "," +
                 step.ref(step_seam_vertex) + "," + step.ref(cylinder_seam_curve) + ",.T.)");

    const int cone_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
    const int cone_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cone_seam_edge_curve) + ",.F.)");
    const int cone_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(transition_edge_curve) + ",.T.)");
    const int cone_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cone_seam_edge_curve) + ",.T.)");
    const int cone_lateral_loop =
        step.add("EDGE_LOOP('',(" + step.ref(cone_top_oriented) + "," +
                 step.ref(cone_seam_forward) + "," + step.ref(cone_bottom_oriented) + "," +
                 step.ref(cone_seam_backward) + "))");
    const int cone_lateral_bound =
        step.add("FACE_BOUND(''," + step.ref(cone_lateral_loop) + ",.T.)");
    const int cone_lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(cone_lateral_bound) + ")," +
                 step.ref(cone_surface) + ",.T.)");

    const int cylinder_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(transition_edge_curve) + ",.F.)");
    const int cylinder_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cylinder_seam_edge_curve) + ",.F.)");
    const int cylinder_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(floor_edge_curve) + ",.T.)");
    const int cylinder_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cylinder_seam_edge_curve) + ",.T.)");
    const int cylinder_lateral_loop =
        step.add("EDGE_LOOP('',(" + step.ref(cylinder_top_oriented) + "," +
                 step.ref(cylinder_seam_forward) + "," + step.ref(cylinder_bottom_oriented) + "," +
                 step.ref(cylinder_seam_backward) + "))");
    const int cylinder_lateral_bound =
        step.add("FACE_BOUND(''," + step.ref(cylinder_lateral_loop) + ",.T.)");
    const int cylinder_lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(cylinder_lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CountersinkBundle {
        top_edge_curve,
        floor_edge_curve,
        cone_lateral_face,
        cylinder_lateral_face,
    };
  };

  const CountersinkBundle bundle = add_countersink_bundle();

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = countersink_export.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bundle.top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop_id = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(top_inner_loop_id) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," +
                                step.ref(top_inner_bound) + ")," +
                                step.ref(top_plane.plane_id) + ",.T.)");

  const bool floor_circle_forward = countersink_export.floor_area > 0.0;
  const int floor_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bundle.floor_edge_curve) + "," +
               std::string(floor_circle_forward ? ".T." : ".F.") + ")");
  const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
  const int floor_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
  const int floor_face = step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                  step.ref(floor_plane.plane_id) + ",.T.)");

  std::vector<int> side_face_ids;
  side_face_ids.reserve(countersink_export.outer_face_indices.size());
  for (int face_index : countersink_export.outer_face_indices) {
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
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(floor_face) << ","
              << step.ref(bundle.cone_lateral_face) << ","
              << step.ref(bundle.cylinder_lateral_face);
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


bool write_step_file_prismatic_multi_blind_bore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticMultiBlindBoreExport& bore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      const bool same_orientation = key.a == from && key.b == to;
      const int oriented_edge_id =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(oriented_edge_id);
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = bore_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = bore_export.bottom_outer_loop;
  const Vec3 axis = bore_export.bores.front().bore.axis;
  const Vec3 ref_direction = bore_export.bores.front().bore.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_center = reconstruction.faces[bore_export.top_face_index].fit.centroid;
  const Vec3 bottom_center = reconstruction.faces[bore_export.bottom_face_index].fit.centroid;
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");
    const int local_top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int local_bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(local_top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(local_bottom_axis) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
    const int bottom_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                 step.ref(bottom_cyl_pcurve) + "," + step.ref(bottom_plane_pcurve) +
                 "),.PCURVE_S1.)");
    const int bottom_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(bottom_seam_vertex) + "," + step.ref(bottom_surface_curve) + ",.T.)");

    const int lateral_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
    const int lateral_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.F.)");
    const int lateral_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.T.)");
    const int lateral_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.T.)");
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");
    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  struct BoreFaceBundle {
    int floor_plane {};
    CylinderBundle cylinder;
  };

  std::vector<BoreFaceBundle> bore_bundles;
  bore_bundles.reserve(bore_export.bores.size());
  for (const SimplePrismaticMultiBlindBore& bore : bore_export.bores) {
    const Vec3 floor_center = bore.bore.base_center;
    const int floor_center_point = step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
                                        format_double(floor_center.y) + "," +
                                        format_double(floor_center.z) + "))");
    const int floor_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                                    step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int floor_plane = step.add("PLANE(''," + step.ref(floor_axis) + ")");
    bore_bundles.push_back(BoreFaceBundle {floor_plane, add_cylinder_bundle(bore.bore, top_plane, floor_plane)});
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  std::vector<int> top_bound_ids;
  top_bound_ids.reserve(1 + bore_export.bores.size());
  top_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < bore_export.bores.size(); ++index) {
    const bool top_circle_forward = bore_export.bores[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bore_bundles[index].cylinder.top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> floor_face_ids;
  floor_face_ids.reserve(bore_export.bores.size());
  for (std::size_t index = 0; index < bore_export.bores.size(); ++index) {
    const bool floor_circle_forward = bore_export.bores[index].floor_area > 0.0;
    const int floor_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bore_bundles[index].cylinder.bottom_edge_curve) + "," +
                 std::string(floor_circle_forward ? ".T." : ".F.") + ")");
    const int floor_loop_id = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
    const int floor_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop_id) + ",.T.)");
    floor_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                      step.ref(bore_bundles[index].floor_plane) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(bore_export.outer_face_indices.size());
  for (int face_index : bore_export.outer_face_indices) {
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
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (std::size_t index = 0; index < bore_export.bores.size(); ++index) {
    shell_faces << "," << step.ref(floor_face_ids[index]) << ","
                << step.ref(bore_bundles[index].cylinder.lateral_face);
  }
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

bool write_step_file_prismatic_multi_blind_counterbore(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticMultiBlindCounterboreExport& counterbore_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      const bool same_orientation = key.a == from && key.b == to;
      const int oriented_edge_id =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(oriented_edge_id);
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = counterbore_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = counterbore_export.bottom_outer_loop;
  const Vec3 axis = counterbore_export.counterbores.front().upper_bore.axis;
  const Vec3 ref_direction = counterbore_export.counterbores.front().upper_bore.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_center = reconstruction.faces[counterbore_export.top_face_index].fit.centroid;
  const Vec3 bottom_center = reconstruction.faces[counterbore_export.bottom_face_index].fit.centroid;
  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," + format_double(axis.y) + "," +
               format_double(axis.z) + "))");
  const int plane_ref_direction =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
  const int top_plane = step.add("PLANE(''," + step.ref(top_axis) + ")");
  const int bottom_plane = step.add("PLANE(''," + step.ref(bottom_axis) + ")");

  struct CylinderBundle {
    int top_edge_curve {};
    int bottom_edge_curve {};
    int seam_edge_curve {};
    int lateral_face {};
  };

  auto add_cylinder_bundle =
      [&](const SimpleCylinderPrimitive& cylinder, int top_plane_id, int bottom_plane_id) -> CylinderBundle {
    const Vec3 local_top_center = cylinder.base_center + cylinder.axis * cylinder.height;
    const Vec3 bottom_seam = cylinder.base_center + cylinder.ref_direction * cylinder.radius;
    const Vec3 top_seam = local_top_center + cylinder.ref_direction * cylinder.radius;
    const double circumference = 2.0 * kPi * cylinder.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int bottom_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
                 format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

    const int local_base_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(cylinder.base_center.x) + "," +
                 format_double(cylinder.base_center.y) + "," + format_double(cylinder.base_center.z) + "))");
    const int local_top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(local_top_center.x) + "," +
                 format_double(local_top_center.y) + "," + format_double(local_top_center.z) + "))");
    const int local_axis_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                 format_double(cylinder.axis.y) + "," + format_double(cylinder.axis.z) + "))");
    const int local_ref_direction =
        step.add("DIRECTION('',(" + format_double(cylinder.ref_direction.x) + "," +
                 format_double(cylinder.ref_direction.y) + "," +
                 format_double(cylinder.ref_direction.z) + "))");

    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(cylinder.radius) + ")");
    const int local_top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_top_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");
    const int local_bottom_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(local_base_center_point) + "," +
                 step.ref(local_axis_direction) + "," + step.ref(local_ref_direction) + ")");

    const int top_circle = step.add("CIRCLE(''," + step.ref(local_top_axis) + "," +
                                    format_double(cylinder.radius) + ")");
    const int top_cyl_pcurve_geom_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
    const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int top_cyl_pcurve_vec =
        step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
    const int top_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int top_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");

    const int top_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int top_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int top_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(top_plane_origin_2d) + "," +
                 step.ref(top_plane_ref_2d) + ")");
    const int top_circle_2d = step.add("CIRCLE(''," + step.ref(top_circle_axis_2d) + "," +
                                       format_double(cylinder.radius) + ")");
    const int top_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_plane_pcurve =
        step.add("PCURVE(''," + step.ref(top_plane_id) + "," + step.ref(top_circle_repr) + ")");
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                       format_double(cylinder.axis.y) + "," +
                                       format_double(cylinder.axis.z) + "))");
    const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
    const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                   step.ref(seam_line_vec) + ")");
    const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
    const int seam_u0_line = step.add("LINE(''," + step.ref(seam_u0_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
    const int seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                      step.ref(seam_v_vec) + ")");
    const int seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
    const int seam_curve = step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" +
                                    step.ref(seam_u1_pcurve) + "," + step.ref(seam_u0_pcurve) +
                                    "),.PCURVE_S1.)");
    const int seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(top_seam_vertex) + "," + step.ref(seam_curve) + ",.T.)");

    const int bottom_circle = step.add("CIRCLE(''," + step.ref(local_bottom_axis) + "," +
                                       format_double(cylinder.radius) + ")");
    const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," +
                 step.ref(top_cyl_pcurve_vec) + ")");
    const int bottom_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
    const int bottom_plane_origin_2d = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int bottom_plane_ref_2d = step.add("DIRECTION('',(1.0,0.0))");
    const int bottom_circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(bottom_plane_origin_2d) + "," +
                 step.ref(bottom_plane_ref_2d) + ")");
    const int bottom_circle_2d =
        step.add("CIRCLE(''," + step.ref(bottom_circle_axis_2d) + "," +
                 format_double(cylinder.radius) + ")");
    const int bottom_circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int bottom_plane_pcurve =
        step.add("PCURVE(''," + step.ref(bottom_plane_id) + "," + step.ref(bottom_circle_repr) + ")");
    const int bottom_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
                 step.ref(bottom_cyl_pcurve) + "," + step.ref(bottom_plane_pcurve) +
                 "),.PCURVE_S1.)");
    const int bottom_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
                 step.ref(bottom_seam_vertex) + "," + step.ref(bottom_surface_curve) + ",.T.)");

    const int lateral_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
    const int lateral_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.F.)");
    const int lateral_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.T.)");
    const int lateral_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(seam_edge_curve) + ",.T.)");
    const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
                                      step.ref(lateral_seam_forward) + "," +
                                      step.ref(lateral_bottom_oriented) + "," +
                                      step.ref(lateral_seam_backward) + "))");
    const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
    const int lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");
    return CylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
  };

  struct CounterboreFaceBundle {
    int step_plane {};
    int floor_plane {};
    CylinderBundle upper_cylinder;
    CylinderBundle lower_cylinder;
  };

  std::vector<CounterboreFaceBundle> bundles;
  bundles.reserve(counterbore_export.counterbores.size());
  for (const SimplePrismaticMultiBlindCounterbore& counterbore : counterbore_export.counterbores) {
    const Vec3 step_center = counterbore.upper_bore.base_center;
    const Vec3 floor_center = counterbore.lower_bore.base_center;
    const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                       format_double(step_center.y) + "," +
                                       format_double(step_center.z) + "))");
    const int floor_center_point = step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
                                        format_double(floor_center.y) + "," +
                                        format_double(floor_center.z) + "))");
    const int step_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                                   step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int floor_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                                    step.ref(axis_direction) + "," + step.ref(plane_ref_direction) + ")");
    const int step_plane = step.add("PLANE(''," + step.ref(step_axis) + ")");
    const int floor_plane = step.add("PLANE(''," + step.ref(floor_axis) + ")");
    bundles.push_back(CounterboreFaceBundle {
        step_plane,
        floor_plane,
        add_cylinder_bundle(counterbore.upper_bore, top_plane, step_plane),
        add_cylinder_bundle(counterbore.lower_bore, step_plane, floor_plane),
    });
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  std::vector<int> top_bound_ids;
  top_bound_ids.reserve(1 + counterbore_export.counterbores.size());
  top_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool top_circle_forward = counterbore_export.counterbores[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].upper_cylinder.top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  std::vector<int> step_face_ids;
  std::vector<int> floor_face_ids;
  step_face_ids.reserve(counterbore_export.counterbores.size());
  floor_face_ids.reserve(counterbore_export.counterbores.size());
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    const bool step_outer_circle_forward = counterbore_export.counterbores[index].step_outer_area > 0.0;
    const int step_outer_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].upper_cylinder.bottom_edge_curve) + "," +
                 std::string(step_outer_circle_forward ? ".T." : ".F.") + ")");
    const int step_outer_loop = step.add("EDGE_LOOP('',(" + step.ref(step_outer_oriented) + "))");
    const int step_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(step_outer_loop) + ",.T.)");

    const bool step_inner_circle_forward = counterbore_export.counterbores[index].step_inner_area > 0.0;
    const int step_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].lower_cylinder.top_edge_curve) + "," +
                 std::string(step_inner_circle_forward ? ".T." : ".F.") + ")");
    const int step_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(step_inner_oriented) + "))");
    const int step_inner_bound =
        step.add("FACE_BOUND(''," + step.ref(step_inner_loop) + ",.T.)");
    step_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(step_outer_bound) + "," +
                                     step.ref(step_inner_bound) + ")," +
                                     step.ref(bundles[index].step_plane) + ",.T.)"));

    const bool floor_circle_forward = counterbore_export.counterbores[index].floor_area > 0.0;
    const int floor_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].lower_cylinder.bottom_edge_curve) + "," +
                 std::string(floor_circle_forward ? ".T." : ".F.") + ")");
    const int floor_loop = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
    const int floor_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop) + ",.T.)");
    floor_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                      step.ref(bundles[index].floor_plane) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(counterbore_export.outer_face_indices.size());
  for (int face_index : counterbore_export.outer_face_indices) {
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
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (std::size_t index = 0; index < counterbore_export.counterbores.size(); ++index) {
    shell_faces << "," << step.ref(step_face_ids[index]) << "," << step.ref(floor_face_ids[index]) << ","
                << step.ref(bundles[index].upper_cylinder.lateral_face) << ","
                << step.ref(bundles[index].lower_cylinder.lateral_face);
  }
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

bool write_step_file_prismatic_multi_blind_countersink(
    const std::filesystem::path& path,
    const ReconstructionResult& reconstruction,
    const SimplePrismaticMultiBlindCountersinkExport& countersink_export) {
  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);

  std::vector<int> point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    const Vec3& vertex = reconstruction.vertices[index];
    point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                   format_double(vertex.y) + "," + format_double(vertex.z) + "))");
  }

  std::vector<int> vertex_point_ids(reconstruction.vertices.size(), 0);
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
  }

  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
  auto get_line_edge_curve_id = [&](int from, int to) -> int {
    const EdgeKey key = make_edge_key(from, to);
    const auto found = edge_curve_ids.find(key);
    if (found != edge_curve_ids.end()) {
      return found->second;
    }

    const Vec3 start = reconstruction.vertices[key.a];
    const Vec3 end = reconstruction.vertices[key.b];
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  auto add_line_loop = [&](const std::vector<int>& loop) -> int {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = get_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      const bool same_orientation = key.a == from && key.b == to;
      const int oriented_edge_id =
          step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                   std::string(same_orientation ? ".T." : ".F.") + ")");
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(oriented_edge_id);
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  };

  std::vector<int> top_outer_loop = countersink_export.top_outer_loop;
  std::vector<int> bottom_outer_loop = countersink_export.bottom_outer_loop;
  const Vec3 axis = countersink_export.countersinks.front().countersink.axis;
  const Vec3 ref_direction = countersink_export.countersinks.front().countersink.ref_direction;
  const PlaneBasis axis_basis = basis_from_normal(axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const Vec3 top_plane_origin =
      countersink_export.countersinks.front().countersink.base_center +
      axis * countersink_export.countersinks.front().countersink.height;
  const Vec3 bottom_plane_origin = reconstruction.faces[countersink_export.bottom_face_index].fit.centroid;
  const StepPlaneSurface top_plane =
      add_step_plane_surface(step, top_plane_origin, axis, ref_direction);
  const StepPlaneSurface bottom_plane =
      add_step_plane_surface(step, bottom_plane_origin, axis, ref_direction);

  auto add_plane_circle_pcurve = [&](const StepPlaneSurface& plane,
                                     const Vec3& circle_center,
                                     double radius,
                                     const Vec3& circle_ref_direction) -> int {
    const Vec3 delta = circle_center - plane.origin;
    const double center_u = dot(delta, plane.basis.u);
    const double center_v = dot(delta, plane.basis.v);
    const double ref_u_raw = dot(circle_ref_direction, plane.basis.u);
    const double ref_v_raw = dot(circle_ref_direction, plane.basis.v);
    const double ref_scale = std::sqrt(ref_u_raw * ref_u_raw + ref_v_raw * ref_v_raw);
    const double ref_u = ref_scale <= 1e-12 ? 1.0 : ref_u_raw / ref_scale;
    const double ref_v = ref_scale <= 1e-12 ? 0.0 : ref_v_raw / ref_scale;

    const int center_2d =
        step.add("CARTESIAN_POINT('',(" + format_double(center_u) + "," +
                 format_double(center_v) + "))");
    const int ref_2d = step.add("DIRECTION('',(" + format_double(ref_u) + "," +
                                format_double(ref_v) + "))");
    const int circle_axis_2d =
        step.add("AXIS2_PLACEMENT_2D(''," + step.ref(center_2d) + "," + step.ref(ref_2d) + ")");
    const int circle_2d =
        step.add("CIRCLE(''," + step.ref(circle_axis_2d) + "," + format_double(radius) + ")");
    const int circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    return step.add("PCURVE(''," + step.ref(plane.plane_id) + "," + step.ref(circle_repr) + ")");
  };

  struct CountersinkBundle {
    int top_edge_curve {};
    int floor_edge_curve {};
    int cone_lateral_face {};
    int cylinder_lateral_face {};
    int floor_plane_id {};
  };

  auto add_countersink_bundle =
      [&](const SimplePrismaticMultiBlindCountersink& feature) -> CountersinkBundle {
    const SimpleConicalFrustumPrimitive& countersink = feature.countersink;
    const SimpleCylinderPrimitive& bore = feature.bore;
    const Vec3 top_center = countersink.base_center + countersink.axis * countersink.height;
    const Vec3 step_center = countersink.base_center;
    const Vec3 floor_center = bore.base_center;
    const StepPlaneSurface floor_plane_surface =
        add_step_plane_surface(step, floor_center, axis, ref_direction);
    const double cone_height = countersink.height;
    const double radius_delta = countersink.top_radius - countersink.base_radius;
    const double cone_angle = std::atan2(radius_delta, cone_height);
    const double slant_height = std::sqrt(cone_height * cone_height + radius_delta * radius_delta);
    if (slant_height <= 1e-12) {
      throw std::runtime_error("Cannot export zero-height blind countersink");
    }

    const Vec3 top_seam = top_center + countersink.ref_direction * countersink.top_radius;
    const Vec3 step_seam = step_center + countersink.ref_direction * countersink.base_radius;
    const Vec3 floor_seam = floor_center + bore.ref_direction * bore.radius;

    const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                       format_double(top_seam.y) + "," +
                                       format_double(top_seam.z) + "))");
    const int step_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(step_seam.x) + "," +
                 format_double(step_seam.y) + "," + format_double(step_seam.z) + "))");
    const int floor_seam_point =
        step.add("CARTESIAN_POINT('',(" + format_double(floor_seam.x) + "," +
                 format_double(floor_seam.y) + "," + format_double(floor_seam.z) + "))");
    const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
    const int step_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(step_seam_point) + ")");
    const int floor_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(floor_seam_point) + ")");

    const int axis_direction_id =
        step.add("DIRECTION('',(" + format_double(countersink.axis.x) + "," +
                 format_double(countersink.axis.y) + "," + format_double(countersink.axis.z) + "))");
    const int ref_direction_id =
        step.add("DIRECTION('',(" + format_double(countersink.ref_direction.x) + "," +
                 format_double(countersink.ref_direction.y) + "," +
                 format_double(countersink.ref_direction.z) + "))");
    const int top_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                 format_double(top_center.y) + "," + format_double(top_center.z) + "))");
    const int step_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                 format_double(step_center.y) + "," + format_double(step_center.z) + "))");
    const int floor_center_point =
        step.add("CARTESIAN_POINT('',(" + format_double(floor_center.x) + "," +
                 format_double(floor_center.y) + "," + format_double(floor_center.z) + "))");

    const int cone_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int cone_surface =
        step.add("CONICAL_SURFACE(''," + step.ref(cone_axis) + "," +
                 format_double(countersink.base_radius) + "," + format_double(cone_angle) + ")");
    const int top_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int step_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(step_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int cylinder_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
    const int cylinder_surface =
        step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
                 format_double(bore.radius) + ")");
    const int floor_axis =
        step.add("AXIS2_PLACEMENT_3D(''," + step.ref(floor_center_point) + "," +
                 step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");

    const int top_circle =
        step.add("CIRCLE(''," + step.ref(top_axis) + "," + format_double(countersink.top_radius) + ")");
    const int top_cone_pcurve_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(slant_height) + "))");
    const int u_dir = step.add("DIRECTION('',(1.0,0.0))");
    const int u_vec = step.add("VECTOR(''," + step.ref(u_dir) + ",1.0)");
    const int top_cone_pcurve_line =
        step.add("LINE(''," + step.ref(top_cone_pcurve_point) + "," + step.ref(u_vec) + ")");
    const int top_cone_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cone_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int top_cone_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(top_cone_pcurve_repr) + ")");
    const int top_plane_pcurve =
        add_plane_circle_pcurve(top_plane, top_center, countersink.top_radius, countersink.ref_direction);
    const int top_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cone_pcurve) +
                 "," + step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
    const int top_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(top_surface_curve) + ",.T.)");

    const int transition_circle =
        step.add("CIRCLE(''," + step.ref(step_axis) + "," + format_double(countersink.base_radius) + ")");
    const int cone_transition_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int cone_transition_line =
        step.add("LINE(''," + step.ref(cone_transition_point) + "," + step.ref(u_vec) + ")");
    const int cone_transition_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cone_transition_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cone_transition_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(cone_transition_repr) + ")");
    const int cylinder_top_pcurve_point =
        step.add("CARTESIAN_POINT('',(0.0," + format_double(bore.height) + "))");
    const int cylinder_top_pcurve_line =
        step.add("LINE(''," + step.ref(cylinder_top_pcurve_point) + "," + step.ref(u_vec) + ")");
    const int cylinder_top_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cylinder_top_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cylinder_top_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(cylinder_top_pcurve_repr) + ")");
    const int transition_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(transition_circle) + ",(" +
                 step.ref(cone_transition_pcurve) + "," + step.ref(cylinder_top_pcurve) +
                 "),.PCURVE_S1.)");
    const int transition_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(step_seam_vertex) + "," + step.ref(step_seam_vertex) + "," +
                 step.ref(transition_surface_curve) + ",.T.)");

    const int floor_circle =
        step.add("CIRCLE(''," + step.ref(floor_axis) + "," + format_double(bore.radius) + ")");
    const int floor_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int floor_cyl_pcurve_line =
        step.add("LINE(''," + step.ref(floor_cyl_pcurve_geom_point) + "," + step.ref(u_vec) + ")");
    const int floor_cyl_pcurve_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(floor_cyl_pcurve_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int floor_cyl_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(floor_cyl_pcurve_repr) + ")");
    const int floor_plane_pcurve =
        add_plane_circle_pcurve(floor_plane_surface, floor_center, bore.radius, bore.ref_direction);
    const int floor_surface_curve =
        step.add("SURFACE_CURVE(''," + step.ref(floor_circle) + ",(" +
                 step.ref(floor_cyl_pcurve) + "," + step.ref(floor_plane_pcurve) +
                 "),.PCURVE_S1.)");
    const int floor_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(floor_seam_vertex) + "," +
                 step.ref(floor_seam_vertex) + "," + step.ref(floor_surface_curve) + ",.T.)");

    const int cone_seam_line_dir =
        step.add("DIRECTION('',(" + format_double(countersink.axis.x) + "," +
                 format_double(countersink.axis.y) + "," + format_double(countersink.axis.z) + "))");
    const int cone_seam_line_vec = step.add("VECTOR(''," + step.ref(cone_seam_line_dir) + ",1.0)");
    const int cone_seam_line =
        step.add("LINE(''," + step.ref(step_seam_point) + "," + step.ref(cone_seam_line_vec) + ")");
    const int cone_seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int cone_seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
    const int cone_seam_v_vec = step.add("VECTOR(''," + step.ref(cone_seam_v_dir) + ",1.0)");
    const int cone_seam_u0_line =
        step.add("LINE(''," + step.ref(cone_seam_u0_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cone_seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cone_seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cone_seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(cone_seam_u0_repr) + ")");
    const int cone_seam_u1_point = step.add("CARTESIAN_POINT('',(" + format_double(2.0 * kPi) + ",0.0))");
    const int cone_seam_u1_line =
        step.add("LINE(''," + step.ref(cone_seam_u1_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cone_seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cone_seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cone_seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cone_surface) + "," + step.ref(cone_seam_u1_repr) + ")");
    const int cone_seam_curve =
        step.add("SEAM_CURVE(''," + step.ref(cone_seam_line) + ",(" + step.ref(cone_seam_u1_pcurve) +
                 "," + step.ref(cone_seam_u0_pcurve) + "),.PCURVE_S1.)");
    const int cone_seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(step_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
                 step.ref(cone_seam_curve) + ",.T.)");

    const double circumference = 2.0 * kPi * bore.radius;
    const int cylinder_seam_line =
        step.add("LINE(''," + step.ref(floor_seam_point) + "," + step.ref(cone_seam_line_vec) + ")");
    const int cylinder_seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
    const int cylinder_seam_u0_line =
        step.add("LINE(''," + step.ref(cylinder_seam_u0_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cylinder_seam_u0_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cylinder_seam_u0_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cylinder_seam_u0_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(cylinder_seam_u0_repr) + ")");
    const int cylinder_seam_u1_point =
        step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
    const int cylinder_seam_u1_line =
        step.add("LINE(''," + step.ref(cylinder_seam_u1_point) + "," + step.ref(cone_seam_v_vec) + ")");
    const int cylinder_seam_u1_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(cylinder_seam_u1_line) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    const int cylinder_seam_u1_pcurve =
        step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(cylinder_seam_u1_repr) + ")");
    const int cylinder_seam_curve =
        step.add("SEAM_CURVE(''," + step.ref(cylinder_seam_line) + ",(" +
                 step.ref(cylinder_seam_u1_pcurve) + "," + step.ref(cylinder_seam_u0_pcurve) +
                 "),.PCURVE_S1.)");
    const int cylinder_seam_edge_curve =
        step.add("EDGE_CURVE(''," + step.ref(floor_seam_vertex) + "," +
                 step.ref(step_seam_vertex) + "," + step.ref(cylinder_seam_curve) + ",.T.)");

    const int cone_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.F.)");
    const int cone_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cone_seam_edge_curve) + ",.F.)");
    const int cone_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(transition_edge_curve) + ",.T.)");
    const int cone_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cone_seam_edge_curve) + ",.T.)");
    const int cone_lateral_loop =
        step.add("EDGE_LOOP('',(" + step.ref(cone_top_oriented) + "," +
                 step.ref(cone_seam_forward) + "," + step.ref(cone_bottom_oriented) + "," +
                 step.ref(cone_seam_backward) + "))");
    const int cone_lateral_bound =
        step.add("FACE_BOUND(''," + step.ref(cone_lateral_loop) + ",.T.)");
    const int cone_lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(cone_lateral_bound) + ")," +
                 step.ref(cone_surface) + ",.T.)");

    const int cylinder_top_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(transition_edge_curve) + ",.F.)");
    const int cylinder_seam_forward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cylinder_seam_edge_curve) + ",.F.)");
    const int cylinder_bottom_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(floor_edge_curve) + ",.T.)");
    const int cylinder_seam_backward =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(cylinder_seam_edge_curve) + ",.T.)");
    const int cylinder_lateral_loop =
        step.add("EDGE_LOOP('',(" + step.ref(cylinder_top_oriented) + "," +
                 step.ref(cylinder_seam_forward) + "," + step.ref(cylinder_bottom_oriented) + "," +
                 step.ref(cylinder_seam_backward) + "))");
    const int cylinder_lateral_bound =
        step.add("FACE_BOUND(''," + step.ref(cylinder_lateral_loop) + ",.T.)");
    const int cylinder_lateral_face =
        step.add("ADVANCED_FACE('',(" + step.ref(cylinder_lateral_bound) + ")," +
                 step.ref(cylinder_surface) + ",.T.)");

    return CountersinkBundle {
        top_edge_curve,
        floor_edge_curve,
        cone_lateral_face,
        cylinder_lateral_face,
        floor_plane_surface.plane_id,
    };
  };

  std::vector<CountersinkBundle> bundles;
  bundles.reserve(countersink_export.countersinks.size());
  for (const SimplePrismaticMultiBlindCountersink& countersink : countersink_export.countersinks) {
    bundles.push_back(add_countersink_bundle(countersink));
  }

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  std::vector<int> top_bound_ids;
  top_bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)"));
  for (std::size_t index = 0; index < countersink_export.countersinks.size(); ++index) {
    const bool top_circle_forward = countersink_export.countersinks[index].top_inner_area > 0.0;
    const int top_inner_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].top_edge_curve) + "," +
                 std::string(top_circle_forward ? ".T." : ".F.") + ")");
    const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
    top_bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)"));
  }
  std::ostringstream top_bound_refs;
  for (std::size_t index = 0; index < top_bound_ids.size(); ++index) {
    if (index > 0) {
      top_bound_refs << ",";
    }
    top_bound_refs << step.ref(top_bound_ids[index]);
  }
  const int top_face = step.add("ADVANCED_FACE('',(" + top_bound_refs.str() + ")," +
                                step.ref(top_plane.plane_id) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + ")," +
                                   step.ref(bottom_plane.plane_id) + ",.F.)");

  std::vector<int> floor_face_ids;
  floor_face_ids.reserve(countersink_export.countersinks.size());
  for (std::size_t index = 0; index < countersink_export.countersinks.size(); ++index) {
    const bool floor_circle_forward = countersink_export.countersinks[index].floor_area > 0.0;
    const int floor_oriented =
        step.add("ORIENTED_EDGE('',*,*," + step.ref(bundles[index].floor_edge_curve) + "," +
                 std::string(floor_circle_forward ? ".T." : ".F.") + ")");
    const int floor_loop = step.add("EDGE_LOOP('',(" + step.ref(floor_oriented) + "))");
    const int floor_outer_bound =
        step.add("FACE_OUTER_BOUND(''," + step.ref(floor_loop) + ",.T.)");
    floor_face_ids.push_back(step.add("ADVANCED_FACE('',(" + step.ref(floor_outer_bound) + ")," +
                                      step.ref(bundles[index].floor_plane_id) + ",.T.)"));
  }

  std::vector<int> side_face_ids;
  side_face_ids.reserve(countersink_export.outer_face_indices.size());
  for (int face_index : countersink_export.outer_face_indices) {
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
    const int axis_placement = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                        step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(axis_placement) + ")");

    std::vector<int> bound_ids;
    bound_ids.reserve(face.loops.size());
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      const int edge_loop = add_line_loop(face.loops[loop_index]);
      if (loop_index == 0) {
        bound_ids.push_back(step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      } else {
        bound_ids.push_back(step.add("FACE_BOUND(''," + step.ref(edge_loop) + ",.T.)"));
      }
    }

    std::ostringstream bound_refs;
    for (std::size_t index = 0; index < bound_ids.size(); ++index) {
      if (index > 0) {
        bound_refs << ",";
      }
      bound_refs << step.ref(bound_ids[index]);
    }
    side_face_ids.push_back(
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)"));
  }

  std::ostringstream shell_faces;
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face);
  for (std::size_t index = 0; index < bundles.size(); ++index) {
    shell_faces << "," << step.ref(floor_face_ids[index]) << ","
                << step.ref(bundles[index].cone_lateral_face) << ","
                << step.ref(bundles[index].cylinder_lateral_face);
  }
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

