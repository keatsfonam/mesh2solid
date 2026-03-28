bool write_step_file_prismatic_countersink(const std::filesystem::path& path,
                                           const ReconstructionResult& reconstruction,
                                           const SimplePrismaticCountersinkExport& countersink_export) {
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
  const PlaneBasis axis_basis = basis_from_normal(countersink_export.countersink.axis);
  if (projected_loop_area(top_outer_loop, reconstruction.vertices, axis_basis) < 0.0) {
    std::reverse(top_outer_loop.begin(), top_outer_loop.end());
  }
  if (projected_loop_area(bottom_outer_loop, reconstruction.vertices, axis_basis) > 0.0) {
    std::reverse(bottom_outer_loop.begin(), bottom_outer_loop.end());
  }

  const SimpleConicalFrustumPrimitive& countersink = countersink_export.countersink;
  const SimpleCylinderPrimitive& bore = countersink_export.bore;
  const Vec3 top_center = countersink.base_center + countersink.axis * countersink.height;
  const Vec3 step_center = countersink.base_center;
  const Vec3 bottom_center = bore.base_center;
  const Vec3 axis = countersink.axis;
  const Vec3 ref_direction = countersink.ref_direction;
  const double cone_height = countersink.height;
  const double radius_delta = countersink.top_radius - countersink.base_radius;
  const double cone_angle = std::atan2(radius_delta, cone_height);
  const double slant_height = std::sqrt(cone_height * cone_height + radius_delta * radius_delta);
  if (slant_height <= 1e-12) {
    return false;
  }

  const StepPlaneSurface top_plane = add_step_plane_surface(step, top_center, axis, ref_direction);
  const StepPlaneSurface bottom_plane = add_step_plane_surface(step, bottom_center, axis, ref_direction);

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

  const Vec3 top_seam = top_center + ref_direction * countersink.top_radius;
  const Vec3 step_seam = step_center + ref_direction * countersink.base_radius;
  const Vec3 bottom_seam = bottom_center + ref_direction * bore.radius;

  const int top_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(top_seam.x) + "," +
                                     format_double(top_seam.y) + "," +
                                     format_double(top_seam.z) + "))");
  const int step_seam_point = step.add("CARTESIAN_POINT('',(" + format_double(step_seam.x) + "," +
                                      format_double(step_seam.y) + "," +
                                      format_double(step_seam.z) + "))");
  const int bottom_seam_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_seam.x) + "," +
               format_double(bottom_seam.y) + "," + format_double(bottom_seam.z) + "))");
  const int top_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(top_seam_point) + ")");
  const int step_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(step_seam_point) + ")");
  const int bottom_seam_vertex = step.add("VERTEX_POINT(''," + step.ref(bottom_seam_point) + ")");

  const int axis_direction_id =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," +
               format_double(axis.y) + "," + format_double(axis.z) + "))");
  const int ref_direction_id =
      step.add("DIRECTION('',(" + format_double(ref_direction.x) + "," +
               format_double(ref_direction.y) + "," + format_double(ref_direction.z) + "))");
  const int top_center_point = step.add("CARTESIAN_POINT('',(" + format_double(top_center.x) + "," +
                                     format_double(top_center.y) + "," +
                                     format_double(top_center.z) + "))");
  const int step_center_point = step.add("CARTESIAN_POINT('',(" + format_double(step_center.x) + "," +
                                      format_double(step_center.y) + "," +
                                      format_double(step_center.z) + "))");
  const int bottom_center_point =
      step.add("CARTESIAN_POINT('',(" + format_double(bottom_center.x) + "," +
               format_double(bottom_center.y) + "," + format_double(bottom_center.z) + "))");

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
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction_id) + "," + step.ref(ref_direction_id) + ")");
  const int cylinder_surface =
      step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," + format_double(bore.radius) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
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
      add_plane_circle_pcurve(top_plane, top_center, countersink.top_radius, ref_direction);
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

  const int bottom_circle =
      step.add("CIRCLE(''," + step.ref(bottom_axis) + "," + format_double(bore.radius) + ")");
  const int bottom_cyl_pcurve_geom_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int bottom_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(bottom_cyl_pcurve_geom_point) + "," + step.ref(u_vec) + ")");
  const int bottom_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(bottom_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int bottom_cyl_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_pcurve =
      add_plane_circle_pcurve(bottom_plane, bottom_center, bore.radius, ref_direction);
  const int bottom_surface_curve =
      step.add("SURFACE_CURVE(''," + step.ref(bottom_circle) + ",(" +
               step.ref(bottom_cyl_pcurve) + "," + step.ref(bottom_plane_pcurve) +
               "),.PCURVE_S1.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
               step.ref(bottom_seam_vertex) + "," + step.ref(bottom_surface_curve) + ",.T.)");

  const int cone_seam_line_dir = step.add("DIRECTION('',(" + format_double(axis.x) + "," +
                                          format_double(axis.y) + "," + format_double(axis.z) + "))");
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
      step.add("LINE(''," + step.ref(bottom_seam_point) + "," + step.ref(cone_seam_line_vec) + ")");
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
      step.add("EDGE_CURVE(''," + step.ref(bottom_seam_vertex) + "," +
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
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.T.)");
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

  const int top_outer_loop_id = add_line_loop(top_outer_loop);
  const int top_outer_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(top_outer_loop_id) + ",.T.)");
  const bool top_circle_forward = countersink_export.top_inner_area > 0.0;
  const int top_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + "," +
               std::string(top_circle_forward ? ".T." : ".F.") + ")");
  const int top_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(top_inner_oriented) + "))");
  const int top_inner_bound = step.add("FACE_BOUND(''," + step.ref(top_inner_loop) + ",.T.)");
  const int top_face =
      step.add("ADVANCED_FACE('',(" + step.ref(top_outer_bound) + "," + step.ref(top_inner_bound) + ")," +
               step.ref(top_plane.plane_id) + ",.T.)");

  const int bottom_outer_loop_id = add_line_loop(bottom_outer_loop);
  const int bottom_outer_bound =
      step.add("FACE_OUTER_BOUND(''," + step.ref(bottom_outer_loop_id) + ",.T.)");
  const bool bottom_circle_forward = countersink_export.bottom_inner_area > 0.0;
  const int bottom_inner_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + "," +
               std::string(bottom_circle_forward ? ".T." : ".F.") + ")");
  const int bottom_inner_loop = step.add("EDGE_LOOP('',(" + step.ref(bottom_inner_oriented) + "))");
  const int bottom_inner_bound =
      step.add("FACE_BOUND(''," + step.ref(bottom_inner_loop) + ",.T.)");
  const int bottom_face =
      step.add("ADVANCED_FACE('',(" + step.ref(bottom_outer_bound) + "," + step.ref(bottom_inner_bound) + ")," +
               step.ref(bottom_plane.plane_id) + ",.F.)");

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
    const int face_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(plane_origin) + "," +
                                   step.ref(face_normal) + "," + step.ref(face_ref) + ")");
    const int plane = step.add("PLANE(''," + step.ref(face_axis) + ")");

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
  shell_faces << step.ref(top_face) << "," << step.ref(bottom_face) << ","
              << step.ref(cone_lateral_face) << "," << step.ref(cylinder_lateral_face);
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

