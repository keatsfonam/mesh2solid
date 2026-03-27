int add_step_cylinder_solid(StepBuilder& step,
                            const StepExportContext& context_ids,
                            const SimpleCylinderPrimitive& cylinder) {
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
  const int seam_line_vec =
      step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line = step.add("LINE(''," + step.ref(bottom_seam_point) + "," +
                                 step.ref(seam_line_vec) + ")");

  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec =
      step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
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

  const int top_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.T.)");
  const int top_loop = step.add("EDGE_LOOP('',(" + step.ref(top_oriented_edge) + "))");
  const int top_bound = step.add("FACE_BOUND(''," + step.ref(top_loop) + ",.T.)");
  const int top_face = step.add("ADVANCED_FACE('',(" + step.ref(top_bound) + ")," +
                                step.ref(top_plane) + ",.T.)");

  const int bottom_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.F.)");
  const int bottom_loop = step.add("EDGE_LOOP('',(" + step.ref(bottom_oriented_edge) + "))");
  const int bottom_bound = step.add("FACE_BOUND(''," + step.ref(bottom_loop) + ",.T.)");
  const int bottom_face = step.add("ADVANCED_FACE('',(" + step.ref(bottom_bound) + ")," +
                                   step.ref(bottom_plane) + ",.F.)");

  const int closed_shell = step.add("CLOSED_SHELL('',(" + step.ref(lateral_face) + "," +
                                   step.ref(top_face) + "," + step.ref(bottom_face) + "))");
  return step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
}

bool write_step_file_simple_cylinders(const std::filesystem::path& path,
                                      const std::vector<SimpleCylinderPrimitive>& cylinders) {
  if (cylinders.empty()) {
    return false;
  }

  StepBuilder step;
  const StepExportContext context_ids = build_step_export_context(step);
  std::vector<int> solid_ids;
  solid_ids.reserve(cylinders.size());
  for (const SimpleCylinderPrimitive& cylinder : cylinders) {
    solid_ids.push_back(add_step_cylinder_solid(step, context_ids, cylinder));
  }

  std::ostringstream items;
  for (std::size_t index = 0; index < solid_ids.size(); ++index) {
    if (index != 0) {
      items << ",";
    }
    items << step.ref(solid_ids[index]);
  }
  items << "," << step.ref(context_ids.global_axis);

  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + items.str() + ")," +
               step.ref(context_ids.context) + ")");
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
