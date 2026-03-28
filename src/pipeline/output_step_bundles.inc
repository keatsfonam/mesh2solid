std::optional<StepSmoothBandBundle> add_step_cylindrical_band_bundle(
    StepBuilder& step,
    const GenericSmoothExtrudedBand& band) {
  if (band.top_profile_points.size() < 3 ||
      band.top_profile_points.size() != band.bottom_profile_points.size()) {
    return std::nullopt;
  }

  const std::optional<CircleFit2D> top_circle = fit_circle_2d(band.top_profile_points);
  const std::optional<CircleFit2D> bottom_circle = fit_circle_2d(band.bottom_profile_points);
  if (!top_circle.has_value() || !bottom_circle.has_value()) {
    return std::nullopt;
  }

  const double radius = 0.5 * (top_circle->radius + bottom_circle->radius);
  if (!std::isfinite(radius) || radius <= 1e-6) {
    return std::nullopt;
  }

  const double radius_tolerance =
      band.prefer_cylindrical_surface ? std::max(radius * 0.03, 0.08) : std::max(radius * 0.025, 1e-4);
  if (std::abs(top_circle->radius - bottom_circle->radius) > radius_tolerance ||
      top_circle->max_error > radius_tolerance ||
      bottom_circle->max_error > radius_tolerance ||
      !circle_fit_centers_coincident(*top_circle, *bottom_circle, radius_tolerance)) {
    return std::nullopt;
  }

  const Vec2 center {
      0.5 * (top_circle->center.x + bottom_circle->center.x),
      0.5 * (top_circle->center.y + bottom_circle->center.y),
  };

  auto project_point_to_circle = [&](const Vec2& point) {
    Vec2 radial {point.x - center.x, point.y - center.y};
    const double radial_length = std::sqrt(radial.x * radial.x + radial.y * radial.y);
    if (radial_length <= 1e-9) {
      return Vec2 {center.x + radius, center.y};
    }
    radial.x /= radial_length;
    radial.y /= radial_length;
    return Vec2 {center.x + radial.x * radius, center.y + radial.y * radius};
  };

  const Vec2 top_start_2d = project_point_to_circle(band.top_profile_points.front());
  const Vec2 top_end_2d = project_point_to_circle(band.top_profile_points.back());
  const Vec2 bottom_start_2d = project_point_to_circle(band.bottom_profile_points.front());
  const Vec2 bottom_end_2d = project_point_to_circle(band.bottom_profile_points.back());

  auto lift_radial = [&](const Vec2& point) {
    return band.basis.u * (point.x - center.x) + band.basis.v * (point.y - center.y);
  };

  Vec3 start_radial = lift_radial(top_start_2d) + lift_radial(bottom_start_2d);
  if (length_squared(start_radial) <= 1e-12) {
    start_radial = lift_radial(top_start_2d);
  }
  Vec3 end_radial = lift_radial(top_end_2d) + lift_radial(bottom_end_2d);
  if (length_squared(end_radial) <= 1e-12) {
    end_radial = lift_radial(top_end_2d);
  }
  start_radial = normalized(start_radial);
  end_radial = normalized(end_radial);
  if (length_squared(start_radial) <= 1e-12 || length_squared(end_radial) <= 1e-12) {
    return std::nullopt;
  }

  const Vec3 axis = normalized(band.axis);
  if (length_squared(axis) <= 1e-12) {
    return std::nullopt;
  }

  const double sweep_radians = std::acos(clamp(dot(start_radial, end_radial), -1.0, 1.0));
  const double sweep_degrees = sweep_radians * 180.0 / kPi;
  if (!std::isfinite(sweep_degrees) || sweep_degrees < 12.0 || sweep_degrees > 180.0) {
    return std::nullopt;
  }

  const bool same_sense = dot(cross(start_radial, end_radial), axis) >= 0.0;
  const Vec3 center_offset = band.basis.u * center.x + band.basis.v * center.y;
  const Vec3 bottom_center = center_offset + axis * band.bottom_plane;
  const Vec3 top_center = center_offset + axis * band.top_plane;
  const Vec3 bottom_start = bottom_center + start_radial * radius;
  const Vec3 bottom_end = bottom_center + end_radial * radius;
  const Vec3 top_start = top_center + start_radial * radius;
  const Vec3 top_end = top_center + end_radial * radius;

  auto add_point_vertex = [&](const Vec3& point) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," +
                                  format_double(point.z) + "))");
    return std::pair<int, int> {point_id, step.add("VERTEX_POINT(''," + step.ref(point_id) + ")")};
  };

  const auto [bottom_center_point, _bottom_center_vertex_unused] = add_point_vertex(bottom_center);
  const auto [top_center_point, _top_center_vertex_unused] = add_point_vertex(top_center);
  const auto [bottom_start_point, bottom_start_vertex] = add_point_vertex(bottom_start);
  const auto [bottom_end_point, bottom_end_vertex] = add_point_vertex(bottom_end);
  const auto [top_start_point, top_start_vertex] = add_point_vertex(top_start);
  const auto [top_end_point, top_end_vertex] = add_point_vertex(top_end);
  (void)_bottom_center_vertex_unused;
  (void)_top_center_vertex_unused;

  const int axis_direction =
      step.add("DIRECTION('',(" + format_double(axis.x) + "," +
               format_double(axis.y) + "," + format_double(axis.z) + "))");
  const int ref_direction =
      step.add("DIRECTION('',(" + format_double(start_radial.x) + "," +
               format_double(start_radial.y) + "," + format_double(start_radial.z) + "))");

  const int cylinder_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface =
      step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
               format_double(radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(bottom_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");

  const int top_circle_id = step.add("CIRCLE(''," + step.ref(top_axis) + "," + format_double(radius) + ")");
  const int bottom_circle_id =
      step.add("CIRCLE(''," + step.ref(bottom_axis) + "," + format_double(radius) + ")");
  const int top_trimmed_curve =
      step.add("TRIMMED_CURVE(''," + step.ref(top_circle_id) + ",(" + step.ref(top_start_point) + "),(" +
               step.ref(top_end_point) + ")," + std::string(same_sense ? ".T." : ".F.") + ",.CARTESIAN.)");
  const int bottom_trimmed_curve =
      step.add("TRIMMED_CURVE(''," + step.ref(bottom_circle_id) + ",(" + step.ref(bottom_start_point) + "),(" +
               step.ref(bottom_end_point) + ")," + std::string(same_sense ? ".T." : ".F.") + ",.CARTESIAN.)");

  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_start_vertex) + "," + step.ref(top_end_vertex) + "," +
               step.ref(top_trimmed_curve) + ",.T.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_start_vertex) + "," +
               step.ref(bottom_end_vertex) + "," + step.ref(bottom_trimmed_curve) + ",.T.)");

  auto add_line_edge = [&](const Vec3& start,
                           int from_point,
                           int from_vertex,
                           const Vec3& end,
                           int to_vertex) {
    const Vec3 delta = end - start;
    const double edge_length = length(delta);
    if (edge_length <= 1e-12) {
      throw std::runtime_error("Cannot export zero-length cylindrical band seam edge");
    }
    const Vec3 direction = normalized(delta);
    const int direction_id =
        step.add("DIRECTION('',(" + format_double(direction.x) + "," + format_double(direction.y) + "," +
                 format_double(direction.z) + "))");
    const int vector_id = step.add("VECTOR(''," + step.ref(direction_id) + "," +
                                   format_double(edge_length) + ")");
    const int line_id = step.add("LINE(''," + step.ref(from_point) + "," + step.ref(vector_id) + ")");
    return step.add("EDGE_CURVE(''," + step.ref(from_vertex) + "," + step.ref(to_vertex) + "," +
                    step.ref(line_id) + ",.T.)");
  };

  const int start_seam_edge_curve =
      add_line_edge(bottom_start, bottom_start_point, bottom_start_vertex, top_start, top_start_vertex);
  const int end_seam_edge_curve =
      add_line_edge(bottom_end, bottom_end_point, bottom_end_vertex, top_end, top_end_vertex);

  const int top_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.T.)");
  const int end_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(end_seam_edge_curve) + ",.F.)");
  const int bottom_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.F.)");
  const int start_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(start_seam_edge_curve) + ",.T.)");
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(top_oriented_edge) + "," +
                                    step.ref(end_seam_oriented) + "," +
                                    step.ref(bottom_oriented_edge) + "," +
                                    step.ref(start_seam_oriented) + "))");
  const int lateral_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(cylinder_surface) + ",.T.)");

  return StepSmoothBandBundle {
      top_edge_curve,
      bottom_edge_curve,
      start_seam_edge_curve,
      end_seam_edge_curve,
      lateral_face,
  };
}

StepCylinderBundle add_step_cylinder_bundle(StepBuilder& step,
                                            const StepExportContext& context_ids,
                                            const SimpleCylinderPrimitive& cylinder,
                                            const StepPlaneSurface& top_plane,
                                            const StepPlaneSurface& bottom_plane) {
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

  const int cylinder_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int cylinder_surface =
      step.add("CYLINDRICAL_SURFACE(''," + step.ref(cylinder_axis) + "," +
               format_double(cylinder.radius) + ")");
  const int top_axis = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(top_center_point) + "," +
                                step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");
  const int bottom_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(base_center_point) + "," +
               step.ref(axis_direction) + "," + step.ref(ref_direction) + ")");

  auto add_plane_circle_pcurve = [&](const StepPlaneSurface& plane,
                                     const Vec3& circle_center) -> int {
    const Vec3 delta = circle_center - plane.origin;
    const double center_u = dot(delta, plane.basis.u);
    const double center_v = dot(delta, plane.basis.v);
    const double ref_u_raw = dot(cylinder.ref_direction, plane.basis.u);
    const double ref_v_raw = dot(cylinder.ref_direction, plane.basis.v);
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
        step.add("CIRCLE(''," + step.ref(circle_axis_2d) + "," + format_double(cylinder.radius) + ")");
    const int circle_repr =
        step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(circle_2d) + ")," +
                 step.ref(context_ids.context_2d) + ")");
    return step.add("PCURVE(''," + step.ref(plane.plane_id) + "," + step.ref(circle_repr) + ")");
  };

  const int top_circle = step.add("CIRCLE(''," + step.ref(top_axis) + "," +
                                  format_double(cylinder.radius) + ")");
  const int top_cyl_pcurve_geom_point =
      step.add("CARTESIAN_POINT('',(0.0," + format_double(cylinder.height) + "))");
  const int top_cyl_pcurve_dir = step.add("DIRECTION('',(1.0,0.0))");
  const int top_cyl_pcurve_vec = step.add("VECTOR(''," + step.ref(top_cyl_pcurve_dir) + ",1.0)");
  const int top_cyl_pcurve_line =
      step.add("LINE(''," + step.ref(top_cyl_pcurve_geom_point) + "," +
               step.ref(top_cyl_pcurve_vec) + ")");
  const int top_cyl_pcurve_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(top_cyl_pcurve_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int top_cyl_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(top_cyl_pcurve_repr) + ")");
  const int top_plane_pcurve = add_plane_circle_pcurve(top_plane, top_center);
  const int top_surface_curve =
      step.add("SURFACE_CURVE(''," + step.ref(top_circle) + ",(" + step.ref(top_cyl_pcurve) + "," +
               step.ref(top_plane_pcurve) + "),.PCURVE_S1.)");
  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_seam_vertex) + "," + step.ref(top_seam_vertex) + "," +
               step.ref(top_surface_curve) + ",.T.)");

  const int seam_line_dir = step.add("DIRECTION('',(" + format_double(cylinder.axis.x) + "," +
                                     format_double(cylinder.axis.y) + "," +
                                     format_double(cylinder.axis.z) + "))");
  const int seam_line_vec = step.add("VECTOR(''," + step.ref(seam_line_dir) + ",1.0)");
  const int seam_line =
      step.add("LINE(''," + step.ref(bottom_seam_point) + "," + step.ref(seam_line_vec) + ")");
  const int seam_u0_point = step.add("CARTESIAN_POINT('',(0.0,0.0))");
  const int seam_v_dir = step.add("DIRECTION('',(0.0,1.0))");
  const int seam_v_vec = step.add("VECTOR(''," + step.ref(seam_v_dir) + ",1.0)");
  const int seam_u0_line =
      step.add("LINE(''," + step.ref(seam_u0_point) + "," + step.ref(seam_v_vec) + ")");
  const int seam_u0_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u0_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u0_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u0_repr) + ")");
  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line =
      step.add("LINE(''," + step.ref(seam_u1_point) + "," + step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_ids.context_2d) + ")");
  const int seam_u1_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," + step.ref(seam_u1_repr) + ")");
  const int seam_curve =
      step.add("SEAM_CURVE(''," + step.ref(seam_line) + ",(" + step.ref(seam_u1_pcurve) + "," +
               step.ref(seam_u0_pcurve) + "),.PCURVE_S1.)");
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
  const int bottom_cyl_pcurve =
      step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
               step.ref(bottom_cyl_pcurve_repr) + ")");
  const int bottom_plane_pcurve = add_plane_circle_pcurve(bottom_plane, cylinder.base_center);
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
  const int lateral_loop =
      step.add("EDGE_LOOP('',(" + step.ref(lateral_top_oriented) + "," +
               step.ref(lateral_seam_forward) + "," + step.ref(lateral_bottom_oriented) + "," +
               step.ref(lateral_seam_backward) + "))");
  const int lateral_bound = step.add("FACE_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face =
      step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
               step.ref(cylinder_surface) + ",.T.)");

  return StepCylinderBundle {top_edge_curve, bottom_edge_curve, seam_edge_curve, lateral_face};
}

StepSmoothBandBundle add_step_smooth_band_bundle(StepBuilder& step,
                                                 const GenericSmoothExtrudedBand& band) {
  if (band.top_profile_points.size() < 2 || band.top_profile_points.size() != band.bottom_profile_points.size()) {
    throw std::runtime_error("Invalid smooth band profile");
  }

  if (const std::optional<StepSmoothBandBundle> cylindrical_bundle =
          add_step_cylindrical_band_bundle(step, band);
      cylindrical_bundle.has_value()) {
    return *cylindrical_bundle;
  }

  auto lift_profile = [&](const std::vector<Vec2>& profile, double plane) {
    std::vector<Vec3> points;
    points.reserve(profile.size());
    for (const Vec2& point : profile) {
      points.push_back(band.basis.u * point.x + band.basis.v * point.y + band.axis * plane);
    }
    return points;
  };

  const std::vector<Vec3> top_points = lift_profile(band.top_profile_points, band.top_plane);
  const std::vector<Vec3> bottom_points = lift_profile(band.bottom_profile_points, band.bottom_plane);

  const int top_curve = add_step_b_spline_curve(step, top_points);
  const int bottom_curve = add_step_b_spline_curve(step, bottom_points);
  const int surface = add_step_b_spline_surface(step, bottom_points, top_points);

  auto add_vertex_point = [&](const Vec3& point) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    return std::pair<int, int> {point_id, step.add("VERTEX_POINT(''," + step.ref(point_id) + ")")};
  };

  const auto [top_start_point, top_start_vertex] = add_vertex_point(top_points.front());
  const auto [top_end_point, top_end_vertex] = add_vertex_point(top_points.back());
  const auto [bottom_start_point, bottom_start_vertex] = add_vertex_point(bottom_points.front());
  const auto [bottom_end_point, bottom_end_vertex] = add_vertex_point(bottom_points.back());

  const int top_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(top_start_vertex) + "," + step.ref(top_end_vertex) + "," +
               step.ref(top_curve) + ",.T.)");
  const int bottom_edge_curve =
      step.add("EDGE_CURVE(''," + step.ref(bottom_start_vertex) + "," +
               step.ref(bottom_end_vertex) + "," + step.ref(bottom_curve) + ",.T.)");

  auto add_line_edge = [&](const Vec3& start,
                           int from_point,
                           int from_vertex,
                           const Vec3& end,
                           int /*to_point*/,
                           int to_vertex) {
    const Vec3 delta = end - start;
    const double edge_length = length(delta);
    if (edge_length <= 1e-12) {
      throw std::runtime_error("Cannot export zero-length smooth band seam edge");
    }
    const Vec3 direction = normalized(delta);
    const int direction_id =
        step.add("DIRECTION('',(" + format_double(direction.x) + "," + format_double(direction.y) +
                 "," + format_double(direction.z) + "))");
    const int vector_id = step.add("VECTOR(''," + step.ref(direction_id) + "," +
                                   format_double(edge_length) + ")");
    const int line_id = step.add("LINE(''," + step.ref(from_point) + "," + step.ref(vector_id) + ")");
    return step.add("EDGE_CURVE(''," + step.ref(from_vertex) + "," + step.ref(to_vertex) + "," +
                    step.ref(line_id) + ",.T.)");
  };

  const int start_seam_edge_curve = add_line_edge(bottom_points.front(),
                                                  bottom_start_point,
                                                  bottom_start_vertex,
                                                  top_points.front(),
                                                  top_start_point,
                                                  top_start_vertex);
  const int end_seam_edge_curve = add_line_edge(bottom_points.back(),
                                                bottom_end_point,
                                                bottom_end_vertex,
                                                top_points.back(),
                                                top_end_point,
                                                top_end_vertex);

  const int top_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(top_edge_curve) + ",.T.)");
  const int end_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(end_seam_edge_curve) + ",.F.)");
  const int bottom_oriented_edge =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(bottom_edge_curve) + ",.F.)");
  const int start_seam_oriented =
      step.add("ORIENTED_EDGE('',*,*," + step.ref(start_seam_edge_curve) + ",.T.)");
  const int lateral_loop = step.add("EDGE_LOOP('',(" + step.ref(top_oriented_edge) + "," +
                                    step.ref(end_seam_oriented) + "," +
                                    step.ref(bottom_oriented_edge) + "," +
                                    step.ref(start_seam_oriented) + "))");
  const int lateral_bound = step.add("FACE_OUTER_BOUND(''," + step.ref(lateral_loop) + ",.T.)");
  const int lateral_face = step.add("ADVANCED_FACE('',(" + step.ref(lateral_bound) + ")," +
                                    step.ref(surface) + ",.T.)");

  return StepSmoothBandBundle {
      top_edge_curve,
      bottom_edge_curve,
      start_seam_edge_curve,
      end_seam_edge_curve,
      lateral_face,
  };
}
