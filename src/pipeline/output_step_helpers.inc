std::pair<std::vector<int>, std::vector<double>> open_uniform_knot_data(int control_point_count,
                                                                        int degree) {
  if (control_point_count < 2) {
    return {{}, {}};
  }
  const int clamped_degree = std::max(1, std::min(degree, control_point_count - 1));
  const int distinct_knot_count = control_point_count - clamped_degree + 1;
  std::vector<int> multiplicities(distinct_knot_count, 1);
  multiplicities.front() = clamped_degree + 1;
  multiplicities.back() = clamped_degree + 1;
  std::vector<double> knots;
  knots.reserve(distinct_knot_count);
  for (int index = 0; index < distinct_knot_count; ++index) {
    knots.push_back(static_cast<double>(index));
  }
  return {multiplicities, knots};
}

int add_step_b_spline_curve(StepBuilder& step,
                            const std::vector<Vec3>& points,
                            int degree = 3) {
  if (points.size() < 2) {
    throw std::runtime_error("Cannot export B-spline curve with fewer than 2 points");
  }

  const int clamped_degree = std::max(1, std::min(degree, static_cast<int>(points.size()) - 1));
  std::ostringstream point_refs;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const Vec3& point = points[index];
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    if (index > 0) {
      point_refs << ",";
    }
    point_refs << step.ref(point_id);
  }

  const auto [multiplicities, knots] = open_uniform_knot_data(static_cast<int>(points.size()), clamped_degree);
  if (multiplicities.empty() || knots.empty()) {
    throw std::runtime_error("Unable to compute B-spline knot data");
  }

  std::ostringstream multiplicity_values;
  for (std::size_t index = 0; index < multiplicities.size(); ++index) {
    if (index > 0) {
      multiplicity_values << ",";
    }
    multiplicity_values << multiplicities[index];
  }
  std::ostringstream knot_values;
  for (std::size_t index = 0; index < knots.size(); ++index) {
    if (index > 0) {
      knot_values << ",";
    }
    knot_values << format_double(knots[index]);
  }

  return step.add("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(clamped_degree) + ",(" +
                  point_refs.str() + "),.UNSPECIFIED.,.F.,.F.,(" + multiplicity_values.str() +
                  "),(" + knot_values.str() + "),.UNSPECIFIED.)");
}

int add_step_b_spline_surface(StepBuilder& step,
                              const std::vector<Vec3>& bottom_points,
                              const std::vector<Vec3>& top_points,
                              int degree = 3) {
  if (bottom_points.size() < 2 || bottom_points.size() != top_points.size()) {
    throw std::runtime_error("Cannot export B-spline surface without matching profile rows");
  }

  const int u_degree = std::max(1, std::min(degree, static_cast<int>(bottom_points.size()) - 1));
  const int v_degree = 1;
  std::ostringstream control_rows;
  for (std::size_t index = 0; index < bottom_points.size(); ++index) {
    const int bottom_point =
        step.add("CARTESIAN_POINT('',(" + format_double(bottom_points[index].x) + "," +
                 format_double(bottom_points[index].y) + "," +
                 format_double(bottom_points[index].z) + "))");
    const int top_point = step.add("CARTESIAN_POINT('',(" + format_double(top_points[index].x) + "," +
                                      format_double(top_points[index].y) + "," +
                                      format_double(top_points[index].z) + "))");
    if (index > 0) {
      control_rows << ",";
    }
    control_rows << "(" << step.ref(bottom_point) << "," << step.ref(top_point) << ")";
  }

  const auto [u_multiplicities, u_knots] =
      open_uniform_knot_data(static_cast<int>(bottom_points.size()), u_degree);
  const auto [v_multiplicities, v_knots] = open_uniform_knot_data(2, v_degree);
  if (u_multiplicities.empty() || u_knots.empty() ||
      v_multiplicities.empty() || v_knots.empty()) {
    throw std::runtime_error("Unable to compute B-spline surface knot data");
  }

  auto emit_int_list = [](const std::vector<int>& values) {
    std::ostringstream output;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output << ",";
      }
      output << values[index];
    }
    return output.str();
  };
  auto emit_double_list = [](const std::vector<double>& values) {
    std::ostringstream output;
    for (std::size_t index = 0; index < values.size(); ++index) {
      if (index > 0) {
        output << ",";
      }
      output << format_double(values[index]);
    }
    return output.str();
  };

  return step.add("B_SPLINE_SURFACE_WITH_KNOTS(''," + std::to_string(u_degree) + "," +
                  std::to_string(v_degree) + ",(" + control_rows.str() + "),.UNSPECIFIED.,.F.,.F.,.F.,(" +
                  emit_int_list(u_multiplicities) + "),(" + emit_int_list(v_multiplicities) +
                  "),(" + emit_double_list(u_knots) + "),(" + emit_double_list(v_knots) +
                  "),.UNSPECIFIED.)");
}

StepExportContext build_step_export_context(StepBuilder& step) {
  const int application_context = step.add(
      "APPLICATION_CONTEXT('configuration controlled 3d designs of mechanical parts and assemblies')");
  const int application_protocol =
      step.add("APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994," +
               step.ref(application_context) + ")");
  (void)application_protocol;
  const int design_context =
      step.add("DESIGN_CONTEXT(''," + step.ref(application_context) + ",'design')");
  const int mechanical_context =
      step.add("MECHANICAL_CONTEXT(''," + step.ref(application_context) + ",'mechanical')");
  const int product = step.add("PRODUCT('mesh2solid','mesh2solid','',(" + step.ref(mechanical_context) + "))");
  const int product_formation = step.add(
      "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('',''," + step.ref(product) + ",.NOT_KNOWN.)");
  const int product_definition =
      step.add("PRODUCT_DEFINITION('',''," + step.ref(product_formation) + "," + step.ref(design_context) +
               ")");
  const int product_shape =
      step.add("PRODUCT_DEFINITION_SHAPE('',''," + step.ref(product_definition) + ")");
  const int length_unit = step.add("(LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.))");
  const int angle_unit = step.add("(NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.))");
  const int solid_angle_unit = step.add("(NAMED_UNIT(*) SOLID_ANGLE_UNIT() SI_UNIT($,.STERADIAN.))");
  const int uncertainty =
      step.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06)," + step.ref(length_unit) +
               ",'distance_accuracy_value','confusion accuracy')");
  const int context = step.add("(GEOMETRIC_REPRESENTATION_CONTEXT(3) "
                               "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((" +
                               step.ref(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((" +
                               step.ref(length_unit) + "," + step.ref(angle_unit) + "," +
                               step.ref(solid_angle_unit) + ")) REPRESENTATION_CONTEXT('',''))");
  const int context_2d = step.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) "
                                  "PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )");

  const int origin = step.add("CARTESIAN_POINT('',(0.0,0.0,0.0))");
  const int z_direction = step.add("DIRECTION('',(0.0,0.0,1.0))");
  const int x_direction = step.add("DIRECTION('',(1.0,0.0,0.0))");
  const int global_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin) + "," + step.ref(z_direction) + "," +
               step.ref(x_direction) + ")");

  return StepExportContext {product_shape, context, context_2d, global_axis};
}

StepPlaneSurface add_step_plane_surface(StepBuilder& step,
                                        const Vec3& origin,
                                        const Vec3& normal,
                                        const Vec3& ref_direction) {
  const Vec3 local_normal = normalized(normal);
  Vec3 local_ref = ref_direction - local_normal * dot(ref_direction, local_normal);
  if (length_squared(local_ref) <= 1e-12) {
    local_ref = basis_from_normal(local_normal).u;
  } else {
    local_ref = normalized(local_ref);
  }
  const Vec3 local_v = normalized(cross(local_normal, local_ref));
  const int origin_id = step.add("CARTESIAN_POINT('',(" + format_double(origin.x) + "," +
                               format_double(origin.y) + "," + format_double(origin.z) + "))");
  const int normal_id = step.add("DIRECTION('',(" + format_double(local_normal.x) + "," +
                               format_double(local_normal.y) + "," +
                               format_double(local_normal.z) + "))");
  const int ref_id = step.add("DIRECTION('',(" + format_double(local_ref.x) + "," +
                            format_double(local_ref.y) + "," + format_double(local_ref.z) + "))");
  const int axis_id = step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin_id) + "," +
                               step.ref(normal_id) + "," + step.ref(ref_id) + ")");
  return StepPlaneSurface {
      step.add("PLANE(''," + step.ref(axis_id) + ")"),
      origin,
      local_normal,
      PlaneBasis {local_ref, local_v},
  };
}
