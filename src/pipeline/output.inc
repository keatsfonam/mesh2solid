std::string step_header() {
  std::ostringstream output;
  output << "ISO-10303-21;\n";
  output << "HEADER;\n";
  output << "FILE_DESCRIPTION(('mesh2solid advanced brep'),'2;1');\n";
  output << "FILE_NAME('reconstruction.step','2026-03-25T00:00:00',('Codex'),('OpenAI'),"
         << "'mesh2solid','mesh2solid','');\n";
  output << "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\n";
  output << "ENDSEC;\n";
  output << "DATA;\n";
  return output.str();
}

struct CircleFit2DExport {
  Vec2 center {};
  double radius {};
  double rms_error {};
  double max_error {};
};

struct SimpleCylinderExport {
  Vec3 base_center {};
  Vec3 axis {};
  Vec3 ref_direction {};
  double radius {};
  double height {};
};

bool solve_linear_3x3_export(double matrix[3][4], std::array<double, 3>& solution) {
  for (int column = 0; column < 3; ++column) {
    int pivot = column;
    double pivot_value = std::abs(matrix[pivot][column]);
    for (int row = column + 1; row < 3; ++row) {
      const double candidate = std::abs(matrix[row][column]);
      if (candidate > pivot_value) {
        pivot = row;
        pivot_value = candidate;
      }
    }
    if (pivot_value <= 1e-12) {
      return false;
    }
    if (pivot != column) {
      for (int entry = column; entry < 4; ++entry) {
        std::swap(matrix[column][entry], matrix[pivot][entry]);
      }
    }

    const double divisor = matrix[column][column];
    for (int entry = column; entry < 4; ++entry) {
      matrix[column][entry] /= divisor;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      if (std::abs(factor) <= 1e-12) {
        continue;
      }
      for (int entry = column; entry < 4; ++entry) {
        matrix[row][entry] -= factor * matrix[column][entry];
      }
    }
  }

  solution = {matrix[0][3], matrix[1][3], matrix[2][3]};
  return true;
}

std::optional<CircleFit2DExport> fit_circle_2d_export(const std::vector<Vec2>& points) {
  if (points.size() < 3) {
    return std::nullopt;
  }

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_xx = 0.0;
  double sum_yy = 0.0;
  double sum_xy = 0.0;
  double sum_xxx = 0.0;
  double sum_yyy = 0.0;
  double sum_xxy = 0.0;
  double sum_xyy = 0.0;

  for (const Vec2& point : points) {
    const double x = point.x;
    const double y = point.y;
    const double xx = x * x;
    const double yy = y * y;
    sum_x += x;
    sum_y += y;
    sum_xx += xx;
    sum_yy += yy;
    sum_xy += x * y;
    sum_xxx += xx * x;
    sum_yyy += yy * y;
    sum_xxy += xx * y;
    sum_xyy += x * yy;
  }

  double system[3][4] = {
      {sum_xx, sum_xy, sum_x, -(sum_xxx + sum_xyy)},
      {sum_xy, sum_yy, sum_y, -(sum_xxy + sum_yyy)},
      {sum_x, sum_y, static_cast<double>(points.size()), -(sum_xx + sum_yy)},
  };
  std::array<double, 3> solution {};
  if (!solve_linear_3x3_export(system, solution)) {
    return std::nullopt;
  }

  const Vec2 center {-0.5 * solution[0], -0.5 * solution[1]};
  const double radius_squared = center.x * center.x + center.y * center.y - solution[2];
  if (radius_squared <= 1e-12) {
    return std::nullopt;
  }

  CircleFit2DExport fit;
  fit.center = center;
  fit.radius = std::sqrt(radius_squared);
  double error_sum = 0.0;
  for (const Vec2& point : points) {
    const double dx = point.x - fit.center.x;
    const double dy = point.y - fit.center.y;
    const double error = std::abs(std::sqrt(dx * dx + dy * dy) - fit.radius);
    error_sum += error * error;
    fit.max_error = std::max(fit.max_error, error);
  }
  fit.rms_error = std::sqrt(error_sum / static_cast<double>(points.size()));
  return fit;
}

std::optional<SimpleCylinderExport> detect_simple_cylinder_export(
    const ReconstructionResult& reconstruction) {
  if (reconstruction.faces.size() < 10 || reconstruction.vertices.size() < 12) {
    return std::nullopt;
  }

  std::vector<int> candidate_caps;
  for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
    const ReconstructedFace& face = reconstruction.faces[face_index];
    if (face.loops.size() == 1 && face.loops.front().size() >= 8) {
      candidate_caps.push_back(static_cast<int>(face_index));
    }
  }
  if (candidate_caps.size() < 2) {
    return std::nullopt;
  }

  std::sort(candidate_caps.begin(), candidate_caps.end(), [&reconstruction](int lhs, int rhs) {
    return reconstruction.faces[lhs].area > reconstruction.faces[rhs].area;
  });
  if (candidate_caps.size() > 6) {
    candidate_caps.resize(6);
  }

  for (std::size_t i = 0; i < candidate_caps.size(); ++i) {
    for (std::size_t j = i + 1; j < candidate_caps.size(); ++j) {
      const ReconstructedFace& face_a = reconstruction.faces[candidate_caps[i]];
      const ReconstructedFace& face_b = reconstruction.faces[candidate_caps[j]];
      const double angle = angle_degrees_unsigned(face_a.fit.normal, face_b.fit.normal);
      if (angle > 8.0) {
        continue;
      }

      Vec3 axis = normalized(face_a.fit.normal);
      if (dot(axis, face_a.fit.centroid - face_b.fit.centroid) < 0.0) {
        axis = axis * -1.0;
      }
      const PlaneBasis basis = basis_from_normal(axis);

      const auto project_loop = [&](const std::vector<int>& loop) {
        std::vector<Vec2> projected;
        projected.reserve(loop.size());
        for (int vertex_index : loop) {
          const Vec3& vertex = reconstruction.vertices[vertex_index];
          projected.push_back({dot(vertex, basis.u), dot(vertex, basis.v)});
        }
        return projected;
      };

      const std::optional<CircleFit2DExport> circle_a = fit_circle_2d_export(project_loop(face_a.loops.front()));
      const std::optional<CircleFit2DExport> circle_b = fit_circle_2d_export(project_loop(face_b.loops.front()));
      if (!circle_a.has_value() || !circle_b.has_value()) {
        continue;
      }

      const double radius = (circle_a->radius + circle_b->radius) * 0.5;
      if (radius <= 1e-6) {
        continue;
      }

      const double radius_tolerance = std::max(radius * 0.03, 1e-4);
      if (std::abs(circle_a->radius - circle_b->radius) > radius_tolerance ||
          circle_a->max_error > radius_tolerance || circle_b->max_error > radius_tolerance) {
        continue;
      }

      const Vec2 center_delta {circle_a->center.x - circle_b->center.x,
                               circle_a->center.y - circle_b->center.y};
      if (std::sqrt(center_delta.x * center_delta.x + center_delta.y * center_delta.y) >
          radius_tolerance) {
        continue;
      }

      const double plane_a = dot(face_a.fit.centroid, axis);
      const double plane_b = dot(face_b.fit.centroid, axis);
      const double top_plane = std::max(plane_a, plane_b);
      const double bottom_plane = std::min(plane_a, plane_b);
      const double height = top_plane - bottom_plane;
      if (height <= radius_tolerance) {
        continue;
      }

      bool all_side_faces_match = true;
      for (std::size_t face_index = 0; face_index < reconstruction.faces.size(); ++face_index) {
        if (static_cast<int>(face_index) == candidate_caps[i] ||
            static_cast<int>(face_index) == candidate_caps[j]) {
          continue;
        }
        const ReconstructedFace& side_face = reconstruction.faces[face_index];
        if (side_face.loops.size() != 1 || side_face.loops.front().size() != 4) {
          all_side_faces_match = false;
          break;
        }

        int top_vertices = 0;
        int bottom_vertices = 0;
        for (int vertex_index : side_face.loops.front()) {
          const double coordinate = dot(reconstruction.vertices[vertex_index], axis);
          if (std::abs(coordinate - top_plane) <= radius_tolerance) {
            ++top_vertices;
          } else if (std::abs(coordinate - bottom_plane) <= radius_tolerance) {
            ++bottom_vertices;
          }
        }
        if (top_vertices != 2 || bottom_vertices != 2) {
          all_side_faces_match = false;
          break;
        }
      }
      if (!all_side_faces_match) {
        continue;
      }

      const Vec2 center {
          (circle_a->center.x + circle_b->center.x) * 0.5,
          (circle_a->center.y + circle_b->center.y) * 0.5,
      };
      const Vec3 axis_origin = basis.u * center.x + basis.v * center.y;
      return SimpleCylinderExport {
          axis_origin + axis * bottom_plane,
          axis,
          basis.u,
          radius,
          height,
      };
    }
  }

  return std::nullopt;
}

class StepBuilder {
 public:
  int add(const std::string& body) {
    const int id = next_id_++;
    entities_.push_back("#" + std::to_string(id) + "=" + body + ";");
    return id;
  }

  std::string ref(int id) const { return "#" + std::to_string(id); }

  std::string emit() const {
    std::ostringstream output;
    for (const std::string& entity : entities_) {
      output << entity << "\n";
    }
    return output.str();
  }

 private:
  int next_id_ {1};
  std::vector<std::string> entities_;
};

bool write_step_file_simple_cylinder(const std::filesystem::path& path,
                                     const SimpleCylinderExport& cylinder) {
  StepBuilder step;
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
               step.ref(context_2d) + ")");
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
               step.ref(context_2d) + ")");
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
               step.ref(context_2d) + ")");
  const int seam_u0_pcurve = step.add("PCURVE(''," + step.ref(cylinder_surface) + "," +
                                      step.ref(seam_u0_repr) + ")");

  const int seam_u1_point =
      step.add("CARTESIAN_POINT('',(" + format_double(circumference) + ",0.0))");
  const int seam_u1_line = step.add("LINE(''," + step.ref(seam_u1_point) + "," +
                                    step.ref(seam_v_vec) + ")");
  const int seam_u1_repr =
      step.add("DEFINITIONAL_REPRESENTATION('',(" + step.ref(seam_u1_line) + ")," +
               step.ref(context_2d) + ")");
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
               step.ref(context_2d) + ")");
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
               step.ref(context_2d) + ")");
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
  const int manifold_solid_brep =
      step.add("MANIFOLD_SOLID_BREP('mesh2solid'," + step.ref(closed_shell) + ")");
  const int shape_representation =
      step.add("ADVANCED_BREP_SHAPE_REPRESENTATION('',(" + step.ref(manifold_solid_brep) + "," +
               step.ref(global_axis) + ")," + step.ref(context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(product_shape) + "," +
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

bool write_step_file(const std::filesystem::path& path, const ReconstructionResult& reconstruction) {
  if (reconstruction.outcome != ReconstructionOutcome::SolidCreated) {
    return false;
  }

  if (const std::optional<SimpleCylinderExport> cylinder =
          detect_simple_cylinder_export(reconstruction);
      cylinder.has_value()) {
    return write_step_file_simple_cylinder(path, *cylinder);
  }

  StepBuilder step;
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

  const int origin = step.add("CARTESIAN_POINT('',(0.0,0.0,0.0))");
  const int z_direction = step.add("DIRECTION('',(0.0,0.0,1.0))");
  const int x_direction = step.add("DIRECTION('',(1.0,0.0,0.0))");
  const int global_axis =
      step.add("AXIS2_PLACEMENT_3D(''," + step.ref(origin) + "," + step.ref(z_direction) + "," +
               step.ref(x_direction) + ")");

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
  auto get_edge_curve_id = [&](int from, int to) -> int {
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
    const int line_id = step.add("LINE(''," + step.ref(point_ids[key.a]) + "," + step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertex_point_ids[key.a]) + "," +
                 step.ref(vertex_point_ids[key.b]) + "," + step.ref(line_id) + ",.T.)");
    edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  };

  std::vector<int> face_ids;
  face_ids.reserve(reconstruction.faces.size());
  for (const ReconstructedFace& face : reconstruction.faces) {
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
      const auto& loop = face.loops[loop_index];
      std::ostringstream edge_refs;
      for (std::size_t index = 0; index < loop.size(); ++index) {
        const int from = loop[index];
        const int to = loop[(index + 1) % loop.size()];
        const int edge_curve_id = get_edge_curve_id(from, to);
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
      const int edge_loop = step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
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
    const int advanced_face =
        step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane) + ",.T.)");
    face_ids.push_back(advanced_face);
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
               step.ref(global_axis) + ")," + step.ref(context) + ")");
  step.add("SHAPE_DEFINITION_REPRESENTATION(" + step.ref(product_shape) + "," +
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

void write_ascii_stl(const std::filesystem::path& path, const MeshModel& mesh) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write STL file: " + path.string());
  }

  output << "solid cleaned_mesh\n";
  for (const Triangle& triangle : mesh.triangles) {
    output << "  facet normal " << format_double(triangle.normal.x) << " "
           << format_double(triangle.normal.y) << " " << format_double(triangle.normal.z) << "\n";
    output << "    outer loop\n";
    for (int vertex_index : triangle.vertices) {
      const Vec3& vertex = mesh.vertices[vertex_index];
      output << "      vertex " << format_double(vertex.x) << " " << format_double(vertex.y) << " "
             << format_double(vertex.z) << "\n";
    }
    output << "    endloop\n";
    output << "  endfacet\n";
  }
  output << "endsolid cleaned_mesh\n";
}

std::string backend_description() {
#if defined(MESH2SOLID_WITH_CGAL) && defined(MESH2SOLID_WITH_OCCT)
  return "internal_fallback + cgal + opencascade";
#elif defined(MESH2SOLID_WITH_CGAL)
  return "internal_fallback + cgal";
#elif defined(MESH2SOLID_WITH_OCCT)
  return "internal_fallback + opencascade";
#else
  return "internal_fallback";
#endif
}

std::string regions_json(const std::vector<PlaneRegion>& regions) {
  std::ostringstream output;
  output << "{\n  \"regions\": [\n";
  for (std::size_t index = 0; index < regions.size(); ++index) {
    const PlaneRegion& region = regions[index];
    output << "    {\n";
    output << "      \"id\": " << region.id << ",\n";
    output << "      \"triangle_count\": " << region.triangle_indices.size() << ",\n";
    output << "      \"vertex_count\": " << region.vertex_indices.size() << ",\n";
    output << "      \"area\": " << format_double(region.area) << ",\n";
    output << "      \"unresolved\": " << bool_json(region.unresolved) << ",\n";
    output << "      \"fit\": {\n";
    output << "        \"normal\": " << vec3_json(region.fit.normal) << ",\n";
    output << "        \"centroid\": " << vec3_json(region.fit.centroid) << ",\n";
    output << "        \"d\": " << format_double(region.fit.d) << ",\n";
    output << "        \"rms_error\": " << format_double(region.fit.rms_error) << ",\n";
    output << "        \"max_error\": " << format_double(region.fit.max_error) << ",\n";
    output << "        \"confidence\": " << format_double(region.fit.confidence) << "\n";
    output << "      },\n";
    output << "      \"neighbors\": [";
    for (std::size_t neighbor = 0; neighbor < region.neighbor_region_ids.size(); ++neighbor) {
      if (neighbor > 0) {
        output << ",";
      }
      output << region.neighbor_region_ids[neighbor];
    }
    output << "]\n";
    output << "    }";
    if (index + 1 < regions.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string constraints_json(const ConstraintGraph& graph) {
  std::ostringstream output;
  output << "{\n  \"constraints\": [\n";
  for (std::size_t index = 0; index < graph.constraints.size(); ++index) {
    const Constraint& constraint = graph.constraints[index];
    output << "    {\n";
    output << "      \"region_a\": " << constraint.region_a << ",\n";
    output << "      \"region_b\": " << constraint.region_b << ",\n";
    output << "      \"type\": \"" << escape_json(constraint_type_to_string(constraint.type)) << "\",\n";
    output << "      \"applied\": " << bool_json(constraint.applied) << ",\n";
    output << "      \"score\": " << format_double(constraint.score) << ",\n";
    output << "      \"rationale\": \"" << escape_json(constraint.rationale) << "\"\n";
    output << "    }";
    if (index + 1 < graph.constraints.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string reconstruction_debug_json(const ReconstructionResult& reconstruction) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"method\": \"" << escape_json(reconstruction_method_to_string(reconstruction.method))
         << "\",\n";
  output << "  \"vertices\": [\n";
  for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
    output << "    " << vec3_json(reconstruction.vertices[index]);
    if (index + 1 < reconstruction.vertices.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ],\n";
  output << "  \"faces\": [\n";
  for (std::size_t index = 0; index < reconstruction.faces.size(); ++index) {
    const ReconstructedFace& face = reconstruction.faces[index];
    output << "    {\n";
    output << "      \"region_id\": " << face.region_id << ",\n";
    output << "      \"area\": " << format_double(face.area) << ",\n";
    output << "      \"confidence\": " << format_double(face.confidence) << ",\n";
    output << "      \"fit\": {\n";
    output << "        \"normal\": " << vec3_json(face.fit.normal) << ",\n";
    output << "        \"centroid\": " << vec3_json(face.fit.centroid) << ",\n";
    output << "        \"d\": " << format_double(face.fit.d) << "\n";
    output << "      },\n";
    output << "      \"loops\": [\n";
    for (std::size_t loop_index = 0; loop_index < face.loops.size(); ++loop_index) {
      output << "        [";
      for (std::size_t vertex_index = 0; vertex_index < face.loops[loop_index].size(); ++vertex_index) {
        if (vertex_index > 0) {
          output << ",";
        }
        output << face.loops[loop_index][vertex_index];
      }
      output << "]";
      if (loop_index + 1 < face.loops.size()) {
        output << ",";
      }
      output << "\n";
    }
    output << "      ]\n";
    output << "    }";
    if (index + 1 < reconstruction.faces.size()) {
      output << ",";
    }
    output << "\n";
  }
  output << "  ]\n}\n";
  return output.str();
}

std::string report_json(const RunReport& report, const AnalyzeOptions& options) {
  std::ostringstream output;
  output << "{\n";
  output << "  \"input\": \"" << escape_json(options.input_path.string()) << "\",\n";
  output << "  \"preset\": \"" << escape_json(options.preset) << "\",\n";
  output << "  \"backend\": \"" << escape_json(report.backend) << "\",\n";
  output << "  \"mesh_stats_before\": {\n";
  output << "    \"vertex_count\": " << report.repair.before.vertex_count << ",\n";
  output << "    \"triangle_count\": " << report.repair.before.triangle_count << ",\n";
  output << "    \"connected_components\": " << report.repair.before.connected_components << ",\n";
  output << "    \"open_edge_count\": " << report.repair.before.open_edge_count << ",\n";
  output << "    \"non_manifold_edge_count\": " << report.repair.before.non_manifold_edge_count << ",\n";
  output << "    \"surface_area\": " << format_double(report.repair.before.surface_area) << ",\n";
  output << "    \"bbox_diagonal\": " << format_double(report.repair.before.bbox_diagonal) << "\n";
  output << "  },\n";
  output << "  \"mesh_stats_after\": {\n";
  output << "    \"vertex_count\": " << report.repair.after.vertex_count << ",\n";
  output << "    \"triangle_count\": " << report.repair.after.triangle_count << ",\n";
  output << "    \"connected_components\": " << report.repair.after.connected_components << ",\n";
  output << "    \"open_edge_count\": " << report.repair.after.open_edge_count << ",\n";
  output << "    \"non_manifold_edge_count\": " << report.repair.after.non_manifold_edge_count << ",\n";
  output << "    \"surface_area\": " << format_double(report.repair.after.surface_area) << ",\n";
  output << "    \"bbox_diagonal\": " << format_double(report.repair.after.bbox_diagonal) << "\n";
  output << "  },\n";
  output << "  \"repair\": {\n";
  output << "    \"welded_vertices_removed\": " << report.repair.welded_vertices_removed << ",\n";
  output << "    \"duplicate_triangles_removed\": " << report.repair.duplicate_triangles_removed << ",\n";
  output << "    \"degenerate_triangles_removed\": " << report.repair.degenerate_triangles_removed << ",\n";
  output << "    \"tiny_components_removed\": " << report.repair.tiny_components_removed << ",\n";
  output << "    \"tiny_component_triangles_removed\": "
         << report.repair.tiny_component_triangles_removed << ",\n";
  output << "    \"orientation_flips\": " << report.repair.orientation_flips << ",\n";
  output << "    \"warnings\": [";
  for (std::size_t index = 0; index < report.repair.warnings.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << "\"" << escape_json(report.repair.warnings[index]) << "\"";
  }
  output << "]\n";
  output << "  },\n";
  output << "  \"plane_regularization\": {\n";
  output << "    \"method\": \"" << escape_json(report.plane_regularization.method) << "\",\n";
  output << "    \"applied\": " << bool_json(report.plane_regularization.applied) << ",\n";
  output << "    \"adjusted_planes\": " << report.plane_regularization.adjusted_planes << "\n";
  output << "  },\n";
  output << "  \"tolerances\": {\n";
  output << "    \"vertex_weld\": " << format_double(report.tolerances.vertex_weld) << ",\n";
  output << "    \"plane_distance\": " << format_double(report.tolerances.plane_distance) << ",\n";
  output << "    \"plane_merge_distance\": " << format_double(report.tolerances.plane_merge_distance) << ",\n";
  output << "    \"collinear_distance\": " << format_double(report.tolerances.collinear_distance) << ",\n";
  output << "    \"normal_angle_degrees\": " << format_double(report.tolerances.normal_angle_degrees) << ",\n";
  output << "    \"merge_angle_degrees\": " << format_double(report.tolerances.merge_angle_degrees) << ",\n";
  output << "    \"parallel_angle_degrees\": " << format_double(report.tolerances.parallel_angle_degrees)
         << ",\n";
  output << "    \"perpendicular_angle_degrees\": "
         << format_double(report.tolerances.perpendicular_angle_degrees) << "\n";
  output << "  },\n";
  output << "  \"regions\": {\n";
  output << "    \"count\": " << report.regions.size() << ",\n";
  output << "    \"unresolved_count\": "
         << std::count_if(report.regions.begin(), report.regions.end(),
                          [](const PlaneRegion& region) { return region.unresolved; })
         << "\n";
  output << "  },\n";
  output << "  \"constraints\": {\n";
  output << "    \"count\": " << report.constraint_graph.constraints.size() << "\n";
  output << "  },\n";
  output << "  \"reconstruction\": {\n";
  output << "    \"outcome\": \""
         << escape_json(reconstruction_outcome_to_string(report.reconstruction.outcome)) << "\",\n";
  output << "    \"method\": \""
         << escape_json(reconstruction_method_to_string(report.reconstruction.method)) << "\",\n";
  output << "    \"face_count\": " << report.reconstruction.faces.size() << ",\n";
  output << "    \"vertex_count\": " << report.reconstruction.vertices.size() << ",\n";
  output << "    \"edge_split_insertions\": " << report.reconstruction.edge_split_insertions << ",\n";
  output << "    \"open_edge_count\": " << report.reconstruction.open_edge_count << ",\n";
  output << "    \"non_manifold_edge_count\": " << report.reconstruction.non_manifold_edge_count << ",\n";
  output << "    \"omitted_region_ids\": [";
  for (std::size_t index = 0; index < report.reconstruction.omitted_region_ids.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << report.reconstruction.omitted_region_ids[index];
  }
  output << "],\n";
  output << "    \"problematic_regions\": [";
  for (std::size_t index = 0; index < report.reconstruction.problematic_regions.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    const RegionTopologyIssue& issue = report.reconstruction.problematic_regions[index];
    output << "{\"region_id\":" << issue.region_id << ",\"edge_count\":" << issue.edge_count << "}";
  }
  output << "],\n";
  output << "    \"shell_gap_score\": " << format_double(report.reconstruction.shell_gap_score) << ",\n";
  output << "    \"confidence\": " << format_double(report.reconstruction.confidence) << ",\n";
  output << "    \"step_written\": " << bool_json(report.reconstruction.step_written) << ",\n";
  output << "    \"failure_reasons\": [";
  for (std::size_t index = 0; index < report.reconstruction.failure_reasons.size(); ++index) {
    if (index > 0) {
      output << ",";
    }
    output << "\"" << escape_json(report.reconstruction.failure_reasons[index]) << "\"";
  }
  output << "]\n";
  output << "  }\n";
  output << "}\n";
  return output.str();
}

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("Unable to write file: " + path.string());
  }
  output << text;
}
