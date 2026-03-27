class StepLineLoopWriter {
 public:
  StepLineLoopWriter(StepBuilder& builder,
                     const StepExportContext& export_context,
                     const ReconstructionResult& mesh)
      : step(builder),
        context_ids(export_context),
        reconstruction(mesh),
        point_ids(mesh.vertices.size(), 0),
        vertex_point_ids(mesh.vertices.size(), 0) {
    for (std::size_t index = 0; index < reconstruction.vertices.size(); ++index) {
      const Vec3& vertex = reconstruction.vertices[index];
      point_ids[index] = step.add("CARTESIAN_POINT('',(" + format_double(vertex.x) + "," +
                                 format_double(vertex.y) + "," + format_double(vertex.z) + "))");
      vertex_point_ids[index] = step.add("VERTEX_POINT(''," + step.ref(point_ids[index]) + ")");
    }
  }

  int add_line_loop(const std::vector<int>& loop) {
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
  }

  int add_plane_circle_pcurve(const StepPlaneSurface& plane,
                              const Vec3& circle_center,
                              double radius,
                              const Vec3& circle_ref_direction) {
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
  }

  int line_edge_curve_id(int from, int to) { return get_line_edge_curve_id(from, to); }

  StepBuilder& step;
  const StepExportContext& context_ids;
  const ReconstructionResult& reconstruction;

 private:
  int get_line_edge_curve_id(int from, int to) {
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
  }

  std::vector<int> point_ids;
  std::vector<int> vertex_point_ids;
  std::unordered_map<EdgeKey, int, EdgeKeyHash> edge_curve_ids;
};
