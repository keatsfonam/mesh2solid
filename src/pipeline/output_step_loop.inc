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

struct StepSyntheticVertexInfo {
  int point_id {};
  int vertex_id {};
  Vec3 position {};
};

class StepSyntheticLoopWriter {
 public:
  StepSyntheticLoopWriter(StepBuilder& builder, const std::vector<Vec3>& base_vertices)
      : step(builder) {
    vertices.reserve(base_vertices.size());
    for (const Vec3& vertex : base_vertices) {
      append_vertex(vertex);
    }
  }

  int add_vertex(const Vec3& point) {
    append_vertex(point);
    return static_cast<int>(vertices.size()) - 1;
  }

  const Vec3& position(int vertex_index) const { return vertices[vertex_index].position; }

  int point_id(int vertex_index) const { return vertices[vertex_index].point_id; }

  int vertex_id(int vertex_index) const { return vertices[vertex_index].vertex_id; }

  int add_line_edge_curve_id(int from, int to) {
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
    const int line_id = step.add("LINE(''," + step.ref(vertices[key.a].point_id) + "," +
                                 step.ref(vector_id) + ")");
    const int edge_curve_id =
        step.add("EDGE_CURVE(''," + step.ref(vertices[key.a].vertex_id) + "," +
                 step.ref(vertices[key.b].vertex_id) + "," + step.ref(line_id) + ",.T.)");
    line_edge_curve_ids.emplace(key, edge_curve_id);
    return edge_curve_id;
  }

  int add_oriented_edge(int edge_curve_id, bool same_orientation) {
    return step.add("ORIENTED_EDGE('',*,*," + step.ref(edge_curve_id) + "," +
                    std::string(same_orientation ? ".T." : ".F.") + ")");
  }

  int add_line_loop(const std::vector<int>& loop) {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < loop.size(); ++index) {
      const int from = loop[index];
      const int to = loop[(index + 1) % loop.size()];
      const int edge_curve_id = add_line_edge_curve_id(from, to);
      const EdgeKey key = make_edge_key(from, to);
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(add_oriented_edge(edge_curve_id, key.a == from && key.b == to));
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  }

  int add_edge_loop(const std::vector<std::pair<int, bool>>& edges) {
    std::ostringstream edge_refs;
    for (std::size_t index = 0; index < edges.size(); ++index) {
      if (index > 0) {
        edge_refs << ",";
      }
      edge_refs << step.ref(add_oriented_edge(edges[index].first, edges[index].second));
    }
    return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
  }

 private:
  void append_vertex(const Vec3& point) {
    const int point_id = step.add("CARTESIAN_POINT('',(" + format_double(point.x) + "," +
                                  format_double(point.y) + "," + format_double(point.z) + "))");
    const int vertex_id = step.add("VERTEX_POINT(''," + step.ref(point_id) + ")");
    vertices.push_back({point_id, vertex_id, point});
  }

  StepBuilder& step;
  std::vector<StepSyntheticVertexInfo> vertices;
  std::unordered_map<EdgeKey, int, EdgeKeyHash> line_edge_curve_ids;
};

struct StepLoopCurveReplacement {
  LoopSubsequenceMatch match;
  int edge_curve_id {};
};

int add_step_loop_with_curve_replacements(
    StepBuilder& step,
    StepLineLoopWriter& writer,
    const std::vector<int>& loop,
    const std::vector<StepLoopCurveReplacement>& replacements) {
  if (loop.size() < 2 || replacements.empty()) {
    return writer.add_line_loop(loop);
  }

  std::unordered_map<std::size_t, std::size_t> replacement_by_start;
  std::vector<bool> covered(loop.size(), false);
  for (std::size_t replacement_index = 0; replacement_index < replacements.size();
       ++replacement_index) {
    const StepLoopCurveReplacement& replacement = replacements[replacement_index];
    if (replacement.match.span == 0 || replacement.match.span >= loop.size()) {
      return writer.add_line_loop(loop);
    }
    if (!replacement_by_start.emplace(replacement.match.start, replacement_index).second) {
      return writer.add_line_loop(loop);
    }
    for (std::size_t offset = 0; offset < replacement.match.span; ++offset) {
      const std::size_t edge_slot = (replacement.match.start + offset) % loop.size();
      if (covered[edge_slot]) {
        return writer.add_line_loop(loop);
      }
      covered[edge_slot] = true;
    }
  }

  std::ostringstream edge_refs;
  bool first_edge = true;
  const auto append_oriented_edge = [&](int curve_id, bool same_orientation) {
    if (!first_edge) {
      edge_refs << ",";
    }
    first_edge = false;
    edge_refs << step.ref(
        step.add("ORIENTED_EDGE('',*,*," + step.ref(curve_id) + "," +
                 std::string(same_orientation ? ".T." : ".F.") + ")"));
  };

  std::size_t edge_index = 0;
  bool found_start = false;
  for (std::size_t candidate = 0; candidate < loop.size(); ++candidate) {
    if (!covered[candidate]) {
      edge_index = candidate;
      found_start = true;
      break;
    }
  }
  if (!found_start) {
    edge_index = replacements.front().match.start % loop.size();
  }

  std::size_t processed_edges = 0;
  while (processed_edges < loop.size()) {
    const auto replacement_found = replacement_by_start.find(edge_index);
    if (replacement_found != replacement_by_start.end()) {
      const StepLoopCurveReplacement& replacement = replacements[replacement_found->second];
      append_oriented_edge(replacement.edge_curve_id, replacement.match.forward);
      edge_index = (edge_index + replacement.match.span) % loop.size();
      processed_edges += replacement.match.span;
      continue;
    }

    const int from = loop[edge_index];
    const int to = loop[(edge_index + 1) % loop.size()];
    const int line_edge_curve_id = writer.line_edge_curve_id(from, to);
    const EdgeKey key = make_edge_key(from, to);
    append_oriented_edge(line_edge_curve_id, key.a == from && key.b == to);
    edge_index = (edge_index + 1) % loop.size();
    processed_edges += 1;
  }

  return step.add("EDGE_LOOP('',(" + edge_refs.str() + "))");
}

int add_step_planar_face_from_edge_loops(StepBuilder& step,
                                         int plane_id,
                                         const std::vector<int>& edge_loop_ids,
                                         std::size_t outer_loop_index = 0,
                                         bool same_sense = true) {
  std::vector<int> bound_ids;
  bound_ids.reserve(edge_loop_ids.size());
  for (std::size_t loop_index = 0; loop_index < edge_loop_ids.size(); ++loop_index) {
    const std::string bound_prefix = loop_index == outer_loop_index ? "FACE_OUTER_BOUND" : "FACE_BOUND";
    bound_ids.push_back(step.add(bound_prefix + "(''," + step.ref(edge_loop_ids[loop_index]) + ",.T.)"));
  }

  std::ostringstream bound_refs;
  for (std::size_t index = 0; index < bound_ids.size(); ++index) {
    if (index > 0) {
      bound_refs << ",";
    }
    bound_refs << step.ref(bound_ids[index]);
  }
  return step.add("ADVANCED_FACE('',(" + bound_refs.str() + ")," + step.ref(plane_id) + "," +
                  std::string(same_sense ? ".T." : ".F.") + ")");
}

int add_step_face_from_single_outer_loop(StepBuilder& step,
                                         int surface_id,
                                         int edge_loop_id,
                                         bool same_sense = true) {
  const int bound_id = step.add("FACE_OUTER_BOUND(''," + step.ref(edge_loop_id) + ",.T.)");
  return step.add("ADVANCED_FACE('',(" + step.ref(bound_id) + ")," + step.ref(surface_id) + "," +
                  std::string(same_sense ? ".T." : ".F.") + ")");
}

int add_step_planar_face_from_loops(StepBuilder& step,
                                    StepLineLoopWriter& writer,
                                    int plane_id,
                                    const std::vector<std::vector<int>>& loops,
                                    std::size_t outer_loop_index = 0,
                                    bool same_sense = true) {
  std::vector<int> edge_loop_ids;
  edge_loop_ids.reserve(loops.size());
  for (const std::vector<int>& loop : loops) {
    edge_loop_ids.push_back(writer.add_line_loop(loop));
  }
  return add_step_planar_face_from_edge_loops(
      step, plane_id, edge_loop_ids, outer_loop_index, same_sense);
}
