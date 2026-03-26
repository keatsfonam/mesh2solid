ReconstructionResult reconstruct_faceted_mesh_fallback(const MeshModel& mesh,
                                                       double solid_threshold) {
  ReconstructionResult result;
  result.method = ReconstructionMethod::FacetedMeshFallback;
  result.vertices = mesh.vertices;
  result.faces.reserve(mesh.triangles.size());

  for (const Triangle& triangle : mesh.triangles) {
    if (triangle.area <= 1e-12) {
      continue;
    }

    ReconstructedFace face;
    face.region_id = triangle.region_id;
    face.loops.push_back(
        {triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]});

    Vec3 normal = triangle.normal;
    if (length_squared(normal) <= 1e-12) {
      const Vec3& a = mesh.vertices[triangle.vertices[0]];
      const Vec3& b = mesh.vertices[triangle.vertices[1]];
      const Vec3& c = mesh.vertices[triangle.vertices[2]];
      normal = normalized(cross(b - a, c - a));
    }
    if (length_squared(normal) <= 1e-12) {
      continue;
    }

    Vec3 centroid {};
    for (int vertex_index : triangle.vertices) {
      centroid += mesh.vertices[vertex_index];
    }
    centroid = centroid / 3.0;

    face.fit.normal = normalized(normal);
    face.fit.centroid = centroid;
    face.fit.d = -dot(face.fit.normal, face.fit.centroid);
    face.fit.rms_error = 0.0;
    face.fit.max_error = 0.0;
    face.fit.confidence = 1.0;
    face.confidence = 1.0;
    face.area = triangle.area;
    result.faces.push_back(std::move(face));
  }

  assess_shell_topology(result, mesh, solid_threshold);
  return result;
}
