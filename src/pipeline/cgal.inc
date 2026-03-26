#if defined(MESH2SOLID_WITH_CGAL)
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Regularization/regularize_planes.h>
#include <CGAL/number_utils.h>
#include <CGAL/property_map.h>
#include <boost/property_map/property_map.hpp>

struct CgalPlaneIndexMap {
  using key_type = std::size_t;
  using value_type = int;
  using reference = int;
  using category = boost::readable_property_map_tag;

  const std::vector<int>* assignments {};
};

inline int get(const CgalPlaneIndexMap& map, std::size_t index) {
  return (*map.assignments)[index];
}

PlaneRegularizationReport regularize_plane_regions_with_cgal(const MeshModel& mesh,
                                                             std::vector<PlaneRegion>& regions,
                                                             const Tolerances& tolerances) {
  PlaneRegularizationReport report;
  report.method = "cgal_regularize_planes";
  if (regions.size() < 2) {
    return report;
  }

  using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
  using Point_3 = Kernel::Point_3;
  using Plane_3 = Kernel::Plane_3;

  std::vector<Point_3> points;
  std::vector<int> plane_assignments;
  std::vector<Plane_3> planes;
  points.reserve(mesh.vertices.size());
  plane_assignments.reserve(mesh.vertices.size());
  planes.reserve(regions.size());

  for (const PlaneRegion& region : regions) {
    const Vec3 normal = normalized(region.fit.normal);
    planes.emplace_back(normal.x, normal.y, normal.z, region.fit.d);
    for (int vertex_index : region.vertex_indices) {
      const Vec3& vertex = mesh.vertices[vertex_index];
      points.emplace_back(vertex.x, vertex.y, vertex.z);
      plane_assignments.push_back(region.id);
    }
  }

  if (points.empty() || plane_assignments.empty()) {
    return report;
  }

  CgalPlaneIndexMap plane_index_map {&plane_assignments};
  CGAL::regularize_planes(points, CGAL::Identity_property_map<Point_3>(), planes,
                          CGAL::Identity_property_map<Plane_3>(), plane_index_map, Kernel(),
                          true, true, true, false, tolerances.parallel_angle_degrees,
                          tolerances.plane_merge_distance);

  std::size_t adjusted_planes = 0;
  for (std::size_t index = 0; index < regions.size(); ++index) {
    const Plane_3& plane = planes[index];
    const auto orthogonal = plane.orthogonal_vector();
    Vec3 normal {
        CGAL::to_double(orthogonal.x()),
        CGAL::to_double(orthogonal.y()),
        CGAL::to_double(orthogonal.z()),
    };

    double d = CGAL::to_double(plane.d());
    const double scale = length(normal);
    if (scale <= 1e-12) {
      continue;
    }

    normal = normal / scale;
    d /= scale;
    if (dot(normal, regions[index].fit.normal) < 0.0) {
      normal = normal * -1.0;
      d *= -1.0;
    }

    const double angle_delta = angle_degrees_unsigned(normal, regions[index].fit.normal);
    const double offset_delta = std::abs(d - regions[index].fit.d);
    if (angle_delta > 1e-6 || offset_delta > 1e-6) {
      ++adjusted_planes;
    }

    regions[index].fit.normal = normal;
    regions[index].fit.d = d;
    const double centroid_distance =
        dot(regions[index].fit.normal, regions[index].fit.centroid) + regions[index].fit.d;
    regions[index].fit.centroid =
        regions[index].fit.centroid - regions[index].fit.normal * centroid_distance;
  }

  report.applied = adjusted_planes > 0;
  report.adjusted_planes = adjusted_planes;
  return report;
}
#else
PlaneRegularizationReport regularize_plane_regions_with_cgal(const MeshModel&,
                                                             std::vector<PlaneRegion>&,
                                                             const Tolerances&) {
  return {};
}
#endif
