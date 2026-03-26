#pragma once

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace stl2solid {

struct Vec3 {
  double x {};
  double y {};
  double z {};
};

struct Bounds {
  Vec3 min {};
  Vec3 max {};
  bool valid {false};
};

struct Triangle {
  std::array<int, 3> vertices {};
  Vec3 normal {};
  double area {};
  int region_id {-1};
};

struct MeshStats {
  std::size_t vertex_count {};
  std::size_t triangle_count {};
  std::size_t connected_components {};
  std::size_t open_edge_count {};
  std::size_t non_manifold_edge_count {};
  double surface_area {};
  double bbox_diagonal {};
};

struct MeshModel {
  std::vector<Vec3> vertices;
  std::vector<Triangle> triangles;
  Bounds bounds;
  std::string source_format;
};

struct Tolerances {
  double vertex_weld {};
  double plane_distance {};
  double plane_merge_distance {};
  double collinear_distance {};
  double normal_angle_degrees {};
  double merge_angle_degrees {};
  double parallel_angle_degrees {};
  double perpendicular_angle_degrees {};
  double min_region_confidence {};
  double min_component_area_ratio {};
  std::size_t min_component_triangles {};
};

struct RepairReport {
  MeshStats before;
  MeshStats after;
  std::size_t welded_vertices_removed {};
  std::size_t duplicate_triangles_removed {};
  std::size_t degenerate_triangles_removed {};
  std::size_t tiny_components_removed {};
  std::size_t tiny_component_triangles_removed {};
  std::size_t orientation_flips {};
  std::vector<std::string> warnings;
};

struct PlaneFit {
  Vec3 normal {};
  double d {};
  Vec3 centroid {};
  double rms_error {};
  double max_error {};
  double confidence {};
};

struct PlaneRegion {
  int id {};
  std::vector<int> triangle_indices;
  std::vector<int> vertex_indices;
  std::vector<int> neighbor_region_ids;
  double area {};
  bool unresolved {};
  PlaneFit fit;
};

enum class ConstraintType {
  CoplanarMerge,
  Parallel,
  Perpendicular,
};

struct Constraint {
  int region_a {};
  int region_b {};
  ConstraintType type {ConstraintType::CoplanarMerge};
  bool applied {};
  double score {};
  std::string rationale;
};

struct ConstraintGraph {
  std::vector<Constraint> constraints;
};

struct ReconstructedFace {
  int region_id {};
  PlaneFit fit;
  std::vector<std::vector<int>> loops;
  double area {};
  double confidence {};
};

struct RegionTopologyIssue {
  int region_id {};
  std::size_t edge_count {};
};

enum class ReconstructionOutcome {
  AnalysisOnly,
  ShellOnly,
  SolidCreated,
};

struct ReconstructionResult {
  ReconstructionOutcome outcome {ReconstructionOutcome::AnalysisOnly};
  std::vector<Vec3> vertices;
  std::vector<ReconstructedFace> faces;
  std::vector<std::string> failure_reasons;
  std::vector<int> skipped_region_ids;
  std::size_t edge_split_insertions {};
  std::size_t open_edge_count {};
  std::size_t non_manifold_edge_count {};
  std::vector<int> omitted_region_ids;
  std::vector<RegionTopologyIssue> problematic_regions;
  double shell_gap_score {};
  double confidence {};
  bool step_written {};
};

struct AnalyzeOptions {
  std::filesystem::path input_path;
  std::filesystem::path output_dir;
  std::string preset {"mechanical"};
  double solid_threshold {0.75};
};

struct RunReport {
  MeshModel cleaned_mesh;
  RepairReport repair;
  std::vector<PlaneRegion> regions;
  ConstraintGraph constraint_graph;
  ReconstructionResult reconstruction;
  Tolerances tolerances;
  std::string backend;
};

RunReport analyze(const AnalyzeOptions& options);
void write_outputs(const AnalyzeOptions& options, RunReport& report);

std::string reconstruction_outcome_to_string(ReconstructionOutcome outcome);
std::string constraint_type_to_string(ConstraintType type);

}  // namespace stl2solid
