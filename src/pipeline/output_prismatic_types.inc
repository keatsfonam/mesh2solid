struct SimplePrismaticBoreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBore {
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive bore {};
};

struct SimplePrismaticMultiBoreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBore> bores;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBlindBoreExport {
  int top_face_index {};
  int floor_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> floor_loop;
  double top_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBlindBore {
  double top_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive bore {};
};

struct SimplePrismaticMultiBlindBoreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBlindBore> bores;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBlindCounterbore {
  double top_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
};

struct SimplePrismaticMultiBlindCountersink {
  double top_inner_area {};
  double floor_area {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive bore {};
};

struct SimplePrismaticMultiBlindCountersinkExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBlindCountersink> countersinks;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBlindCounterboreExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticMultiBlindCounterbore> counterbores;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBlindCounterboreExport {
  int top_face_index {};
  int step_face_index {};
  int floor_face_index {};
  std::vector<int> top_outer_loop;
  double top_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  double floor_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBlindCountersinkExport {
  int top_face_index {};
  int floor_face_index {};
  std::vector<int> top_outer_loop;
  double top_inner_area {};
  double floor_area {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticCounterboreExport {
  int top_face_index {};
  int bottom_face_index {};
  int step_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticCountersinkExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive bore {};
  std::vector<int> outer_face_indices;
};

// Phase-1 canonical prismatic section model:
// shared-cap axial features all reduce to the same 2.5D description:
// paired cap loops plus one or more feature columns whose radius profile is
// constant, stepped, or tapered along the shared axis.
enum class PrismaticAxialThroughFeatureKind {
  bore,
  countersink,
  counterbore,
};

struct SimplePrismaticAxialThroughFeature {
  double top_inner_area {};
  double bottom_inner_area {};
  double step_outer_area {};
  double step_inner_area {};
  PrismaticAxialThroughFeatureKind kind {PrismaticAxialThroughFeatureKind::bore};
  SimpleCylinderPrimitive bore {};
  SimpleConicalFrustumPrimitive countersink {};
  SimpleCylinderPrimitive upper_bore {};
  SimpleCylinderPrimitive lower_bore {};
};

struct SimplePrismaticAxialThroughFeatureExport {
  int top_face_index {};
  int bottom_face_index {};
  std::vector<int> top_outer_loop;
  std::vector<int> bottom_outer_loop;
  std::vector<SimplePrismaticAxialThroughFeature> features;
  std::vector<int> outer_face_indices;
};

using PrismaticAxialSectionColumn = SimplePrismaticAxialThroughFeature;
using PrismaticAxialSectionModel = SimplePrismaticAxialThroughFeatureExport;

struct PrismaticBlindSectionModel {
  int top_face_index {};
  int support_bottom_face_index {-1};
  std::vector<int> top_outer_loop;
  std::vector<int> support_bottom_outer_loop;
  std::vector<PrismaticAxialSectionColumn> features;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticBossExport {
  int base_face_index {};
  int top_face_index {};
  std::vector<int> base_outer_loop;
  double base_inner_area {};
  double top_area {};
  SimpleCylinderPrimitive boss {};
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticMultiBoss {
  double base_inner_area {};
  double top_area {};
  SimpleCylinderPrimitive boss {};
};

struct SimplePrismaticMultiBossExport {
  int bottom_face_index {};
  int base_face_index {};
  std::vector<int> bottom_outer_loop;
  std::vector<int> base_outer_loop;
  std::vector<SimplePrismaticMultiBoss> bosses;
  std::vector<int> outer_face_indices;
};

struct SimplePrismaticStandoffExport {
  int top_face_index {};
  int base_face_index {};
  int bottom_face_index {};
  std::vector<int> base_outer_loop;
  std::vector<int> bottom_outer_loop;
  double top_outer_area {};
  double top_inner_area {};
  double base_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive outer_cylinder {};
  SimpleCylinderPrimitive inner_cylinder {};
  std::vector<int> outer_face_indices;
};

struct PrismaticSupportSectionColumn {
  double base_inner_area {};
  double top_outer_area {};
  double top_inner_area {};
  double bottom_inner_area {};
  SimpleCylinderPrimitive outer_cylinder {};
  bool has_inner_cylinder {};
  SimpleCylinderPrimitive inner_cylinder {};
};

struct PrismaticSupportSectionModel {
  int base_face_index {};
  int support_bottom_face_index {-1};
  std::vector<int> base_outer_loop;
  std::vector<int> support_bottom_outer_loop;
  std::vector<PrismaticSupportSectionColumn> features;
  std::vector<int> outer_face_indices;
};
