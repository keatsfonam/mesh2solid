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

struct StepExportContext {
  int product_shape {};
  int context {};
  int context_2d {};
  int global_axis {};
};

struct StepPlaneSurface {
  int plane_id {};
  Vec3 origin {};
  Vec3 normal {};
  PlaneBasis basis {};
};

struct StepCylinderBundle {
  int top_edge_curve {};
  int bottom_edge_curve {};
  int seam_edge_curve {};
  int lateral_face {};
};

struct StepSmoothBandBundle {
  int top_edge_curve {};
  int bottom_edge_curve {};
  int start_seam_edge_curve {};
  int end_seam_edge_curve {};
  int lateral_face {};
};
