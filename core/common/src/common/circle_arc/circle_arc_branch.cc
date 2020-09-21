#include "common/circle_arc/circle_arc_branch.h"

namespace common {

void CircleArcBranch::RetFinalStates(std::vector<Vec3f> *p_states) const {
  for (const auto &arc : circle_arc_vec_) {
    p_states->emplace_back(arc.final_state());
  }
}

void CircleArcBranch::RetAllSampledStates(std::vector<Vec3f> *p_states) const {
  for (const auto &arc : circle_arc_vec_) {
    std::vector<Vec3f> samples;
    arc.GetSampledStates(0.2, &samples);
    for (const auto &state : samples) {
      p_states->emplace_back(state);
    }
  }
}

void CircleArcBranch::CalculateCircleArcBranch() {
  int n_arcs = curvature_vec_.size();
  if ((int)length_vec_.size() != n_arcs) {
    std::cerr << "[CircleArcBranch] ERROR - Size of vec are not equal"
              << std::endl;
    assert(false);
  }
  for (int i = 0; i < n_arcs; ++i) {
    CircleArc arc(start_state_, curvature_vec_[i], length_vec_[i]);
    circle_arc_vec_.emplace_back(arc);
  }
}

}  // namespace common