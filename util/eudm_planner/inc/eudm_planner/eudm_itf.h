#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

namespace planning {
namespace eudm {

struct LaneChangeInfo {
  bool forbid_lane_change_left = false;
  bool forbid_lane_change_right = false;
  bool lane_change_left_unsafe_by_occu = false;
  bool lane_change_right_unsafe_by_occu = false;
  bool left_solid_lane = false;
  bool right_solid_lane = false;
  bool recommend_lc_left = false;
  bool recommend_lc_right = false;
};

struct Task {
  bool is_under_ctrl = false;
  double user_desired_vel;
  int user_perferred_behavior = 0;
  LaneChangeInfo lc_info;
};

}  // namespace eudm
}  // namespace planning

#endif
