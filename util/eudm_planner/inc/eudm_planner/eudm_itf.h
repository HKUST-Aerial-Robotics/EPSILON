#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "hkust_msg_transformer/Freespace.h"
#include "hkust_msg_transformer/LaneSet.h"
#include "hkust_msg_transformer/MotSet.h"
#include "hkust_msg_transformer/PlainOutput.h"
#include "hkust_msg_transformer/Vins.h"

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

struct RawInput {
  double stamp;
  hkust_msg_transformer::Vins state;
  hkust_msg_transformer::MotSet mot_set;
  hkust_msg_transformer::LaneSet lane_set;
  hkust_msg_transformer::Freespace freespace;
};

struct PlainState {
  double timestamp;
  double x;
  double y;
  double angle;
  double vel;
  double acc;
  double curvature;

  PlainState() {}
  PlainState(double _timestamp, double _x, double _y, double _angle,
             double _vel, double _acc, double _curvature)
      : timestamp(_timestamp),
        x(_x),
        y(_y),
        angle(_angle),
        vel(_vel),
        acc(_acc),
        curvature(_curvature) {}
};

struct PlainCost {
  int valid_sample_index_ub;
  double efficiency = 0.0;
  double safety = 0.0;
  double navigation = 0.0;
  double weight = 0.0;

  PlainCost() {}
  PlainCost(int _valid_sample_index_ub, double _efficiency, double _safety,
            double _navigation, double _weight)
      : valid_sample_index_ub(_valid_sample_index_ub),
        efficiency(_efficiency),
        safety(_safety),
        navigation(_navigation),
        weight(_weight) {}
};

struct OutputElement {
  // * total states
  bool valid = false;
  bool risky = false;
  std::string sim_info;
  // * macro states
  std::vector<int> lat_behaviors;
  std::vector<int> lon_behaviors;
  // * macro cmd
  std::vector<int> lat_cmds;
  std::vector<int> lon_cmds;

  double final_cost = 0.0; // f = g + h
  std::vector<PlainCost> stage_costs; // g: progress cost
  PlainCost tail_cost; // h: tail cost
  // * micro states
  std::vector<PlainState> ego_traj;
  std::unordered_map<int, std::vector<PlainState>> surround_trajs;
};

struct PlainOutput {
  bool valid = false;
  int winner_element_id;
  int original_winner_id;
  double best_cost;
  double ongoing_action_duration;
  std::vector<OutputElement> elements;

  PlainState ego_state;

  double plan_stamp = 0.0;
  double time_cost = 0.0;
};

/**
 * @brief: interface for eudm planner
 * @param:
 */
class EudmItf {
 public:
  virtual ~EudmItf() = default;
  virtual void Init(const std::string config_path, double work_rate) = 0;
  virtual bool Plan(const RawInput& raw, const Task& task,
                    PlainOutput* output) = 0;
  virtual void Publish() = 0;
  virtual void Clean() = 0;
};

EudmItf* Create();

}  // namespace eudm
}  // namespace planning

#endif
