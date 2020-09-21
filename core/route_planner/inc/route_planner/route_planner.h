#ifndef _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_
#define _CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_

#include <memory>
#include <random>
#include <set>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/state/state.h"

namespace planning {

class RoutePlanner : public Planner {
 public:
  enum NaviMode { kRandomExpansion, kAssignedTarget };
  enum NaviStatus { kReadyToGo, kInProgress, kFinished };

  std::string Name() override;

  ErrorType Init(const std::string config) override;

  ErrorType RunOnce() override;

  void set_navi_mode(const NaviMode& mode) { navi_mode_ = mode; };
  void set_ego_state(const common::State& state) { ego_state_ = state; }
  void set_nearest_lane_id(const int& id) { nearest_lane_id_ = id; }
  void set_lane_net(const common::LaneNet& lane_net) {
    lane_net_ = lane_net;
    if_get_lane_net_ = true;
  }

  std::vector<int> navi_path() const { return navi_path_; }
  common::Lane navi_lane() const { return navi_lane_; }

  bool if_get_lane_net() const { return if_get_lane_net_; }

  decimal_t navi_cur_arc_len() const { return navi_cur_arc_len_; }

 private:
  ErrorType GetChildLaneIds(const int lane_id, std::vector<int>* child_ids);

  ErrorType GetNaviPathByRandomExpansion();

  bool CheckIfArriveTargetLane();
  ErrorType CheckNaviProgress();

  ErrorType NaviLoopRandomExpansion();
  ErrorType NaviLoopAssignedTarget();

  common::LaneNet lane_net_;
  common::State ego_state_;

  int nearest_lane_id_;

  NaviStatus navi_status_ = kReadyToGo;
  NaviMode navi_mode_ = kRandomExpansion;

  bool if_restart_ = true;
  bool if_get_lane_net_ = false;

  // decimal_t navi_path_max_length_ = 1500;
  decimal_t navi_path_max_length_ = 200;
  decimal_t navi_start_arc_length_{0.0};
  // decimal_t navi_tail_remain_ = 200;
  decimal_t navi_path_length_{0.0};
  decimal_t navi_cur_arc_len_{0.0};

  std::vector<int> navi_path_;
  common::Lane navi_lane_;

  std::random_device rd_gen_;
};

}  // namespace planning

#endif  //_CORE_ROUTE_PLANNER_INC_ROUTE_PLANNER_H_