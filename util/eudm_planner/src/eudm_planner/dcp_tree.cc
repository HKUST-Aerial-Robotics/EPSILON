/**
 * @file behavior_tree.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#include "eudm_planner/dcp_tree.h"

namespace planning {
DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time)
    : tree_height_(tree_height), layer_time_(layer_time) {
  last_layer_time_ = layer_time_;
  GenerateActionScript();
}

DcpTree::DcpTree(const int& tree_height, const decimal_t& layer_time,
                 const decimal_t& last_layer_time)
    : tree_height_(tree_height),
      layer_time_(layer_time),
      last_layer_time_(last_layer_time) {
  GenerateActionScript();
}

ErrorType DcpTree::UpdateScript() { GenerateActionScript(); }

std::vector<DcpTree::DcpAction> DcpTree::AppendActionSequence(
    const std::vector<DcpAction>& seq_in, const DcpAction& a,
    const int& n) const {
  std::vector<DcpAction> seq = seq_in;
  for (int i = 0; i < n; ++i) {
    seq.push_back(a);
  }
  return seq;
}

ErrorType DcpTree::GenerateActionScript() {
  action_script_.clear();
  std::vector<DcpAction> ongoing_action_seq;
  for (int lon = 0; lon < static_cast<int>(DcpLonAction::MAX_COUNT); lon++) {
    ongoing_action_seq.clear();
    ongoing_action_seq.push_back(
        DcpAction(DcpLonAction(lon), ongoing_action_.lat, ongoing_action_.t));

    for (int h = 1; h < tree_height_; ++h) {
      for (int lat = 0; lat < static_cast<int>(DcpLatAction::MAX_COUNT);
           lat++) {
        if (lat != static_cast<int>(ongoing_action_.lat)) {
          auto actions = AppendActionSequence(
              ongoing_action_seq,
              DcpAction(DcpLonAction(lon), DcpLatAction(lat), layer_time_),
              tree_height_ - h);
          action_script_.push_back(actions);
        }
      }
      ongoing_action_seq.push_back(
          DcpAction(DcpLonAction(lon), ongoing_action_.lat, layer_time_));
    }
    action_script_.push_back(ongoing_action_seq);
  }
  // override the last layer time
  for (auto& action_seq : action_script_) {
    action_seq.back().t = last_layer_time_;
  }
  return kSuccess;
}

decimal_t DcpTree::planning_horizon() const {
  if (action_script_.empty()) return 0.0;
  decimal_t planning_horizon = 0.0;
  for (const auto& a : action_script_[0]) {
    planning_horizon += a.t;
  }
  return planning_horizon;
}

}  // namespace planning
