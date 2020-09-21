#include "route_planner/route_planner.h"

namespace planning {

std::string RoutePlanner::Name() {
  return std::string("Generic route planner");
}

ErrorType RoutePlanner::Init(const std::string config) {
  navi_status_ = kReadyToGo;
  return kSuccess;
}

ErrorType RoutePlanner::RunOnce() {
  switch (navi_mode_) {
    case kRandomExpansion:
      NaviLoopRandomExpansion();
      break;
    case kAssignedTarget:
      NaviLoopAssignedTarget();
      break;
    default:
      assert(false);
  }
  // ~ if navi lane is not valid, we should tell the caller
  if (!navi_lane_.IsValid()) {
    printf("[RP]Err - fail to output valid navi lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType RoutePlanner::NaviLoopRandomExpansion() {
  switch (navi_status_) {
    case kReadyToGo:
      if (GetNaviPathByRandomExpansion() != kSuccess) {
        printf("[RP]Err - fail to find navi path.\n");
        break;
      }
      navi_status_ = kInProgress;
      break;
    case kInProgress:
      if (CheckNaviProgress() != kSuccess) {
        printf("[RP]Err - InProgress but fail to check navi progress.\n");
        printf("[RP]Switching back to kReadyToGo.\n");
        navi_status_ = kReadyToGo;
      }
      break;
    case kFinished:
      printf("[RP]Finish trip.\n");
      if (if_restart_) {
        printf("[RP]Restart mission.\n");
        navi_status_ = kReadyToGo;
      }
      break;
    default:
      assert(false);
  }
  return kSuccess;
}

ErrorType RoutePlanner::NaviLoopAssignedTarget() { return kSuccess; }

ErrorType RoutePlanner::GetNaviPathByRandomExpansion() {
  navi_path_.clear();
  navi_path_.push_back(nearest_lane_id_);
  decimal_t dist_acc = 0;
  int cur_id = nearest_lane_id_;
  while (dist_acc < navi_path_max_length_) {
    std::vector<int> child_ids;
    GetChildLaneIds(cur_id, &child_ids);
    int n_child = static_cast<int>(child_ids.size());
    // ~ we should break here, otherwise there may be divided by zero
    if (n_child == 0) break;
    int rand_num = std::floor(rd_gen_() / (rd_gen_.max() / n_child));
    int rand_id = child_ids[rand_num];
    dist_acc += lane_net_.lane_set.at(rand_id).length;
    navi_path_.push_back(rand_id);
    cur_id = rand_id;
  }

  vec_Vecf<2> raw_samples;
  for (const auto &id : navi_path_) {
    if (raw_samples.empty() &&
        (int)lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1; i < (int)lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &navi_lane_) !=
      kSuccess) {
    printf("[RP]Err - fail to fitting lane with %d samples.\n",
           static_cast<int>(raw_samples.size()));
    return kWrongStatus;
  }

  Vec2f pos(ego_state_.vec_position(0), ego_state_.vec_position(1));
  if (navi_lane_.GetArcLengthByVecPosition(pos, &navi_start_arc_length_) !=
      kSuccess) {
    printf("[RP]Err - fail to locate ego state on navi lane.\n");
    return kWrongStatus;
  }
  navi_lane_.GetArcLengthByVecPosition(pos, &navi_cur_arc_len_);

  navi_path_length_ = navi_lane_.end() - navi_start_arc_length_;
  if (navi_path_length_ < 0.0) {
    printf("[PubgLane]Err - fail to get legal navi path length.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType RoutePlanner::CheckNaviProgress() {
  if (!navi_lane_.IsValid()) return kWrongStatus;
  if (navi_path_length_ < 0.0) return kWrongStatus;

  // decimal_t ratio =
  //     (navi_cur_arc_len_ - navi_start_arc_length_) / navi_path_length_;
  // std::cout << "[PubgLane] Navi progress ratio: " << ratio << std::endl;

  // if (ratio >= 0.0) {
  navi_status_ = kFinished;
  // }
  return kSuccess;
}

bool RoutePlanner::CheckIfArriveTargetLane() {
  std::cout << "[rp] Nearest = " << nearest_lane_id_ << std::endl;
  if (nearest_lane_id_ == *navi_path_.rbegin()) {
    return true;
  } else {
    return false;
  }
}

ErrorType RoutePlanner::GetChildLaneIds(const int lane_id,
                                        std::vector<int> *child_ids) {
  auto it = lane_net_.lane_set.find(lane_id);
  if (it == lane_net_.lane_set.end()) {
    return kWrongStatus;
  } else {
    // ~ note this is an assign
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

}  // namespace planning
