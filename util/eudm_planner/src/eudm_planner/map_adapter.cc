#include "eudm_planner/map_adapter.h"

namespace planning {

bool EudmPlannerMapAdapter::IsValid() { return is_valid_; }

ErrorType EudmPlannerMapAdapter::GetEgoState(State *state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetEgoId(int *id) {
  if (!is_valid_) return kWrongStatus;
  *id = map_->ego_id();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetEgoVehicle(common::Vehicle *vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetEgoLaneIdByPosition(
    const std::vector<int> &navi_path, int *lane_id) {
  if (!is_valid_) {
    printf("[GetEgoLaneIdByPosition]Interface not valid.\n");
    return kWrongStatus;
  }

  int ego_lane_id = kInvalidLaneId;
  decimal_t distance_to_lane;
  decimal_t arc_len;

  Vec3f state_3dof(map_->ego_vehicle().state().vec_position(0),
                   map_->ego_vehicle().state().vec_position(1),
                   map_->ego_vehicle().state().angle);

  if (map_->GetNearestLaneIdUsingState(state_3dof, navi_path, &ego_lane_id,
                                       &distance_to_lane,
                                       &arc_len) != kSuccess) {
    printf("[GetEgoLaneIdByPosition]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  *lane_id = ego_lane_id;
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  std::set<std::tuple<decimal_t, decimal_t, int>> dist_set;
  if (map_->GetNearestLaneIdUsingState(state, navi_path, id, distance,
                                       arc_len) != kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) {
  if (!is_valid_) {
    printf("[GetNearestLaneIdUsingState]Interface not valid.\n");
    return kWrongStatus;
  }
  if (map_->IsTopologicallyReachable(lane_id, path, num_lane_changes, res) !=
      kSuccess) {
    printf("[GetNearestLaneIdUsingState]Cannot get nearest lane.\n");
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetRightLaneId(const int lane_id,
                                                int *r_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.r_change_avbl) {
      *r_lane_id = it->second.r_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetLeftLaneId(const int lane_id,
                                               int *l_lane_id) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (it->second.l_change_avbl) {
      *l_lane_id = it->second.l_lane_id;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetLaneByLaneId(const int lane_id,
                                                 Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
    if (!lane->IsValid()) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetChildLaneIds(const int lane_id,
                                                 std::vector<int> *child_ids) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    // ~ note this is an assign
    child_ids->assign(it->second.child_id.begin(), it->second.child_id.end());
  }
  return kSuccess;
}

bool EudmPlannerMapAdapter::IsLaneConsistent(const int lane_id_old,
                                             const int lane_id_new) {
  if (lane_id_new == lane_id_old) return true;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  const int max_nodes_expanded = 20;
  int expanded_nodes = 1;
  std::set<int> visited_set;
  std::list<int> queue;
  visited_set.insert(lane_id_old);
  queue.push_back(lane_id_old);
  //* BFS
  int cur_id;
  while (!queue.empty() && expanded_nodes < max_nodes_expanded) {
    cur_id = queue.front();
    queue.pop_front();
    expanded_nodes++;

    std::vector<int> child_ids;
    auto it = p_semantic_lane_set->semantic_lanes.find(cur_id);
    if (it == p_semantic_lane_set->semantic_lanes.end()) {
      continue;
    } else {
      child_ids = it->second.child_id;
    }
    if (child_ids.empty()) continue;
    for (auto &id : child_ids) {
      if (visited_set.count(id) == 0) {
        visited_set.insert(id);
        queue.push_back(id);
      }
    }
  }

  if (visited_set.find(lane_id_new) != visited_set.end()) return true;
  return false;
}

ErrorType EudmPlannerMapAdapter::GetFatherLaneIds(
    const int lane_id, std::vector<int> *father_ids) {
  if (!is_valid_) return kWrongStatus;
  auto p_semantic_lane_set = map_->semantic_lane_set_cptr();
  auto it = p_semantic_lane_set->semantic_lanes.find(lane_id);
  if (it == p_semantic_lane_set->semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    father_ids->assign(it->second.father_id.begin(),
                       it->second.father_id.end());
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetLocalLaneSamplesByState(
    const State &state, const int lane_id, const std::vector<int> &navi_path,
    const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
    vec_Vecf<2> *samples) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLocalLaneSamplesByState(state, lane_id, navi_path,
                                       max_reflane_dist, max_backward_dist,
                                       samples) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetRefLaneForStateByBehavior(
    const State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality, Lane *lane) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetRefLaneForStateByBehavior(state, navi_path, behavior,
                                         max_forward_len, max_back_len,
                                         is_high_quality, lane) != kSuccess) {
    return kWrongStatus;
  }
  if (!lane->IsValid()) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                                    leading_vehicle,
                                    distance_residual_ratio) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType
EudmPlannerMapAdapter::GetLeadingAndFollowingVehiclesFrenetStateOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
    common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
    bool *has_following_vehicle, common::Vehicle *following_vehicle,
    common::FrenetState *following_fs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
          ref_lane, ref_state, vehicle_set, has_leading_vehicle,
          leading_vehicle, leading_fs, has_following_vehicle, following_vehicle,
          following_fs) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetSurroundingVehicles(
    common::VehicleSet *surrounding_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *surrounding_vehicle_set = map_->surrounding_vehicles();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetKeyVehicles(
    common::VehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *key_vehicle_set = map_->key_vehicles();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetKeySemanticVehicles(
    common::SemanticVehicleSet *key_vehicle_set) {
  if (!is_valid_) return kWrongStatus;
  *key_vehicle_set = map_->semantic_key_vehicles();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::GetWholeLaneNet(common::LaneNet *lane_net) {
  if (!is_valid_) return kWrongStatus;
  *lane_net = map_->whole_lane_net();
  return kSuccess;
}

ErrorType EudmPlannerMapAdapter::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
  if (!is_valid_) return kWrongStatus;
  if (map_->CheckCollisionUsingState(param_a, state_a, param_b, state_b, res) !=
      kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

void EudmPlannerMapAdapter::set_map(std::shared_ptr<IntegratedMap> map_ptr) {
  map_ = map_ptr;
  is_valid_ = true;
}

}  // namespace planning