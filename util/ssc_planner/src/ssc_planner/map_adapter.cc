/**
 * @file map_adpater.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for map adapter
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include "ssc_planner/map_adapter.h"

namespace planning {

ErrorType SscPlannerAdapter::set_map(std::shared_ptr<IntegratedMap> map) {
  map_ = map;  // maintain a snapshop of the environment
  is_valid_ = true;
  return kSuccess;
}

bool SscPlannerAdapter::IsValid() { return is_valid_; }

decimal_t SscPlannerAdapter::GetTimeStamp() { return map_->time_stamp(); }

ErrorType SscPlannerAdapter::GetEgoVehicle(Vehicle* vehicle) {
  if (!is_valid_) return kWrongStatus;
  *vehicle = map_->ego_vehicle();
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetEgoState(State* state) {
  if (!is_valid_) return kWrongStatus;
  *state = map_->ego_vehicle().state();
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetLocalReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetForwardTrajectories(
    std::vector<LateralBehavior>* behaviors,
    vec_E<vec_E<common::Vehicle>>* trajs,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().forward_behaviors.size() < 1) return kWrongStatus;
  *behaviors = map_->ego_behavior().forward_behaviors;
  *trajs = map_->ego_behavior().forward_trajs;
  *sur_trajs = map_->ego_behavior().surround_trajs;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetEgoDiscretBehavior(
    LateralBehavior* lat_behavior) {
  if (!is_valid_) return kWrongStatus;
  if (map_->ego_behavior().lat_behavior == common::LateralBehavior::kUndefined)
    return kWrongStatus;
  *lat_behavior = map_->ego_behavior().lat_behavior;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetLaneByLaneId(const int lane_id, Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto semantic_lane_set = map_->semantic_lane_set();
  auto it = semantic_lane_set.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    *lane = it->second.lane;
  }
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetEgoReferenceLane(Lane* lane) {
  if (!is_valid_) return kWrongStatus;
  auto ref_lane = map_->ego_behavior().ref_lane;
  if (!ref_lane.IsValid()) {
    printf("[GetEgoReferenceLane]No reference lane existing.\n");
    return kWrongStatus;
  }
  *lane = ref_lane;
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetObstacleMap(GridMap2D* grid_map) {
  if (!is_valid_) return kWrongStatus;
  *grid_map = map_->obstacle_map();
  return kSuccess;
}

ErrorType SscPlannerAdapter::CheckIfCollision(
    const common::VehicleParam& vehicle_param, const State& state, bool* res) {
  if (!is_valid_) return kWrongStatus;
  map_->CheckCollisionUsingStateAndVehicleParam(vehicle_param, state, res);
  return kSuccess;
}

ErrorType SscPlannerAdapter::GetObstacleGrids(
    std::set<std::array<decimal_t, 2>>* obs_grids) {
  if (!is_valid_) return kWrongStatus;
  *obs_grids = map_->obstacle_grids();
  return kSuccess;
}

}  // namespace planning