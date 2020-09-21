#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_MAP_INTERFACE_H_

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {
class BehaviorPlannerMapItf {
 public:
  using State = common::State;
  using Lane = common::Lane;
  using Behavior = common::SemanticBehavior;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;

  virtual bool IsValid() = 0;
  virtual ErrorType GetEgoState(State *state) = 0;
  virtual ErrorType GetEgoId(int *id) = 0;
  virtual ErrorType GetEgoVehicle(common::Vehicle *vehicle) = 0;
  virtual ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                           int *lane_id) = 0;
  virtual ErrorType GetNearestLaneIdUsingState(
      const Vec3f &state, const std::vector<int> &navi_path, int *id,
      decimal_t *distance, decimal_t *arc_len) = 0;
  virtual ErrorType IsTopologicallyReachable(const int lane_id,
                                             const std::vector<int> &path,
                                             int *num_lane_changes,
                                             bool *res) = 0;
  virtual ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) = 0;
  virtual ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) = 0;
  virtual ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) = 0;
  virtual ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) = 0;
  virtual ErrorType GetChildLaneIds(const int lane_id,
                                    std::vector<int> *child_ids) = 0;
  virtual ErrorType GetFatherLaneIds(const int lane_id,
                                     std::vector<int> *father_ids) = 0;
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) = 0;
  virtual ErrorType GetLocalLaneSamplesByState(
      const State &state, const int lane_id, const std::vector<int> &navi_path,
      const decimal_t max_reflane_dist, const decimal_t max_backward_dist,
      vec_Vecf<2> *samples) = 0;
  virtual ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality,
      Lane *lane) = 0;
  virtual ErrorType GetWholeLaneNet(common::LaneNet *lane_net) = 0;
  virtual ErrorType CheckCollisionUsingState(
      const common::VehicleParam &param_a, const common::State &state_a,
      const common::VehicleParam &param_b, const common::State &state_b,
      bool *res) = 0;
  virtual ErrorType CheckIfCollision(const common::VehicleParam &vehicle_param,
                                     const State &state, bool *res) = 0;
  virtual ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle, decimal_t *distance_residual_ratio) = 0;
  virtual ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                                  decimal_t *speed_limit) = 0;
  virtual ErrorType GetPredictedBehavior(
      const int vehicle_id, common::LateralBehavior *lat_behavior) = 0;
};

}  // namespace planning

#endif