#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_MAP_ADAPTER_H_

#include <iostream>
#include <set>

#include "common/basics/semantics.h"
#include "eudm_planner/map_interface.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

class EudmPlannerMapAdapter : public EudmPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  bool IsValid() override;
  ErrorType GetEgoState(State *state) override;
  ErrorType GetEgoId(int *id) override;
  ErrorType GetEgoVehicle(common::Vehicle *vehicle) override;
  ErrorType GetEgoLaneIdByPosition(const std::vector<int> &navi_path,
                                   int *lane_id) override;
  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) override;
  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) override;
  bool IsLaneConsistent(const int lane_id_old, const int lane_id_new) override;
  ErrorType GetRightLaneId(const int lane_id, int *r_lane_id) override;
  ErrorType GetLeftLaneId(const int lane_id, int *l_lane_id) override;
  ErrorType GetChildLaneIds(const int lane_id,
                            std::vector<int> *child_ids) override;
  ErrorType GetFatherLaneIds(const int lane_id,
                             std::vector<int> *father_ids) override;
  ErrorType GetLaneByLaneId(const int lane_id, Lane *lane) override;
  ErrorType GetLocalLaneSamplesByState(const State &state, const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) override;
  ErrorType GetRefLaneForStateByBehavior(
      const State &state, const std::vector<int> &navi_path,
      const LateralBehavior &behavior, const decimal_t &max_forward_len,
      const decimal_t &max_back_len, const bool is_high_quality, Lane *lane);
  ErrorType GetKeyVehicles(common::VehicleSet *key_vehicle_set) override;
  ErrorType GetSurroundingVehicles(
      common::VehicleSet *key_vehicle_set) override;
  ErrorType GetKeySemanticVehicles(
      common::SemanticVehicleSet *key_vehicle_set) override;
  ErrorType GetWholeLaneNet(common::LaneNet *lane_net) override;
  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b,
                                     bool *res) override;
  ErrorType GetLeadingVehicleOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
      common::Vehicle *leading_vehicle,
      decimal_t *distance_residual_ratio) override;
  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) override;

  void set_map(std::shared_ptr<IntegratedMap> map_ptr);

  std::shared_ptr<IntegratedMap> map() { return map_; }

 private:
  std::shared_ptr<IntegratedMap> map_;
  bool is_valid_ = false;
};

}  // namespace planning

#endif