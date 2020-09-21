/**
 * @file map_adapter.h
 * @author HKUST Aerial Robotics Group
 * @brief map adapter (inherits map interface) for ssc planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_ADAPTER_H__

#include "common/basics/semantics.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "ssc_planner/map_interface.h"

namespace planning {

class SscPlannerAdapter : public SscPlannerMapItf {
 public:
  using IntegratedMap = semantic_map_manager::SemanticMapManager;
  bool IsValid() override;
  decimal_t GetTimeStamp() override;
  ErrorType GetEgoVehicle(Vehicle* vehicle) override;
  ErrorType GetEgoState(State* state) override;
  ErrorType GetEgoReferenceLane(Lane* lane) override;
  ErrorType GetLocalReferenceLane(Lane* lane) override;
  ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) override;
  ErrorType GetObstacleMap(GridMap2D* grid_map) override;
  ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                             const State& state, bool* res) override;
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) override;
  ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) override;
  ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>* sur_trajs)
      override;
  ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) override;
  ErrorType set_map(std::shared_ptr<IntegratedMap> map);

 private:
  std::shared_ptr<IntegratedMap> map_;
  bool is_valid_ = false;
};

}  // namespace planning

#endif