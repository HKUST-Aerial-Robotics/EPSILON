/**
 * @file map_interface.h
 * @author HKUST Aerial Robotics Group
 * @brief map interface for ssc planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__

#include <array>
#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {
class SscPlannerMapItf {
 public:
  using ObstacleMapType = uint8_t;
  using State = common::State;
  using Lane = common::Lane;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using Behavior = common::SemanticBehavior;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  virtual bool IsValid() = 0;
  virtual decimal_t GetTimeStamp() = 0;
  virtual ErrorType GetEgoVehicle(Vehicle* vehicle) = 0;
  virtual ErrorType GetEgoState(State* state) = 0;
  virtual ErrorType GetEgoReferenceLane(Lane* lane) = 0;
  virtual ErrorType GetLocalReferenceLane(Lane* lane) = 0;
  virtual ErrorType GetLaneByLaneId(const int lane_id, Lane* lane) = 0;
  virtual ErrorType GetObstacleMap(GridMap2D* grid_map) = 0;
  virtual ErrorType CheckIfCollision(const common::VehicleParam& vehicle_param,
                                     const State& state, bool* res) = 0;
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs) = 0;
  virtual ErrorType GetForwardTrajectories(
      std::vector<LateralBehavior>* behaviors,
      vec_E<vec_E<common::Vehicle>>* trajs,
      vec_E<std::unordered_map<int, vec_E<Vehicle>>>* sur_trajs) = 0;
  virtual ErrorType GetEgoDiscretBehavior(LateralBehavior* lat_behavior) = 0;
  virtual ErrorType GetObstacleGrids(
      std::set<std::array<decimal_t, 2>>* obs_grids) = 0;
};

}  // namespace planning

#endif  // _UTIL_SSC_PLANNER_INC_SSC_PLANNER_MAP_INTERFACE_H__