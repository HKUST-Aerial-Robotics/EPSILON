/**
 * @file ssc_planner.h
 * @author HKUST Aerial Robotics Group
 * @brief planner using spatio-temporal corridor
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_
#define _UTIL_SSC_PLANNER_INC_SSC_SEARCH_H_

#include <memory>
#include <set>
#include <string>
#include <thread>

#include "common/basics/basics.h"
#include "common/interface/planner.h"
#include "common/lane/lane.h"
#include "common/primitive/frenet_primitive.h"
#include "common/spline/spline_generator.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_bezier_traj.h"
#include "common/trajectory/frenet_primitive_traj.h"
#include "ssc_config.pb.h"
#include "ssc_planner/map_interface.h"
#include "ssc_planner/ssc_map.h"

namespace planning {

class SscPlanner : public Planner {
 public:
  using ObstacleMapType = uint8_t;
  using SscMapDataType = uint8_t;

  using Lane = common::Lane;
  using State = common::State;
  using Vehicle = common::Vehicle;
  using LateralBehavior = common::LateralBehavior;
  using FrenetState = common::FrenetState;
  using FrenetTrajectory = common::FrenetTrajectory;
  using FrenetPrimitive = common::FrenetPrimitive;
  using FrenetBezierTrajectory = common::FrenetBezierTrajectory;
  using FrenetPrimitiveTrajectory = common::FrenetPrimitiveTrajectory;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  typedef common::BezierSpline<5, 2> BezierSpline;

  SscPlanner() = default;

  /**
   * @brief setters
   */
  ErrorType set_map_interface(SscPlannerMapItf* map_itf);

  ErrorType set_initial_state(const State& state);
  /**
   * @brief getters
   */
  SscMap* p_ssc_map() const { return p_ssc_map_; }

  FrenetState ego_frenet_state() const { return ego_frenet_state_; }

  Vehicle ego_vehicle() const { return ego_vehicle_; }

  vec_E<vec_E<Vehicle>> forward_trajs() const { return forward_trajs_; }

  vec_E<BezierSpline> qp_trajs() const { return qp_trajs_; }

  decimal_t time_origin() const { return time_origin_; }

  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs()
      const {
    return sur_vehicle_trajs_fs_;
  }

  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
  surround_forward_trajs_fs() const {
    return surround_forward_trajs_fs_;
  };

  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs() const {
    return forward_trajs_fs_;
  }

  vec_E<Vec2f> ego_vehicle_contour_fs() const {
    return fs_ego_vehicle_.vertices;
  }

  common::FsVehicle fs_ego_vehicle() const { return fs_ego_vehicle_; }

  // BezierSpline bezier_spline() const { return bezier_spline_; }

  std::unique_ptr<FrenetTrajectory> trajectory() const {
    if (!is_lateral_independent_) {
      return std::unique_ptr<FrenetPrimitiveTrajectory>(
          new FrenetPrimitiveTrajectory(low_spd_alternative_traj_));
    }
    return std::unique_ptr<FrenetBezierTrajectory>(
        new FrenetBezierTrajectory(trajectory_));
  }

  common::StateTransformer state_transformer() const { return stf_; }

  decimal_t time_cost() const { return time_cost_; }

  common::FrenetState initial_frenet_state() const {
    return initial_frenet_state_;
  }

  /**
   * @brief Initialize the planner with config path
   */
  std::string Name() override;

  /**
   * @brief Initialize the planner with config path
   */
  ErrorType Init(const std::string config_path) override;

  /**
   * @brief Run one planning round with given states
   */
  ErrorType RunOnce() override;

 private:
  ErrorType ReadConfig(const std::string config_path);

  ErrorType CorridorFeasibilityCheck(
      const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes);
  /**
   * @brief transform all the states in a batch
   */
  ErrorType StateTransformForInputData();

  ErrorType RunQpOptimization();

  ErrorType ValidateTrajectory(const FrenetTrajectory& traj);
  /**
   * @brief transform all the states using openmp
   */
  ErrorType StateTransformUsingOpenMp(const vec_E<State>& global_state_vec,
                                      const vec_E<Vec2f>& global_point_vec,
                                      vec_E<FrenetState>* frenet_state_vec,
                                      vec_E<Vec2f>* fs_point_vec) const;

  ErrorType StateTransformSingleThread(const vec_E<State>& global_state_vec,
                                       const vec_E<Vec2f>& global_point_vec,
                                       vec_E<FrenetState>* frenet_state_vec,
                                       vec_E<Vec2f>* fs_point_vec) const;

  ErrorType UpdateTrajectoryWithCurrentBehavior();

  Vehicle ego_vehicle_;
  LateralBehavior ego_behavior_;
  FrenetState ego_frenet_state_;
  Lane nav_lane_local_;
  decimal_t time_origin_{0.0};

  State initial_state_;
  bool has_initial_state_ = false;

  common::FrenetState initial_frenet_state_;

  GridMap2D grid_map_;
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  vec_E<vec_E<Vehicle>> forward_trajs_;
  std::vector<LateralBehavior> forward_behaviors_;
  vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_forward_trajs_;

  vec_E<Vec2f> obstacle_grids_fs_;

  // Initial solution for optimization
  common::FsVehicle fs_ego_vehicle_;
  vec_E<vec_E<common::FsVehicle>> forward_trajs_fs_;
  std::unordered_map<int, vec_E<common::FsVehicle>> sur_vehicle_trajs_fs_;
  vec_E<std::unordered_map<int, vec_E<common::FsVehicle>>>
      surround_forward_trajs_fs_;

  vec_E<BezierSpline> qp_trajs_;
  vec_E<FrenetPrimitive> primitive_trajs_;
  std::vector<LateralBehavior> valid_behaviors_;
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> corridors_;
  vec_E<vec_E<common::FrenetState>> ref_states_list_;

  bool is_lateral_independent_ = true;
  FrenetBezierTrajectory trajectory_;
  FrenetPrimitiveTrajectory low_spd_alternative_traj_;
  vec_E<common::SpatioTemporalSemanticCubeNd<2>> final_corridor_;
  vec_E<common::FrenetState> final_ref_states_;

  common::StateTransformer stf_;
  // Map
  SscPlannerMapItf* map_itf_;
  bool map_valid_ = false;
  SscMap* p_ssc_map_;

  decimal_t stamp_ = 0.0;
  decimal_t time_cost_ = 0.0;

  planning::ssc::Config cfg_;
};

}  // namespace planning

#endif