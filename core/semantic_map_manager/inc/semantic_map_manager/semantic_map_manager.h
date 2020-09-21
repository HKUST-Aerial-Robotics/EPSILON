#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <list>
#include <memory>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/lane/lane.h"
#include "common/lane/lane_generator.h"
#include "common/math/calculations.h"
#include "common/mobil/mobil_behavior_prediction.h"
#include "common/rss/rss_checker.h"
#include "motion_predictor/onlane_fs_predictor.h"
#include "semantic_map_manager/config_loader.h"
#include "semantic_map_manager/traffic_signal_manager.h"
#include "vehicle_model/idm_model.h"

namespace semantic_map_manager {

class SemanticMapManager {
 public:
  using ObstacleMapType = uint8_t;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;
  using State = common::State;
  using Lane = common::Lane;
  using LateralBehavior = common::LateralBehavior;
  using SemanticLane = common::SemanticLane;

  SemanticMapManager() {}
  SemanticMapManager(const int &id, const std::string &agent_config_path);
  SemanticMapManager(const int &id, const decimal_t surrounding_search_radius,
                     bool enable_openloop_prediction, bool use_right_hand_axis);
  ~SemanticMapManager() {}

  ErrorType CheckCollisionUsingGlobalPosition(const Vec2f &p_w,
                                              bool *res) const;

  ErrorType GetObstacleMapValueUsingGlobalPosition(const Vec2f &p_w,
                                                   ObstacleMapType *res);

  ErrorType CheckCollisionUsingStateAndVehicleParam(
      const common::VehicleParam &vehicle_param, const common::State &state,
      bool *res);

  ErrorType CheckCollisionUsingState(const common::VehicleParam &param_a,
                                     const common::State &state_a,
                                     const common::VehicleParam &param_b,
                                     const common::State &state_b, bool *res);

  ErrorType CheckCollisionUsingStateVec(
      const vec_E<common::State> state_vec) const;

  ErrorType GetDistanceToLanesUsing3DofState(
      const Vec3f &state,
      std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const;

  ErrorType UpdateSemanticMap(
      const double &time_stamp, const common::Vehicle &ego_vehicle,
      const common::LaneNet &whole_lane_net,
      const common::LaneNet &surrounding_lane_net,
      const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
      const std::set<std::array<decimal_t, 2>> &obstacle_grids,
      const common::VehicleSet &surrounding_vehicles);

  ErrorType GetNearestLaneIdUsingState(const Vec3f &state,
                                       const std::vector<int> &navi_path,
                                       int *id, decimal_t *distance,
                                       decimal_t *arc_len) const;

  ErrorType NaiveRuleBasedLateralBehaviorPrediction(
      const common::Vehicle &vehicle, const int nearest_lane_id,
      common::ProbDistOfLatBehaviors *lat_probs);

  ErrorType MobilRuleBasedBehaviorPrediction(
      const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
      common::ProbDistOfLatBehaviors *res);

  ErrorType TrajectoryPredictionForVehicle(const common::Vehicle &vehicle,
                                           const common::Lane &lane,
                                           const decimal_t &t_pred,
                                           const decimal_t &t_step,
                                           vec_E<common::State> *traj);

  ErrorType IsTopologicallyReachable(const int lane_id,
                                     const std::vector<int> &path,
                                     int *num_lane_changes, bool *res) const;

  ErrorType GetRefLaneForStateByBehavior(const common::State &state,
                                         const std::vector<int> &navi_path,
                                         const LateralBehavior &behavior,
                                         const decimal_t &max_forward_len,
                                         const decimal_t &max_back_len,
                                         const bool is_high_quality,
                                         common::Lane *lane) const;
  ErrorType GetTargetLaneId(const int lane_id, const LateralBehavior &behavior,
                            int *target_lane_id) const;

  ErrorType GetLocalLaneSamplesByState(const common::State &state,
                                       const int lane_id,
                                       const std::vector<int> &navi_path,
                                       const decimal_t max_reflane_dist,
                                       const decimal_t max_backward_dist,
                                       vec_Vecf<2> *samples) const;
  ErrorType GetLeadingVehicleOnLane(const common::Lane &ref_lane,
                                    const common::State &ref_state,
                                    const common::VehicleSet &vehicle_set,
                                    const decimal_t &lat_range,
                                    common::Vehicle *leading_vehicle,
                                    decimal_t *distance_residual_ratio) const;
  ErrorType GetFollowingVehicleOnLane(const common::Lane &ref_lane,
                                      const common::State &ref_state,
                                      const common::VehicleSet &vehicle_set,
                                      const decimal_t &lat_range,
                                      common::Vehicle *leading_vehicle) const;

  ErrorType GetLeadingAndFollowingVehiclesFrenetStateOnLane(
      const common::Lane &ref_lane, const common::State &ref_state,
      const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
      common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
      bool *has_following_vehicle, common::Vehicle *following_vehicle,
      common::FrenetState *following_fs) const;

  ErrorType SaveMapToLog();

  bool IsLocalLaneContainsLane(const int &local_lane_id,
                               const int &seg_lane_id) const;

  ErrorType GetSpeedLimit(const State &state, const Lane &lane,
                          decimal_t *speed_limit) const;

  ErrorType GetTrafficStoppingState(const State &state, const Lane &lane,
                                    State *stopping_state) const;

  ErrorType GetEgoNearestLaneId(int *ego_lane_id) const;

  inline double time_stamp() const { return time_stamp_; }

  inline int ego_id() const { return ego_id_; }

  inline common::Vehicle ego_vehicle() const { return ego_vehicle_; }

  inline common::GridMapND<ObstacleMapType, 2> obstacle_map() const {
    return obstacle_map_;
  }
  inline common::GridMapND<ObstacleMapType, 2> *obstacle_map_ptr() {
    return &obstacle_map_;
  }
  inline std::set<std::array<decimal_t, 2>> obstacle_grids() const {
    return obstacle_grids_;
  }
  inline common::VehicleSet surrounding_vehicles() const {
    return surrounding_vehicles_;
  }
  inline common::VehicleSet key_vehicles() const { return key_vehicles_; }
  inline common::LaneNet whole_lane_net() const { return whole_lane_net_; }
  inline common::LaneNet surrounding_lane_net() const {
    return surrounding_lane_net_;
  }
  inline common::SemanticLaneSet semantic_lane_set() const {
    return semantic_lane_set_;
  }
  inline const common::SemanticLaneSet *semantic_lane_set_cptr() const {
    const common::SemanticLaneSet *ptr = &semantic_lane_set_;
    return ptr;
  }
  inline common::SemanticBehavior ego_behavior() const { return ego_behavior_; }
  inline common::SemanticVehicleSet semantic_surrounding_vehicles() const {
    return semantic_surrounding_vehicles_;
  }
  inline common::SemanticVehicleSet semantic_key_vehicles() const {
    return semantic_key_vehicles_;
  }
  inline AgentConfigInfo agent_config_info() const {
    return agent_config_info_;
  }
  inline std::vector<int> key_vehicle_ids() const { return key_vehicle_ids_; }

  inline std::vector<int> uncertain_vehicle_ids() const {
    return uncertain_vehicle_ids_;
  }

  inline std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs()
      const {
    return openloop_pred_trajs_;
  }

  inline vec_E<common::SpeedLimit> RetTrafficInfoSpeedLimit() const {
    return traffic_singal_manager_.speed_limit_list();
  }

  inline vec_E<common::TrafficLight> RetTrafficInfoTrafficLight() const {
    return traffic_singal_manager_.traffic_light_list();
  }

  inline std::unordered_map<int, common::Lane> local_lanes() const {
    return local_lanes_;
  }

  inline void set_ego_id(const int &in) { ego_id_ = in; }
  inline void set_obstacle_map(
      const common::GridMapND<ObstacleMapType, 2> &in) {
    obstacle_map_ = in;
  }
  inline void set_obstacle_grids(const std::set<std::array<decimal_t, 2>> &in) {
    obstacle_grids_ = in;
  }
  inline void set_ego_vehicle(const common::Vehicle &in) { ego_vehicle_ = in; }
  inline void set_surrounding_vehicles(const common::VehicleSet &in) {
    surrounding_vehicles_ = in;
  }
  inline void set_whole_lane_net(const common::LaneNet &in) {
    whole_lane_net_ = in;
  }
  inline void set_surrounding_lane_net(const common::LaneNet &in) {
    surrounding_lane_net_ = in;
  }
  inline void set_semantic_lane_set(const common::SemanticLaneSet &in) {
    semantic_lane_set_ = in;
  }
  inline void set_ego_behavior(const common::SemanticBehavior &in) {
    ego_behavior_ = in;
  }
  inline void set_uncertain_vehicle_ids(
      const std::vector<int> &uncertain_vehicle_ids) {
    uncertain_vehicle_ids_ = uncertain_vehicle_ids;
  }

 private:
  ErrorType UpdateSemanticLaneSet();

  ErrorType UpdateLocalLanesAndFastLut();

  ErrorType UpdateSemanticVehicles();

  ErrorType UpdateKeyVehicles();

  ErrorType OpenloopTrajectoryPrediction();

  ErrorType GetDistanceOnLaneNet(const int &lane_id_0,
                                 const decimal_t &arc_len_0,
                                 const int &lane_id_1,
                                 const decimal_t &arc_len_1,
                                 decimal_t *dist) const;

  ErrorType SampleLane(const common::Lane &lane, const decimal_t &s0,
                       const decimal_t &s1, const decimal_t &step,
                       vec_E<Vecf<2>> *samples, decimal_t *accum_dist) const;

  void GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
      const decimal_t &node_id, const decimal_t &node_length,
      const decimal_t &aggre_length, const std::vector<int> &path_to_node,
      std::vector<std::vector<int>> *all_paths);

  void GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
      const decimal_t &node_id, const decimal_t &node_length,
      const decimal_t &aggre_length, const std::vector<int> &path_to_node,
      std::vector<std::vector<int>> *all_paths);

  ErrorType GetLocalLaneUsingLaneIds(const common::State &state,
                                     const std::vector<int> &lane_ids,
                                     const decimal_t max_reflane_dist,
                                     const decimal_t max_backward_dist,
                                     const bool &is_high_quality,
                                     common::Lane *lane);

  ErrorType GetLaneBySampledPoints(const vec_Vecf<2> &samples,
                                   const bool &is_high_quality,
                                   common::Lane *lane) const;

  double time_stamp_{0.0};

  decimal_t pred_time_ = 5.0;
  decimal_t pred_step_ = 0.2;

  decimal_t nearest_lane_range_ = 1.5;
  decimal_t lane_range_ = 10.0;

  decimal_t max_distance_to_lane_ = 2.0;

  bool has_fast_lut_ = false;
  std::unordered_map<int, common::Lane> local_lanes_;
  std::unordered_map<int, std::vector<int>> local_to_segment_lut_;
  std::unordered_map<int, std::set<int>> segment_to_local_lut_;

  decimal_t local_lane_length_forward_ = 250.0;
  decimal_t local_lane_length_backward_ = 150.0;

  int ego_id_;
  std::string agent_config_path_;
  AgentConfigInfo agent_config_info_;
  bool use_right_hand_axis_ = true;

  common::Vehicle ego_vehicle_;
  GridMap2D obstacle_map_;
  std::set<std::array<decimal_t, 2>> obstacle_grids_;
  // * surrounding vehicles is constructed by radius
  common::VehicleSet surrounding_vehicles_;
  // * semantic version of surrounding vehicles
  common::SemanticVehicleSet semantic_surrounding_vehicles_;
  // * key vehicles on lane based on selection strategy
  // * key vehicles is a subset of surrounding vehicle
  common::VehicleSet key_vehicles_;
  // * key vehicles with semantics
  common::SemanticVehicleSet semantic_key_vehicles_;
  std::vector<int> key_vehicle_ids_;
  std::vector<int> uncertain_vehicle_ids_;

  common::LaneNet whole_lane_net_;
  common::LaneNet surrounding_lane_net_;
  common::SemanticLaneSet semantic_lane_set_;
  common::SemanticBehavior ego_behavior_;

  // * open loop prediction only for collision checking for onlane mp
  std::unordered_map<int, vec_E<common::State>> openloop_pred_trajs_;

  TicToc global_timer_;
  TrafficSignalManager traffic_singal_manager_;
  ConfigLoader *p_config_loader_;

  common::RssChecker rss_checker_;

  // * For highway-like lane structure only
  bool is_simple_lane_structure_ = false;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_SEMANTIC_MAP_H_