#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

SemanticMapManager::SemanticMapManager(const int &id,
                                       const std::string &agent_config_path)
    : ego_id_(id), agent_config_path_(agent_config_path) {
  p_config_loader_ = new ConfigLoader();
  p_config_loader_->set_ego_id(ego_id_);
  p_config_loader_->set_agent_config_path(agent_config_path_);
  p_config_loader_->ParseAgentConfig(&agent_config_info_);
  global_timer_.tic();
}

SemanticMapManager::SemanticMapManager(
    const int &id, const decimal_t surrounding_search_radius,
    bool enable_openloop_prediction, bool use_right_hand_axis) {
  // default constructor
  ego_id_ = id;
  agent_config_info_.surrounding_search_radius = surrounding_search_radius;
  agent_config_info_.enable_openloop_prediction = enable_openloop_prediction;
  agent_config_info_.enable_tracking_noise = false;
  agent_config_info_.enable_log = false;
  agent_config_info_.enable_fast_lane_lut = true;
  use_right_hand_axis_ = use_right_hand_axis;
  is_simple_lane_structure_ = true;
}

ErrorType SemanticMapManager::UpdateSemanticMap(
    const double &time_stamp, const common::Vehicle &ego_vehicle,
    const common::LaneNet &whole_lane_net,
    const common::LaneNet &surrounding_lane_net,
    const common::GridMapND<ObstacleMapType, 2> &obstacle_map,
    const std::set<std::array<decimal_t, 2>> &obstacle_grids,
    const common::VehicleSet &surrounding_vehicles) {
  TicToc timer;
  time_stamp_ = time_stamp;
  set_ego_vehicle(ego_vehicle);
  set_whole_lane_net(whole_lane_net);
  set_surrounding_lane_net(surrounding_lane_net);
  set_obstacle_map(obstacle_map);
  set_obstacle_grids(obstacle_grids);
  set_surrounding_vehicles(surrounding_vehicles);

  // * update lanes and topologies
  UpdateSemanticLaneSet();

  // * update key lanes and its LUT
  if (agent_config_info_.enable_fast_lane_lut) {
    UpdateLocalLanesAndFastLut();
  }

  // * update semantic info for vehicles
  UpdateSemanticVehicles();

  // * update selected key vehicles
  UpdateKeyVehicles();

  // * openloop prediction for all semantic vehicles
  if (agent_config_info_.enable_openloop_prediction) {
    OpenloopTrajectoryPrediction();
  }

  if (agent_config_info_.enable_log) {
    SaveMapToLog();
  }
  return kSuccess;
}

ErrorType SemanticMapManager::SaveMapToLog() {
  std::ofstream record_file_stream;
  record_file_stream.open(agent_config_info_.log_file,
                          std::ofstream::out | std::ofstream::app);
  record_file_stream.setf(std::ios_base::fixed);

  common::VehicleSet vehicle_set = surrounding_vehicles_;
  vehicle_set.vehicles.insert(std::make_pair(ego_vehicle_.id(), ego_vehicle_));
  for (auto &v : vehicle_set.vehicles) {
    record_file_stream << v.second.state().time_stamp << "," << v.first << ","
                       << v.second.state().vec_position[0] << ","
                       << v.second.state().vec_position[1] << ","
                       << v.second.state().velocity << ","
                       << v.second.state().acceleration << ","
                       << v.second.state().angle << ","
                       << v.second.state().curvature << ","
                       << v.second.state().steer << std::endl;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::NaiveRuleBasedLateralBehaviorPrediction(
    const common::Vehicle &vehicle, const int nearest_lane_id,
    common::ProbDistOfLatBehaviors *lat_probs) {
  if (nearest_lane_id == kInvalidLaneId) {
    lat_probs->is_valid = false;
    return kWrongStatus;
  }

  SemanticLane nearest_lane =
      semantic_lane_set_.semantic_lanes.at(nearest_lane_id);
  common::StateTransformer stf(nearest_lane.lane);

  common::FrenetState fs;
  if (stf.GetFrenetStateFromState(vehicle.state(), &fs) != kSuccess) {
    lat_probs->is_valid = false;
    return kWrongStatus;
  }

  decimal_t prob_lcl = 0.0;
  decimal_t prob_lcr = 0.0;
  decimal_t prob_lk = 0.0;

  const decimal_t lat_distance_threshold = 0.4;
  const decimal_t lat_vel_threshold = 0.35;
  if (use_right_hand_axis_) {
    if (fs.vec_dt[0] > lat_distance_threshold &&
        fs.vec_dt[1] > lat_vel_threshold &&
        nearest_lane.l_lane_id != kInvalidLaneId &&
        nearest_lane.l_change_avbl) {
      prob_lcl = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcl.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else if (fs.vec_dt[0] < -lat_distance_threshold &&
               fs.vec_dt[1] < -lat_vel_threshold &&
               nearest_lane.r_lane_id != kInvalidLaneId &&
               nearest_lane.r_change_avbl) {
      prob_lcr = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcr.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else {
      prob_lk = 1.0;
    }
  } else {
    if (fs.vec_dt[0] > lat_distance_threshold &&
        fs.vec_dt[1] > lat_vel_threshold &&
        nearest_lane.r_lane_id != kInvalidLaneId &&
        nearest_lane.r_change_avbl) {
      prob_lcr = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcr.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else if (fs.vec_dt[0] < -lat_distance_threshold &&
               fs.vec_dt[1] < -lat_vel_threshold &&
               nearest_lane.l_lane_id != kInvalidLaneId &&
               nearest_lane.l_change_avbl) {
      prob_lcl = 1.0;
      printf(
          "[NaivePrediction]vehicle %d lane id %d, lat d %lf lat dd %lf, "
          "behavior "
          "lcl.\n",
          vehicle.id(), nearest_lane_id, fs.vec_dt[0], fs.vec_dt[1]);
    } else {
      prob_lk = 1.0;
    }
  }

  lat_probs->SetEntry(common::LateralBehavior::kLaneChangeLeft, prob_lcl);
  lat_probs->SetEntry(common::LateralBehavior::kLaneChangeRight, prob_lcr);
  lat_probs->SetEntry(common::LateralBehavior::kLaneKeeping, prob_lk);
  lat_probs->is_valid = true;

  return kSuccess;
}

ErrorType SemanticMapManager::MobilRuleBasedBehaviorPrediction(
    const common::Vehicle &vehicle, const common::VehicleSet &nearby_vehicles,
    common::ProbDistOfLatBehaviors *res) {
  decimal_t lane_radius = agent_config_info_.surrounding_search_radius;

  vec_E<common::Lane> lanes;
  vec_E<common::Vehicle> leading_vehicles;
  vec_E<common::Vehicle> following_vehicles;
  vec_E<common::FrenetState> leading_frenet_states;
  vec_E<common::FrenetState> follow_frenet_states;

  std::vector<LateralBehavior> behaviors{LateralBehavior::kLaneKeeping,
                                         LateralBehavior::kLaneChangeLeft,
                                         LateralBehavior::kLaneChangeRight};

  for (const auto &behavior : behaviors) {
    // ~ Prepare lane
    common::Lane ref_lane;
    GetRefLaneForStateByBehavior(vehicle.state(), std::vector<int>(), behavior,
                                 lane_radius, lane_radius, false, &ref_lane);

    // ~ Prepare leading and following vehicle
    bool has_leading_vehicle = false, has_following_vehicle = false;
    common::Vehicle leading_vehicle, following_vehicle;
    common::FrenetState leading_frenet_state, following_frenet_state;
    GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ref_lane, vehicle.state(), nearby_vehicles, &has_leading_vehicle,
        &leading_vehicle, &leading_frenet_state, &has_following_vehicle,
        &following_vehicle, &following_frenet_state);

    // ~ essemble
    lanes.push_back(ref_lane);
    leading_vehicles.push_back(leading_vehicle);
    following_vehicles.push_back(following_vehicle);
    leading_frenet_states.push_back(leading_frenet_state);
    follow_frenet_states.push_back(following_frenet_state);
  }

  if (common::MobilBehaviorPrediction::LateralBehaviorPrediction(
          vehicle, lanes, leading_vehicles, leading_frenet_states,
          following_vehicles, follow_frenet_states, nearby_vehicles,
          res) != kSuccess) {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetLeadingAndFollowingVehiclesFrenetStateOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, bool *has_leading_vehicle,
    common::Vehicle *leading_vehicle, common::FrenetState *leading_fs,
    bool *has_following_vehicle, common::Vehicle *following_vehicle,
    common::FrenetState *following_fs) const {
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;
  GetLeadingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                          leading_vehicle, &distance_residual_ratio);
  GetFollowingVehicleOnLane(ref_lane, ref_state, vehicle_set, lat_range,
                            following_vehicle);

  common::StateTransformer stf(ref_lane);
  *has_leading_vehicle = false;
  *has_following_vehicle = false;

  if (leading_vehicle->id() != kInvalidAgentId) {
    if (stf.GetFrenetStateFromState(leading_vehicle->state(), leading_fs) ==
        kSuccess) {
      *has_leading_vehicle = true;
    } else {
      return kWrongStatus;
    }
  }
  if (following_vehicle->id() != kInvalidAgentId) {
    if (stf.GetFrenetStateFromState(following_vehicle->state(), following_fs) ==
        kSuccess) {
      *has_following_vehicle = true;
    } else {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetEgoNearestLaneId(int *ego_lane_id) const {
  int nearest_lane_id;
  decimal_t distance, arclen;
  if (GetNearestLaneIdUsingState(ego_vehicle_.state().ToXYTheta(),
                                 std::vector<int>(), &nearest_lane_id,
                                 &distance, &arclen) != kSuccess) {
    return kWrongStatus;
  }
  *ego_lane_id = nearest_lane_id;
  return kSuccess;
}

ErrorType SemanticMapManager::UpdateSemanticVehicles() {
  // * construct semantic vehicle set
  // * necessary info: vehicle, nearest_lane_id(w.o. navi_path),
  // * lat_behavior w.r.t nearest lane
  // * other info: pred_traj, ref_lane
  common::SemanticVehicleSet semantic_vehicles_tmp;
  for (const auto &v : surrounding_vehicles_.vehicles) {
    common::SemanticVehicle semantic_vehicle;
    semantic_vehicle.vehicle = v.second;
    GetNearestLaneIdUsingState(
        semantic_vehicle.vehicle.state().ToXYTheta(), std::vector<int>(),
        &semantic_vehicle.nearest_lane_id, &semantic_vehicle.dist_to_lane,
        &semantic_vehicle.arc_len_onlane);

    NaiveRuleBasedLateralBehaviorPrediction(
        semantic_vehicle.vehicle, semantic_vehicle.nearest_lane_id,
        &semantic_vehicle.probs_lat_behaviors);
    semantic_vehicle.probs_lat_behaviors.GetMaxProbBehavior(
        &semantic_vehicle.lat_behavior);

    decimal_t max_backward_len = 10.0;
    decimal_t forward_lane_len =
        std::max(semantic_vehicle.vehicle.state().velocity * 10.0, 50.0);
    GetRefLaneForStateByBehavior(
        semantic_vehicle.vehicle.state(), std::vector<int>(),
        semantic_vehicle.lat_behavior, forward_lane_len, max_backward_len,
        false, &semantic_vehicle.lane);

    semantic_vehicles_tmp.semantic_vehicles.insert(
        std::pair<int, common::SemanticVehicle>(semantic_vehicle.vehicle.id(),
                                                semantic_vehicle));
  }
  {
    semantic_surrounding_vehicles_.semantic_vehicles.swap(
        semantic_vehicles_tmp.semantic_vehicles);
  }
  return kSuccess;
}

ErrorType SemanticMapManager::OpenloopTrajectoryPrediction() {
  openloop_pred_trajs_.clear();
  for (const auto &p_sv : semantic_surrounding_vehicles_.semantic_vehicles) {
    auto semantic_vehicle = p_sv.second;
    vec_E<common::State> traj;
    TrajectoryPredictionForVehicle(semantic_vehicle.vehicle,
                                   semantic_vehicle.lane, pred_time_,
                                   pred_step_, &traj);
    openloop_pred_trajs_.insert({{semantic_vehicle.vehicle.id(), traj}});
  }
  return kSuccess;
}

ErrorType SemanticMapManager::UpdateSemanticLaneSet() {
  semantic_lane_set_.clear();
  // Update semantic lane set
  {
    for (const auto &pe : surrounding_lane_net_.lane_set) {
      common::SemanticLane semantic_lane;
      semantic_lane.id = pe.second.id;
      semantic_lane.dir = pe.second.dir;
      semantic_lane.child_id = pe.second.child_id;
      semantic_lane.father_id = pe.second.father_id;
      semantic_lane.l_lane_id = pe.second.l_lane_id;
      semantic_lane.l_change_avbl = pe.second.l_change_avbl;
      semantic_lane.r_lane_id = pe.second.r_lane_id;
      semantic_lane.r_change_avbl = pe.second.r_change_avbl;
      semantic_lane.behavior = pe.second.behavior;
      semantic_lane.length = pe.second.length;

      vec_Vecf<2> samples;
      for (const auto &pt : pe.second.lane_points) {
        samples.push_back(pt);
      }
      if (common::LaneGenerator::GetLaneBySamplePoints(
              samples, &semantic_lane.lane) != kSuccess) {
        continue;
      }

      semantic_lane_set_.semantic_lanes.insert(
          std::pair<int, common::SemanticLane>(semantic_lane.id,
                                               semantic_lane));
    }
  }
  // Check the consistency of semantic map
  {
    for (auto &semantic_lane : semantic_lane_set_.semantic_lanes) {
      if (semantic_lane.second.l_change_avbl) {
        auto l_it = semantic_lane_set_.semantic_lanes.find(
            semantic_lane.second.l_lane_id);
        if (l_it == semantic_lane_set_.semantic_lanes.end()) {
          semantic_lane.second.l_change_avbl = false;
          semantic_lane.second.l_lane_id = kInvalidLaneId;
        }
      }
      if (semantic_lane.second.r_change_avbl) {
        auto r_it = semantic_lane_set_.semantic_lanes.find(
            semantic_lane.second.r_lane_id);
        if (r_it == semantic_lane_set_.semantic_lanes.end()) {
          semantic_lane.second.r_change_avbl = false;
          semantic_lane.second.r_lane_id = kInvalidLaneId;
        }
      }
      for (auto it = semantic_lane.second.father_id.begin();
           it < semantic_lane.second.father_id.end();) {
        auto father_it = semantic_lane_set_.semantic_lanes.find(*it);
        if (father_it == semantic_lane_set_.semantic_lanes.end()) {
          it = semantic_lane.second.father_id.erase(it);
        } else {
          ++it;
        }
      }
      for (auto it = semantic_lane.second.child_id.begin();
           it < semantic_lane.second.child_id.end();) {
        auto child_it = semantic_lane_set_.semantic_lanes.find(*it);
        if (child_it == semantic_lane_set_.semantic_lanes.end()) {
          it = semantic_lane.second.child_id.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::UpdateLocalLanesAndFastLut() {
  common::State ego_state = ego_vehicle_.state();
  int cur_lane_id;
  decimal_t dist_tmp, arc_len_tmp;
  if (kSuccess != GetNearestLaneIdUsingState(ego_state.ToXYTheta(),
                                             std::vector<int>(), &cur_lane_id,
                                             &dist_tmp, &arc_len_tmp)) {
    return kWrongStatus;
  }

  local_lanes_.clear();
  local_to_segment_lut_.clear();
  segment_to_local_lut_.clear();

  // * currently we consider ego lane and its adjacent lanes (trunk lanes)
  std::vector<int> root_lane_ids;
  {
    root_lane_ids.push_back(cur_lane_id);
    // ~ left
    if (whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id > 0) {
      int l_id = whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id;
      root_lane_ids.push_back(l_id);
      // ~ left -> left
      if (whole_lane_net_.lane_set.at(l_id).l_lane_id > 0) {
        int ll_id = whole_lane_net_.lane_set.at(l_id).l_lane_id;
        root_lane_ids.push_back(ll_id);
      }
    }
    // ~ right
    if (whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id > 0) {
      int r_id = whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id;
      root_lane_ids.push_back(r_id);
      // ~ right -> right
      if (whole_lane_net_.lane_set.at(r_id).r_lane_id > 0) {
        int rr_id = whole_lane_net_.lane_set.at(r_id).r_lane_id;
        root_lane_ids.push_back(rr_id);
      }
    }
  }

  std::vector<std::vector<int>> lane_ids_expand_front;
  for (const auto root_id : root_lane_ids) {
    decimal_t arc_len;
    semantic_lane_set_.semantic_lanes.at(root_id)
        .lane.GetArcLengthByVecPosition(ego_state.vec_position, &arc_len);
    decimal_t length_remain =
        semantic_lane_set_.semantic_lanes.at(root_id).length - arc_len;
    // printf("[XXX]root: %d, arc_len: %lf, remain: %lf\n", root_id, arc_len,
    //        length_remain);

    // ~ Get forward lane paths
    std::vector<std::vector<int>> all_paths_forward;
    GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
        root_id, length_remain, 0.0, std::vector<int>(), &all_paths_forward);

    // for (const auto path : all_paths_forward) {
    //   printf("[XXX]Forward:  ");
    //   for (const auto id : path) {
    //     printf("%d ", id);
    //   }
    //   printf("\n");
    // }

    // ~ Get backward lane paths
    std::vector<std::vector<int>> all_paths_backward;
    GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
        root_id, arc_len, 0.0, std::vector<int>(), &all_paths_backward);
    // for (const auto path : all_paths_backward) {
    //   printf("[XXX]Backward:  ");
    //   for (const auto id : path) {
    //     printf("%d ", id);
    //   }
    //   printf("\n");
    // }

    // ~ assemble forward and backward
    std::vector<std::vector<int>> assembled_paths;
    for (const auto &f_path : all_paths_forward) {
      for (const auto &b_path : all_paths_backward) {
        std::vector<int> path = f_path;
        path.erase(path.begin());
        path.insert(path.begin(), b_path.begin(), b_path.end());
        assembled_paths.push_back(path);
      }
    }
    // for (const auto path : assembled_paths) {
    //   printf("[XXX]assembled:  ");
    //   for (const auto id : path) {
    //     printf("%d ", id);
    //   }
    //   printf("\n");
    // }

    // ~ Fit local lanes and construct LUTs
    int local_lane_cnt = local_lanes_.size();
    for (const auto &path : assembled_paths) {
      common::Lane lane;
      if (kSuccess !=
          GetLocalLaneUsingLaneIds(ego_state, path, local_lane_length_forward_,
                                   local_lane_length_backward_, true, &lane)) {
        continue;
      }
      int local_id = local_lane_cnt++;
      local_lanes_.insert(std::pair<int, common::Lane>(local_id, lane));

      local_to_segment_lut_.insert(
          std::pair<int, std::vector<int>>(local_id, path));

      for (const auto &id : path) {
        segment_to_local_lut_[id].insert(local_id);
      }
    }
  }

  // for (const auto &entry : local_to_segment_lut_) {
  //   printf("\n[XXX]local: %d - segment: ", entry.first);
  //   for (const auto& id: entry.second) {
  //     printf("%d ", id);
  //   }
  //   printf("\n");
  // }

  // for (const auto &entry : segment_to_local_lut_) {
  //   printf("[XXX]segment: %d - local: ", entry.first);
  //   for (const auto& id: entry.second) {
  //     printf("%d ", id);
  //   }
  //   printf("\n");
  // }

  has_fast_lut_ = true;
  return kSuccess;
}

void SemanticMapManager::GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
    const decimal_t &node_id, const decimal_t &node_length,
    const decimal_t &aggre_length, const std::vector<int> &path_to_node,
    std::vector<std::vector<int>> *all_paths) {
  // * Traverse FORWARD
  // check cycle here?

  decimal_t node_aggre_length = node_length + aggre_length;
  auto path = path_to_node;
  path.push_back(node_id);

  if (node_aggre_length >= local_lane_length_forward_ ||
      whole_lane_net_.lane_set.at(node_id).child_id.empty()) {
    // stop recursion
    all_paths->push_back(path);
    return;
  } else {
    // expand
    auto child_ids = whole_lane_net_.lane_set.at(node_id).child_id;
    for (const auto child_id : child_ids) {
      decimal_t child_length = whole_lane_net_.lane_set.at(child_id).length;
      GetAllForwardLaneIdPathsWithMinimumLengthByRecursion(
          child_id, child_length, node_aggre_length, path, all_paths);
    }
  }
}

void SemanticMapManager::GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
    const decimal_t &node_id, const decimal_t &node_length,
    const decimal_t &aggre_length, const std::vector<int> &path_to_node,
    std::vector<std::vector<int>> *all_paths) {
  // * Traverse BACKWARD
  // check cycle here?

  decimal_t node_aggre_length = node_length + aggre_length;
  auto path = path_to_node;
  path.push_back(node_id);

  if (node_aggre_length >= local_lane_length_backward_ ||
      whole_lane_net_.lane_set.at(node_id).father_id.empty()) {
    // stop recursion
    // backward, reverse path
    std::reverse(path.begin(), path.end());
    all_paths->push_back(path);
    return;
  } else {
    // expand
    auto father_ids = whole_lane_net_.lane_set.at(node_id).father_id;
    for (const auto father_id : father_ids) {
      decimal_t father_length = whole_lane_net_.lane_set.at(father_id).length;
      GetAllBackwardLaneIdPathsWithMinimumLengthByRecursion(
          father_id, father_length, node_aggre_length, path, all_paths);
    }
  }
}

ErrorType SemanticMapManager::GetDistanceToLanesUsing3DofState(
    const Vec3f &state,
    std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> *res) const {
  for (const auto &p : semantic_lane_set_.semantic_lanes) {
    decimal_t arc_len;
    p.second.lane.GetArcLengthByVecPosition(Vec2f(state(0), state(1)),
                                            &arc_len);

    // decimal_t arc_len2;
    // p.second.lane.GetArcLengthByVecPositionUsingBinarySearch(
    //     Vec2f(state(0), state(1)), &arc_len2);

    // if (std::fabs(arc_len - arc_len2) > 1.0) {
    //   printf("[XXX]lane_id: %d, arc_len1: %lf, arc_len2: %lf\n", p.second.id,
    //          arc_len, arc_len2);
    // }

    Vec2f pt;
    p.second.lane.GetPositionByArcLength(arc_len, &pt);
    double dist = std::hypot((state(0) - pt(0)), (state(1) - pt(1)));

    if (dist > lane_range_) continue;

    decimal_t lane_angle;
    p.second.lane.GetOrientationByArcLength(arc_len, &lane_angle);
    decimal_t angle_diff = normalize_angle(lane_angle - state(2));

    res->insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
        dist, arc_len, angle_diff, p.second.id));
  }
#if 0
  for (auto iter = res->begin(); iter != res->end(); ++iter) {
    std::cout << "[Dist] distance = " << std::get<0>(*iter)
              << " , angle_diff = " << std::get<1>(*iter)
              << " , id = " << std::get<2>(*iter) << std::endl;
  }
  std::cout << "\n";
#endif
  return kSuccess;
}

ErrorType SemanticMapManager::CheckCollisionUsingState(
    const common::VehicleParam &param_a, const common::State &state_a,
    const common::VehicleParam &param_b, const common::State &state_b,
    bool *res) {
  common::OrientedBoundingBox2D obb_a, obb_b;
  common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
      param_a, state_a, &obb_a);
  common::SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
      param_b, state_b, &obb_b);
  *res = common::ShapeUtils::CheckIfOrientedBoundingBoxIntersect(obb_a, obb_b);
  return kSuccess;
}

ErrorType SemanticMapManager::CheckCollisionUsingStateAndVehicleParam(
    const common::VehicleParam &vehicle_param, const common::State &state,
    bool *res) {
  // check static collision
  {
    // TODO: (@denny.ding) add static collision checking
    common::Vehicle vehicle;
    vehicle.set_state(state);
    vehicle.set_param(vehicle_param);
    vec_E<Vec2f> vertices;
    common::ShapeUtils::GetVerticesOfOrientedBoundingBox(
        vehicle.RetOrientedBoundingBox(), &vertices);
    bool is_collision = false;
    for (auto &v : vertices) {
      CheckCollisionUsingGlobalPosition(v, &is_collision);
      if (is_collision) {
        *res = is_collision;
        return kSuccess;
      }
    }
  }

  // check dynamic collision
  if (agent_config_info_.enable_openloop_prediction) {
    for (const auto &v : semantic_surrounding_vehicles_.semantic_vehicles) {
      auto state_stamp = state.time_stamp;
      auto obstacle_init_stamp = v.second.vehicle.state().time_stamp;
      auto openloop_pred_traj = openloop_pred_trajs_.at(v.first);
      int access_index =
          std::round((state_stamp - obstacle_init_stamp) / pred_step_);
      int num_pred_states = static_cast<int>(openloop_pred_traj.size());
      if (access_index < 0 || access_index >= num_pred_states) continue;
      bool is_collision = false;
      CheckCollisionUsingState(vehicle_param, state, v.second.vehicle.param(),
                               openloop_pred_traj[access_index], &is_collision);
      if (is_collision) {
        *res = is_collision;
        return kSuccess;
      }
    }
  }
  *res = false;
  return kSuccess;
}

ErrorType SemanticMapManager::CheckCollisionUsingGlobalPosition(
    const Vec2f &p_w, bool *res) const {
  std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
  return obstacle_map_.CheckIfEqualUsingGlobalPosition(p, GridMap2D::OCCUPIED,
                                                       res);
}

ErrorType SemanticMapManager::GetObstacleMapValueUsingGlobalPosition(
    const Vec2f &p_w, ObstacleMapType *res) {
  std::array<decimal_t, 2> p = {{p_w(0), p_w(1)}};
  return obstacle_map_.GetValueUsingGlobalPosition(p, res);
}

ErrorType SemanticMapManager::IsTopologicallyReachable(
    const int lane_id, const std::vector<int> &path, int *num_lane_changes,
    bool *res) const {
  if (semantic_lane_set_.semantic_lanes.count(lane_id) == 0) {
    printf("[IsTopologicallyReachable]fail to get lane id %d.\n", lane_id);
    return kWrongStatus;
  }
  // ~ check whether any node of the path is reachable from lane id
  const int max_expansion_nodes = 20;

  // ~ BFS starting from lane_id;
  std::unordered_map<int, int> num_lc_map;
  std::set<int> visited_set;
  std::list<int> queue;
  visited_set.insert(lane_id);
  queue.push_back(lane_id);
  num_lc_map.insert(std::make_pair(lane_id, 0));

  int expanded_nodes = 1;
  int cur_id = lane_id;
  bool is_reachable = false;
  while (!queue.empty() && expanded_nodes < max_expansion_nodes) {
    cur_id = queue.front();
    queue.pop_front();
    expanded_nodes++;

    if (std::find(path.begin(), path.end(), cur_id) != path.end()) {
      *num_lane_changes = num_lc_map.at(cur_id);
      is_reachable = true;
      break;
    }
    std::vector<int> child_ids;
    auto it = semantic_lane_set_.semantic_lanes.find(cur_id);
    if (it == semantic_lane_set_.semantic_lanes.end()) {
      continue;
    } else {
      child_ids = it->second.child_id;
      if (it->second.l_change_avbl) child_ids.push_back(it->second.l_lane_id);
      if (it->second.r_change_avbl) child_ids.push_back(it->second.r_lane_id);
    }
    if (child_ids.empty()) continue;
    for (auto &id : child_ids) {
      if (visited_set.count(id) == 0) {
        visited_set.insert(id);
        queue.push_back(id);
        if (it->second.l_change_avbl && id == it->second.l_lane_id) {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id) + 1));
        } else if (it->second.r_change_avbl && id == it->second.r_lane_id) {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id) + 1));
        } else {
          num_lc_map.insert(std::make_pair(id, num_lc_map.at(cur_id)));
        }
      }
    }
  }

  if (!is_reachable) {
    *res = false;
  } else {
    *res = true;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetNearestLaneIdUsingState(
    const Vec3f &state, const std::vector<int> &navi_path, int *id,
    decimal_t *distance, decimal_t *arc_len) const {
  // tuple: dist, arc_len, angle_diff, id
  std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>> lanes_in_dist;
  if (GetDistanceToLanesUsing3DofState(state, &lanes_in_dist) != kSuccess) {
    return kWrongStatus;
  }

  if (lanes_in_dist.empty()) {
    printf("[GetNearestLaneIdUsingState]No nearest lane found.\n");
    return kWrongStatus;
  }

  //   if (navi_path.empty()) {
  //     *id = std::get<3>(*lanes_in_dist.begin());
  //     *distance = std::get<0>(*lanes_in_dist.begin());
  //     *arc_len = std::get<1>(*lanes_in_dist.begin());
  //   } else {
  // #if 0
  //     // bool has_nearest_lane_found = false;
  //     std::map<decimal_t, std::pair<int, decimal_t>> surrounding_lanes;
  //     for (auto &t : lanes_in_dist) {
  //       int lane_id = std::get<3>(t);
  //       decimal_t cur_distance = std::get<0>(t);
  //       bool is_reachable = false;
  //       int num_lane_changes;
  //       if (IsTopologicallyReachable(lane_id, navi_path, &num_lane_changes,
  //                                    &is_reachable) != kSuccess) {
  //         continue;
  //       }
  //       if (is_reachable) {
  //         const decimal_t lane_change_dist_cost = 0.1;
  //         decimal_t cost =
  //             num_lane_changes * lane_change_dist_cost + cur_distance;
  //         surrounding_lanes.emplace(cost, std::make_pair(lane_id,
  //         cur_distance));
  //       }
  //     }
  //     if (surrounding_lanes.empty()) {
  //       return kWrongStatus;
  //     }
  //     *id = surrounding_lanes.begin()->second.first;
  //     *distance = surrounding_lanes.begin()->second.second;
  // #else
  //     *id = std::get<3>(*lanes_in_dist.begin());
  //     *distance = std::get<0>(*lanes_in_dist.begin());
  //     *arc_len = std::get<1>(*lanes_in_dist.begin());
  // #endif
  //   }

  // * Get candidate lanes within a small range, then sort by angle_diff
  // tuple: angle_diff, dist, arc_len, id
  std::set<std::tuple<decimal_t, decimal_t, decimal_t, int>>
      lanes_in_angle_diff;
  for (const auto &ele : lanes_in_dist) {
    if (std::get<0>(ele) > nearest_lane_range_) {
      break;
    }
    lanes_in_angle_diff.insert(std::tuple<decimal_t, decimal_t, decimal_t, int>(
        fabs(std::get<2>(ele)), std::get<0>(ele), std::get<1>(ele),
        std::get<3>(ele)));
  }
  if (lanes_in_angle_diff.empty() ||
      std::get<0>(*lanes_in_angle_diff.begin()) > kPi / 2) {
    // Use the nearest lane with suitable angle diff
    for (const auto &ele : lanes_in_dist) {
      if (std::get<2>(ele) < kPi / 2) {
        *id = std::get<3>(ele);
        *distance = std::get<0>(ele);
        *arc_len = std::get<1>(ele);
        return kSuccess;
      }
    }
    // Otherwise, use the nearest lane
    *id = std::get<3>(*lanes_in_dist.begin());
    *distance = std::get<0>(*lanes_in_dist.begin());
    *arc_len = std::get<1>(*lanes_in_dist.begin());
    // printf(
    //     "[GetNearestLaneIdUsingState]No suitable lane in %f m, use the
    //     nearest " "one, dist: %lf, id: %d\n", nearest_lane_range_, *distance,
    //     *id);
    return kSuccess;
  }
  // * Use the lane with minimum angle diff
  *id = std::get<3>(*lanes_in_angle_diff.begin());
  *distance = std::get<1>(*lanes_in_angle_diff.begin());
  *arc_len = std::get<2>(*lanes_in_angle_diff.begin());

  if (std::get<3>(*lanes_in_angle_diff.begin()) !=
      std::get<3>(*lanes_in_dist.begin())) {
    // printf(
    //     "[GetNearestLaneIdUsingState]Use minimum angle diff lane, "
    //     "angle_diff: %lf, dist: %lf, id: %d\n",
    //     std::get<0>(*lanes_in_angle_diff.begin()), *distance, *id);
  }

  // *id = std::get<3>(*lanes_in_dist.begin());
  // *distance = std::get<0>(*lanes_in_dist.begin());
  // *arc_len = std::get<1>(*lanes_in_dist.begin());
  // printf("[GetNearestLaneIdUsingState]angle_diff: %lf, dist: %lf, id: %d\n",
  //        std::get<2>(*lanes_in_dist.begin()), *distance, *id);
  return kSuccess;
}

ErrorType SemanticMapManager::TrajectoryPredictionForVehicle(
    const common::Vehicle &vehicle, const common::Lane &lane,
    const decimal_t &t_pred, const decimal_t &t_step,
    vec_E<common::State> *traj) {
  planning::OnLaneFsPredictor::GetPredictedTrajectory(lane, vehicle, t_pred,
                                                      t_step, traj);
  return kSuccess;
}

ErrorType SemanticMapManager::UpdateKeyVehicles() {
  // ~ directly use semantic surrounding vehicle as key vehicle
  semantic_key_vehicles_ = semantic_surrounding_vehicles_;
  key_vehicles_ = surrounding_vehicles_;
#if 1
  decimal_t search_radius = agent_config_info_.surrounding_search_radius;

  std::map<int, decimal_t> key_lane_ids;  // lane_id, length to the cur_pos
  int cur_lane_id;
  decimal_t cur_lane_dist;
  decimal_t cur_arc_len;

  if (GetNearestLaneIdUsingState(ego_vehicle_.state().ToXYTheta(),
                                 std::vector<int>(), &cur_lane_id,
                                 &cur_lane_dist, &cur_arc_len) != kSuccess) {
    return kWrongStatus;
  }

  // Find key lane ids
  {
    std::vector<int> mid_lane_ids;
    // left and right lanes
    // id - reference arc_len of start points w.r.t to ego vehicle
    key_lane_ids.insert(std::pair<int, decimal_t>(cur_lane_id, -cur_arc_len));
    mid_lane_ids.push_back(cur_lane_id);
    if (whole_lane_net_.lane_set.at(cur_lane_id).l_change_avbl) {
      key_lane_ids.insert(std::pair<int, decimal_t>(
          whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id, -cur_arc_len));
      mid_lane_ids.push_back(
          whole_lane_net_.lane_set.at(cur_lane_id).l_lane_id);
    }
    if (whole_lane_net_.lane_set.at(cur_lane_id).r_change_avbl) {
      key_lane_ids.insert(std::pair<int, decimal_t>(
          whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id, -cur_arc_len));
      mid_lane_ids.push_back(
          whole_lane_net_.lane_set.at(cur_lane_id).r_lane_id);
    }

    // successors
    decimal_t len_front =
        whole_lane_net_.lane_set.at(cur_lane_id).length - cur_arc_len;
    for (const int &id : mid_lane_ids) {
      // ! bug!
      decimal_t len_sum = len_front;
      if (len_sum >= search_radius) continue;
      std::set<std::pair<decimal_t, int>> id_to_expand;
      id_to_expand.insert(std::pair<decimal_t, int>(len_sum, id));
      while (!id_to_expand.empty()) {
        auto it = id_to_expand.begin();
        decimal_t len_expand = it->first;
        int id_expand = it->second;
        id_to_expand.erase(it);
        std::vector<int> succ_ids =
            whole_lane_net_.lane_set.at(id_expand).child_id;
        for (const auto &succ_id : succ_ids) {
          key_lane_ids.insert(std::pair<int, decimal_t>(succ_id, len_sum));
          decimal_t len_tmp =
              whole_lane_net_.lane_set.at(succ_id).length + len_expand;
          if (len_tmp < search_radius) {
            id_to_expand.insert(std::pair<decimal_t, int>(len_tmp, succ_id));
          }
        }
      }
    }
    // predecessors
    auto it = std::find(mid_lane_ids.begin(), mid_lane_ids.end(), cur_lane_id);
    mid_lane_ids.erase(it);
    decimal_t len_rear = -cur_arc_len;
    for (const auto &id : mid_lane_ids) {
      decimal_t len_sum = len_rear;
      if (fabs(len_sum) >= search_radius) continue;
      std::set<std::pair<decimal_t, int>> id_to_expand;
      id_to_expand.insert(std::pair<decimal_t, int>(len_sum, id));
      while (!id_to_expand.empty()) {
        auto it = id_to_expand.begin();
        decimal_t len_expand = it->first;
        int id_expand = it->second;
        id_to_expand.erase(it);
        std::vector<int> pred_ids =
            whole_lane_net_.lane_set.at(id_expand).father_id;
        for (const auto &pred_id : pred_ids) {
          decimal_t len_tmp =
              -whole_lane_net_.lane_set.at(pred_id).length + len_expand;
          key_lane_ids.insert(std::pair<int, decimal_t>(pred_id, len_tmp));
          if (fabs(len_tmp) < search_radius) {
            id_to_expand.insert(std::pair<decimal_t, int>(len_tmp, pred_id));
          }
        }
      }
    }
  }

  decimal_t max_radius = 170.0;
  decimal_t min_radius = 30.0;
  decimal_t dec_comfort = 1.6;
  decimal_t t_pl = 5.0;
  decimal_t t_comfort =
      std::max(t_pl, ego_vehicle_.state().velocity / dec_comfort);
  decimal_t pl_horizon_length = ego_vehicle_.state().velocity * t_comfort + 100;

  decimal_t front_range = std::min(pl_horizon_length, max_radius);
  front_range = std::max(min_radius, front_range);

  // get key surrounding vehicles
  // ~ Here we use an assumption:
  // ~ All nearby/reachable lanes have similar length and start from similar
  // ~ arc_len
  key_vehicle_ids_.clear();
  semantic_key_vehicles_.semantic_vehicles.clear();
  key_vehicles_.vehicles.clear();
  {
    for (const auto v : semantic_surrounding_vehicles_.semantic_vehicles) {
      if (v.second.dist_to_lane > max_distance_to_lane_) {
        continue;
      }
      int v_lane_id = v.second.nearest_lane_id;
      auto it = key_lane_ids.find(v_lane_id);
      if (it != key_lane_ids.end()) {
        decimal_t len_offset = it->second;
        decimal_t dist = len_offset + v.second.arc_len_onlane;
        // * Front
        if (dist >= 0 && fabs(dist) < front_range) {
          int v_id = v.first;
          if (v.second.nearest_lane_id == cur_lane_id &&
              v.second.arc_len_onlane < cur_arc_len)
            continue;
          key_vehicle_ids_.push_back(v_id);
          semantic_key_vehicles_.semantic_vehicles.insert(
              std::pair<int, common::SemanticVehicle>(
                  v_id,
                  semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)));
          key_vehicles_.vehicles.insert(std::pair<int, common::Vehicle>(
              v_id, semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)
                        .vehicle));
        }
        // * Rear
        decimal_t s_margin =
            std::max(20.0, fabs(v.second.vehicle.state().velocity) * t_pl);
        if (dist < 0 && s_margin > fabs(dist)) {
          int v_id = v.first;
          if (v.second.nearest_lane_id == cur_lane_id &&
              v.second.arc_len_onlane < cur_arc_len)
            continue;
          key_vehicle_ids_.push_back(v_id);
          semantic_key_vehicles_.semantic_vehicles.insert(
              std::pair<int, common::SemanticVehicle>(
                  v_id,
                  semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)));
          key_vehicles_.vehicles.insert(std::pair<int, common::Vehicle>(
              v_id, semantic_surrounding_vehicles_.semantic_vehicles.at(v_id)
                        .vehicle));
        }
      }
    }
  }
  // std::cout << "[XXX] ";
  // for (const auto &id : nearby_key_vehicle_ids) {
  //   std::cout << id << " ";
  // }
  // std::cout << "\n";
#endif

  return kSuccess;
}

ErrorType SemanticMapManager::GetLocalLaneSamplesByState(
    const common::State &state, const int lane_id,
    const std::vector<int> &navi_path, const decimal_t max_reflane_dist,
    const decimal_t max_backward_dist, vec_Vecf<2> *samples) const {
  if (semantic_lane_set_.semantic_lanes.count(lane_id) == 0) {
    printf("[GetLocalLaneSamplesByState]fail to get lane id %d.\n", lane_id);
    return kWrongStatus;
  }

  decimal_t arclen = 0.0;
  common::Lane target_lane = semantic_lane_set_.semantic_lanes.at(lane_id).lane;
  target_lane.GetArcLengthByVecPosition(state.vec_position, &arclen);
  decimal_t accum_dist_backward = 0.0;
  std::vector<int> ids_back;
  {
    if (arclen < max_backward_dist) {
      accum_dist_backward += arclen;
      int id_tmp = lane_id;
      while (accum_dist_backward < max_backward_dist) {
        std::vector<int> father_ids =
            semantic_lane_set_.semantic_lanes.at(id_tmp).father_id;
        if (!father_ids.empty()) {
          // TODO: double check the logic for front
          int father_id = father_ids.front();
          for (auto &id : father_ids) {
            if (std::find(navi_path.begin(), navi_path.end(), id) !=
                navi_path.end()) {
              father_id = id;
              break;
            }
          }
          accum_dist_backward +=
              semantic_lane_set_.semantic_lanes.at(father_id).length;
          ids_back.push_back(father_id);
          id_tmp = father_id;
        } else {
          break;
        }
      }
    } else {
      accum_dist_backward = arclen;
    }
  }

  std::reverse(ids_back.begin(), ids_back.end());

  std::vector<int> ids_front;
  decimal_t accum_dist_forward = 0.0;
  {
    decimal_t dist_remain_target_lane =
        semantic_lane_set_.semantic_lanes.at(lane_id).length - arclen;
    if (dist_remain_target_lane < max_reflane_dist) {
      int id_tmp = lane_id;
      ids_front.push_back(lane_id);
      accum_dist_forward += dist_remain_target_lane;
      while (accum_dist_forward < max_reflane_dist) {
        std::vector<int> child_ids =
            semantic_lane_set_.semantic_lanes.at(id_tmp).child_id;
        // TODO: double check the logic for front
        if (!child_ids.empty()) {
          int child_id = child_ids.front();
          for (auto &id : child_ids) {
            if (std::find(navi_path.begin(), navi_path.end(), id) !=
                navi_path.end()) {
              child_id = id;
              break;
            }
          }
          accum_dist_forward +=
              semantic_lane_set_.semantic_lanes.at(child_id).length;
          ids_front.push_back(child_id);
          id_tmp = child_id;
        } else {
          break;
        }
      }
    } else {
      ids_front.push_back(lane_id);
      accum_dist_forward = dist_remain_target_lane;
    }
  }

  std::vector<int> lane_id_all;
  lane_id_all.insert(lane_id_all.end(), ids_back.begin(), ids_back.end());
  lane_id_all.insert(lane_id_all.end(), ids_front.begin(), ids_front.end());

  vec_Vecf<2> raw_samples;
  for (const auto &id : lane_id_all) {
    if (raw_samples.empty() &&
        (int)surrounding_lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(
          surrounding_lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1;
         i < (int)surrounding_lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(
          surrounding_lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  common::Lane long_lane;
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
      kSuccess) {
    return kWrongStatus;
  }

  decimal_t acc_dist_tmp;
  decimal_t sample_start =
      std::max(0.0, accum_dist_backward - max_backward_dist);
  decimal_t forward_sample_len = std::min(max_reflane_dist, accum_dist_forward);
  SampleLane(long_lane, sample_start,
             sample_start + forward_sample_len +
                 std::min(accum_dist_backward, max_backward_dist),
             1.0, samples, &acc_dist_tmp);

  return kSuccess;
}

ErrorType SemanticMapManager::GetLocalLaneUsingLaneIds(
    const common::State &state, const std::vector<int> &lane_ids,
    const decimal_t forward_length, const decimal_t backward_length,
    const bool &is_high_quality, common::Lane *lane) {
  vec_Vecf<2> raw_samples;
  for (const auto &id : lane_ids) {
    if (raw_samples.empty() &&
        whole_lane_net_.lane_set.at(id).lane_points.size() > 0) {
      raw_samples.push_back(whole_lane_net_.lane_set.at(id).lane_points[0]);
    }
    for (int i = 1; i < whole_lane_net_.lane_set.at(id).lane_points.size();
         ++i) {
      raw_samples.push_back(whole_lane_net_.lane_set.at(id).lane_points[i]);
    }
  }

  common::Lane long_lane;
  if (common::LaneGenerator::GetLaneBySamplePoints(raw_samples, &long_lane) !=
      kSuccess) {
    return kWrongStatus;
  }
  decimal_t arc_len;
  long_lane.GetArcLengthByVecPosition(state.vec_position, &arc_len);

  vec_Vecf<2> samples;
  decimal_t acc_dist_tmp;
  decimal_t sample_start = std::max(0.0, arc_len - backward_length);
  decimal_t sample_end = std::min(arc_len + forward_length, long_lane.end());
  SampleLane(long_lane, sample_start, sample_end, 1.0, &samples, &acc_dist_tmp);

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetRefLaneForStateByBehavior(
    const common::State &state, const std::vector<int> &navi_path,
    const LateralBehavior &behavior, const decimal_t &max_forward_len,
    const decimal_t &max_back_len, const bool is_high_quality,
    common::Lane *lane) const {
  Vec3f state_3dof(state.vec_position(0), state.vec_position(1), state.angle);
  int current_lane_id;
  decimal_t distance_to_lane;
  decimal_t arc_len;
  if (GetNearestLaneIdUsingState(state_3dof, navi_path, &current_lane_id,
                                 &distance_to_lane, &arc_len) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get nearest lane.\n");
    return kWrongStatus;
  }

  if (distance_to_lane > max_distance_to_lane_) {
    return kWrongStatus;
  }

  int target_lane_id;
  if (GetTargetLaneId(current_lane_id, behavior, &target_lane_id) != kSuccess) {
    // printf(
    //     "[GetRefLaneForStateByBehavior]fail to get target lane from lane %d "
    //     "with behavior %d.\n",
    //     current_lane_id, static_cast<int>(behavior));
    return kWrongStatus;
  }

  if (agent_config_info_.enable_fast_lane_lut && has_fast_lut_) {
    if (segment_to_local_lut_.end() !=
        segment_to_local_lut_.find(target_lane_id)) {
      // * here we just select the first local lane from several candidates
      int id = *segment_to_local_lut_.at(target_lane_id).begin();
      *lane = local_lanes_.at(id);
      return kSuccess;
    }
  }

  // ~ the reflane length should be consist with maximum speed and maximum
  // ~ forward simulation time, the current setup is for 30m/s x 7.5s forward
  vec_Vecf<2> samples;
  if (GetLocalLaneSamplesByState(state, target_lane_id, navi_path,
                                 max_forward_len, max_back_len,
                                 &samples) != kSuccess) {
    printf("[GetRefLaneForStateByBehavior]Cannot get local lane samples.\n");
    return kWrongStatus;
  }

  if (kSuccess != GetLaneBySampledPoints(samples, is_high_quality, lane)) {
    return kWrongStatus;
  }

  return kSuccess;
}

ErrorType SemanticMapManager::GetLaneBySampledPoints(
    const vec_Vecf<2> &samples, const bool &is_high_quality,
    common::Lane *lane) const {
  if (is_high_quality) {
    double d = 0.0;
    std::vector<decimal_t> para;
    para.push_back(d);

    int num_samples = static_cast<int>(samples.size());
    for (int i = 1; i < num_samples; i++) {
      double dx = samples[i](0) - samples[i - 1](0);
      double dy = samples[i](1) - samples[i - 1](1);
      d += std::hypot(dx, dy);
      para.push_back(d);
    }

    const int num_segments = 20;
    Eigen::ArrayXf breaks =
        Eigen::ArrayXf::LinSpaced(num_segments, para.front(), para.back());

    const decimal_t regulator = (double)1e6;
    if (common::LaneGenerator::GetLaneBySampleFitting(
            samples, para, breaks, regulator, lane) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    if (common::LaneGenerator::GetLaneBySamplePoints(samples, lane) !=
        kSuccess) {
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::SampleLane(const common::Lane &lane,
                                         const decimal_t &s0,
                                         const decimal_t &s1,
                                         const decimal_t &step,
                                         vec_E<Vecf<2>> *samples,
                                         decimal_t *accum_dist) const {
  Vecf<2> pt;
  for (decimal_t s = s0; s < s1; s += step) {
    lane.GetPositionByArcLength(s, &pt);
    samples->push_back(pt);
    (*accum_dist) += step;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetTargetLaneId(const int lane_id,
                                              const LateralBehavior &behavior,
                                              int *target_lane_id) const {
  auto it = semantic_lane_set_.semantic_lanes.find(lane_id);
  if (it == semantic_lane_set_.semantic_lanes.end()) {
    return kWrongStatus;
  } else {
    if (behavior == common::LateralBehavior::kLaneKeeping ||
        behavior == common::LateralBehavior::kUndefined) {
      *target_lane_id = lane_id;
    } else if (behavior == common::LateralBehavior::kLaneChangeLeft) {
      if (it->second.l_change_avbl) {
        *target_lane_id = it->second.l_lane_id;
      } else {
        return kWrongStatus;
      }
    } else if (behavior == common::LateralBehavior::kLaneChangeRight) {
      if (it->second.r_change_avbl) {
        *target_lane_id = it->second.r_lane_id;
      } else {
        return kWrongStatus;
      }
    } else {
      assert(false);
    }
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetLeadingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *leading_vehicle,
    decimal_t *distance_residual_ratio) const {
  /**
   *    [>>>>]    ------->    [>>>>]
   *      | offset               |
   *      |                      |
   *  ------------------------------------ref lane
   *      lane_pt
   */
  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  Vecf<2> lane_pt;
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    return kWrongStatus;
  }
  ref_lane.GetPositionByArcLength(ref_fs.vec_s[0], &lane_pt);
  // Vecf<2> offset = ref_state.vec_position - lane_pt;

  const decimal_t lane_width = 3.5;
  const decimal_t search_lat_radius = lat_range;
  const decimal_t max_forward_search_dist = 120.0;
  decimal_t search_lon_offset = 0.0;
  decimal_t resolution = search_lat_radius / 1.4;

  int leading_vehicle_id = kInvalidAgentId;

  bool find_leading_vehicle_in_set = false;
  bool find_occupied = false;
  common::Vehicle virtual_vehicle;

  for (decimal_t s = ref_fs.vec_s[0] + resolution + search_lon_offset;
       s < ref_fs.vec_s[0] + max_forward_search_dist + search_lon_offset;
       s += resolution) {
    decimal_t delta_s = s - ref_fs.vec_s[0];
    ref_lane.GetPositionByArcLength(s, &lane_pt);
    // lane_pt = lane_pt + offset;

    for (const auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      if ((lane_pt - entry.second.state().vec_position).squaredNorm() <
          search_lat_radius * search_lat_radius) {
        find_leading_vehicle_in_set = true;
        leading_vehicle_id = entry.first;
        *distance_residual_ratio =
            (max_forward_search_dist - delta_s) / max_forward_search_dist;
        break;
      }
    }

    if (find_leading_vehicle_in_set) break;
  }

  if (find_leading_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(leading_vehicle_id);
    *leading_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetFollowingVehicleOnLane(
    const common::Lane &ref_lane, const common::State &ref_state,
    const common::VehicleSet &vehicle_set, const decimal_t &lat_range,
    common::Vehicle *following_vehicle) const {
  common::StateTransformer stf(ref_lane);
  common::FrenetState ref_fs;
  if (stf.GetFrenetStateFromState(ref_state, &ref_fs) != kSuccess) {
    // printf("[FollowingVehicleOnLane]Cannot get ref state frenet state.\n");
    return kWrongStatus;
  }

  const decimal_t lane_width_tol = lat_range;
  decimal_t max_backward_search_dist =
      std::min(ref_fs.vec_s[0] - ref_lane.begin(), 100.0);

  int following_vehicle_id = kInvalidAgentId;
  Vecf<2> lane_pt;

  bool find_following_vehicle_in_set = false;

  for (decimal_t delta_s = lane_width_tol / 1.4;
       delta_s < max_backward_search_dist - 2.0 * lane_width_tol;
       delta_s += lane_width_tol / 1.4) {
    ref_lane.GetPositionByArcLength(ref_fs.vec_s[0] - delta_s, &lane_pt);
    for (auto &entry : vehicle_set.vehicles) {
      if (entry.second.id() == kInvalidAgentId) continue;
      if ((lane_pt - entry.second.state().vec_position).norm() <
          lane_width_tol) {
        find_following_vehicle_in_set = true;
        following_vehicle_id = entry.first;
        break;
      }
    }

    if (find_following_vehicle_in_set) break;
  }

  if (find_following_vehicle_in_set) {
    auto it = vehicle_set.vehicles.find(following_vehicle_id);
    *following_vehicle = it->second;
  } else {
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType SemanticMapManager::GetSpeedLimit(const State &state,
                                            const Lane &lane,
                                            decimal_t *speed_limit) const {
  return traffic_singal_manager_.GetSpeedLimit(state, lane, speed_limit);
}

ErrorType SemanticMapManager::GetTrafficStoppingState(
    const State &state, const Lane &lane, State *stopping_state) const {
  return traffic_singal_manager_.GetTrafficStoppingState(state, lane,
                                                         stopping_state);
}

bool SemanticMapManager::IsLocalLaneContainsLane(const int &local_lane_id,
                                                 const int &seg_lane_id) const {
  if (!has_fast_lut_) return false;
  auto ids = local_to_segment_lut_.at(local_lane_id);
  if (ids.end() != std::find(ids.begin(), ids.end(), seg_lane_id)) {
    return true;
  }
  return false;
}

// TODO(lu.zhang): Use general graph search instead in the future
ErrorType SemanticMapManager::GetDistanceOnLaneNet(const int &lane_id_0,
                                                   const decimal_t &arc_len_0,
                                                   const int &lane_id_1,
                                                   const decimal_t &arc_len_1,
                                                   decimal_t *dist) const {
  std::unordered_set<int> visited_list;
  std::set<std::pair<decimal_t, int>> pq;
  pq.insert(std::pair<decimal_t, int>(0.0, lane_id_0));
  decimal_t cost_lane_change = 10.0;

  int tar_node = lane_id_1;
  // decimal_t cost_aggre = -arc_len_0;

  while (!pq.empty()) {
    int cur_node = pq.begin()->second;

    if (cur_node == tar_node) {
      // finish
      break;
    }
    visited_list.insert(cur_node);
    std::vector<std::pair<int, decimal_t>> succ_nodes;

    // get successors
    {
      // child lane
      if (!whole_lane_net_.lane_set.at(cur_node).child_id.empty()) {
        auto ids = whole_lane_net_.lane_set.at(cur_node).child_id;
        auto cost = whole_lane_net_.lane_set.at(cur_node).length;
        for (const auto &id : ids) {
          succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
        }
      }

      // left lane
      if (whole_lane_net_.lane_set.at(cur_node).l_change_avbl) {
        auto id = whole_lane_net_.lane_set.at(cur_node).l_lane_id;
        auto cost = cost_lane_change;
        succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
      }

      // right lane
      if (whole_lane_net_.lane_set.at(cur_node).r_change_avbl) {
        auto id = whole_lane_net_.lane_set.at(cur_node).r_lane_id;
        auto cost = cost_lane_change;
        succ_nodes.push_back(std::pair<int, decimal_t>(id, cost));
      }
    }

    // for (int i = 0; i < succ_nodes.size(); ++i) {
    //   auto id = succ_nodes[i].first;
    //   auto it = visited_list.find(id);
    // }
  }

  return kSuccess;
}

}  // namespace semantic_map_manager
