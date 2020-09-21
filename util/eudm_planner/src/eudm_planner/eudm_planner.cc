#include "eudm_planner/eudm_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "common/rss/rss_checker.h"

namespace planning {

std::string EudmPlanner::Name() { return std::string("Eudm behavior planner"); }

ErrorType EudmPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading eudm planner config\n");
  using namespace google::protobuf;
  int fd = open(config_path.c_str(), O_RDONLY);
  io::FileInputStream fstream(fd);
  TextFormat::Parse(&fstream, &cfg_);
  if (!cfg_.IsInitialized()) {
    LOG(ERROR) << "failed to parse config from " << config_path;
    assert(false);
  }
  return kSuccess;
}

ErrorType EudmPlanner::GetSimParam(const planning::eudm::ForwardSimDetail& cfg,
                                   OnLaneForwardSimulation::Param* sim_param) {
  sim_param->idm_param.kMinimumSpacing = cfg.lon().idm().min_spacing();
  sim_param->idm_param.kDesiredHeadwayTime = cfg.lon().idm().head_time();
  sim_param->idm_param.kAcceleration = cfg.lon().limit().acc();
  sim_param->idm_param.kComfortableBrakingDeceleration =
      cfg.lon().limit().soft_brake();
  sim_param->idm_param.kHardBrakingDeceleration =
      cfg.lon().limit().hard_brake();
  sim_param->idm_param.kExponent = cfg.lon().idm().exponent();
  sim_param->max_lon_acc_jerk = cfg.lon().limit().acc_jerk();
  sim_param->max_lon_brake_jerk = cfg.lon().limit().brake_jerk();
  sim_param->steer_control_gain = cfg.lat().pure_pursuit().gain();
  sim_param->steer_control_max_lookahead_dist =
      cfg.lat().pure_pursuit().max_lookahead_dist();
  sim_param->steer_control_min_lookahead_dist =
      cfg.lat().pure_pursuit().min_lookahead_dist();
  sim_param->max_lat_acceleration_abs = cfg.lat().limit().acc();
  sim_param->max_lat_jerk_abs = cfg.lat().limit().jerk();
  sim_param->max_curvature_abs = cfg.lat().limit().curvature();
  sim_param->max_steer_angle_abs = cfg.lat().limit().steer_angle();
  sim_param->max_steer_rate = cfg.lat().limit().steer_rate();
  sim_param->auto_decelerate_if_lat_failed = cfg.auto_dec_if_lat_failed();
  return kSuccess;
}

ErrorType EudmPlanner::Init(const std::string config) {
  ReadConfig(config);

  dcp_tree_ptr_ = new DcpTree(cfg_.sim().duration().tree_height(),
                              cfg_.sim().duration().layer(),
                              cfg_.sim().duration().last_layer());
  LOG(INFO) << "[Eudm]Init.";
  LOG(INFO) << "[Eudm]ActionScript size: "
            << dcp_tree_ptr_->action_script().size() << std::endl;

  GetSimParam(cfg_.sim().ego(), &ego_sim_param_);
  GetSimParam(cfg_.sim().agent(), &agent_sim_param_);

  rss_config_ = common::RssChecker::RssConfig(
      cfg_.safety().rss().response_time(),
      cfg_.safety().rss().longitudinal_acc_max(),
      cfg_.safety().rss().longitudinal_brake_min(),
      cfg_.safety().rss().longitudinal_brake_max(),
      cfg_.safety().rss().lateral_acc_max(),
      cfg_.safety().rss().lateral_brake_min(),
      cfg_.safety().rss().lateral_brake_max(),
      cfg_.safety().rss().lateral_miu());

  rss_config_strict_as_front_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_front().response_time(),
      cfg_.safety().rss_strict_as_front().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_front().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_acc_max(),
      cfg_.safety().rss_strict_as_front().lateral_brake_min(),
      cfg_.safety().rss_strict_as_front().lateral_brake_max(),
      cfg_.safety().rss_strict_as_front().lateral_miu());

  rss_config_strict_as_rear_ = common::RssChecker::RssConfig(
      cfg_.safety().rss_strict_as_rear().response_time(),
      cfg_.safety().rss_strict_as_rear().longitudinal_acc_max(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_min(),
      cfg_.safety().rss_strict_as_rear().longitudinal_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_acc_max(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_min(),
      cfg_.safety().rss_strict_as_rear().lateral_brake_max(),
      cfg_.safety().rss_strict_as_rear().lateral_miu());

  return kSuccess;
}

ErrorType EudmPlanner::TranslateDcpActionToLonLatBehavior(
    const DcpAction& action, LateralBehavior* lat,
    LongitudinalBehavior* lon) const {
  switch (action.lat) {
    case DcpLatAction::kLaneKeeping: {
      *lat = LateralBehavior::kLaneKeeping;
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      *lat = LateralBehavior::kLaneChangeLeft;
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      *lat = LateralBehavior::kLaneChangeRight;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Lateral action translation error!";
      return kWrongStatus;
    }
  }

  switch (action.lon) {
    case DcpLonAction::kMaintain: {
      *lon = LongitudinalBehavior::kMaintain;
      break;
    }
    case DcpLonAction::kAccelerate: {
      *lon = LongitudinalBehavior::kAccelerate;
      break;
    }
    case DcpLonAction::kDecelerate: {
      *lon = LongitudinalBehavior::kDecelerate;
      break;
    }
    default: {
      LOG(ERROR) << "[Eudm]Longitudinal action translation error!";
      return kWrongStatus;
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::ClassifyActionSeq(
    const std::vector<DcpAction>& action_seq, decimal_t* operation_at_seconds,
    common::LateralBehavior* lat_behavior, bool* is_cancel_operation) const {
  decimal_t duration = 0.0;
  decimal_t operation_at = 0.0;
  bool find_lat_active_behavior = false;
  *is_cancel_operation = false;
  for (const auto& action : action_seq) {
    if (!find_lat_active_behavior) {
      if (action.lat == DcpLatAction::kLaneChangeLeft) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeLeft;
        find_lat_active_behavior = true;
      }
      if (action.lat == DcpLatAction::kLaneChangeRight) {
        *operation_at_seconds = duration;
        *lat_behavior = common::LateralBehavior::kLaneChangeRight;
        find_lat_active_behavior = true;
      }
    } else {
      if (action.lat == DcpLatAction::kLaneKeeping) {
        *is_cancel_operation = true;
      }
    }
    duration += action.t;
  }
  if (!find_lat_active_behavior) {
    *operation_at_seconds = duration + cfg_.sim().duration().layer();
    *lat_behavior = common::LateralBehavior::kLaneKeeping;
    *is_cancel_operation = false;
  }
  return kSuccess;
}

ErrorType EudmPlanner::PrepareMultiThreadContainers(const int n_sequence) {
  LOG(INFO) << "[Eudm][Process]Prepare multi-threading - " << n_sequence
            << " threads.";

  sim_res_.clear();
  sim_res_.resize(n_sequence, 0);

  risky_res_.clear();
  risky_res_.resize(n_sequence, 0);

  sim_info_.clear();
  sim_info_.resize(n_sequence, std::string(""));

  final_cost_.clear();
  final_cost_.resize(n_sequence, 0.0);

  progress_cost_.clear();
  progress_cost_.resize(n_sequence);

  tail_cost_.clear();
  tail_cost_.resize(n_sequence);

  forward_trajs_.clear();
  forward_trajs_.resize(n_sequence);

  forward_lat_behaviors_.clear();
  forward_lat_behaviors_.resize(n_sequence);

  forward_lon_behaviors_.clear();
  forward_lon_behaviors_.resize(n_sequence);

  surround_trajs_.clear();
  surround_trajs_.resize(n_sequence);
  return kSuccess;
}

ErrorType EudmPlanner::GetSurroundingForwardSimAgents(
    const common::SemanticVehicleSet& surrounding_semantic_vehicles,
    ForwardSimAgentSet* fsagents) const {
  for (const auto& psv : surrounding_semantic_vehicles.semantic_vehicles) {
    ForwardSimAgent fsagent;

    int id = psv.second.vehicle.id();
    fsagent.id = id;
    fsagent.vehicle = psv.second.vehicle;

    // * lon
    fsagent.sim_param = agent_sim_param_;

    common::State state = psv.second.vehicle.state();
    // ~ If other vehicles' acc > 0, we assume constant velocity
    if (state.acceleration >= 0) {
      fsagent.sim_param.idm_param.kDesiredVelocity = state.velocity;
    } else {
      decimal_t est_vel =
          std::max(0.0, state.velocity + state.acceleration * sim_time_total_);
      fsagent.sim_param.idm_param.kDesiredVelocity = est_vel;
    }

    // * lat
    fsagent.lat_probs = psv.second.probs_lat_behaviors;
    fsagent.lat_behavior = psv.second.lat_behavior;

    fsagent.lane = psv.second.lane;
    fsagent.stf = common::StateTransformer(fsagent.lane);

    // * other
    fsagent.lat_range = cfg_.sim().agent().cooperative_lat_range();

    fsagents->forward_sim_agents.insert(std::make_pair(id, fsagent));
  }

  return kSuccess;
}

ErrorType EudmPlanner::RunEudm() {
  // * get relevant information
  common::SemanticVehicleSet surrounding_semantic_vehicles;
  if (map_itf_->GetKeySemanticVehicles(&surrounding_semantic_vehicles) !=
      kSuccess) {
    LOG(ERROR) << "[Eudm][Fatal]fail to get key semantic vehicles. Exit";
    return kWrongStatus;
  }

  ForwardSimAgentSet surrounding_fsagents;
  GetSurroundingForwardSimAgents(surrounding_semantic_vehicles,
                                 &surrounding_fsagents);

  auto action_script = dcp_tree_ptr_->action_script();
  int n_sequence = action_script.size();

  // * prepare for multi-threading
  std::vector<std::thread> thread_set(n_sequence);
  PrepareMultiThreadContainers(n_sequence);

  // * threading
  // TODO(@lu.zhang) Use thread pool?
  TicToc timer;
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i] =
        std::thread(&EudmPlanner::SimulateActionSequence, this, ego_vehicle_,
                    surrounding_fsagents, action_script[i], i);
  }
  for (int i = 0; i < n_sequence; ++i) {
    thread_set[i].join();
  }

  LOG(INFO) << "[Eudm][Process]Multi-thread forward simulation finished!";

  // * finish multi-threading, summary simulation results
  bool sim_success = false;
  int num_valid_behaviors = 0;
  for (int i = 0; i < static_cast<int>(sim_res_.size()); ++i) {
    if (sim_res_[i] == 1) {
      sim_success = true;
      num_valid_behaviors++;
    }
  }

  for (int i = 0; i < n_sequence; ++i) {
    std::ostringstream line_info;
    line_info << "[Eudm][Result]" << i << " [";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLonActionName(a.lon);
    }
    line_info << "|";
    for (const auto& a : action_script[i]) {
      line_info << DcpTree::RetLatActionName(a.lat);
    }
    line_info << "]";
    line_info << "[s:" << sim_res_[i] << "|r:" << risky_res_[i]
              << "|c:" << std::fixed << std::setprecision(3) << final_cost_[i]
              << "]";
    line_info << " " << sim_info_[i] << "\n";
    if (sim_res_[i]) {
      line_info << "[Eudm][Result][e;s;n;w:";
      for (const auto& c : progress_cost_[i]) {
        line_info << std::fixed << std::setprecision(2)
                  << c.efficiency.ego_to_desired_vel << "_"
                  << c.efficiency.leading_to_desired_vel << ";" << c.safety.rss
                  << "_" << c.safety.occu_lane << ";"
                  << c.navigation.lane_change_preference << ";" << c.weight;
        line_info << "|";
      }
      line_info << "]";
    }
    LOG(WARNING) << line_info.str();
  }
  LOG(WARNING) << "[Eudm][Result]Sim status: " << sim_success << " with "
               << num_valid_behaviors << " behaviors.";
  if (!sim_success) {
    LOG(ERROR) << "[Eudm][Fatal]Fail to find any valid behavior. Exit";
    return kWrongStatus;
  }

  // * evaluate
  if (EvaluateMultiThreadSimResults(&winner_id_, &winner_score_) != kSuccess) {
    LOG(ERROR)
        << "[Eudm][Fatal]fail to evaluate multi-thread sim results. Exit";
    return kWrongStatus;
  }
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoBehaviorsUsingAction(
    const DcpAction& action, ForwardSimEgoAgent* ego_fsagent) const {
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    printf("[Eudm]Translate action error\n");
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;
  return kSuccess;
}

ErrorType EudmPlanner::UpdateSimSetupForScenario(
    const std::vector<DcpAction>& action_seq,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * Get the type of lateral action sequence
  common::LateralBehavior seq_lat_behavior;
  decimal_t operation_at_seconds;
  bool is_cancel_behavior;
  ClassifyActionSeq(action_seq, &operation_at_seconds, &seq_lat_behavior,
                    &is_cancel_behavior);
  ego_fsagent->operation_at_seconds = operation_at_seconds;
  ego_fsagent->is_cancel_behavior = is_cancel_behavior;
  ego_fsagent->seq_lat_behavior = seq_lat_behavior;

  // * get action sequence type
  if (is_cancel_behavior) {
    ego_fsagent->lat_behavior_longterm = LateralBehavior::kLaneKeeping;
    ego_fsagent->seq_lat_mode = LatSimMode::kChangeThenCancel;
  } else {
    if (seq_lat_behavior == LateralBehavior::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneKeep;
    } else if (action_seq.front().lat == DcpLatAction::kLaneKeeping) {
      ego_fsagent->seq_lat_mode = LatSimMode::kKeepThenChange;
    } else {
      ego_fsagent->seq_lat_mode = LatSimMode::kAlwaysLaneChange;
    }
    ego_fsagent->lat_behavior_longterm = seq_lat_behavior;
  }

  // * lon
  decimal_t desired_vel = std::floor(ego_fsagent->vehicle.state().velocity);
  simulator::IntelligentDriverModel::Param idm_param_tmp;
  idm_param_tmp = ego_sim_param_.idm_param;

  switch (action_seq[1].lon) {
    case DcpLonAction::kAccelerate: {
      idm_param_tmp.kDesiredVelocity = std::min(
          desired_vel + cfg_.sim().acc_cmd_vel_gap(), desired_velocity_);
      idm_param_tmp.kMinimumSpacing *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      idm_param_tmp.kDesiredHeadwayTime *=
          (1.0 - cfg_.sim().ego().lon_aggressive_ratio());
      break;
    }
    case DcpLonAction::kDecelerate: {
      idm_param_tmp.kDesiredVelocity =
          std::min(std::max(desired_vel - cfg_.sim().dec_cmd_vel_gap(), 0.0),
                   desired_velocity_);
      break;
    }
    case DcpLonAction::kMaintain: {
      idm_param_tmp.kDesiredVelocity = std::min(desired_vel, desired_velocity_);
      break;
    }
    default: {
      printf("[Eudm]Error - Lon action not valid\n");
      assert(false);
    }
  }
  ego_fsagent->sim_param = ego_sim_param_;
  ego_fsagent->sim_param.idm_param = idm_param_tmp;
  ego_fsagent->lat_range = cfg_.sim().ego().cooperative_lat_range();

  return kSuccess;
}

ErrorType EudmPlanner::UpdateSimSetupForLayer(
    const DcpAction& action, const ForwardSimAgentSet& other_fsagent,
    ForwardSimEgoAgent* ego_fsagent) const {
  // * action -> behavior
  LateralBehavior lat_behavior;
  LongitudinalBehavior lon_behavior;
  if (TranslateDcpActionToLonLatBehavior(action, &lat_behavior,
                                         &lon_behavior) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->lat_behavior = lat_behavior;
  ego_fsagent->lon_behavior = lon_behavior;

  // * related lanes
  auto state = ego_fsagent->vehicle.state();
  decimal_t forward_lane_len =
      std::min(std::max(state.velocity * cfg_.sim().ref_line().len_vel_coeff(),
                        cfg_.sim().ref_line().forward_len_min()),
               cfg_.sim().ref_line().forward_len_max());

  common::Lane lane_current;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), LateralBehavior::kLaneKeeping,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_current) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->current_lane = lane_current;
  ego_fsagent->current_stf = common::StateTransformer(lane_current);

  common::Lane lane_target;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_target) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->target_lane = lane_target;
  ego_fsagent->target_stf = common::StateTransformer(lane_target);

  common::Lane lane_longterm;
  if (map_itf_->GetRefLaneForStateByBehavior(
          state, std::vector<int>(), ego_fsagent->lat_behavior_longterm,
          forward_lane_len, cfg_.sim().ref_line().backward_len_max(), false,
          &lane_longterm) != kSuccess) {
    return kWrongStatus;
  }
  ego_fsagent->longterm_lane = lane_longterm;
  ego_fsagent->longterm_stf = common::StateTransformer(lane_longterm);

  // * Gap finding for lane-changing behaviors
  if (ego_fsagent->lat_behavior != LateralBehavior::kLaneKeeping) {
    common::VehicleSet other_vehicles;
    for (const auto& pv : other_fsagent.forward_sim_agents) {
      other_vehicles.vehicles.insert(
          std::make_pair(pv.first, pv.second.vehicle));
    }

    bool has_front_vehicle = false, has_rear_vehicle = false;
    common::Vehicle front_vehicle, rear_vehicle;
    common::FrenetState front_fs, rear_fs;
    map_itf_->GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        ego_fsagent->target_lane, state, other_vehicles, &has_front_vehicle,
        &front_vehicle, &front_fs, &has_rear_vehicle, &rear_vehicle, &rear_fs);

    ego_fsagent->target_gap_ids(0) =
        has_front_vehicle ? front_vehicle.id() : -1;
    ego_fsagent->target_gap_ids(1) = has_rear_vehicle ? rear_vehicle.id() : -1;

    if (cfg_.safety().rss_for_layers_enable()) {
      // * Strict RSS check here
      // * Disable the action that is apparently not valid
      common::FrenetState ego_fs;
      if (kSuccess != ego_fsagent->target_stf.GetFrenetStateFromState(
                          ego_fsagent->vehicle.state(), &ego_fs)) {
        return kWrongStatus;
      }
      decimal_t s_ego_fbumper = ego_fs.vec_s[0] +
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();
      decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                                ego_fsagent->vehicle.param().length() / 2.0 +
                                ego_fsagent->vehicle.param().d_cr();

      if (has_front_vehicle) {
        decimal_t s_front_rbumper = front_fs.vec_s[0] -
                                    front_vehicle.param().length() / 2.0 +
                                    front_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            front_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Front,
            rss_config_strict_as_rear_, &rss_dist);

        if (s_front_rbumper - s_ego_fbumper < rss_dist) {
          // violate strict RSS
          return kWrongStatus;
        }
      }

      if (has_rear_vehicle) {
        decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                   rear_vehicle.param().length() / 2.0 +
                                   rear_vehicle.param().d_cr();
        decimal_t rss_dist;
        common::RssChecker::CalculateSafeLongitudinalDistance(
            ego_fsagent->vehicle.state().velocity,
            rear_vehicle.state().velocity,
            common::RssChecker::LongitudinalDirection::Rear,
            rss_config_strict_as_front_, &rss_dist);

        if (s_ego_rbumper - s_rear_fbumper < rss_dist) {
          // violate strict RSS
          return kWrongStatus;
        }
      }
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::SimulateScenario(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id,
    const int& sub_seq_id, std::vector<int>* sub_sim_res,
    std::vector<int>* sub_risky_res, std::vector<std::string>* sub_sim_info,
    std::vector<std::vector<CostStructure>>* sub_progress_cost,
    std::vector<CostStructure>* sub_tail_cost,
    vec_E<vec_E<common::Vehicle>>* sub_forward_trajs,
    std::vector<std::vector<LateralBehavior>>* sub_forward_lat_behaviors,
    std::vector<std::vector<LongitudinalBehavior>>* sub_forward_lon_behaviors,
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>>*
        sub_surround_trajs) {
  // * declare variables which will be used to track traces from multiple layers
  vec_E<common::Vehicle> ego_traj_multilayers{ego_vehicle};

  std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multilayers;
  for (const auto& p_fsa : surrounding_fsagents.forward_sim_agents) {
    surround_trajs_multilayers.insert(std::pair<int, vec_E<common::Vehicle>>(
        p_fsa.first, vec_E<common::Vehicle>({p_fsa.second.vehicle})));
  }

  std::vector<LateralBehavior> ego_lat_behavior_multilayers;
  std::vector<LongitudinalBehavior> ego_lon_behavior_multilayers;
  std::vector<CostStructure> cost_multilayers;
  std::set<int> risky_ids_multilayers;

  // * Setup ego longitudinal sim config
  ForwardSimEgoAgent ego_fsagent_this_layer;
  ego_fsagent_this_layer.vehicle = ego_vehicle;
  UpdateSimSetupForScenario(action_seq, &ego_fsagent_this_layer);

  ForwardSimAgentSet surrounding_fsagents_this_layer = surrounding_fsagents;

  int action_ref_lane_id = ego_lane_id_;
  bool is_sub_seq_risky = false;
  std::vector<DcpAction> action_seq_sim = action_seq;

  // * For each action in action sequence
  for (int i = 0; i < static_cast<int>(action_seq_sim.size()); ++i) {
    auto action_this_layer = action_seq_sim[i];

    // For each action, we can update context here.
    // such as lane, stf, desired_vel, social_force_masks
    // For each action, the context info will not change, so we can use it in
    // every step. A low-level reactive lane-changing controller can be
    // implemented without a lot of computation cost.
    // * update setup for this layer
    if (kSuccess != UpdateSimSetupForLayer(action_this_layer,
                                           surrounding_fsagents_this_layer,
                                           &ego_fsagent_this_layer)) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Update setup F)");
      return kWrongStatus;
    }

    // TODO(lu.zhang): MOBIL/RSS check here?
    // * Disable the action that is apparently not valid
    // if (ego_lat_behavior_this_layer != LateralBehavior::kLaneKeeping) {
    //   if (!CheckLaneChangingFeasibilityUsingMobil(
    //           ego_semantic_vehicle_this_layer,
    //           semantic_vehicle_set_this_layer)) {
    //     return kWrongStatus;
    //   }
    // }

    // * simulate this action (layer)
    vec_E<common::Vehicle> ego_traj_multisteps;
    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs_multisteps;
    if (SimulateSingleAction(action_this_layer, ego_fsagent_this_layer,
                             surrounding_fsagents_this_layer,
                             &ego_traj_multisteps,
                             &surround_trajs_multisteps) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] +=
          std::string("(Sim ") + std::to_string(i) + std::string(" F)");
      return kWrongStatus;
    }

    // * update ForwardSimAgent
    ego_fsagent_this_layer.vehicle.set_state(
        ego_traj_multisteps.back().state());
    for (auto it = surrounding_fsagents_this_layer.forward_sim_agents.begin();
         it != surrounding_fsagents_this_layer.forward_sim_agents.end(); ++it) {
      it->second.vehicle.set_state(
          surround_trajs_multisteps.at(it->first).back().state());
    }

    // * enforce strict safety check
    bool is_strictly_safe = false;
    int collided_id = 0;
    TicToc timer;
    if (StrictSafetyCheck(ego_traj_multisteps, surround_trajs_multisteps,
                          &is_strictly_safe, &collided_id) != kSuccess) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Check F)");
      return kWrongStatus;
    }
    // LOG(INFO) << "[RssTime]safety check time per action: " << timer.toc();

    if (!is_strictly_safe) {
      (*sub_sim_res)[sub_seq_id] = 0;
      (*sub_sim_info)[sub_seq_id] += std::string("(Strict F:") +
                                     std::to_string(collided_id) +
                                     std::string(")");
      return kWrongStatus;
    }

    // * If lateral action finished, update simulation action sequence
    int current_lane_id;
    if (CheckIfLateralActionFinished(
            ego_fsagent_this_layer.vehicle.state(), action_ref_lane_id,
            ego_fsagent_this_layer.lat_behavior, &current_lane_id)) {
      action_ref_lane_id = current_lane_id;
      if (kSuccess != UpdateLateralActionSequence(i, &action_seq_sim)) {
        (*sub_sim_res)[sub_seq_id] = 0;
        (*sub_sim_info)[sub_seq_id] += std::string("(Update Lat F)");
        return kWrongStatus;
      }
      ego_fsagent_this_layer.lat_behavior_longterm =
          LateralBehavior::kLaneKeeping;
    }

    // * trace
    ego_traj_multilayers.insert(ego_traj_multilayers.end(),
                                ego_traj_multisteps.begin(),
                                ego_traj_multisteps.end());
    ego_lat_behavior_multilayers.push_back(ego_fsagent_this_layer.lat_behavior);
    ego_lon_behavior_multilayers.push_back(ego_fsagent_this_layer.lon_behavior);

    for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
      int id = v.first;
      surround_trajs_multilayers.at(id).insert(
          surround_trajs_multilayers.at(id).end(),
          surround_trajs_multisteps.at(id).begin(),
          surround_trajs_multisteps.at(id).end());
    }

    CostStructure cost;
    bool verbose = false;
    std::set<int> risky_ids;
    bool is_risky_action = false;
    CostFunction(action_this_layer, ego_fsagent_this_layer,
                 surrounding_fsagents_this_layer, ego_traj_multisteps,
                 surround_trajs_multisteps, verbose, &cost, &is_risky_action,
                 &risky_ids);
    if (is_risky_action) {
      is_sub_seq_risky = true;
      for (const auto& id : risky_ids) {
        risky_ids_multilayers.insert(id);
      }
    }
    cost.weight = cost.weight * pow(cfg_.cost().discount_factor(), i);
    cost.valid_sample_index_ub = ego_traj_multilayers.size();
    cost_multilayers.push_back(cost);
  }

  (*sub_sim_res)[sub_seq_id] = 1;
  (*sub_risky_res)[sub_seq_id] = is_sub_seq_risky ? 1 : 0;
  if (is_sub_seq_risky) {
    std::string risky_id_list;
    for (auto it = risky_ids_multilayers.begin();
         it != risky_ids_multilayers.end(); it++) {
      risky_id_list += " " + std::to_string(*it);
    }
    (*sub_sim_info)[sub_seq_id] += std::string("(Risky)") + risky_id_list;
  }
  (*sub_progress_cost)[sub_seq_id] = cost_multilayers;
  (*sub_forward_trajs)[sub_seq_id] = ego_traj_multilayers;
  (*sub_forward_lat_behaviors)[sub_seq_id] = ego_lat_behavior_multilayers;
  (*sub_forward_lon_behaviors)[sub_seq_id] = ego_lon_behavior_multilayers;
  (*sub_surround_trajs)[sub_seq_id] = surround_trajs_multilayers;

  return kSuccess;
}

ErrorType EudmPlanner::SimulateActionSequence(
    const common::Vehicle& ego_vehicle,
    const ForwardSimAgentSet& surrounding_fsagents,
    const std::vector<DcpAction>& action_seq, const int& seq_id) {
  if (pre_deleted_seq_ids_.find(seq_id) != pre_deleted_seq_ids_.end()) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = std::string("(Pre-deleted)");
    return kWrongStatus;
  }

  // ~ For each ego sequence, we may further branch here, which will create
  // ~ multiple sub threads. Currently, we use n_sub_threads = 1
  // TODO(@lu.zhang) Preliminary safety assessment here
  int n_sub_threads = 1;

  std::vector<int> sub_sim_res(n_sub_threads);
  std::vector<int> sub_risky_res(n_sub_threads);
  std::vector<std::string> sub_sim_info(n_sub_threads);
  std::vector<std::vector<CostStructure>> sub_progress_cost(n_sub_threads);
  std::vector<CostStructure> sub_tail_cost(n_sub_threads);
  vec_E<vec_E<common::Vehicle>> sub_forward_trajs(n_sub_threads);
  std::vector<std::vector<LateralBehavior>> sub_forward_lat_behaviors(
      n_sub_threads);
  std::vector<std::vector<LongitudinalBehavior>> sub_forward_lon_behaviors(
      n_sub_threads);
  vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> sub_surround_trajs(
      n_sub_threads);

  SimulateScenario(ego_vehicle, surrounding_fsagents, action_seq, seq_id, 0,
                   &sub_sim_res, &sub_risky_res, &sub_sim_info,
                   &sub_progress_cost, &sub_tail_cost, &sub_forward_trajs,
                   &sub_forward_lat_behaviors, &sub_forward_lon_behaviors,
                   &sub_surround_trajs);

  if (sub_sim_res.front() == 0) {
    sim_res_[seq_id] = 0;
    sim_info_[seq_id] = sub_sim_info.front();
    return kWrongStatus;
  }

  // ~ Here use the default scenario
  sim_res_[seq_id] = 1;
  risky_res_[seq_id] = sub_risky_res.front();
  sim_info_[seq_id] = sub_sim_info.front();
  progress_cost_[seq_id] = sub_progress_cost.front();
  tail_cost_[seq_id] = sub_tail_cost.front();
  forward_trajs_[seq_id] = sub_forward_trajs.front();
  forward_lat_behaviors_[seq_id] = sub_forward_lat_behaviors.front();
  forward_lon_behaviors_[seq_id] = sub_forward_lon_behaviors.front();
  surround_trajs_[seq_id] = sub_surround_trajs.front();

  return kSuccess;
}

ErrorType EudmPlanner::UpdateLateralActionSequence(
    const int cur_idx, std::vector<DcpAction>* action_seq) const {
  if (cur_idx == static_cast<int>(action_seq->size()) - 1) {
    return kSuccess;
  }

  switch ((*action_seq)[cur_idx].lat) {
    case DcpLatAction::kLaneKeeping: {
      // * no need to update
      break;
    }
    case DcpLatAction::kLaneChangeLeft: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * LLLLL -> LLKKK
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // * LLKKK -> LLRRR
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeRight;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * LLRRR -> x
          return kWrongStatus;
        }
      }
      break;
    }
    case DcpLatAction::kLaneChangeRight: {
      for (int i = cur_idx + 1; i < static_cast<int>(action_seq->size()); ++i) {
        if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeRight) {
          // * RRRRR -> RRKKK
          (*action_seq)[i].lat = DcpLatAction::kLaneKeeping;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneKeeping) {
          // * RRKKK -> RRLLL
          (*action_seq)[i].lat = DcpLatAction::kLaneChangeLeft;
        } else if ((*action_seq)[i].lat == DcpLatAction::kLaneChangeLeft) {
          // * RRLLL -> x
          return kWrongStatus;
        }
      }
      break;
    }

    default: {
      std::cout << "[Eudm]Error - Invalid lateral behavior" << std::endl;
      assert(false);
    }
  }

  return kSuccess;
}

bool EudmPlanner::CheckIfLateralActionFinished(
    const common::State& cur_state, const int& action_ref_lane_id,
    const LateralBehavior& lat_behavior, int* current_lane_id) const {
  if (lat_behavior == LateralBehavior::kLaneKeeping) {
    return false;
  }

  decimal_t current_lane_dist;
  decimal_t arc_len;
  map_itf_->GetNearestLaneIdUsingState(cur_state.ToXYTheta(),
                                       std::vector<int>(), current_lane_id,
                                       &current_lane_dist, &arc_len);
  // std::cout << "[Eudm] current_lane_id: " << *current_lane_id << std::endl;
  // std::cout << "[Eudm] action_ref_lane_id: " << action_ref_lane_id <<
  // std::endl;

  std::vector<int> potential_lane_ids;
  GetPotentialLaneIds(action_ref_lane_id, lat_behavior, &potential_lane_ids);

  // ~ Lane change
  auto it = std::find(potential_lane_ids.begin(), potential_lane_ids.end(),
                      *current_lane_id);
  if (it == potential_lane_ids.end()) {
    return false;
  } else {
    return true;
  }
}

ErrorType EudmPlanner::RunOnce() {
  TicToc timer_runonce;
  // * Get current nearest lane id
  if (!map_itf_) {
    LOG(ERROR) << "[Eudm]map interface not initialized. Exit";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Eudm]no ego vehicle found.";
    return kWrongStatus;
  }
  ego_id_ = ego_vehicle_.id();
  time_stamp_ = ego_vehicle_.state().time_stamp;

  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Eudm]------ Eudm Cycle Begins (stamp): " << time_stamp_
               << " ------- ";

  int ego_lane_id_by_pos = kInvalidLaneId;
  if (map_itf_->GetEgoLaneIdByPosition(std::vector<int>(),
                                       &ego_lane_id_by_pos) != kSuccess) {
    LOG(ERROR) << "[Eudm]Fatal (Exit) ego not on lane.";
    return kWrongStatus;
  }
  LOG(WARNING) << std::fixed << std::setprecision(3)
               << "[Eudm][Input]Ego plan state (x,y,theta,v,a,k):("
               << ego_vehicle_.state().vec_position[0] << ","
               << ego_vehicle_.state().vec_position[1] << ","
               << ego_vehicle_.state().angle << ","
               << ego_vehicle_.state().velocity << ","
               << ego_vehicle_.state().acceleration << ","
               << ego_vehicle_.state().curvature << ")"
               << " lane id:" << ego_lane_id_by_pos;
  LOG(WARNING) << "[Eudm][Setup]Desired vel:" << desired_velocity_
               << " sim_time total:" << sim_time_total_
               << " lc info[f_l,f_r,us_ol,us_or,solid_l,solid_r]:"
               << lc_info_.forbid_lane_change_left << ","
               << lc_info_.forbid_lane_change_right << ","
               << lc_info_.lane_change_left_unsafe_by_occu << ","
               << lc_info_.lane_change_right_unsafe_by_occu << ","
               << lc_info_.left_solid_lane << "," << lc_info_.right_solid_lane;
  ego_lane_id_ = ego_lane_id_by_pos;

  const decimal_t forward_rss_check_range = 130.0;
  const decimal_t backward_rss_check_range = 130.0;
  const decimal_t forward_lane_len = forward_rss_check_range;
  const decimal_t backward_lane_len = backward_rss_check_range;
  if (map_itf_->GetRefLaneForStateByBehavior(
          ego_vehicle_.state(), std::vector<int>(),
          LateralBehavior::kLaneKeeping, forward_lane_len, backward_lane_len,
          false, &rss_lane_) != kSuccess) {
    LOG(ERROR) << "[Eudm]No Rss lane available. Rss disabled";
  }

  if (rss_lane_.IsValid()) {
    rss_stf_ = common::StateTransformer(rss_lane_);
  }

  pre_deleted_seq_ids_.clear();
  int n_sequence = dcp_tree_ptr_->action_script().size();
  for (int i = 0; i < n_sequence; i++) {
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    int num_actions = action_seq.size();
    for (int j = 1; j < num_actions; j++) {
      if ((action_seq[j - 1].lat == DcpLatAction::kLaneChangeLeft &&
           action_seq[j].lat == DcpLatAction::kLaneChangeRight) ||
          (action_seq[j - 1].lat == DcpLatAction::kLaneChangeRight &&
           action_seq[j].lat == DcpLatAction::kLaneChangeLeft)) {
        pre_deleted_seq_ids_.insert(i);
      }
    }
  }

  TicToc timer;
  if (RunEudm() != kSuccess) {
    LOG(ERROR) << std::fixed << std::setprecision(4)
               << "[Eudm]****** Eudm Cycle FAILED (stamp): " << time_stamp_
               << " time cost " << timer.toc() << " ms.";
    return kWrongStatus;
  }
  auto action_script = dcp_tree_ptr_->action_script();
  std::ostringstream line_info;
  line_info << "[Eudm]SUCCESS id:" << winner_id_ << " [";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLonActionName(a.lon);
  }
  line_info << "|";
  for (const auto& a : action_script[winner_id_]) {
    line_info << DcpTree::RetLatActionName(a.lat);
  }
  line_info << "] cost: " << std::fixed << std::setprecision(3) << winner_score_
            << " time cost: " << timer.toc() << " ms.";
  LOG(WARNING) << line_info.str();

  time_cost_ = timer_runonce.toc();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSinglePolicyTrajs(
    const std::vector<CostStructure>& progress_cost,
    const CostStructure& tail_cost, const std::vector<DcpAction>& action_seq,
    decimal_t* score) {
  decimal_t score_tmp = 0.0;
  for (const auto& c : progress_cost) {
    score_tmp += c.ave();
  }
  *score = score_tmp + tail_cost.ave();
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateMultiThreadSimResults(int* winner_id,
                                                     decimal_t* winner_cost) {
  decimal_t min_cost = kInf;
  int best_id = 0;
  int num_sequences = sim_res_.size();
  for (int i = 0; i < num_sequences; ++i) {
    if (sim_res_[i] == 0) {
      continue;
    }
    decimal_t cost = 0.0;
    auto action_seq = dcp_tree_ptr_->action_script()[i];
    EvaluateSinglePolicyTrajs(progress_cost_[i], tail_cost_[i], action_seq,
                              &cost);
    final_cost_[i] = cost;
    if (cost < min_cost) {
      min_cost = cost;
      best_id = i;
    }
  }
  *winner_cost = min_cost;
  *winner_id = best_id;
  return kSuccess;
}

ErrorType EudmPlanner::EvaluateSafetyStatus(
    const vec_E<common::Vehicle>& traj_a, const vec_E<common::Vehicle>& traj_b,
    decimal_t* cost, bool* is_rss_safe, int* risky_id) {
  if (traj_a.size() != traj_b.size()) {
    return kWrongStatus;
  }
  if (!cfg_.safety().rss_check_enable() || !rss_lane_.IsValid()) {
    return kSuccess;
  }
  int num_states = static_cast<int>(traj_a.size());
  decimal_t cost_tmp = 0.0;
  bool ret_is_rss_safe = true;
  const int check_per_state = 1;
  for (int i = 0; i < num_states; i += check_per_state) {
    bool is_rss_safe = true;
    common::RssChecker::LongitudinalViolateType type;
    decimal_t rss_vel_low, rss_vel_up;
    common::RssChecker::RssCheck(traj_a[i], traj_b[i], rss_stf_, rss_config_,
                                 &is_rss_safe, &type, &rss_vel_low,
                                 &rss_vel_up);
    if (!is_rss_safe) {
      ret_is_rss_safe = false;
      *risky_id = traj_b.size() ? traj_b[0].id() : 0;
      if (cfg_.cost().safety().rss_cost_enable()) {
        if (type == common::RssChecker::LongitudinalViolateType::TooFast) {
          cost_tmp +=
              cfg_.cost().safety().rss_over_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_over_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_up));
        } else if (type ==
                   common::RssChecker::LongitudinalViolateType::TooSlow) {
          cost_tmp +=
              cfg_.cost().safety().rss_lack_speed_linear_coeff() *
              traj_a[i].state().velocity *
              pow(10, cfg_.cost().safety().rss_lack_speed_power_coeff() *
                          fabs(traj_a[i].state().velocity - rss_vel_low));
        }
      }
    }
  }
  *is_rss_safe = ret_is_rss_safe;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::StrictSafetyCheck(
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool* is_safe, int* collided_id) {
  if (!cfg_.safety().strict_check_enable()) {
    *is_safe = true;
    return kSuccess;
  }

  int num_points_ego = ego_traj.size();
  if (num_points_ego == 0) {
    *is_safe = true;
    return kSuccess;
  }
  // strict collision check
  for (auto it = surround_trajs.begin(); it != surround_trajs.end(); it++) {
    int num_points_other = it->second.size();
    if (num_points_other != num_points_ego) {
      *is_safe = false;
      LOG(ERROR) << "[Eudm]unsafe due to incomplete sim record for vehicle";
      return kSuccess;
    }
    for (int i = 0; i < num_points_ego; i++) {
      common::Vehicle inflated_a, inflated_b;
      common::SemanticsUtils::InflateVehicleBySize(
          ego_traj[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_a);
      common::SemanticsUtils::InflateVehicleBySize(
          it->second[i], cfg_.safety().strict().inflation_w(),
          cfg_.safety().strict().inflation_h(), &inflated_b);
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(inflated_a.param(), inflated_a.state(),
                                         inflated_b.param(), inflated_b.state(),
                                         &is_collision);
      if (is_collision) {
        *is_safe = false;
        *collided_id = it->second[i].id();
        return kSuccess;
      }
    }
  }
  *is_safe = true;
  return kSuccess;
}

ErrorType EudmPlanner::CostFunction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent,
    const ForwardSimAgentSet& other_fsagent,
    const vec_E<common::Vehicle>& ego_traj,
    const std::unordered_map<int, vec_E<common::Vehicle>>& surround_trajs,
    bool verbose, CostStructure* cost, bool* is_risky,
    std::set<int>* risky_ids) {
  decimal_t duration = action.t;

  auto ego_lon_behavior_this_layer = ego_fsagent.lon_behavior;
  auto ego_lat_behavior_this_layer = ego_fsagent.lat_behavior;

  auto seq_lat_behavior = ego_fsagent.seq_lat_behavior;
  auto is_cancel_behavior = ego_fsagent.is_cancel_behavior;

  common::VehicleSet vehicle_set;
  for (const auto& v : other_fsagent.forward_sim_agents) {
    vehicle_set.vehicles.insert(std::make_pair(v.first, v.second.vehicle));
  }

  decimal_t ego_velocity = ego_fsagent.vehicle.state().velocity;
  // f = c1 * fabs(v_ego - v_user), if v_ego < v_user
  // f = c2 * fabs(v_ego - v_user - vth), if v_ego > v_user + vth
  // unit of this cost is velocity (finally multiplied by duration)
  CostStructure cost_tmp;
  if (ego_fsagent.vehicle.state().velocity < desired_velocity_) {
    cost_tmp.efficiency.ego_to_desired_vel =
        cfg_.cost().effciency().ego_lack_speed_to_desired_unit_cost() *
        fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_);
  } else {
    if (ego_fsagent.vehicle.state().velocity >
        desired_velocity_ +
            cfg_.cost().effciency().ego_desired_speed_tolerate_gap()) {
      cost_tmp.efficiency.ego_to_desired_vel =
          cfg_.cost().effciency().ego_over_speed_to_desired_unit_cost() *
          fabs(ego_fsagent.vehicle.state().velocity - desired_velocity_ -
               cfg_.cost().effciency().ego_desired_speed_tolerate_gap());
    }
  }

  // f = ratio * c1 * fabs(v_ego - v_user) , if v_ego < v_user && v_ego
  // > v_leading
  // unit of this cost is velocity (finally multiplied by duration)
  common::Vehicle leading_vehicle;
  decimal_t distance_residual_ratio = 0.0;
  if (map_itf_->GetLeadingVehicleOnLane(
          ego_fsagent.target_lane, ego_fsagent.vehicle.state(), vehicle_set,
          ego_fsagent.lat_range, &leading_vehicle,
          &distance_residual_ratio) == kSuccess) {
    decimal_t distance_to_leading_vehicle =
        (leading_vehicle.state().vec_position -
         ego_fsagent.vehicle.state().vec_position)
            .norm();
    if (ego_fsagent.vehicle.state().velocity < desired_velocity_ &&
        leading_vehicle.state().velocity < desired_velocity_ &&
        distance_to_leading_vehicle <
            cfg_.cost().effciency().leading_distance_th()) {
      decimal_t ego_blocked_by_leading_velocity =
          ego_fsagent.vehicle.state().velocity >
                  leading_vehicle.state().velocity
              ? ego_fsagent.vehicle.state().velocity -
                    leading_vehicle.state().velocity
              : 0.0;
      decimal_t leading_to_desired_velocity =
          leading_vehicle.state().velocity < desired_velocity_
              ? desired_velocity_ - leading_vehicle.state().velocity
              : 0.0;
      cost_tmp.efficiency.leading_to_desired_vel =
          std::max(cfg_.cost().effciency().min_distance_ratio(),
                   distance_residual_ratio) *
          (cfg_.cost().effciency().ego_speed_blocked_by_leading_unit_cost() *
               ego_blocked_by_leading_velocity +
           cfg_.cost()
                   .effciency()
                   .leading_speed_blocked_desired_vel_unit_cost() *
               leading_to_desired_velocity);
    }
  }

  // * safety
  for (const auto& surround_traj : surround_trajs) {
    decimal_t safety_cost = 0.0;
    bool is_safe = true;
    int risky_id = 0;
    EvaluateSafetyStatus(ego_traj, surround_traj.second, &safety_cost, &is_safe,
                         &risky_id);
    if (!is_safe) {
      risky_ids->insert(risky_id);
      *is_risky = true;
    }
    cost_tmp.safety.rss += safety_cost;
  }

  if (cfg_.cost().safety().occu_lane_enable()) {
    if (lc_info_.forbid_lane_change_left &&
        seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    } else if (lc_info_.forbid_lane_change_right &&
               seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
      cost_tmp.safety.occu_lane =
          ego_velocity * cfg_.cost().safety().occu_lane_unit_cost();
    }
  }

  // * navigation
  if (seq_lat_behavior == LateralBehavior::kLaneChangeLeft ||
      seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
    if (is_cancel_behavior) {
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          cfg_.cost().user().cancel_operation_unit_cost();
    } else {
      cost_tmp.navigation.lane_change_preference =
          std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                   ego_velocity) *
          (seq_lat_behavior == LateralBehavior::kLaneChangeLeft
               ? cfg_.cost().navigation().lane_change_left_unit_cost()
               : cfg_.cost().navigation().lane_change_right_unit_cost());
      if (lc_info_.recommend_lc_left &&
          seq_lat_behavior == LateralBehavior::kLaneChangeLeft) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_left_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeLeft) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      } else if (lc_info_.recommend_lc_right &&
                 seq_lat_behavior == LateralBehavior::kLaneChangeRight) {
        cost_tmp.navigation.lane_change_preference =
            -std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                      ego_velocity) *
            cfg_.cost().navigation().lane_change_right_recommendation_reward();
        if (action.lat != DcpLatAction::kLaneChangeRight) {
          cost_tmp.navigation.lane_change_preference +=
              std::max(cfg_.cost().navigation().lane_change_unit_cost_vel_lb(),
                       ego_velocity) *
              cfg_.cost().user().late_operate_unit_cost();
        }
      }
    }
  }
  cost_tmp.weight = duration;
  *cost = cost_tmp;
  return kSuccess;
}

ErrorType EudmPlanner::GetSimTimeSteps(const DcpAction& action,
                                       std::vector<decimal_t>* dt_steps) const {
  decimal_t sim_time_resolution = cfg_.sim().duration().step();
  decimal_t sim_time_total = action.t;
  int n_1 = std::floor(sim_time_total / sim_time_resolution);
  decimal_t dt_remain = sim_time_total - n_1 * sim_time_resolution;
  std::vector<decimal_t> steps(n_1, sim_time_resolution);
  if (fabs(dt_remain) > kEPS) {
    steps.insert(steps.begin(), dt_remain);
  }
  *dt_steps = steps;

  return kSuccess;
}

ErrorType EudmPlanner::SimulateSingleAction(
    const DcpAction& action, const ForwardSimEgoAgent& ego_fsagent_this_layer,
    const ForwardSimAgentSet& surrounding_fsagents_this_layer,
    vec_E<common::Vehicle>* ego_traj,
    std::unordered_map<int, vec_E<common::Vehicle>>* surround_trajs) {
  // ~ Prepare containers
  ego_traj->clear();
  surround_trajs->clear();
  for (const auto& v : surrounding_fsagents_this_layer.forward_sim_agents) {
    surround_trajs->insert(std::pair<int, vec_E<common::Vehicle>>(
        v.first, vec_E<common::Vehicle>()));
  }

  // ~ Simulation time steps
  std::vector<decimal_t> dt_steps;
  GetSimTimeSteps(action, &dt_steps);

  ForwardSimEgoAgent ego_fsagent_this_step = ego_fsagent_this_layer;
  ForwardSimAgentSet surrounding_fsagents_this_step =
      surrounding_fsagents_this_layer;

  for (int i = 0; i < static_cast<int>(dt_steps.size()); i++) {
    decimal_t sim_time_step = dt_steps[i];

    common::State ego_state_cache_this_step;
    std::unordered_map<int, State>
        others_state_cache_this_step;  // id - state_output

    common::VehicleSet all_sim_vehicles;  // include ego vehicle
    all_sim_vehicles.vehicles.insert(std::make_pair(
        ego_fsagent_this_step.vehicle.id(), ego_fsagent_this_step.vehicle));
    for (const auto& v : surrounding_fsagents_this_step.forward_sim_agents) {
      all_sim_vehicles.vehicles.insert(
          std::make_pair(v.first, v.second.vehicle));
    }

    // * For ego agent
    {
      all_sim_vehicles.vehicles.at(ego_id_).set_id(kInvalidAgentId);

      common::State state_output;
      if (kSuccess != EgoAgentForwardSim(ego_fsagent_this_step,
                                         all_sim_vehicles, sim_time_step,
                                         &state_output)) {
        return kWrongStatus;
      }

      common::Vehicle v_tmp = ego_fsagent_this_step.vehicle;
      v_tmp.set_state(state_output);
      ego_traj->push_back(v_tmp);
      ego_state_cache_this_step = state_output;

      all_sim_vehicles.vehicles.at(ego_id_).set_id(ego_id_);
    }

    // * For surrounding agents
    {
      for (const auto& p_fsa :
           surrounding_fsagents_this_step.forward_sim_agents) {
        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(kInvalidAgentId);

        common::State state_output;
        if (kSuccess !=
            SurroundingAgentForwardSim(p_fsa.second, all_sim_vehicles,
                                       sim_time_step, &state_output)) {
          return kWrongStatus;
        }

        common::Vehicle v_tmp = p_fsa.second.vehicle;
        v_tmp.set_state(state_output);
        surround_trajs->at(p_fsa.first).push_back(v_tmp);
        others_state_cache_this_step.insert(
            std::make_pair(p_fsa.first, state_output));

        all_sim_vehicles.vehicles.at(p_fsa.first).set_id(p_fsa.first);
      }
    }

    // * update sim state after steps
    ego_fsagent_this_step.vehicle.set_state(ego_state_cache_this_step);
    for (const auto& ps : others_state_cache_this_step) {
      surrounding_fsagents_this_step.forward_sim_agents.at(ps.first)
          .vehicle.set_state(ps.second);
    }
  }

  return kSuccess;
}

ErrorType EudmPlanner::SurroundingAgentForwardSim(
    const ForwardSimAgent& fsagent, const common::VehicleSet& all_sim_vehicles,
    const decimal_t& sim_time_step, common::State* state_out) const {
  common::Vehicle leading_vehicle;
  common::State state_output;
  decimal_t distance_residual_ratio = 0.0;
  map_itf_->GetLeadingVehicleOnLane(fsagent.lane, fsagent.vehicle.state(),
                                    all_sim_vehicles, fsagent.lat_range,
                                    &leading_vehicle, &distance_residual_ratio);
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          fsagent.stf, fsagent.vehicle, leading_vehicle, sim_time_step,
          fsagent.sim_param, &state_output) != kSuccess) {
    return kWrongStatus;
  }
  *state_out = state_output;
  return kSuccess;
}

ErrorType EudmPlanner::EgoAgentForwardSim(
    const ForwardSimEgoAgent& ego_fsagent,
    const common::VehicleSet& all_sim_vehicles, const decimal_t& sim_time_step,
    common::State* state_out) const {
  common::State state_output;

  if (ego_fsagent.lat_behavior == LateralBehavior::kLaneKeeping) {
    // * Lane keeping, only consider leading vehicle on ego lane
    common::Vehicle leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.target_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // ~ with leading vehicle
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          leading_vehicle.param(), leading_vehicle.state(), &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    // TODO(lu.zhang): consider lateral social force to get lateral offset
    decimal_t lat_track_offset = 0.0;
    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLK(
            ego_fsagent.target_stf, ego_fsagent.vehicle, leading_vehicle,
            lat_track_offset, sim_time_step, ego_fsagent.sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  } else {
    // * Lane changing, consider multiple vehicles
    common::Vehicle current_leading_vehicle;
    decimal_t distance_residual_ratio = 0.0;
    if (map_itf_->GetLeadingVehicleOnLane(
            ego_fsagent.current_lane, ego_fsagent.vehicle.state(),
            all_sim_vehicles, ego_fsagent.lat_range, &current_leading_vehicle,
            &distance_residual_ratio) == kSuccess) {
      // ~ with leading vehicle
      bool is_collision = false;
      map_itf_->CheckCollisionUsingState(
          ego_fsagent.vehicle.param(), ego_fsagent.vehicle.state(),
          current_leading_vehicle.param(), current_leading_vehicle.state(),
          &is_collision);
      if (is_collision) {
        return kWrongStatus;
      }
    }

    common::Vehicle gap_front_vehicle;
    if (ego_fsagent.target_gap_ids(0) != -1) {
      gap_front_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(0));
    }
    common::Vehicle gap_rear_vehicle;
    if (ego_fsagent.target_gap_ids(1) != -1) {
      gap_rear_vehicle =
          all_sim_vehicles.vehicles.at(ego_fsagent.target_gap_ids(1));
    }

    // TODO(lu.zhang): consider lateral social force to get lateral offset
    decimal_t lat_track_offset = 0.0;
    auto sim_param = ego_fsagent.sim_param;

    if (gap_rear_vehicle.id() != kInvalidAgentId &&
        cfg_.sim().ego().evasive().evasive_enable()) {
      common::FrenetState ego_on_tarlane_fs;
      common::FrenetState rear_on_tarlane_fs;
      if (ego_fsagent.target_stf.GetFrenetStateFromState(
              ego_fsagent.vehicle.state(), &ego_on_tarlane_fs) == kSuccess &&
          ego_fsagent.target_stf.GetFrenetStateFromState(
              gap_rear_vehicle.state(), &rear_on_tarlane_fs) == kSuccess) {
        // * rss check for evasive behavior
        bool is_rss_safe = true;
        common::RssChecker::LongitudinalViolateType type;
        decimal_t rss_vel_low, rss_vel_up;
        common::RssChecker::RssCheck(ego_fsagent.vehicle, gap_rear_vehicle,
                                     ego_fsagent.target_stf,
                                     rss_config_strict_as_rear_, &is_rss_safe,
                                     &type, &rss_vel_low, &rss_vel_up);
        if (!is_rss_safe) {
          if (type == common::RssChecker::LongitudinalViolateType::TooSlow) {
            sim_param.idm_param.kDesiredVelocity = std::max(
                sim_param.idm_param.kDesiredVelocity,
                rss_vel_low + cfg_.sim().ego().evasive().lon_extraspeed());
            sim_param.idm_param.kDesiredHeadwayTime =
                cfg_.sim().ego().evasive().head_time();
            sim_param.idm_param.kAcceleration =
                cfg_.sim().ego().evasive().lon_acc();
            sim_param.max_lon_acc_jerk = cfg_.sim().ego().evasive().lon_jerk();
          }
        }
        if (cfg_.sim().ego().evasive().virtual_barrier_enable()) {
          if (ego_on_tarlane_fs.vec_s[0] - rear_on_tarlane_fs.vec_s[0] <
              gap_rear_vehicle.param().length() +
                  cfg_.sim().ego().evasive().virtual_barrier_tic() *
                      gap_rear_vehicle.state().velocity) {
            lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
          }
          common::FrenetState front_on_tarlane_fs;
          if (gap_front_vehicle.id() != kInvalidAgentId &&
              ego_fsagent.target_stf.GetFrenetStateFromState(
                  gap_front_vehicle.state(), &front_on_tarlane_fs) ==
                  kSuccess) {
            if (front_on_tarlane_fs.vec_s[0] - ego_on_tarlane_fs.vec_s[0] <
                ego_fsagent.vehicle.param().length() +
                    cfg_.sim().ego().evasive().virtual_barrier_tic() *
                        ego_fsagent.vehicle.state().velocity) {
              lat_track_offset = ego_on_tarlane_fs.vec_dt[0];
            }
          }
        }
      }
    }

    if (planning::OnLaneForwardSimulation::PropagateOnceAdvancedLC(
            ego_fsagent.current_stf, ego_fsagent.target_stf,
            ego_fsagent.vehicle, current_leading_vehicle, gap_front_vehicle,
            gap_rear_vehicle, lat_track_offset, sim_time_step, sim_param,
            &state_output) != kSuccess) {
      return kWrongStatus;
    }
  }

  *state_out = state_output;

  return kSuccess;
}

ErrorType EudmPlanner::JudgeBehaviorByLaneId(
    const int ego_lane_id_by_pos, LateralBehavior* behavior_by_lane_id) {
  if (ego_lane_id_by_pos == ego_lane_id_) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  auto it = std::find(potential_lk_lane_ids_.begin(),
                      potential_lk_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcl = std::find(potential_lcl_lane_ids_.begin(),
                          potential_lcl_lane_ids_.end(), ego_lane_id_by_pos);
  auto it_lcr = std::find(potential_lcr_lane_ids_.begin(),
                          potential_lcr_lane_ids_.end(), ego_lane_id_by_pos);

  if (it != potential_lk_lane_ids_.end()) {
    // ~ if routing information is available, here
    // ~ we still need to check whether the change is consist with the
    *behavior_by_lane_id = common::LateralBehavior::kLaneKeeping;
    return kSuccess;
  }

  if (it_lcl != potential_lcl_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeLeft;
    return kSuccess;
  }

  if (it_lcr != potential_lcr_lane_ids_.end()) {
    *behavior_by_lane_id = common::LateralBehavior::kLaneChangeRight;
    return kSuccess;
  }

  *behavior_by_lane_id = common::LateralBehavior::kUndefined;
  return kSuccess;
}

ErrorType EudmPlanner::UpdateEgoLaneId(const int new_ego_lane_id) {
  ego_lane_id_ = new_ego_lane_id;
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneKeeping,
  //                     &potential_lk_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_, common::LateralBehavior::kLaneChangeLeft,
  //                     &potential_lcl_lane_ids_);
  // GetPotentialLaneIds(ego_lane_id_,
  // common::LateralBehavior::kLaneChangeRight,
  //                     &potential_lcr_lane_ids_);
  return kSuccess;
}

ErrorType EudmPlanner::GetPotentialLaneIds(
    const int source_lane_id, const LateralBehavior& beh,
    std::vector<int>* candidate_lane_ids) const {
  candidate_lane_ids->clear();
  if (beh == common::LateralBehavior::kUndefined ||
      beh == common::LateralBehavior::kLaneKeeping) {
    map_itf_->GetChildLaneIds(source_lane_id, candidate_lane_ids);
  } else if (beh == common::LateralBehavior::kLaneChangeLeft) {
    int l_lane_id;
    if (map_itf_->GetLeftLaneId(source_lane_id, &l_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(l_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(l_lane_id);
    }
  } else if (beh == common::LateralBehavior::kLaneChangeRight) {
    int r_lane_id;
    if (map_itf_->GetRightLaneId(source_lane_id, &r_lane_id) == kSuccess) {
      map_itf_->GetChildLaneIds(r_lane_id, candidate_lane_ids);
      candidate_lane_ids->push_back(r_lane_id);
    }
  } else {
    assert(false);
  }
  return kSuccess;
}

void EudmPlanner::set_map_interface(EudmPlannerMapItf* itf) { map_itf_ = itf; }

void EudmPlanner::set_desired_velocity(const decimal_t desired_vel) {
  desired_velocity_ = std::max(0.0, desired_vel);
}

void EudmPlanner::set_lane_change_info(const LaneChangeInfo& lc_info) {
  lc_info_ = lc_info;
}

decimal_t EudmPlanner::desired_velocity() const { return desired_velocity_; }

int EudmPlanner::winner_id() const { return winner_id_; }

decimal_t EudmPlanner::time_cost() const { return time_cost_; }

void EudmPlanner::UpdateDcpTree(const DcpAction& ongoing_action) {
  dcp_tree_ptr_->set_ongoing_action(ongoing_action);
  dcp_tree_ptr_->UpdateScript();
  sim_time_total_ = dcp_tree_ptr_->planning_horizon();
}

EudmPlannerMapItf* EudmPlanner::map_itf() const { return map_itf_; }

}  // namespace planning