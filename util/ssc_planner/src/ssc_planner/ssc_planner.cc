/**
 * @file ssc_planner.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#include "ssc_planner/ssc_planner.h"

#include <glog/logging.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <omp.h>

// ! Performance can be significantly influenced by multi-core task scheduling
#define USE_OPENMP 0

namespace planning {

std::string SscPlanner::Name() { return std::string("ssc_planner"); }

ErrorType SscPlanner::Init(const std::string config_path) {
  ReadConfig(config_path);

  // * Planner config
  printf("\nSscPlanner Config:\n");
  printf(" -- weight_proximity: %lf\n", cfg_.planner_cfg().weight_proximity());

  LOG(INFO) << "[Ssc]SscPlanner Config:";
  LOG(INFO) << "[Ssc] -- low spd threshold: "
            << cfg_.planner_cfg().low_speed_threshold();
  LOG(INFO) << "[Ssc] -- weight_proximity: "
            << cfg_.planner_cfg().weight_proximity();

  // * SscMap config
  SscMap::Config map_cfg;
  map_cfg.map_size[0] = cfg_.map_cfg().map_size_x();
  map_cfg.map_size[1] = cfg_.map_cfg().map_size_y();
  map_cfg.map_size[2] = cfg_.map_cfg().map_size_z();
  map_cfg.map_resolution[0] = cfg_.map_cfg().map_resl_x();
  map_cfg.map_resolution[1] = cfg_.map_cfg().map_resl_y();
  map_cfg.map_resolution[2] = cfg_.map_cfg().map_resl_z();
  map_cfg.s_back_len = cfg_.map_cfg().s_back_len();
  map_cfg.kMaxLongitudinalVel = cfg_.map_cfg().dyn_bounds().max_lon_vel();
  map_cfg.kMinLongitudinalVel =
      std::max(cfg_.map_cfg().dyn_bounds().min_lon_vel(),
               cfg_.planner_cfg().velocity_singularity_eps());
  map_cfg.kMaxLongitudinalAcc = cfg_.map_cfg().dyn_bounds().max_lon_acc();
  map_cfg.kMaxLongitudinalDecel = cfg_.map_cfg().dyn_bounds().max_lon_dec();
  map_cfg.kMaxLateralVel = cfg_.map_cfg().dyn_bounds().max_lat_vel();
  map_cfg.kMaxLateralAcc = cfg_.map_cfg().dyn_bounds().max_lat_acc();
  map_cfg.kMaxNumOfGridAlongTime = cfg_.map_cfg().max_grids_along_time();
  map_cfg.inflate_steps[0] = cfg_.map_cfg().infl_steps().x_p();
  map_cfg.inflate_steps[1] = cfg_.map_cfg().infl_steps().x_n();
  map_cfg.inflate_steps[2] = cfg_.map_cfg().infl_steps().y_p();
  map_cfg.inflate_steps[3] = cfg_.map_cfg().infl_steps().y_n();
  map_cfg.inflate_steps[4] = cfg_.map_cfg().infl_steps().z_p();
  map_cfg.inflate_steps[5] = cfg_.map_cfg().infl_steps().z_n();
  p_ssc_map_ = new SscMap(map_cfg);

  return kSuccess;
}

ErrorType SscPlanner::ReadConfig(const std::string config_path) {
  printf("\n[EudmPlanner] Loading ssc planner config\n");
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

ErrorType SscPlanner::set_initial_state(const State& state) {
  initial_state_ = state;
  has_initial_state_ = true;
  return kSuccess;
}

ErrorType SscPlanner::RunOnce() {
  stamp_ = map_itf_->GetTimeStamp();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE START: " << stamp_
               << " ********************\n";
  static TicToc ssc_timer;
  ssc_timer.tic();

  static TicToc timer_prepare;
  timer_prepare.tic();
  if (map_itf_->GetEgoVehicle(&ego_vehicle_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego vehicle info.";
    return kWrongStatus;
  }

  // plan state
  if (!has_initial_state_) {
    initial_state_ = ego_vehicle_.state();
  }
  has_initial_state_ = false;

  is_lateral_independent_ =
      initial_state_.velocity > cfg_.planner_cfg().low_speed_threshold()
          ? true
          : false;
  if (map_itf_->GetLocalReferenceLane(&nav_lane_local_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to find ego lane.";
    return kWrongStatus;
  }
  stf_ = common::StateTransformer(nav_lane_local_);

  if (stf_.GetFrenetStateFromState(initial_state_, &initial_frenet_state_) !=
      kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get init state frenet state.";
    return kWrongStatus;
  }

  if (map_itf_->GetEgoDiscretBehavior(&ego_behavior_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get ego behavior.";
    return kWrongStatus;
  }

  if (map_itf_->GetObstacleMap(&grid_map_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle map.";
    return kWrongStatus;
  }

  if (map_itf_->GetObstacleGrids(&obstacle_grids_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get obstacle grids.";
    return kWrongStatus;
  }

  if (map_itf_->GetForwardTrajectories(&forward_behaviors_, &forward_trajs_,
                                       &surround_forward_trajs_) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to get forward trajectories.";
    return kWrongStatus;
  }

  auto t_prepare = timer_prepare.toc();
  LOG(WARNING) << "[Ssc]prepare time cost: " << t_prepare << " ms";

  static TicToc timer_stf;
  timer_stf.tic();
  if (StateTransformForInputData() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to transform state into ff.";
    return kWrongStatus;
  }
  auto t_stf = timer_stf.toc();
  LOG(WARNING) << "[Ssc]state transform time cost: " << t_stf << " ms";

  static TicToc timer_sscmap;  // ! SscMap part sometimes can be very slow
                               // ! (Maybe CPU scheduling?)
  timer_sscmap.tic();
  time_origin_ = initial_state_.time_stamp;
  p_ssc_map_->ResetSscMap(initial_frenet_state_);
  // ~ For closed-loop simulation prediction
  int num_behaviors = forward_behaviors_.size();
  for (int i = 0; i < num_behaviors; ++i) {
    if (!cfg_.planner_cfg().is_fitting_only()) {
      if (p_ssc_map_->ConstructSscMap(surround_forward_trajs_fs_[i],
                                      obstacle_grids_fs_)) {
        LOG(ERROR) << "[Ssc]fail to construct ssc map.";
        return kWrongStatus;
      }
    }
    // ! Notice: No inflation here to save time in eudm project. Improve
    // ! efficiency in the future.
    // TicToc timer_infl;
    // p_ssc_map_->InflateObstacleGrid(ego_vehicle_.param());
    // printf("[SscPlanner] InflateObstacleGrid time cost: %lf ms\n",
    //        timer_infl.toc());
    if (p_ssc_map_->ConstructCorridorUsingInitialTrajectory(
            p_ssc_map_->p_3d_grid(), forward_trajs_fs_[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail to construct corridor for behavior " << i;
      return kWrongStatus;
    }
  }
  if (kSuccess != p_ssc_map_->GetFinalGlobalMetricCubesList()) {
    LOG(ERROR) << "[Ssc]fail to get final corridor";
    return kWrongStatus;
  }
  auto t_sscmap = timer_sscmap.toc();
  LOG(WARNING) << "[Ssc]construct ssc map and corridor time cost: " << t_sscmap
               << " ms";

  static TicToc timer_opt;
  timer_opt.tic();
  if (RunQpOptimization() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail to optimize qp trajectories.\n";
    return kWrongStatus;
  }

  if (UpdateTrajectoryWithCurrentBehavior() != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: current behavior "
               << static_cast<int>(ego_behavior_) << " not valid.";
    LOG(ERROR) << "[Ssc]fail: has " << qp_trajs_.size() << " traj, "
               << valid_behaviors_.size() << " behaviors.";
    return kWrongStatus;
  }

#if 0
  auto traj = trajectory();
  if (ValidateTrajectory(*traj) != kSuccess) {
    LOG(ERROR) << "[Ssc]fail: infeasible traj.";
    return kWrongStatus;
  }
#endif

  auto t_opt = timer_opt.toc();
  LOG(WARNING) << "[Ssc]optimization time cost: " << t_opt << " ms";

  auto t_sum = t_prepare + t_stf + t_sscmap + t_opt;
  time_cost_ = ssc_timer.toc();
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]Sum of time: " << t_sum
               << " ms, diff: " << time_cost_ - t_sum << " ms";
  LOG(WARNING) << std::fixed << std::setprecision(4)
               << "[Ssc]******************** RUNONCE FINISH: " << stamp_ << " +"
               << time_cost_ << " ms ********************\n";

  return kSuccess;
}  // namespace planning

ErrorType SscPlanner::RunQpOptimization() {
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> cube_list =
      p_ssc_map_->final_corridor_vec();
  std::vector<int> if_corridor_valid = p_ssc_map_->if_corridor_valid();
  if (cube_list.empty()) return kWrongStatus;
  if (cube_list.size() != forward_behaviors_.size()) {
    LOG(ERROR) << "[Ssc]cube list " << static_cast<int>(cube_list.size())
               << " not consist with behavior size: "
               << static_cast<int>(forward_behaviors_.size())
               << ", forward traj " << static_cast<int>(forward_trajs_.size())
               << ", flag size " << static_cast<int>(if_corridor_valid.size());
    return kWrongStatus;
  }

  qp_trajs_.clear();
  primitive_trajs_.clear();
  valid_behaviors_.clear();
  corridors_.clear();
  ref_states_list_.clear();
  for (int i = 0; i < static_cast<int>(cube_list.size()); i++) {
    int beh = static_cast<int>(forward_behaviors_[i]);
    if (if_corridor_valid[i] == 0) {
      LOG(ERROR) << "[Ssc]fail: for behavior "
                 << static_cast<int>(forward_behaviors_[i])
                 << " has no valid corridor.";
      continue;
    }

    auto fs_vehicle_traj = forward_trajs_fs_[i];
    int num_states = static_cast<int>(fs_vehicle_traj.size());

    vec_E<Vecf<2>> start_constraints;
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[0], ego_frenet_state_.vec_dt[0]));
    start_constraints.push_back(
        Vecf<2>(std::max(ego_frenet_state_.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                ego_frenet_state_.vec_dt[1]));
    start_constraints.push_back(
        Vecf<2>(ego_frenet_state_.vec_s[2], ego_frenet_state_.vec_dt[2]));

    // printf("[Inconsist]Start sd position (%lf, %lf).\n",
    // start_constraints[0](0),
    //        start_constraints[0](1));
    vec_E<Vecf<2>> end_constraints;
    end_constraints.push_back(
        Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[0],
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[0]));
    end_constraints.push_back(
        Vecf<2>(std::max(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[1],
                         cfg_.planner_cfg().velocity_singularity_eps()),
                fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[1]));
    // end_constraints.push_back(
    //     Vecf<2>(fs_vehicle_traj[num_states - 1].frenet_state.vec_s[2],
    //             fs_vehicle_traj[num_states - 1].frenet_state.vec_dt[2]));
    common::SplineGenerator<5, 2> spline_generator;
    BezierSpline bezier_spline;

    cube_list[i].back().t_ub = fs_vehicle_traj.back().frenet_state.time_stamp;

    if (CorridorFeasibilityCheck(cube_list[i]) != kSuccess) {
      LOG(ERROR) << "[Ssc]fail: corridor not valid for optimization.";
      continue;
    }

    std::vector<decimal_t> ref_stamps;
    vec_E<Vecf<2>> ref_points;
    vec_E<common::FrenetState> ref_states;
    for (int n = 0; n < num_states; n++) {
      ref_stamps.push_back(fs_vehicle_traj[n].frenet_state.time_stamp);
      ref_points.push_back(Vecf<2>(fs_vehicle_traj[n].frenet_state.vec_s[0],
                                   fs_vehicle_traj[n].frenet_state.vec_dt[0]));
      ref_states.push_back(fs_vehicle_traj[n].frenet_state);
    }

    bool bezier_spline_gen_success = true;
    if (spline_generator.GetBezierSplineUsingCorridor(
            cube_list[i], start_constraints, end_constraints, ref_stamps,
            ref_points, cfg_.planner_cfg().weight_proximity(),
            &bezier_spline) != kSuccess) {
      if (is_lateral_independent_) {
        LOG(ERROR) << "[Ssc]fail: solver error for behavior "
                   << static_cast<int>(forward_behaviors_[i]);
        decimal_t t0 = cube_list[i].front().t_lb;
        for (auto& cube : cube_list[i]) {
          LOG(ERROR) << std::fixed << std::setprecision(3) << "[Ssc] t: ["
                     << cube.t_lb - t0 << ", " << cube.t_ub - t0 << "], x: ["
                     << cube.p_lb[0] << ", " << cube.p_ub[0] << "], y: ["
                     << cube.p_lb[1] << ", " << cube.p_ub[1] << "]";
        }
        LOG(ERROR) << "[Ssc]ref points: ";
        for (int k = 0; k < ref_stamps.size(); ++k) {
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << ref_stamps[k] << ", x: " << ref_points[k].x()
                     << ", y: " << ref_points[k].y();
        }
        LOG(ERROR) << "[Ssc]forward traj: ";
        for (int k = 0; k < forward_trajs_[i].size(); ++k) {
          auto v = forward_trajs_[i][k];
          LOG(ERROR) << std::fixed << std::setprecision(4) << "[Ssc]" << k
                     << " t: " << v.state().time_stamp
                     << ", x: " << v.state().vec_position.x()
                     << ", y: " << v.state().vec_position.y()
                     << ", v: " << v.state().velocity;
        }
        LOG(ERROR) << "[Ssc]ref lane range: [" << nav_lane_local_.begin()
                   << ", " << nav_lane_local_.end() << "]";

        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd velocity (" << start_constraints[1](0)
                   << ", " << start_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]Start sd acceleration (" << start_constraints[2](0)
                   << ", " << start_constraints[2](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd position (" << end_constraints[0](0) << ", "
                   << end_constraints[0](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End sd velocity (" << end_constraints[1](0) << ", "
                   << end_constraints[1](1) << ")";
        LOG(ERROR) << std::fixed << std::setprecision(4)
                   << "[Ssc]End state stamp: "
                   << fs_vehicle_traj[num_states - 1].frenet_state.time_stamp;
      }
      bezier_spline_gen_success = false;
    }

    FrenetPrimitive primitive;
    if (!is_lateral_independent_) {
      primitive.Connect(initial_frenet_state_,
                        fs_vehicle_traj.back().frenet_state,
                        initial_frenet_state_.time_stamp,
                        fs_vehicle_traj.back().frenet_state.time_stamp -
                            initial_frenet_state_.time_stamp,
                        is_lateral_independent_);
    }

    if (is_lateral_independent_ && !bezier_spline_gen_success) continue;
    // printf("[SscQP]spline begin stamp: %lf.\n", bezier_spline.begin());
    qp_trajs_.push_back(bezier_spline);
    primitive_trajs_.push_back(primitive);
    corridors_.push_back(cube_list[i]);
    ref_states_list_.push_back(ref_states);
    valid_behaviors_.push_back(forward_behaviors_[i]);
  }
  return kSuccess;
}

ErrorType SscPlanner::UpdateTrajectoryWithCurrentBehavior() {
  int num_valid_behaviors = static_cast<int>(valid_behaviors_.size());
  if (num_valid_behaviors < 1) {
    return kWrongStatus;
  }
  bool find_exact_match_behavior = false;
  int index = 0;
  for (int i = 0; i < num_valid_behaviors; i++) {
    if (valid_behaviors_[i] == ego_behavior_) {
      find_exact_match_behavior = true;
      index = i;
    }
  }
  bool find_candidate_behavior = false;
  LateralBehavior candidate_bahavior = common::LateralBehavior::kLaneKeeping;
  if (!find_exact_match_behavior) {
    for (int i = 0; i < num_valid_behaviors; i++) {
      if (valid_behaviors_[i] == candidate_bahavior) {
        find_candidate_behavior = true;
        index = i;
      }
    }
  }
  if (!find_exact_match_behavior && !find_candidate_behavior)
    return kWrongStatus;

  trajectory_ = FrenetBezierTrajectory(qp_trajs_[index], stf_);
  low_spd_alternative_traj_ =
      FrenetPrimitiveTrajectory(primitive_trajs_[index], stf_);
  final_corridor_ = corridors_[index];
  final_ref_states_ = ref_states_list_[index];
  return kSuccess;
}

ErrorType SscPlanner::CorridorFeasibilityCheck(
    const vec_E<common::SpatioTemporalSemanticCubeNd<2>>& cubes) {
  int num_cubes = static_cast<int>(cubes.size());
  if (num_cubes < 1) {
    LOG(ERROR) << "[Ssc]number of cubes not enough.";
    return kWrongStatus;
  }
  for (int i = 1; i < num_cubes; i++) {
    if (cubes[i - 1].t_ub != cubes[i].t_lb) {
      LOG(ERROR) << "[Ssc]Err- Corridor not consist.";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i - 1].t_lb << ", "
                 << cubes[i - 1].t_ub << "], x: [" << cubes[i - 1].p_lb[0]
                 << ", " << cubes[i - 1].p_ub[0] << "], y: ["
                 << cubes[i - 1].p_lb[1] << ", " << cubes[i - 1].p_ub[1] << "]";
      LOG(ERROR) << "[Ssc]Err - t: [" << cubes[i].t_lb << ", " << cubes[i].t_ub
                 << "], x: [" << cubes[i].p_lb[0] << ", " << cubes[i].p_ub[0]
                 << "], y: [" << cubes[i].p_lb[1] << ", " << cubes[i].p_ub[1]
                 << "]";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::StateTransformForInputData() {
  vec_E<State> global_state_vec;
  vec_E<Vec2f> global_point_vec;
  int num_v;

  // ~ Stage I. Package states and points
  // * Ego vehicle state and vertices
  {
    global_state_vec.push_back(initial_state_);
    vec_E<Vec2f> v_vec;
    common::SemanticsUtils::GetVehicleVertices(ego_vehicle_.param(),
                                               initial_state_, &v_vec);
    num_v = v_vec.size();
    global_point_vec.insert(global_point_vec.end(), v_vec.begin(), v_vec.end());
  }

  // * Ego forward simulation trajs states and vertices
  {
    common::VehicleParam ego_param = ego_vehicle_.param();
    for (int i = 0; i < (int)forward_trajs_.size(); ++i) {
      if (forward_trajs_[i].size() < 1) continue;
      for (int k = 0; k < (int)forward_trajs_[i].size(); ++k) {
        // states
        State traj_state = forward_trajs_[i][k].state();
        global_state_vec.push_back(traj_state);
        // vertices
        vec_E<Vec2f> v_vec;
        common::SemanticsUtils::GetVehicleVertices(ego_param, traj_state,
                                                   &v_vec);
        global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                v_vec.end());
      }
    }
  }

  // * Surrounding vehicle trajs from MPDM
  {
    for (int i = 0; i < surround_forward_trajs_.size(); ++i) {
      for (auto it = surround_forward_trajs_[i].begin();
           it != surround_forward_trajs_[i].end(); ++it) {
        for (int k = 0; k < it->second.size(); ++k) {
          // states
          State traj_state = it->second[k].state();
          global_state_vec.push_back(traj_state);
          // vertices
          vec_E<Vec2f> v_vec;
          common::SemanticsUtils::GetVehicleVertices(it->second[k].param(),
                                                     traj_state, &v_vec);
          global_point_vec.insert(global_point_vec.end(), v_vec.begin(),
                                  v_vec.end());
        }
      }
    }
  }

  // * Obstacle grids
  {
    for (auto it = obstacle_grids_.begin(); it != obstacle_grids_.end(); ++it) {
      Vec2f pt((*it)[0], (*it)[1]);
      global_point_vec.push_back(pt);
    }
  }

  vec_E<FrenetState> frenet_state_vec(global_state_vec.size());
  vec_E<Vec2f> fs_point_vec(global_point_vec.size());

  // ~ Stage II. Do transformation in multi-thread flavor
#if USE_OPENMP
  TicToc timer_stf;
  StateTransformUsingOpenMp(global_state_vec, global_point_vec,
                            &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]OpenMp transform time cost: " << timer_stf.toc()
               << " ms.";
#else
  TicToc timer_stf;
  StateTransformSingleThread(global_state_vec, global_point_vec,
                             &frenet_state_vec, &fs_point_vec);
  LOG(WARNING) << "[Ssc]Single thread transform time cost: " << timer_stf.toc()
               << " ms.";
#endif

  // ~ Stage III. Retrieve states and points
  int offset = 0;
  // * Ego vehicle state and vertices
  {
    fs_ego_vehicle_.frenet_state = frenet_state_vec[offset];
    fs_ego_vehicle_.vertices.clear();
    for (int i = 0; i < num_v; ++i) {
      fs_ego_vehicle_.vertices.push_back(fs_point_vec[offset * num_v + i]);
    }
    offset++;
  }

  // * Ego forward simulation trajs states and vertices
  {
    forward_trajs_fs_.clear();
    if (forward_trajs_.size() < 1) return kWrongStatus;
    for (int j = 0; j < (int)forward_trajs_.size(); ++j) {
      if (forward_trajs_[j].size() < 1) assert(false);
      vec_E<common::FsVehicle> traj_fs;
      for (int k = 0; k < (int)forward_trajs_[j].size(); ++k) {
        common::FsVehicle fs_v;
        fs_v.frenet_state = frenet_state_vec[offset];
        for (int i = 0; i < num_v; ++i) {
          fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
        }
        traj_fs.emplace_back(fs_v);
        offset++;
      }
      forward_trajs_fs_.emplace_back(traj_fs);
    }
  }

  // * Surrounding vehicle trajs from MPDM
  {
    surround_forward_trajs_fs_.clear();
    for (int j = 0; j < surround_forward_trajs_.size(); ++j) {
      std::unordered_map<int, vec_E<common::FsVehicle>> sur_trajs;
      for (auto it = surround_forward_trajs_[j].begin();
           it != surround_forward_trajs_[j].end(); ++it) {
        int v_id = it->first;
        vec_E<common::FsVehicle> traj_fs;
        for (int k = 0; k < it->second.size(); ++k) {
          common::FsVehicle fs_v;
          fs_v.frenet_state = frenet_state_vec[offset];
          for (int i = 0; i < num_v; ++i) {
            fs_v.vertices.push_back(fs_point_vec[offset * num_v + i]);
          }
          traj_fs.emplace_back(fs_v);
          offset++;
        }
        sur_trajs.insert(
            std::pair<int, vec_E<common::FsVehicle>>(v_id, traj_fs));
      }
      surround_forward_trajs_fs_.emplace_back(sur_trajs);
    }
  }

  // * Obstacle grids
  {
    obstacle_grids_fs_.clear();
    for (int i = 0; i < static_cast<int>(obstacle_grids_.size()); ++i) {
      obstacle_grids_fs_.push_back(fs_point_vec[offset * num_v + i]);
    }
  }

  ego_frenet_state_ = fs_ego_vehicle_.frenet_state;
  return kSuccess;
}

ErrorType SscPlanner::StateTransformUsingOpenMp(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();

  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();

  LOG(WARNING) << "[Ssc]OpenMp - Total number of queries: "
               << state_num + point_num;
  omp_set_num_threads(4);
  {
#pragma omp parallel for
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      if (kSuccess != stf_.GetFrenetStateFromState(global_state_vec[i], &fs)) {
        fs.time_stamp = global_state_vec[i].time_stamp;
      }
      *(ptr_state_vec + i) = fs;
    }
  }
  {
#pragma omp parallel for
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::StateTransformSingleThread(
    const vec_E<State>& global_state_vec, const vec_E<Vec2f>& global_point_vec,
    vec_E<FrenetState>* frenet_state_vec, vec_E<Vec2f>* fs_point_vec) const {
  int state_num = global_state_vec.size();
  int point_num = global_point_vec.size();
  auto ptr_state_vec = frenet_state_vec->data();
  auto ptr_point_vec = fs_point_vec->data();
  {
    for (int i = 0; i < state_num; ++i) {
      FrenetState fs;
      stf_.GetFrenetStateFromState(global_state_vec[i], &fs);
      *(ptr_state_vec + i) = fs;
    }
  }
  {
    for (int i = 0; i < point_num; ++i) {
      Vec2f fs_pt;
      stf_.GetFrenetPointFromPoint(global_point_vec[i], &fs_pt);
      *(ptr_point_vec + i) = fs_pt;
    }
  }
  return kSuccess;
}

ErrorType SscPlanner::set_map_interface(SscPlannerMapItf* map_itf) {
  if (map_itf == nullptr) return kIllegalInput;
  map_itf_ = map_itf;
  map_valid_ = true;
  return kSuccess;
}

ErrorType SscPlanner::ValidateTrajectory(const FrenetTrajectory& traj) {
  std::vector<decimal_t> t_vec_xy;
  common::GetRangeVector<decimal_t>(traj.begin(), traj.end(), 0.1, true,
                                    &t_vec_xy);
  common::State state;
  // * check init state
  if (traj.GetState(traj.begin(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]State evaluation error";
    return kWrongStatus;
  }

  if ((state.vec_position - initial_state_.vec_position).norm() > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init position miss match";
    return kWrongStatus;
  }

  if (fabs(state.velocity - initial_state_.velocity) > 0.1) {
    LOG(ERROR) << "[Ssc][Validate]Init vel miss match";
    return kWrongStatus;
  }
  // * check end state
  if (traj.GetState(traj.end(), &state) != kSuccess) {
    LOG(ERROR) << "[Ssc][Validate]End state eval error";
    return kWrongStatus;
  }

  for (const auto t : t_vec_xy) {
    if (traj.GetState(t, &state) != kSuccess) {
      LOG(ERROR) << "[Ssc][Validate]State eval error";
      return kWrongStatus;
    }
    if (fabs(state.curvature) > 0.33) {
      LOG(ERROR) << "[Ssc][Validate]initial_state velocity "
                 << initial_state_.velocity << " Curvature " << state.curvature
                 << " invalid.";
      return kWrongStatus;
    }
  }
  return kSuccess;
}

}  // namespace planning