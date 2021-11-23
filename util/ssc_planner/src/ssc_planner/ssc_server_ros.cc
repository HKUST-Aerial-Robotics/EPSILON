/**
 * @file ssc_server.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc planner server
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

#include "ssc_planner/ssc_server_ros.h"

namespace planning {

SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
  p_ssc_vis_ = new SscVisualizer(nh, ego_id);
}

SscPlannerServer::SscPlannerServer(ros::NodeHandle nh, double work_rate,
                                   int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) {
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);
  p_ssc_vis_ = new SscVisualizer(nh, ego_id);
}

void SscPlannerServer::PushSemanticMap(const SemanticMapManager& smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void SscPlannerServer::PublishData() {
  using common::VisualizationUtil;
  auto current_time = ros::Time::now().toSec();
  // smm visualization
  {
    p_smm_vis_->VisualizeDataWithStamp(ros::Time(current_time), last_smm_);
    p_smm_vis_->SendTfWithStamp(ros::Time(current_time), last_smm_);
  }

  // ssc visualization
  {
    TicToc timer;
    p_ssc_vis_->VisualizeDataWithStamp(ros::Time(current_time), planner_);
    printf("[SscPlannerServer]ssc vis all time cost: %lf ms\n", timer.toc());
  }

  // trajectory feedback
  {
    if (executing_traj_ == nullptr || !executing_traj_->IsValid()) return;

    if (use_sim_state_) {
      printf("[SscPlannerServer]use_sim_state_: true.\n");
    } else {
      printf("[SscPlannerServer]use_sim_state_: false.\n");
    }

    if (use_sim_state_) {
      decimal_t plan_horizon = 1.0 / work_rate_;
      int num_cycles =
          std::floor((current_time - executing_traj_->begin()) / plan_horizon);
      decimal_t ct = executing_traj_->begin() + num_cycles * plan_horizon;
      {
        common::State state;
        if (executing_traj_->GetState(ct, &state) == kSuccess) {
          FilterSingularityState(ctrl_state_hist_, &state);
          ctrl_state_hist_.push_back(state);
          if (ctrl_state_hist_.size() > 100)
            ctrl_state_hist_.erase(ctrl_state_hist_.begin());
          vehicle_msgs::ControlSignal ctrl_msg;
          vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
              common::VehicleControlSignal(state), ros::Time(ct),
              std::string("map"), &ctrl_msg);
          ctrl_signal_pub_.publish(ctrl_msg);
        } else {
          printf(
              "[SscPlannerServer]cannot evaluate state at %lf with begin "
              "%lf.\n",
              ct, executing_traj_->begin());
        }
      }
    }

    // trajectory visualization
    {
      auto color = common::cmap["magenta"];
      if (require_intervention_signal_) color = common::cmap["yellow"];
      visualization_msgs::MarkerArray traj_mk_arr;
      common::VisualizationUtil::GetMarkerArrayByTrajectory(
          *executing_traj_, 0.1, Vecf<3>(0.3, 0.3, 0.3), color, 0.5,
          &traj_mk_arr);
      if (require_intervention_signal_) {
        visualization_msgs::Marker traj_status;
        common::State state_begin;
        executing_traj_->GetState(executing_traj_->begin(), &state_begin);
        Vec3f pos = Vec3f(state_begin.vec_position[0],
                          state_begin.vec_position[1], 5.0);
        common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
            pos, std::string("Intervention Needed!"), common::cmap["red"],
            Vec3f(5.0, 5.0, 5.0), 0, &traj_status);
        traj_mk_arr.markers.push_back(traj_status);
      }
      int num_traj_mks = static_cast<int>(traj_mk_arr.markers.size());
      common::VisualizationUtil::FillHeaderIdInMarkerArray(
          ros::Time(current_time), std::string("map"), last_trajmk_cnt_,
          &traj_mk_arr);
      last_trajmk_cnt_ = num_traj_mks;
      executing_traj_vis_pub_.publish(traj_mk_arr);
    }
  }
}

ErrorType SscPlannerServer::FilterSingularityState(
    const vec_E<common::State>& hist, common::State* filter_state) {
  if (hist.empty()) {
    return kWrongStatus;
  }
  decimal_t duration = filter_state->time_stamp - hist.back().time_stamp;
  decimal_t wheel_base = 2.85;
  decimal_t max_steer = M_PI / 4.0;
  decimal_t singular_velocity = kBigEPS;
  decimal_t max_orientation_rate = tan(max_steer) / 2.85 * singular_velocity;
  decimal_t max_orientation_change = max_orientation_rate * duration;

  if (fabs(filter_state->velocity) < singular_velocity &&
      fabs(normalize_angle(filter_state->angle - hist.back().angle)) >
          max_orientation_change) {
    printf(
        "[SscPlannerServer]Detect singularity velocity %lf angle (%lf, %lf).\n",
        filter_state->velocity, hist.back().angle, filter_state->angle);
    filter_state->angle = hist.back().angle;
    printf("[SscPlannerServer]Filter angle to %lf.\n", hist.back().angle);
  }
  return kSuccess;
}

void SscPlannerServer::Init(const std::string& config_path) {
  planner_.Init(config_path);

  std::string traj_topic = std::string("/vis/agent_") +
                           std::to_string(ego_id_) +
                           std::string("/ssc/exec_traj");
  nh_.param("use_sim_state", use_sim_state_, true);

  ctrl_signal_pub_ = nh_.advertise<vehicle_msgs::ControlSignal>("ctrl", 20);
  map_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("ssc_map", 1);
  executing_traj_vis_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(traj_topic, 1);
}

void SscPlannerServer::Start() {
  if (is_replan_on_) {
    return;
  }
  planner_.set_map_interface(&map_adapter_);
  printf("[SscPlannerServer]Planner server started.\n");
  is_replan_on_ = true;

  std::thread(&SscPlannerServer::MainThread, this).detach();
}

void SscPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void SscPlannerServer::PlanCycleCallback() {
  if (!is_replan_on_) {
    return;
  }
  // printf("input buffer size: %d.\n", p_input_smm_buff_->size_approx());
  // is_map_updated_ = false;  // return when no new map
  while (p_input_smm_buff_->try_dequeue(last_smm_)) {
    is_map_updated_ = true;
  }

  if (!is_map_updated_) return;

  // Update map
  auto map_ptr =
      std::make_shared<semantic_map_manager::SemanticMapManager>(last_smm_);
  map_adapter_.set_map(map_ptr);

  PublishData();

  auto current_time = ros::Time::now().toSec();
  printf("[SscPlannerServer]>>>>>>>current time %lf.\n", current_time);
  if (executing_traj_ == nullptr || !executing_traj_->IsValid() ||
      !use_sim_state_) {
    // Init planning
    if (planner_.RunOnce() != kSuccess) {
      // printf("[SscPlannerServer]Initial planning failed.\n");
      return;
    }

    desired_state_hist_.clear();
    desired_state_hist_.push_back(last_smm_.ego_vehicle().state());
    ctrl_state_hist_.clear();
    ctrl_state_hist_.push_back(last_smm_.ego_vehicle().state());
    executing_traj_ = std::move(planner_.trajectory());
    global_init_stamp_ = executing_traj_->begin();
    printf(
        "[SscPlannerServer]init plan success with stamp: %lf and angle %lf.\n",
        global_init_stamp_, last_smm_.ego_vehicle().state().angle);
    return;
  }

  if (current_time > executing_traj_->end()) {
    printf("[SscPlannerServer]Current time %lf out of [%lf, %lf].\n",
           current_time, executing_traj_->begin(), executing_traj_->end());
    printf("[SscPlannerServer]Mission complete.\n");
    // is_replan_on_ = false;
    executing_traj_.release();
    next_traj_.release();
    return;
  }

  // NOTE: comment this block if you want to disable replan
  {
    if (next_traj_ == nullptr || !next_traj_->IsValid()) {
      Replan();
      return;
    }

    if (next_traj_->IsValid()) {
      executing_traj_ = std::move(next_traj_);
      // next_traj_ = common::Trajectory();
      Replan();
      return;
    }
  }
}

void SscPlannerServer::Replan() {
  if (!is_replan_on_) return;
  if (executing_traj_ == nullptr || !executing_traj_->IsValid()) return;

  decimal_t plan_horizon = 1.0 / work_rate_;
  common::State desired_state;
  decimal_t cur_time = ros::Time::now().toSec();

  int num_cycles_exec = std::floor(
      (executing_traj_->begin() - global_init_stamp_) / plan_horizon);
  int num_cycles_ahead =
      cur_time > executing_traj_->begin()
          ? std::floor((cur_time - global_init_stamp_) / plan_horizon) + 1
          : num_cycles_exec + 1;
  decimal_t t = global_init_stamp_ + plan_horizon * num_cycles_ahead;

  printf(
      "[SscPlannerServer]init stamp: %lf, plan horizon: %lf, num cycles %d.\n",
      global_init_stamp_, plan_horizon, num_cycles_ahead);
  printf(
      "[SscPlannerServer]Replan at cur time %lf with executing traj begin "
      "time: %lf to rounded t: %lf.\n",
      cur_time, executing_traj_->begin(), t);

  if (executing_traj_->GetState(t, &desired_state) != kSuccess) {
    printf("[SscPlannerServer]Cannot get desired state at %lf.\n", t);
    return;
  }
  printf(
      "[SscPlannerServer]t %lf, desired state (x,y,v,a,theta):(%lf, %lf, %lf, "
      "%lf, %lf).\n",
      t, desired_state.vec_position[0], desired_state.vec_position[1],
      desired_state.velocity, desired_state.acceleration, desired_state.angle);

  FilterSingularityState(desired_state_hist_, &desired_state);
  desired_state_hist_.push_back(desired_state);
  if (desired_state_hist_.size() > 100)
    desired_state_hist_.erase(desired_state_hist_.begin());

  planner_.set_initial_state(desired_state);

  time_profile_tool_.tic();
  if (planner_.RunOnce() != kSuccess) {
    printf("[SscPlannerServer]Ssc planner core failed in %lf ms.\n",
           time_profile_tool_.toc());
    require_intervention_signal_ = true;
    return;
  }

  require_intervention_signal_ = false;
  printf("[SscPlannerServer]Ssc planner succeed in %lf ms.\n",
         time_profile_tool_.toc());

  next_traj_ = std::move(planner_.trajectory());
}

}  // namespace planning