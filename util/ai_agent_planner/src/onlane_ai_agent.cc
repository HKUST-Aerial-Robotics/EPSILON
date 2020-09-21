#include <ros/ros.h>
#include <stdlib.h>

#include <chrono>
#include <iostream>
#include <random>

#include "behavior_planner/behavior_server_ros.h"
#include "common/basics/tic_toc.h"
#include "forward_simulator/multimodal_forward.h"
#include "forward_simulator/onlane_forward_simulation.h"
#include "semantic_map_manager/ros_adapter.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "semantic_map_manager/visualizer.h"
#include "sensor_msgs/Joy.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/encoder.h"

DECLARE_BACKWARD;
double fs_work_rate = 50.0;
double visualization_msg_rate = 20.0;
double bp_work_rate = 20.0;
int ego_id;

ros::Publisher ctrl_signal_pub_;
planning::BehaviorPlannerServer* p_bp_server_{nullptr};
moodycamel::ReaderWriterQueue<semantic_map_manager::SemanticMapManager>*
    p_ctrl_input_smm_buff_{nullptr};
semantic_map_manager::Visualizer* p_smm_vis_{nullptr};

common::State desired_state;
bool has_init_state = false;
double desired_vel;

ros::Time next_vis_pub_time;
semantic_map_manager::SemanticMapManager last_smm;
planning::OnLaneForwardSimulation::Param sim_param;

// ~ Add noise
std::mt19937 rng;
double vel_noise = 0.0;
int cnt = 0;
int aggressiveness_level = 3;

int SemanticMapUpdateCallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_bp_server_) p_bp_server_->PushSemanticMap(smm);

  if (!has_init_state) {
    desired_state = smm.ego_vehicle().state();
    has_init_state = true;
    desired_state.print();
  }
  return 0;
}

int BehaviorUpdateBallback(
    const semantic_map_manager::SemanticMapManager& smm) {
  if (p_ctrl_input_smm_buff_) p_ctrl_input_smm_buff_->try_enqueue(smm);
  return 0;
}

void RandomBehavior() {
  if (cnt == 0) {
    std::uniform_real_distribution<double> dist_vel(-2, 5);
    vel_noise = dist_vel(rng);
    p_bp_server_->set_user_desired_velocity(desired_vel + vel_noise);
    printf("[OnlaneAi]%d - desired velocity: %lf\n", ego_id,
           desired_vel + vel_noise);
  }
  cnt++;
  if (cnt >= 2000) cnt = 0;
}

void PublishControl() {
  if (!has_init_state) return;
  if (p_bp_server_ == nullptr) return;
  if (p_ctrl_input_smm_buff_ == nullptr) return;

  bool is_map_updated = false;
  decimal_t previous_stamp = last_smm.time_stamp();
  while (p_ctrl_input_smm_buff_->try_dequeue(last_smm)) {
    is_map_updated = true;
  }
  if (!is_map_updated) return;

  decimal_t delta_t = last_smm.time_stamp() - previous_stamp;
  if (delta_t > 100.0 / fs_work_rate) delta_t = 1.0 / fs_work_rate;

  decimal_t command_vel = p_bp_server_->reference_desired_velocity();
  decimal_t speed_limit;
  if (last_smm.GetSpeedLimit(last_smm.ego_vehicle().state(),
                             last_smm.ego_behavior().ref_lane,
                             &speed_limit) == kSuccess) {
    command_vel = std::min(speed_limit, command_vel);
  }
  common::Vehicle ego_vehicle = last_smm.ego_vehicle();
  ego_vehicle.set_state(desired_state);
  // ~ Add noise
  // command_vel = std::max(command_vel + vel_noise, 0.0);
  sim_param.idm_param.kDesiredVelocity = command_vel;

  common::Vehicle leading_vehicle;
  common::State state;
  decimal_t distance_residual_ratio = 0.0;
  const decimal_t lat_range = 2.2;
  last_smm.GetLeadingVehicleOnLane(last_smm.ego_behavior().ref_lane,
                                   desired_state,
                                   last_smm.surrounding_vehicles(), lat_range,
                                   &leading_vehicle, &distance_residual_ratio);
  if (planning::OnLaneForwardSimulation::PropagateOnce(
          common::StateTransformer(last_smm.ego_behavior().ref_lane),
          ego_vehicle, leading_vehicle, delta_t, sim_param,
          &state) != kSuccess) {
    printf("[AiAgent]Err-Simulation error (with leading vehicle).\n");
    return;
  }

  common::VehicleControlSignal ctrl(state);
  {
    vehicle_msgs::ControlSignal ctrl_msg;
    vehicle_msgs::Encoder::GetRosControlSignalFromControlSignal(
        ctrl, ros::Time::now(), std::string("map"), &ctrl_msg);
    ctrl_signal_pub_.publish(ctrl_msg);
  }
  desired_state = ctrl.state;

  // visualization
  {
    ros::Time tnow = ros::Time::now();
    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += ros::Duration(1.0 / visualization_msg_rate);
      p_smm_vis_->VisualizeDataWithStamp(tnow, last_smm);
      p_smm_vis_->SendTfWithStamp(tnow, last_smm);
    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  next_vis_pub_time = ros::Time::now();

  if (!nh.getParam("ego_id", ego_id)) {
    ROS_ERROR("Failed to get param %d", ego_id);
    assert(false);
  }

  std::string agent_config_path;
  if (!nh.getParam("agent_config_path", agent_config_path)) {
    ROS_ERROR("Failed to get param %s", agent_config_path.c_str());
    assert(false);
  }

  ctrl_signal_pub_ = nh.advertise<vehicle_msgs::ControlSignal>("ctrl", 10);

  int autonomous_level;
  nh.param("desired_vel", desired_vel, 6.0);
  nh.param("autonomous_level", autonomous_level, 2);

  // Get desired velocity noise
  rng.seed(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  // config aggressiveness
  nh.param("aggressiveness_level", aggressiveness_level, 3);
  // std::uniform_int_distribution<int> dist_agg(1, 5);
  // aggressiveness_level = dist_agg(rng);
  // aggressiveness_level = 5;
  planning::MultiModalForward::ParamLookUp(aggressiveness_level, &sim_param);
  printf("[OnlaneAi]%d - aggresive: %d\n", ego_id, aggressiveness_level);

  // Declare smm
  semantic_map_manager::SemanticMapManager semantic_map_manager(
      ego_id, agent_config_path);
  semantic_map_manager::RosAdapter smm_ros_adapter(nh, &semantic_map_manager);
  p_smm_vis_ = new semantic_map_manager::Visualizer(nh, ego_id);

  // Declare bp
  p_bp_server_ = new planning::BehaviorPlannerServer(nh, bp_work_rate, ego_id);
  p_bp_server_->set_user_desired_velocity(desired_vel);
  p_bp_server_->set_autonomous_level(autonomous_level);
  p_bp_server_->set_aggressive_level(aggressiveness_level);
  p_bp_server_->enable_hmi_interface();

  smm_ros_adapter.BindMapUpdateCallback(SemanticMapUpdateCallback);
  p_bp_server_->BindBehaviorUpdateCallback(BehaviorUpdateBallback);

  smm_ros_adapter.Init();
  p_bp_server_->Init();

  p_ctrl_input_smm_buff_ = new moodycamel::ReaderWriterQueue<
      semantic_map_manager::SemanticMapManager>(100);

  p_bp_server_->Start();
  ros::Rate rate(fs_work_rate);
  while (ros::ok()) {
    ros::spinOnce();
    PublishControl();
    // RandomBehavior();
    rate.sleep();
  }
  return 0;
}
