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

common::RssChecker::RssConfig rss_config_strict_as_rear_;
common::RssChecker::RssConfig rss_config_strict_as_front_;
// * test vis
ros::Publisher target_state_pub_;

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

void VisualizeTest(const common::State& target_state,
                   const decimal_t& front_gap, const decimal_t& rear_gap,
                   const decimal_t& gap_length,
                   const common::Vehicle& front_vehicle,
                   const common::Vehicle& rear_vehicle,
                   const decimal_t& rss_fgap, const decimal_t& rss_rgap) {
  visualization_msgs::MarkerArray mks;

  // * target pose
  geometry_msgs::Pose target_pose_ros;
  common::VisualizationUtil::GetRosPoseFrom3DofState(target_state.ToXYTheta(),
                                                     &target_pose_ros);
  target_pose_ros.position.z = 3.0;
  visualization_msgs::Marker target_state_mk;
  common::VisualizationUtil::GetRosMarkerArrowUsingPoseAndNorm(
      target_pose_ros, target_state.velocity, common::cmap.at("red"),
      &target_state_mk);
  target_state_mk.header.frame_id = "map";
  target_state_mk.header.stamp = ros::Time::now();
  target_state_mk.id = 1;
  mks.markers.push_back(target_state_mk);

  // * gap
  std::string gap_info;
  gap_info += std::string("TargetVel: " +
                          common::GetStringByValueWithPrecision<decimal_t>(
                              target_state.velocity, 2) +
                          " m/s\n");
  gap_info += std::string(
      "F: " + common::GetStringByValueWithPrecision<decimal_t>(front_gap, 2) +
      " R: " + common::GetStringByValueWithPrecision<decimal_t>(rear_gap, 2) +
      " Gap: " +
      common::GetStringByValueWithPrecision<decimal_t>(gap_length, 2) + "\n");

  Vec3f text_pos =
      Vec3f(target_state.vec_position(0), target_state.vec_position(1), 3);
  visualization_msgs::Marker text_mk;
  text_mk.header.frame_id = "map";
  text_mk.header.stamp = ros::Time::now();
  common::VisualizationUtil::GetRosMarkerTextUsingPositionAndString(
      text_pos, gap_info, common::cmap.at("white"), Vec3f(1.0, 1.0, 1.0), 2,
      &text_mk);
  mks.markers.push_back(text_mk);

  // * rss gap
  if (front_vehicle.id() != kInvalidAgentId) {
    decimal_t rss_f_len = rss_fgap + (front_vehicle.param().length() / 2.0 -
                                      front_vehicle.param().d_cr());
    common::Point rss_f_pt1, rss_f_pt2;
    rss_f_pt1.x = front_vehicle.state().vec_position.x() -
                  cos(front_vehicle.state().angle) * rss_f_len;
    rss_f_pt1.y = front_vehicle.state().vec_position.y() -
                  sin(front_vehicle.state().angle) * rss_f_len;
    rss_f_pt1.z = 2.0;
    rss_f_pt2.x = front_vehicle.state().vec_position.x();
    rss_f_pt2.y = front_vehicle.state().vec_position.y();
    rss_f_pt2.z = 2.0;
    std::vector<common::Point> rss_f_pts;
    rss_f_pts.push_back(rss_f_pt1);
    rss_f_pts.push_back(rss_f_pt2);

    visualization_msgs::Marker rss_f_mk;
    rss_f_mk.header.frame_id = "map";
    rss_f_mk.header.stamp = ros::Time::now();
    common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
        rss_f_pts, Vec3f(0.5, 0.5, 0.5), common::cmap.at("violet").set_a(0.8),
        3, &rss_f_mk);
    mks.markers.push_back(rss_f_mk);
  }

  if (rear_vehicle.id() != kInvalidAgentId) {
    decimal_t rss_r_len = rss_rgap + (rear_vehicle.param().length() / 2.0 +
                                      rear_vehicle.param().d_cr());
    common::Point rss_r_pt1, rss_r_pt2;
    rss_r_pt1.x = rear_vehicle.state().vec_position.x() +
                  cos(rear_vehicle.state().angle) * rss_r_len;
    rss_r_pt1.y = rear_vehicle.state().vec_position.y() +
                  sin(rear_vehicle.state().angle) * rss_r_len;
    rss_r_pt1.z = 2.5;
    rss_r_pt2.x = rear_vehicle.state().vec_position.x();
    rss_r_pt2.y = rear_vehicle.state().vec_position.y();
    rss_r_pt2.z = 2.5;
    std::vector<common::Point> rss_r_pts;
    rss_r_pts.push_back(rss_r_pt1);
    rss_r_pts.push_back(rss_r_pt2);

    visualization_msgs::Marker rss_r_mk;
    rss_r_mk.header.frame_id = "map";
    rss_r_mk.header.stamp = ros::Time::now();
    common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
        rss_r_pts, Vec3f(0.5, 0.5, 0.5), common::cmap.at("violet").set_a(0.8),
        4, &rss_r_mk);
    mks.markers.push_back(rss_r_mk);
  }

  target_state_pub_.publish(mks);
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

  common::Vehicle ego_vehicle = last_smm.ego_vehicle();
  ego_vehicle.set_state(desired_state);

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

  // * for testing

  common::Lane lane_target;
  if (last_smm.GetRefLaneForStateByBehavior(
          desired_state, std::vector<int>(),
          common::LateralBehavior::kLaneChangeLeft, 150, 150, false,
          &lane_target) == kSuccess) {
    common::StateTransformer stf(lane_target);
    common::FrenetState ego_fs;
    stf.GetFrenetStateFromState(desired_state, &ego_fs);

    decimal_t s_ego_fbumper = ego_fs.vec_s[0] +
                              ego_vehicle.param().length() / 2.0 +
                              ego_vehicle.param().d_cr();
    decimal_t s_ego_rbumper = ego_fs.vec_s[0] -
                              ego_vehicle.param().length() / 2.0 +
                              ego_vehicle.param().d_cr();

    bool has_front_vehicle = false, has_rear_vehicle = false;
    common::Vehicle front_vehicle, rear_vehicle;
    common::FrenetState front_fs, rear_fs;
    last_smm.GetLeadingAndFollowingVehiclesFrenetStateOnLane(
        lane_target, state, last_smm.key_vehicles(), &has_front_vehicle,
        &front_vehicle, &front_fs, &has_rear_vehicle, &rear_vehicle, &rear_fs);

    decimal_t gap_length = (has_front_vehicle && has_rear_vehicle)
                               ? (front_fs.vec_s[0] - rear_fs.vec_s[0])
                               : -1;
    decimal_t front_gap =
        has_front_vehicle ? fabs(ego_fs.vec_s[0] - front_fs.vec_s[0]) : -1;
    decimal_t rear_gap =
        has_rear_vehicle ? fabs(ego_fs.vec_s[0] - rear_fs.vec_s[0]) : -1;

    decimal_t rss_gap_front, rss_gap_rear;

    // * RSS
    if (has_front_vehicle) {
      decimal_t s_front_rbumper = front_fs.vec_s[0] -
                                  front_vehicle.param().length() / 2.0 +
                                  front_vehicle.param().d_cr();
      common::RssChecker::CalculateSafeLongitudinalDistance(
          ego_vehicle.state().velocity, front_vehicle.state().velocity,
          common::RssChecker::LongitudinalDirection::Front,
          rss_config_strict_as_rear_, &rss_gap_front);
      decimal_t front_net_dist = s_front_rbumper - s_ego_fbumper;

      printf("[XXX]front_net_dist: %lf, rss_dist: %lf\n", front_net_dist,
             rss_gap_front);
      if (front_net_dist < rss_gap_front) {
        // violate strict RSS
        printf("[XXX]Rss - Front violate\n");
      }
    }

    if (has_rear_vehicle) {
      decimal_t s_rear_fbumper = rear_fs.vec_s[0] +
                                 rear_vehicle.param().length() / 2.0 +
                                 rear_vehicle.param().d_cr();
      common::RssChecker::CalculateSafeLongitudinalDistance(
          ego_vehicle.state().velocity, rear_vehicle.state().velocity,
          common::RssChecker::LongitudinalDirection::Rear,
          rss_config_strict_as_front_, &rss_gap_rear);
      decimal_t rear_net_dist = s_ego_rbumper - s_rear_fbumper;

      printf("[XXX]rear_net_dist: %lf, rss_dist: %lf\n", rear_net_dist,
             rss_gap_rear);
      if (rear_net_dist < rss_gap_rear) {
        // violate strict RSS
        printf("[XXX]Rss - Rear violate\n");
      }
    }

    common::State target_state;
    planning::OnLaneForwardSimulation::GetTargetStateOnTargetLane(
        lane_target, ego_vehicle, front_vehicle, rear_vehicle, sim_param,
        &target_state);

    VisualizeTest(target_state, front_gap, rear_gap, gap_length, front_vehicle,
                  rear_vehicle, rss_gap_front, rss_gap_rear);
  }

  // * construct control signal
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

  rss_config_strict_as_front_ =
      common::RssChecker::RssConfig(0.1, 0.5, 5.0, 4.5, 1.0, 1.0, 1.0, 0.5);
  rss_config_strict_as_rear_ =
      common::RssChecker::RssConfig(0.1, 0.5, 5.0, 5.0, 1.0, 1.0, 1.0, 0.5);

  ctrl_signal_pub_ = nh.advertise<vehicle_msgs::ControlSignal>("ctrl", 10);
  target_state_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("ctx_test", 10);

  int autonomous_level;
  nh.param("desired_vel", desired_vel, 6.0);
  nh.param("autonomous_level", autonomous_level, 2);

  // Get desired velocity noise
  rng.seed(
      std::chrono::high_resolution_clock::now().time_since_epoch().count());
  // config aggressiveness
  // nh.param("aggressiveness_level", aggressiveness_level, 3);
  std::uniform_int_distribution<int> dist_agg(1, 5);
  // aggressiveness_level = dist_agg(rng);
  aggressiveness_level = 5;
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
