#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud.h>

#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/decoder.h"

#include "phy_simulator/basics.h"
#include "phy_simulator/phy_simulator.h"
#include "phy_simulator/ros_adapter.h"
#include "phy_simulator/visualizer.h"

using namespace phy_simulator;

DECLARE_BACKWARD;
const double simulation_rate = 500.0;
const double gt_msg_rate = 100.0;
const double gt_static_msg_rate = 10.0;
const double visualization_msg_rate = 20.0;

common::VehicleControlSignalSet _signal_set;
std::vector<ros::Subscriber> _ros_sub;

Vec3f initial_state(0, 0, 0);
bool flag_rcv_initial_state = false;

Vec3f goal_state(0, 0, 0);
bool flag_rcv_goal_state = false;

void CtrlSignalCallback(const vehicle_msgs::ControlSignal::ConstPtr& msg,
                        int index) {
  common::VehicleControlSignal ctrl;
  vehicle_msgs::Decoder::GetControlSignalFromRosControlSignal(*msg, &ctrl);
  _signal_set.signal_set[index] = ctrl;
}

void InitialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose.pose,
                                                     &initial_state);
  flag_rcv_initial_state = true;
}

void NavGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  common::VisualizationUtil::Get3DofStateFromRosPose(msg->pose, &goal_state);
  flag_rcv_goal_state = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");

  std::string vehicle_info_path;
  if (!nh.getParam("vehicle_info_path", vehicle_info_path)) {
    ROS_ERROR("Failed to get param %s", vehicle_info_path.c_str());
    assert(false);
  }
  std::string map_path;
  if (!nh.getParam("map_path", map_path)) {
    ROS_ERROR("Failed to get param %s", map_path.c_str());
    assert(false);
  }
  std::string lane_net_path;
  if (!nh.getParam("lane_net_path", lane_net_path)) {
    ROS_ERROR("Failed to get param %s", lane_net_path.c_str());
    assert(false);
  }

  PhySimulation phy_sim(vehicle_info_path, map_path, lane_net_path);

  RosAdapter ros_adapter(nh);
  ros_adapter.set_phy_sim(&phy_sim);

  Visualizer visualizer(nh);
  visualizer.set_phy_sim(&phy_sim);

  auto vehicle_ids = phy_sim.vehicle_ids();
  int num_vehicles = static_cast<int>(vehicle_ids.size());
  _ros_sub.resize(num_vehicles);

  for (int i = 0; i < num_vehicles; i++) {
    auto vehicle_id = vehicle_ids[i];
    std::string topic_name =
        std::string("/ctrl/agent_") + std::to_string(vehicle_id);
    printf("subscribing to %s\n", topic_name.c_str());
    _ros_sub[i] = nh.subscribe<vehicle_msgs::ControlSignal>(
        topic_name, 10, boost::bind(CtrlSignalCallback, _1, vehicle_id));
  }

  for (auto& vehicle_id : vehicle_ids) {
    common::VehicleControlSignal default_signal;
    _signal_set.signal_set.insert(std::pair<int, common::VehicleControlSignal>(
        vehicle_id, default_signal));
  }

  ros::Subscriber ini_pos_sub =
      nh.subscribe("/initialpose", 10, InitialPoseCallback);
  ros::Subscriber goal_pos_sub =
      nh.subscribe("/move_base_simple/goal", 10, NavGoalCallback);

  ros::Rate rate(simulation_rate);
  ros::Time next_gt_pub_time = ros::Time::now();
  ros::Time next_gt_static_pub_time = next_gt_pub_time;
  ros::Time next_vis_pub_time = ros::Time::now();

  std::cout << "[PhySimulation] Initialization finished, waiting for callback"
            << std::endl;

  int gt_msg_counter = 0;
  while (ros::ok()) {
    ros::spinOnce();

    phy_sim.UpdateSimulatorUsingSignalSet(_signal_set, 1.0 / simulation_rate);

    ros::Time tnow = ros::Time::now();
    if (tnow >= next_gt_pub_time) {
      next_gt_pub_time += ros::Duration(1.0 / gt_msg_rate);
      ros_adapter.PublishDynamicDataWithStamp(tnow);
    }

    if (tnow >= next_gt_static_pub_time) {
      next_gt_static_pub_time += ros::Duration(1.0 / gt_static_msg_rate);
      ros_adapter.PublishStaticDataWithStamp(tnow);
    }

    if (tnow >= next_vis_pub_time) {
      next_vis_pub_time += ros::Duration(1.0 / visualization_msg_rate);
      visualizer.VisualizeDataWithStamp(tnow);
    }

    rate.sleep();
  }

  _ros_sub.clear();
  ros::shutdown();
  return 0;
}
