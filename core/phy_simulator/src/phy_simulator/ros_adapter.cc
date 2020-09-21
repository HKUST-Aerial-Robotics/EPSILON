/**
 * @file ros_adapter.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#include "phy_simulator/ros_adapter.h"

namespace phy_simulator {

RosAdapter::RosAdapter() {}

RosAdapter::RosAdapter(ros::NodeHandle nh) : nh_(nh) {
  // arena_info_pub_ = nh_.advertise<vehicle_msgs::ArenaInfo>("arena_info", 10);
  arena_info_static_pub_ =
      nh_.advertise<vehicle_msgs::ArenaInfoStatic>("arena_info_static", 10);
  arena_info_dynamic_pub_ =
      nh_.advertise<vehicle_msgs::ArenaInfoDynamic>("arena_info_dynamic", 10);
}

void RosAdapter::PublishDataWithStamp(const ros::Time& stamp) {
  vehicle_msgs::ArenaInfo msg;
  vehicle_msgs::Encoder::GetRosArenaInfoFromSimulatorData(
      p_phy_sim_->lane_net(), p_phy_sim_->vehicle_set(),
      p_phy_sim_->obstacle_set(), stamp, std::string("map"), &msg);
  arena_info_pub_.publish(msg);
}

void RosAdapter::PublishStaticDataWithStamp(const ros::Time& stamp) {
  vehicle_msgs::ArenaInfoStatic msg;
  vehicle_msgs::Encoder::GetRosArenaInfoStaticFromSimulatorData(
      p_phy_sim_->lane_net(), p_phy_sim_->obstacle_set(), stamp,
      std::string("map"), &msg);
  arena_info_static_pub_.publish(msg);
}

void RosAdapter::PublishDynamicDataWithStamp(const ros::Time& stamp) {
  vehicle_msgs::ArenaInfoDynamic msg;
  vehicle_msgs::Encoder::GetRosArenaInfoDynamicFromSimulatorData(
      p_phy_sim_->vehicle_set(), stamp, std::string("map"), &msg);
  arena_info_dynamic_pub_.publish(msg);
}


}  // namespace phy_simulator