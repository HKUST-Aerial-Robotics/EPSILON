#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_

#include <assert.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "ros/ros.h"
#include "semantic_map_manager/data_renderer.h"
#include "vehicle_msgs/decoder.h"

namespace semantic_map_manager {

class RosAdapter {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  RosAdapter() {}
  RosAdapter(ros::NodeHandle nh, SemanticMapManager* ptr_smm) : nh_(nh) {
    p_smm_ = ptr_smm;
    p_data_renderer_ = new DataRenderer(ptr_smm);
  }
  ~RosAdapter() {}

  void BindMapUpdateCallback(std::function<int(const SemanticMapManager&)> fn);

  void Init();

 private:
  // ! DEPRECATED (@lu.zhang)
  void ArenaInfoCallback(const vehicle_msgs::ArenaInfo::ConstPtr& msg);

  void ArenaInfoStaticCallback(
      const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg);
  void ArenaInfoDynamicCallback(
      const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg);

  ros::NodeHandle nh_;

  // communicate with phy simulator
  ros::Subscriber arena_info_sub_;
  ros::Subscriber arena_info_static_sub_;
  ros::Subscriber arena_info_dynamic_sub_;

  common::Vehicle ego_vehicle_;
  common::VehicleSet vehicle_set_;
  common::LaneNet lane_net_;
  common::ObstacleSet obstacle_set_;

  DataRenderer* p_data_renderer_;
  SemanticMapManager* p_smm_;

  bool get_arena_info_static_ = false;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager&)> private_callback_fn_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_