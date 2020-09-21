#include "semantic_map_manager/ros_adapter.h"

namespace semantic_map_manager {

void RosAdapter::Init() {
  // communicate with phy simulator
  {
    // arena_info_sub_ =
    //     nh_.subscribe("arena_info", 2, &RosAdapter::ArenaInfoCallback, this,
    //                   ros::TransportHints().tcpNoDelay());
    arena_info_static_sub_ = nh_.subscribe(
        "arena_info_static", 2, &RosAdapter::ArenaInfoStaticCallback, this,
        ros::TransportHints().tcpNoDelay());
    arena_info_dynamic_sub_ = nh_.subscribe(
        "arena_info_dynamic", 2, &RosAdapter::ArenaInfoDynamicCallback, this,
        ros::TransportHints().tcpNoDelay());
  }
}

// ! DEPRECATED (@lu.zhang)
void RosAdapter::ArenaInfoCallback(
    const vehicle_msgs::ArenaInfo::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfo(
      *msg, &time_stamp, &lane_net_, &vehicle_set_, &obstacle_set_);
  p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                           obstacle_set_);
  if (has_callback_binded_) {
    private_callback_fn_(*p_smm_);
  }
}

void RosAdapter::ArenaInfoStaticCallback(
    const vehicle_msgs::ArenaInfoStatic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoStatic(
      *msg, &time_stamp, &lane_net_, &obstacle_set_);
  get_arena_info_static_ = true;
}

void RosAdapter::ArenaInfoDynamicCallback(
    const vehicle_msgs::ArenaInfoDynamic::ConstPtr& msg) {
  ros::Time time_stamp;
  vehicle_msgs::Decoder::GetSimulatorDataFromRosArenaInfoDynamic(
      *msg, &time_stamp, &vehicle_set_);

  if (get_arena_info_static_) {
    p_data_renderer_->Render(time_stamp.toSec(), lane_net_, vehicle_set_,
                             obstacle_set_);
    if (has_callback_binded_) {
      private_callback_fn_(*p_smm_);
    }
  }
}

void RosAdapter::BindMapUpdateCallback(
    std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

}  // namespace semantic_map_manager