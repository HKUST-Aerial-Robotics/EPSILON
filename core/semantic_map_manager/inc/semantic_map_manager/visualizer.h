#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_

#include <assert.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {

class Visualizer {
 public:
  using ObstacleMapType = uint8_t;

  Visualizer(ros::NodeHandle nh, int node_id);
  ~Visualizer() {}

  void VisualizeData(const SemanticMapManager &smm);
  void VisualizeDataWithStamp(const ros::Time &stamp,
                              const SemanticMapManager &smm);
  void VisualizeDataWithStampForPlayback(
      const ros::Time &stamp, const SemanticMapManager &smm,
      const std::vector<int> &deleted_lane_ids);
  void SendTfWithStamp(const ros::Time &stamp, const SemanticMapManager &smm);

 private:
  void VisualizeEgoVehicle(const ros::Time &stamp,
                           const common::Vehicle &vehicle);
  void VisualizeSurroundingLaneNet(const ros::Time &stamp,
                                   const common::LaneNet &lane_net,
                                   const std::vector<int> &deleted_lane_ids);
  void VisualizeBehavior(const ros::Time &stamp,
                         const common::SemanticBehavior &behavior);
  void VisualizeSurroundingVehicles(const ros::Time &stamp,
                                    const common::VehicleSet &vehicle_set,
                                    const std::vector<int> &nearby_ids);
  void VisualizeLocalLanes(
      const ros::Time &stamp,
      const std::unordered_map<int, common::Lane> &local_lanes,
      const SemanticMapManager &smm,
      const std::vector<int> &deleted_lane_ids);
  void VisualizeObstacleMap(
      const ros::Time &stamp,
      const common::GridMapND<ObstacleMapType, 2> &obstacle_map);
  void VisualizeIntentionPrediction(
      const ros::Time &stamp, const common::SemanticVehicleSet &s_vehicle_set);
  void VisualizeOpenloopTrajPrediction(
      const ros::Time &stamp,
      const std::unordered_map<int, vec_E<common::State>> &openloop_pred_trajs);
  void VisualizeSpeedLimit(const ros::Time &stamp,
                           const vec_E<common::SpeedLimit> &speed_limits);

  int last_traj_list_marker_cnt_ = 0;
  int last_intention_marker_cnt_ = 0;
  int last_surrounding_vehicle_marker_cnt_ = 0;
  int last_speed_limit_marker_cnt_ = 0;
  int last_surrounding_lanes_cnt_ = 0;
  int last_behavior_marker_cnt_ = 0;

  std::string ego_tf_name_;

  ros::NodeHandle nh_;
  int node_id_;

  ros::Publisher ego_vehicle_pub_;
  ros::Publisher obstacle_map_pub_;
  ros::Publisher surrounding_lane_net_pub_;
  ros::Publisher local_lanes_pub_;
  ros::Publisher behavior_vis_pub_;
  ros::Publisher pred_traj_openloop_vis_pub_;
  ros::Publisher pred_intention_vis_pub_;
  ros::Publisher surrounding_vehicle_vis_pub_;
  ros::Publisher speed_limit_vis_pub_;
  tf::TransformBroadcaster ego_to_map_tf_;
  decimal_t marker_lifetime_{0.05};
};  // Visualizer

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_VISUALIZER_H_