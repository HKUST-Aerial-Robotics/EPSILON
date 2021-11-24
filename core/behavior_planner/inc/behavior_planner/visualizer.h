#ifndef _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_
#define _CORE_BEHAVIOR_PLANNER_INC_BEHAVIOR_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <functional>
#include <iostream>
#include <vector>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"

#include "vehicle_msgs/decoder.h"

#include "behavior_planner/behavior_planner.h"
#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

class BehaviorPlannerVisualizer {
 public:
  BehaviorPlannerVisualizer(ros::NodeHandle nh, BehaviorPlanner* ptr_bp,
                            int ego_id)
      : nh_(nh), ego_id_(ego_id) {
    p_bp_ = ptr_bp;
  }

  void Init() {
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");
    forward_traj_vis_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic, 1);
  }

  void PublishDataWithStamp(const ros::Time& stamp) {
    if (p_bp_ == nullptr) return;
    auto forward_trajs = p_bp_->forward_trajs();
    visualization_msgs::MarkerArray traj_list_marker;
    common::ColorARGB traj_color = common::cmap.at("gold");
    for (const auto& traj : forward_trajs) {
      std::vector<common::Point> points;
      for (const auto& v : traj) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        pt.z = 0.3;
        points.push_back(pt);
        visualization_msgs::Marker point_marker;
        // point_marker.ns = "point";
        common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
            common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, 0,
            &point_marker);
        traj_list_marker.markers.push_back(point_marker);
      }
      visualization_msgs::Marker line_marker;
      // line_marker.ns = "line";
      common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
          points, Vec3f(0.1, 0.1, 0.1), traj_color, 0, &line_marker);
      traj_list_marker.markers.push_back(line_marker);
    }
    int num_markers = static_cast<int>(traj_list_marker.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;
    forward_traj_vis_pub_.publish(traj_list_marker);
  }

 private:
  ros::NodeHandle nh_;
  int ego_id_;

  int last_forward_trajs_marker_cnt_ = 0;
  ros::Publisher forward_traj_vis_pub_;

  BehaviorPlanner* p_bp_{nullptr};
};

}  // namespace planning

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_