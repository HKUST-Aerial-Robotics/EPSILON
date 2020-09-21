#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_ROS_ADAPTER_H_

#include <assert.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <functional>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/rss/rss_checker.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/eudm_utils.h"
#include "hkust_msg_transformer/hkust_encoder.h"
#include "ros/ros.h"
#include "vehicle_msgs/decoder.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {

class EudmPlannerVisualizer {
 public:
  EudmPlannerVisualizer(ros::NodeHandle nh, EudmManager* p_bp_manager,
                        int ego_id)
      : nh_(nh), ego_id_(ego_id) {
    assert(p_bp_manager != nullptr);
    p_bp_manager_ = p_bp_manager;
  }

  void Init() {
    std::string forward_traj_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/forward_trajs");
    std::string lat_intention_topic = std::string("/vis/agent_") +
                                      std::to_string(ego_id_) +
                                      std::string("/predicted_lat_intention");
    std::string behavior_topic = std::string("/vis/agent_") +
                                 std::to_string(ego_id_) +
                                 std::string("/behavior_pc");
    std::string behavior_details_topic = std::string("/vis/agent_") +
                                         std::to_string(ego_id_) +
                                         std::string("/behavior_details");
    std::string opt_behavior_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/optimal_behavior");
    std::string surround_traj_topic = std::string("/vis/agent_") +
                                      std::to_string(ego_id_) +
                                      std::string("/surround_traj_opt");
    std::string opt_graph_topic = std::string("/vis/agent_") +
                                  std::to_string(ego_id_) +
                                  std::string("/opt_graph");
    std::string plain_output_topic = std::string("/hkust/plain_output");
    std::string raw_input_topic = std::string("/hkust/raw_input");
    std::string social_force_topic = std::string("/vis/agent_") +
                                     std::to_string(ego_id_) +
                                     std::string("/social_force_info");

    forward_traj_vis_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(forward_traj_topic, 1);
    lat_intention_vis_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(lat_intention_topic, 1);
    behavior_vis_pub_ =
        nh_.advertise<sensor_msgs::PointCloud>(behavior_topic, 1);
    behavior_details_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        behavior_details_topic, 1);
    surround_traj_vis_pub_ =
        nh_.advertise<sensor_msgs::PointCloud>(surround_traj_topic, 1);
    opt_behavior_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(opt_behavior_topic, 1);
    opt_graph_vis_pub_ =
        nh_.advertise<sensor_msgs::PointCloud>(opt_graph_topic, 1);
    plain_output_pub_ = nh_.advertise<hkust_msg_transformer::PlainOutput>(
        plain_output_topic, 1);
    hkust_raw_input_pub_ =
        nh_.advertise<hkust_msg_transformer::RawInput>(raw_input_topic, 1);
    social_force_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>(social_force_topic, 1);
  }

  void PublishDataWithStamp(const ros::Time& stamp) {
    VisualizeForwardTrajectories(stamp);
    // VisualizePredictedLateralIntentions(stamp);
    VisualizeBehavior(stamp);
    VisualizeOptBehavior(stamp);
    VisualizeSocialForce(stamp);
  }

  void VisualizeSocialForce(const ros::Time& stamp) {
    if (p_bp_manager_->planner().map_itf() == nullptr) return;
    static int last_mks_num = 0;

    // * current social force info
    planning::SocialForceModel::SocialForceInfo current_social_force;
    current_social_force = p_bp_manager_->planner().current_social_force();

    decimal_t z = 5.0;
    auto ego_state = p_bp_manager_->planner().plan_state();
    decimal_t cos_theta = cos(ego_state.angle);
    decimal_t sin_theta = sin(ego_state.angle);
    Vec3f ego_pt(ego_state.vec_position(0), ego_state.vec_position(1), z);
    visualization_msgs::MarkerArray mks;

    {
      // resultant_force
      Vec2f f_res = current_social_force.resultant_force;
      Vec3f f_vec = Vec3f(f_res(0), f_res(1), 0);
      visualization_msgs::Marker mk;
      common::VisualizationUtil::GetRosMarkerArrowUsingOriginAndVector(
          ego_pt, f_vec, &mk);
      common::VisualizationUtil::FillScaleColorInMarker(
          Vec3f(0.2, 0.5, 0), common::cmap.at("red"), &mk);
      mks.markers.push_back(mk);
    }

    {
      // decomposed force
      decimal_t f_lon = current_social_force.decomp_force(0);
      decimal_t f_lat = current_social_force.decomp_force(1);

      Vec3f f_lon_vec = Vec3f(cos_theta * f_lon, sin_theta * f_lon, 0);
      visualization_msgs::Marker mk1;
      common::VisualizationUtil::GetRosMarkerArrowUsingOriginAndVector(
          ego_pt, f_lon_vec, &mk1);
      common::VisualizationUtil::FillScaleColorInMarker(
          Vec3f(0.2, 0.5, 0), common::cmap.at("orange"), &mk1);
      mks.markers.push_back(mk1);

      Vec3f f_lat_vec = Vec3f(-sin_theta * f_lat, cos_theta * f_lat, 0);
      visualization_msgs::Marker mk2;
      common::VisualizationUtil::GetRosMarkerArrowUsingOriginAndVector(
          ego_pt, f_lat_vec, &mk2);
      common::VisualizationUtil::FillScaleColorInMarker(
          Vec3f(0.2, 0.5, 0), common::cmap.at("orange"), &mk2);
      mks.markers.push_back(mk2);
    }

    {
      // force set
      for (const auto& entry : current_social_force.force_set) {
        Vec2f f = entry.second;
        Vec3f f_vec = Vec3f(f(0), f(1), 0);
        visualization_msgs::Marker mk;
        common::VisualizationUtil::GetRosMarkerArrowUsingOriginAndVector(
            ego_pt - Vec3f(0, 0, 0.5), f_vec, &mk);

        auto it = common::cmap.begin();
        int random_index = entry.first % common::cmap.size();
        std::advance(it, random_index);

        common::VisualizationUtil::FillScaleColorInMarker(
            Vec3f(0.2, 0.5, 0), it->second.set_a(0.8), &mk);
        mks.markers.push_back(mk);
      }
    }

    int num_markers = static_cast<int>(mks.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("/map"), last_mks_num, &mks);
    last_mks_num = num_markers;

    social_force_pub_.publish(mks);
  }

  void VisualizeOptBehavior(const ros::Time& stamp) {
    if (p_bp_manager_->planner().map_itf() == nullptr) return;

    planning::eudm::PlainOutput plain_output;
    p_bp_manager_->ConstructPlainOutput(&plain_output);
    visualization_msgs::MarkerArray mk;
    planning::eudm::EudmUtils::ConvertOptimalBehaviorToMarker(plain_output,
                                                              &mk);
    opt_behavior_pub_.publish(mk);

    visualization_msgs::MarkerArray mk_details;
    planning::eudm::EudmUtils::ConvertEudmOutputDetailsToMarker(plain_output,
                                                                &mk_details);
    int num_details_cnt = static_cast<int>(mk_details.markers.size());
    common::VisualizationUtil::FillHeaderIdInMarkerArray(
        stamp, std::string("/map"), last_details_cnt_, &mk_details);
    last_details_cnt_ = num_details_cnt;
    behavior_details_vis_pub_.publish(mk_details);

    sensor_msgs::PointCloud opt_pc;
    planning::eudm::EudmUtils::ConvertOptimalBehaviorToPointCloud(plain_output,
                                                                  &opt_pc);
    opt_graph_vis_pub_.publish(opt_pc);

    // publish data using hkust proto
    {
      hkust_msg_transformer::RawInput raw_input;
      hkust_msg_transformer::Encoder::GetTransformerRawInputFromSemanticmap(
          p_bp_manager_->map(), &raw_input);
      hkust_raw_input_pub_.publish(raw_input);

      hkust_msg_transformer::PlainOutput out_msg;
      planning::eudm::EudmUtils::ConvertPlainOutputToMsg(plain_output,
                                                         &out_msg);
      plain_output_pub_.publish(out_msg);
    }
  }

  void VisualizeBehavior(const ros::Time& stamp) {
    if (p_bp_manager_->planner().map_itf() == nullptr) return;

    planning::eudm::PlainOutput plain_output;
    p_bp_manager_->ConstructPlainOutput(&plain_output);
    sensor_msgs::PointCloud ego_pc, traj_pc;
    planning::eudm::EudmUtils::ConvertPlainOutputToPointCloud(
        plain_output, &ego_pc, &traj_pc);
    ego_pc.header.stamp = stamp;
    behavior_vis_pub_.publish(ego_pc);

    traj_pc.header.stamp = stamp;
    surround_traj_vis_pub_.publish(traj_pc);
  }

  void VisualizeForwardTrajectories(const ros::Time& stamp) {
    auto forward_trajs = p_bp_manager_->planner().forward_trajs();
    int processed_winner_id = p_bp_manager_->processed_winner_id();
    int original_winner_id = p_bp_manager_->original_winner_id();
    visualization_msgs::MarkerArray traj_list_marker;
    common::ColorARGB traj_color(0.5, 0.5, 0.5, 0.5);
    double traj_z = 0.3;
    for (int i = 0; i < static_cast<int>(forward_trajs.size()); ++i) {
      if (i == processed_winner_id) {
        traj_color = common::cmap.at("gold");
        traj_z = 0.4;
      } else if (i == original_winner_id) {
        traj_color = common::cmap.at("spring green");
        traj_z = 0.4;
      } else {
        traj_color = common::ColorARGB(0.5, 0.5, 0.5, 0.5);
        traj_z = 0.3;
      }
      std::vector<common::Point> points;
      for (const auto& v : forward_trajs[i]) {
        common::Point pt(v.state().vec_position(0), v.state().vec_position(1));
        // pt.z = v.state().time_stamp -
        // forward_trajs[i].front().state().time_stamp;
        pt.z = traj_z;
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
        stamp, std::string("/map"), last_forward_trajs_marker_cnt_,
        &traj_list_marker);
    last_forward_trajs_marker_cnt_ = num_markers;
    forward_traj_vis_pub_.publish(traj_list_marker);
  }

  void set_use_sim_state(bool use_sim_state) { use_sim_state_ = use_sim_state; }

 private:
  ros::NodeHandle nh_;
  int ego_id_;
  bool use_sim_state_ = true;

  int last_forward_trajs_marker_cnt_ = 0;
  int last_rss_status_marker_cnt_ = 0;
  int last_lat_intention_marker_cnt_ = 0;
  int last_details_cnt_ = 0;

  ros::Publisher forward_traj_vis_pub_;
  ros::Publisher rss_vis_pub_;
  ros::Publisher lat_intention_vis_pub_;
  ros::Publisher behavior_vis_pub_;
  ros::Publisher behavior_details_vis_pub_;
  ros::Publisher surround_traj_vis_pub_;
  ros::Publisher opt_behavior_pub_;
  ros::Publisher opt_graph_vis_pub_;
  ros::Publisher plain_output_pub_;
  ros::Publisher hkust_raw_input_pub_;
  ros::Publisher social_force_pub_;
  EudmManager* p_bp_manager_{nullptr};
};

}  // namespace planning

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_ROS_ADAPTER_H_