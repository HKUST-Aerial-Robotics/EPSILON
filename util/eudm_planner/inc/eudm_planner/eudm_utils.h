#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_UTILS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_UTILS_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "eudm_planner/eudm_itf.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "visualization_msgs/MarkerArray.h"
namespace planning {

namespace eudm {

class EudmUtils {
 public:
  static void InsertChannelNames(sensor_msgs::PointCloud* pc) {
    const int kChannelNum = 11;
    pc->channels.resize(kChannelNum);
    pc->channels[0].name = "cost_final";
    pc->channels[1].name = "cost_total";
    pc->channels[2].name = "total_efficiency";
    pc->channels[3].name = "total_safety";
    pc->channels[4].name = "total_navigation";
    pc->channels[5].name = "per_efficiency";
    pc->channels[6].name = "per_safety";
    pc->channels[7].name = "per_navigation";
    pc->channels[8].name = "per_weight";
    pc->channels[9].name = "per_lat_behavior";
    pc->channels[10].name = "per_lon_behavior";
  }

  static void ConvertEgoTrajAndCostToPointCloud(
      const planning::eudm::OutputElement& element,
      const double vis_vertical_level, sensor_msgs::PointCloud* pc) {
    // * this corresponds to one action sequence
    if (not element.valid) return;
    int num_macro_actions = element.lat_behaviors.size();
    double cost_final = 0.0, cost_total = 0.0;
    double total_efficiency = 0.0, total_safety = 0.0, total_navigation = 0.0;
    for (auto& cost : element.stage_costs) {
      total_efficiency += cost.efficiency * cost.weight;
      total_safety += cost.safety * cost.weight;
      total_navigation += cost.navigation * cost.weight;
      cost_total +=
          (cost.efficiency + cost.safety + cost.navigation) * cost.weight;
    }
    cost_final = element.final_cost;

    int sample_index_lb = 0;
    for (int i = 0; i < num_macro_actions; ++i) {
      int sample_index_ub = element.stage_costs[i].valid_sample_index_ub;
      double per_efficiency = 0.0, per_safety = 0.0, per_navigation = 0.0,
             per_weight = 0.0, per_lat_behavior = 0.0, per_lon_behavior = 0.0;
      per_efficiency = element.stage_costs[i].efficiency;
      per_safety = element.stage_costs[i].safety;
      per_navigation = element.stage_costs[i].navigation;
      per_weight = element.stage_costs[i].weight;
      if (element.lat_behaviors[i] == 1) {
        per_lat_behavior = 0.5;
      } else if (element.lat_behaviors[i] == 2) {
        per_lat_behavior = 0.0;
      } else if (element.lat_behaviors[i] == 3) {
        per_lat_behavior = 1.0;
      }
      if (element.lon_behaviors[i] == 0) {
        per_lon_behavior = 0.5;
      } else if (element.lon_behaviors[i] == 1) {
        per_lon_behavior = 0.0;
      } else if (element.lon_behaviors[i] == 2) {
        per_lon_behavior = 1.0;
      }

      for (int j = sample_index_lb; j < sample_index_ub; ++j) {
        auto plain_ego_state = element.ego_traj[j];
        geometry_msgs::Point32 pt32;
        pt32.x = plain_ego_state.x;
        pt32.y = plain_ego_state.y;
        pt32.z = vis_vertical_level;
        pc->points.push_back(pt32);
        pc->channels[0].values.push_back(cost_final);  // "cost_final"
        pc->channels[1].values.push_back(cost_total);  // "cost_total";
        pc->channels[2].values.push_back(
            total_efficiency);                           // "total_efficiency";
        pc->channels[3].values.push_back(total_safety);  // "total_safety";
        pc->channels[4].values.push_back(
            total_navigation);  // "total_navigation";
        pc->channels[5].values.push_back(per_efficiency);  // "per_efficiency";
        pc->channels[6].values.push_back(per_safety);      // "per_safety";
        pc->channels[7].values.push_back(per_navigation);  // "per_navigation";
        pc->channels[8].values.push_back(per_weight);      // "per_weight";
        pc->channels[9].values.push_back(
            per_lat_behavior);  // "per_lat_behavior";
        pc->channels[10].values.push_back(
            per_lon_behavior);  // "per_lon_behavior";
      }
      sample_index_lb = sample_index_ub;
    }
  }

  static void ConvertVehicleTrajToPointCloud(
      const planning::eudm::OutputElement& element,
      sensor_msgs::PointCloud* pc) {
    if (not element.valid) return;
    pc->header.frame_id = "/map";
    pc->header.stamp = ros::Time::now();
    pc->channels.resize(1);
    pc->channels[0].name = "time";
    for (auto it = element.surround_trajs.begin();
         it != element.surround_trajs.end(); it++) {
      // for each vehicle
      int num_points = it->second.size();
      for (int i = 0; i < num_points; i++) {
        auto plain_state = it->second[i];
        geometry_msgs::Point32 pt32;
        pt32.x = plain_state.x;
        pt32.y = plain_state.y;
        pt32.z = 0.0;
        pc->points.push_back(pt32);
        pc->channels[0].values.push_back(i);
      }
    }
    int num_points = element.ego_traj.size();
    for (int i = 0; i < num_points; i++) {
      auto plain_state = element.ego_traj[i];
      geometry_msgs::Point32 pt32;
      pt32.x = plain_state.x;
      pt32.y = plain_state.y;
      pt32.z = 0.0;
      pc->points.push_back(pt32);
      pc->channels[0].values.push_back(i);
    }
  }

  static void ConvertPlainOutputToPointCloud(
      const planning::eudm::PlainOutput& plain_out,
      sensor_msgs::PointCloud* ego_pc, sensor_msgs::PointCloud* traj_pc) {
    // convert plain output to pretty pointcloud message
    if (plain_out.elements.empty()) return;
    if (not plain_out.valid) return;

    int num_elements = plain_out.elements.size();
    static const double offset_resolution = 0.5;
    InsertChannelNames(ego_pc);
    for (int i = 0; i < num_elements; ++i) {
      sensor_msgs::PointCloud pc;
      InsertChannelNames(&pc);
      ConvertEgoTrajAndCostToPointCloud(plain_out.elements[i],
                                        i * offset_resolution, &pc);
      ego_pc->points.insert(ego_pc->points.end(), pc.points.begin(),
                            pc.points.end());
      for (size_t j = 0; j < ego_pc->channels.size(); ++j) {
        ego_pc->channels[j].values.insert(ego_pc->channels[j].values.end(),
                                          pc.channels[j].values.begin(),
                                          pc.channels[j].values.end());
      }
    }
    ego_pc->header.frame_id = "/map";
    ego_pc->header.stamp = ros::Time::now();

    ConvertVehicleTrajToPointCloud(
        plain_out.elements[plain_out.winner_element_id], traj_pc);
  }

  static void ConvertOptimalBehaviorToPointCloud(
      const planning::eudm::PlainOutput& plain_out,
      sensor_msgs::PointCloud* pc) {
    if (plain_out.elements.empty()) return;
    if (not plain_out.valid) return;

    pc->header.frame_id = "/opt_graph";
    pc->header.stamp = ros::Time::now();
    pc->channels.resize(1);
    pc->channels[0].name = "channel";

    auto opt_traj = plain_out.elements[plain_out.winner_element_id].ego_traj;
    auto lat_behs =
        plain_out.elements[plain_out.winner_element_id].lat_behaviors;
    auto lon_behs =
        plain_out.elements[plain_out.winner_element_id].lon_behaviors;

    double start_time = opt_traj.front().timestamp;
    // angle
    for (const auto& state : opt_traj) {
      geometry_msgs::Point32 pt;
      pt.x = state.timestamp - start_time;
      pt.y = state.angle;
      pt.z = 0.0;
      pc->points.push_back(pt);
      pc->channels[0].values.push_back(0);
    }

    // velocity
    for (const auto& state : opt_traj) {
      geometry_msgs::Point32 pt;
      pt.x = 10 + state.timestamp - start_time;
      pt.y = state.vel / 10;
      pt.z = 0.0;
      pc->points.push_back(pt);
      pc->channels[0].values.push_back(1);
    }

    // acc
    for (const auto& state : opt_traj) {
      geometry_msgs::Point32 pt;
      pt.x = 20 + state.timestamp - start_time;
      pt.y = state.acc;
      pt.z = 0.0;
      pc->points.push_back(pt);
      pc->channels[0].values.push_back(2);
    }

    // lat_acc
    for (const auto& state : opt_traj) {
      geometry_msgs::Point32 pt;
      pt.x = 30 + state.timestamp - start_time;
      pt.y = state.vel * state.vel * state.curvature;
      pt.z = 0.0;
      pc->points.push_back(pt);
      pc->channels[0].values.push_back(3);
    }

    // behavior
    double x_offs = 40;
    for (const auto& b : lat_behs) {
      geometry_msgs::Point32 pt;
      pt.x = x_offs++;
      pt.y = b;
      pt.z = 0.0;
      pc->points.push_back(pt);
      pc->channels[0].values.push_back(4);
    }
    x_offs = 50;
    for (const auto& b : lon_behs) {
      geometry_msgs::Point32 pt;
      pt.x = x_offs++;
      pt.y = b;
      pt.z = 0.0;
      pc->points.push_back(pt);
      pc->channels[0].values.push_back(5);
    }
  }

  static void ConvertOptimalBehaviorToMarker(
      const planning::eudm::PlainOutput& plain_out,
      visualization_msgs::MarkerArray* markers) {
    if (plain_out.elements.empty()) return;
    if (not plain_out.valid) return;
    static const double offset_resolution = 0.5;
    // int num_elements = plain_out.elements.size();
    auto optimal_element = plain_out.elements[plain_out.winner_element_id];
    visualization_msgs::Marker marker_line;
    marker_line.header.frame_id = "/map";
    marker_line.header.stamp = ros::Time::now();
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.action = visualization_msgs::Marker::MODIFY;
    marker_line.pose.position.x = 0.0;
    marker_line.pose.position.y = 0.0;
    marker_line.pose.position.z = 0.0;
    marker_line.pose.orientation.w = 1.0;
    marker_line.pose.orientation.x = 0.0;
    marker_line.pose.orientation.y = 0.0;
    marker_line.pose.orientation.z = 0.0;
    marker_line.color.a = 1.0;
    marker_line.color.r = 0.0;
    marker_line.color.g = 0.0;
    marker_line.color.b = 0.8;
    marker_line.scale.x = 0.2;
    marker_line.id = 0;
    for (auto& state : optimal_element.ego_traj) {
      geometry_msgs::Point point;
      point.x = state.x;
      point.y = state.y;
      point.z = offset_resolution * plain_out.winner_element_id;
      marker_line.points.push_back(point);
    }
    markers->markers.push_back(marker_line);

    visualization_msgs::Marker marker_txt;
    marker_txt.header.frame_id = "/map";
    marker_txt.header.stamp = ros::Time::now();
    marker_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_txt.action = visualization_msgs::Marker::MODIFY;
    marker_txt.id = 1;

    auto first_pos = optimal_element.ego_traj.front();
    marker_txt.pose.position.x = first_pos.x;
    marker_txt.pose.position.y = first_pos.y;
    marker_txt.pose.position.z =
        offset_resolution * plain_out.winner_element_id;
    FillColorInMarker(1.0, 0.0, 1.0, 0.0, &marker_txt);
    FillScaleInMarker(0.8, 0.8, 0.8, &marker_txt);
    std::ostringstream txt;

    txt << "A: Lat[";
    for (const auto& lat : optimal_element.lat_cmds) {
      txt << RetLatActionName(lat);
    }
    txt << "] Lon[";
    for (const auto& lon : optimal_element.lon_cmds) {
      txt << RetLonActionName(lon);
    }
    txt << "]";

    marker_txt.text = txt.str();
    markers->markers.push_back(marker_txt);
  }

  static void ConvertEudmOutputDetailsToMarker(
      const planning::eudm::PlainOutput& plain_out,
      visualization_msgs::MarkerArray* markers) {
    if (plain_out.elements.empty()) return;
    if (not plain_out.valid) return;
    static const double offset_resolution = 0.5;
    auto timestamp = ros::Time::now();

    auto opt_ele = plain_out.elements[plain_out.winner_element_id];
    if (!opt_ele.valid) return;
    auto pos = opt_ele.ego_traj.front();

    double d = 15.0;
    double x_offset = d * sin(pos.angle);
    double y_offset = -d * cos(pos.angle);

    for (int i = 0; i < plain_out.elements.size(); ++i) {
      auto ele = plain_out.elements[i];
      // if (!ele.valid) continue;

      std::array<double, 4> clr;
      if (ele.valid) {
        clr = {0.8, 0.8, 0.8, 0.8};
      } else {
        clr = {0.5, 0.5, 0.5, 0.5};
      }

      visualization_msgs::Marker m_txt;
      m_txt.header.frame_id = "/map";
      m_txt.header.stamp = timestamp;
      m_txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      m_txt.action = visualization_msgs::Marker::MODIFY;
      m_txt.id = i;

      m_txt.pose.position.x = x_offset + pos.x;
      m_txt.pose.position.y = y_offset + pos.y;
      m_txt.pose.position.z = offset_resolution * i;

      if (i == plain_out.winner_element_id) {
        FillColorInMarker(1.0, 1.0, 1.0, 0.0, &m_txt);
        FillScaleInMarker(0.8, 0.8, 0.8, &m_txt);
      } else {
        FillColorInMarker(clr[0], clr[1], clr[2], clr[3], &m_txt);
        FillScaleInMarker(0.5, 0.5, 0.5, &m_txt);
      }

      std::ostringstream txt;
      txt << std::to_string(i) << " ";
      txt << "A: Lat[";
      for (const auto& lat : ele.lat_cmds) {
        txt << RetLatActionName(lat);
      }
      txt << "] Lon[";
      for (const auto& lon : ele.lon_cmds) {
        txt << RetLonActionName(lon);
      }
      txt << "], cost: " << ele.final_cost;

      if (!ele.sim_info.empty()) {
        m_txt.text = txt.str() + ", " + ele.sim_info;
      } else {
        m_txt.text = txt.str();
      }
      markers->markers.push_back(m_txt);
    }
  }

  static void FillColorInMarker(const double a, const double r, const double g,
                                const double b,
                                visualization_msgs::Marker* marker) {
    marker->color.a = a;
    marker->color.r = r;
    marker->color.g = g;
    marker->color.b = b;
    return;
  }

  static void FillScaleInMarker(const double x, const double y, const double z,
                                visualization_msgs::Marker* marker) {
    marker->scale.x = x;
    marker->scale.y = y;
    marker->scale.z = z;
    return;
  }

  static std::vector<int> RetLatActionIndex(const std::string cmds) {
    std::vector<int> ret;
    for (const char& c : cmds) {
      if (c == 'K') {
        ret.push_back(0);
      } else if (c == 'L') {
        ret.push_back(1);
      } else if (c == 'R') {
        ret.push_back(2);
      } else {
        assert(false);
      }
    }
    return ret;
  }

  static std::string RetLatActionName(const int a) {
    std::string a_str;
    switch (a) {
      case 0: {
        a_str = std::string("K");
        break;
      }
      case 1: {
        a_str = std::string("L");
        break;
      }
      case 2: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  static std::vector<int> RetLatBehaviorIndex(const std::string cmds) {
    std::vector<int> ret;
    for (const char& c : cmds) {
      if (c == 'K') {
        ret.push_back(1);
      } else if (c == 'L') {
        ret.push_back(2);
      } else if (c == 'R') {
        ret.push_back(3);
      } else {
        assert(false);
      }
    }
    return ret;
  }

  static std::string RetLatBehaviorName(const int a) {
    std::string a_str;
    switch (a) {
      case 1: {
        a_str = std::string("K");
        break;
      }
      case 2: {
        a_str = std::string("L");
        break;
      }
      case 3: {
        a_str = std::string("R");
        break;
      }
      default: {
        assert(false);
        break;
      }
    }
    return a_str;
  }

  static std::vector<int> RetLonActionIndex(const std::string cmds) {
    std::vector<int> ret;
    for (const char& c : cmds) {
      if (c == 'M') {
        ret.push_back(0);
      } else if (c == 'A') {
        ret.push_back(1);
      } else if (c == 'D') {
        ret.push_back(2);
      } else {
        assert(false);
      }
    }
    return ret;
  }

  static std::string RetLonActionName(const int a) {
    std::string a_str;
    switch (a) {
      case 0: {
        a_str = std::string("M");
        break;
      }
      case 1: {
        a_str = std::string("A");
        break;
      }
      case 2: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  static void GetPlainStateFromMsg(const hkust_msg_transformer::State& msg,
                                   PlainState* data) {
    data->timestamp = msg.header.stamp.toSec();
    data->x = msg.vec_position.x;
    data->y = msg.vec_position.y;
    data->angle = msg.angle;
    data->vel = msg.velocity;
    data->acc = msg.acceleration;
    data->curvature = msg.curvature;
  }

  static void GetStateMsgFromPlainState(const PlainState& state,
                                        hkust_msg_transformer::State* msg) {
    msg->header.stamp = ros::Time(state.timestamp);
    msg->vec_position.x = state.x;
    msg->vec_position.y = state.y;
    msg->angle = state.angle;
    msg->velocity = state.vel;
    msg->acceleration = state.acc;
    msg->curvature = state.curvature;
  }

  static void GetPlainCostMsgFromPlainCost(
      const PlainCost& cost, hkust_msg_transformer::PlainCost* msg) {
    msg->valid_sample_index_ub = cost.valid_sample_index_ub;
    msg->efficiency = cost.efficiency;
    msg->safety = cost.safety;
    msg->navigation = cost.navigation;
    msg->weight = cost.weight;
  }

  static void GetPlainCostFromMsg(const hkust_msg_transformer::PlainCost& msg,
                                  PlainCost* cost) {
    cost->valid_sample_index_ub = msg.valid_sample_index_ub;
    cost->efficiency = msg.efficiency;
    cost->safety = msg.safety;
    cost->navigation = msg.navigation;
    cost->weight = msg.weight;
  }

  static void ConvertPlainOutputMsgToPlainOutput(
      const hkust_msg_transformer::PlainOutput& msg, PlainOutput* data) {
    data->plan_stamp = msg.header.stamp.toSec();
    data->time_cost = msg.time_cost;
    data->valid = msg.valid;
    data->winner_element_id = msg.winner_id;
    data->best_cost = msg.winner_cost;
    data->original_winner_id = msg.original_winner_id;
    data->ongoing_action_duration = msg.ongoing_action_duration;

    GetPlainStateFromMsg(msg.ego_state, &data->ego_state);

    for (int i = 0; i < static_cast<int>(msg.elements.size()); i++) {
      auto ele_msg = msg.elements[i];
      OutputElement ele;
      ele.valid = ele_msg.valid;
      ele.risky = ele_msg.risky;
      ele.final_cost = ele_msg.final_cost;
      ele.sim_info = ele_msg.sim_info;
      ele.lat_cmds = RetLatActionIndex(ele_msg.lat_cmds);
      ele.lat_behaviors = RetLatBehaviorIndex(ele_msg.lat_behaviors);
      ele.lon_cmds = RetLonActionIndex(ele_msg.lon_cmds);
      ele.lon_behaviors = RetLonActionIndex(ele_msg.lon_behaviors);
      // state costs
      for (const auto& cost_msg : ele_msg.stage_costs) {
        PlainCost cost;
        GetPlainCostFromMsg(cost_msg, &cost);
        ele.stage_costs.push_back(cost);
      }

      // ego traj
      int num_states = static_cast<int>(ele_msg.ego_traj.traj.size());
      for (const auto& s_msg : ele_msg.ego_traj.traj) {
        PlainState state;
        GetPlainStateFromMsg(s_msg, &state);
        ele.ego_traj.push_back(state);
      }
      // surrounding traj
      for (const auto traj_msg : ele_msg.surround_trajs) {
        std::vector<PlainState> traj;
        for (const auto& s_msg : traj_msg.traj) {
          PlainState state;
          GetPlainStateFromMsg(s_msg, &state);
          traj.push_back(state);
        }
        ele.surround_trajs.insert(std::make_pair(traj_msg.id, traj));
      }
      data->elements.push_back(ele);
    }
  }

  static void ConvertPlainOutputToMsg(const PlainOutput& data,
                                      hkust_msg_transformer::PlainOutput* msg) {
    msg->header.frame_id = "map";
    msg->header.stamp = ros::Time(data.plan_stamp);
    msg->time_cost = data.time_cost;
    msg->valid = data.valid;
    msg->winner_id = data.winner_element_id;
    msg->winner_cost = data.best_cost;
    msg->original_winner_id = data.original_winner_id;
    msg->ongoing_action_duration = data.ongoing_action_duration;

    GetStateMsgFromPlainState(data.ego_state, &msg->ego_state);

    for (int i = 0; i < data.elements.size(); ++i) {
      auto ele = data.elements[i];
      hkust_msg_transformer::PlainElement ele_msg;
      ele_msg.id = i;
      ele_msg.valid = ele.valid;
      ele_msg.risky = ele.risky;
      ele_msg.final_cost = ele.final_cost;
      ele_msg.sim_info = ele.sim_info;

      std::string lat_cmds_str;
      for (const auto& c : ele.lat_cmds) {
        lat_cmds_str += RetLatActionName(c);
      }
      ele_msg.lat_cmds = lat_cmds_str;

      std::string lat_beh_str;
      for (const auto& c : ele.lat_behaviors) {
        lat_beh_str += RetLatBehaviorName(c);
      }
      ele_msg.lat_behaviors = lat_beh_str;

      std::string lon_cmds_str;
      for (const auto& c : ele.lon_cmds) {
        lon_cmds_str += RetLonActionName(c);
      }
      ele_msg.lon_cmds = lon_cmds_str;

      std::string lon_beh_str;
      for (const auto& c : ele.lon_behaviors) {
        lon_beh_str += RetLonActionName(c);
      }
      ele_msg.lon_behaviors = lon_beh_str;

      // stage costs
      for (const auto& cost : ele.stage_costs) {
        hkust_msg_transformer::PlainCost cost_msg;
        GetPlainCostMsgFromPlainCost(cost, &cost_msg);
        ele_msg.stage_costs.push_back(cost_msg);
      }

      ele_msg.ego_traj.id = i;
      for (const auto& state : ele.ego_traj) {
        hkust_msg_transformer::State s_msg;
        GetStateMsgFromPlainState(state, &s_msg);
        ele_msg.ego_traj.traj.push_back(s_msg);
      }

      for (const auto traj : ele.surround_trajs) {
        hkust_msg_transformer::PlainTrajectory traj_msg;
        traj_msg.id = traj.first;
        for (const auto& state : traj.second) {
          hkust_msg_transformer::State s_msg;
          GetStateMsgFromPlainState(state, &s_msg);
          traj_msg.traj.push_back(s_msg);
        }
        ele_msg.surround_trajs.push_back(traj_msg);
      }
      msg->elements.push_back(ele_msg);
    }
  }
};

}  // namespace eudm

}  // namespace planning

#endif