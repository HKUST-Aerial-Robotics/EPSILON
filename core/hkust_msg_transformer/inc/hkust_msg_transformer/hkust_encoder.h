#ifndef _CORE_HKUST_MSG_TRANSFORMER_HKUST_ENCODER_H__
#define _CORE_HKUST_MSG_TRANSFORMER_HKUST_ENCODER_H__

#include <algorithm>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"
#include "hkust_msg_transformer/Bbox3D.h"
#include "hkust_msg_transformer/Freespace.h"
#include "hkust_msg_transformer/Lane.h"
#include "hkust_msg_transformer/LaneSet.h"
#include "hkust_msg_transformer/Mot.h"
#include "hkust_msg_transformer/MotSet.h"
#include "hkust_msg_transformer/RawInput.h"
#include "hkust_msg_transformer/State.h"
#include "hkust_msg_transformer/Vins.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace hkust_msg_transformer {

class Encoder {
 public:
  static ErrorType GetTransformerRawInputFromSemanticmap(
      std::shared_ptr<semantic_map_manager::SemanticMapManager> map_ptr,
      hkust_msg_transformer::RawInput* raw_input) {
    if (map_ptr == nullptr) return kSuccess;
    raw_input->header.stamp = ros::Time(map_ptr->time_stamp());
    raw_input->header.frame_id = std::string("/map");

    // state
    {
      GetTransformerStateFromState(map_ptr->ego_vehicle().state(),
                                   &raw_input->state);
    }
    // motset
    {
      GetTransformerMotSetFromVehicleSet(map_ptr->surrounding_vehicles(),
                                         &raw_input->mot_set);
    }
    // laneset
    {
      GetTransformerLaneSetFromLaneNet(map_ptr->surrounding_lane_net(),
                                       &raw_input->lane_set);
    }
    // freespace
    {
      GetTransformerFreespaceFromGridMap2D(map_ptr->obstacle_map_ptr(),
                                           &raw_input->freespace);
    }

    return kSuccess;
  }

  static ErrorType GetTransformerStateFromState(
      const common::State& state, hkust_msg_transformer::State* msg) {
    msg->header.stamp = ros::Time(state.time_stamp);
    msg->header.frame_id = std::string("/map");
    msg->vec_position.x = state.vec_position(0);
    msg->vec_position.y = state.vec_position(1);
    msg->angle = state.angle;
    msg->curvature = state.curvature;
    msg->velocity = state.velocity;
    msg->acceleration = state.acceleration;
    msg->steer = state.steer;
    return kSuccess;
  }

  static ErrorType GetTransformerVinsFromVehicle(
      const common::Vehicle& vehicle, hkust_msg_transformer::Vins* msg) {
    return kSuccess;
  }

  static ErrorType GetTransformerMotFromVehicle(
      const common::Vehicle& vehicle, hkust_msg_transformer::Mot* msg) {
    msg->header.stamp = ros::Time(vehicle.state().time_stamp);
    msg->header.frame_id = std::string("/map");
    msg->id.data = vehicle.id();
    msg->type.data = std::string("\x03");
    // ~ in hkust msg, mot is represented by mass center
    common::State mass_center = vehicle.state();  // rear center
    decimal_t cos_theta = cos(mass_center.angle);
    decimal_t sin_theta = sin(mass_center.angle);
    mass_center.vec_position(0) =
        mass_center.vec_position(0) + vehicle.param().d_cr() * cos_theta;
    mass_center.vec_position(1) =
        mass_center.vec_position(1) + vehicle.param().d_cr() * sin_theta;
    GetTransformerStateFromState(mass_center, &msg->state);
    vec_E<Vec2f> vertices;
    vehicle.RetVehicleVertices(&vertices);
    msg->bbox_3d.left_front_bottom.x = 0.0;  // left front
    msg->bbox_3d.left_front_bottom.y = 0.0;
    msg->bbox_3d.right_back_top.x = vehicle.param().length();
    msg->bbox_3d.right_back_top.y = vehicle.param().width();
    return kSuccess;
  }

  static ErrorType GetTransformerMotSetFromVehicleSet(
      const common::VehicleSet& vehicle_set,
      hkust_msg_transformer::MotSet* msg) {
    // TODO: header not filled
    for (auto& id_vehicle : vehicle_set.vehicles) {
      hkust_msg_transformer::Mot mot;
      GetTransformerMotFromVehicle(id_vehicle.second, &mot);
      msg->mot_set.push_back(mot);
    }
    return kSuccess;
  }

  static ErrorType GetTransformerLaneFromLaneRaw(
      const common::LaneRaw& lane_raw, hkust_msg_transformer::Lane* msg) {
    // TODO: header not filled
    msg->id.data = lane_raw.id;
    msg->left_id.data = lane_raw.l_change_avbl ? lane_raw.l_lane_id : 0;
    msg->right_id.data = lane_raw.r_change_avbl ? lane_raw.r_lane_id : 0;
    if (lane_raw.child_id.size()) {
      msg->next_id.data = lane_raw.child_id[0];
    }
    int num_samples = lane_raw.lane_points.size();
    for (int i = 0; i < num_samples; i++) {
      geometry_msgs::Point pt;
      pt.x = lane_raw.lane_points[i][0];
      pt.y = lane_raw.lane_points[i][1];
      msg->points.push_back(pt);
    }
    return kSuccess;
  }

  static ErrorType GetTransformerLaneSetFromLaneNet(
      const common::LaneNet& lane_net, hkust_msg_transformer::LaneSet* msg) {
    for (auto& id_lane : lane_net.lane_set) {
      hkust_msg_transformer::Lane lane;
      GetTransformerLaneFromLaneRaw(id_lane.second, &lane);
      msg->lane_set.push_back(lane);
    }

    return kSuccess;
  }

  static ErrorType GetTransformerFreespaceFromGridMap2D(
      common::GridMapND<uint8_t, 2>* p_grid,
      hkust_msg_transformer::Freespace* msg) {
    msg->origin.position.x = p_grid->origin()[0];
    msg->origin.position.y = p_grid->origin()[1];
    int height = p_grid->dims_size(0);
    int width = p_grid->dims_size(1);
    msg->height = height;
    msg->width = width;
    msg->data.resize(height * width);
    for (int i = 0; i < width * height; i++) {
      if (*(p_grid->get_data_ptr() + i) ==
          common::GridMapND<uint8_t, 2>::FREE) {
        msg->data[i] = 0;
      } else if (*(p_grid->get_data_ptr() + i) ==
                 common::GridMapND<uint8_t, 2>::UNKNOWN) {
        msg->data[i] = 253;
      } else if (*(p_grid->get_data_ptr() + i) ==
                 common::GridMapND<uint8_t, 2>::OCCUPIED) {
        msg->data[i] = 200;
      }
    }

    return kSuccess;
  }
};

}  // namespace hkust_msg_transformer

#endif  // _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__