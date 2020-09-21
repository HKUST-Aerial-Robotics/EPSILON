#ifndef _CORE_HKUST_MSG_TRANSFORMER_HKUST_DECODER_H__
#define _CORE_HKUST_MSG_TRANSFORMER_HKUST_DECODER_H__

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
#include "hkust_msg_transformer/State.h"
#include "hkust_msg_transformer/Vins.h"

namespace hkust_msg_transformer {

class Decoder {
 public:
  static ErrorType GetStateFromTransformerState(
      const hkust_msg_transformer::State& msg, common::State* state) {
    state->vec_position(0) = msg.vec_position.x;
    state->vec_position(1) = msg.vec_position.y;
    state->angle = msg.angle;
    state->curvature = msg.curvature;
    state->velocity = msg.velocity;
    state->acceleration = msg.acceleration;
    state->steer = msg.steer;
    state->time_stamp = msg.header.stamp.toSec();
    return kSuccess;
  }

  static ErrorType GetVehicleFromTransformerVins(
      const hkust_msg_transformer::Vins& msg, common::Vehicle* vehicle) {
    vehicle->set_id(0);  // ego reserved id 0
    common::State state;
    GetStateFromTransformerState(msg.state, &state);
    state.time_stamp = msg.header.stamp.toSec();
    vehicle->set_state(state);
    return kSuccess;
  }

  static ErrorType GetVehicleFromTransformerMot(
      const hkust_msg_transformer::Mot& msg, common::Vehicle* vehicle) {
    if (msg.type.data != "\x03") {
      // TODO: filter non-vehicle objects
      return kWrongStatus;
    }

    vehicle->set_id(msg.id.data);
    common::VehicleParam param = vehicle->param();
    decimal_t width =
        fabs(msg.bbox_3d.right_back_top.y - msg.bbox_3d.left_front_bottom.y);
    decimal_t length =
        fabs(msg.bbox_3d.right_back_top.x - msg.bbox_3d.left_front_bottom.x);
    decimal_t esti_d_cr = 0.5 * length * 0.6;
    decimal_t esti_wheel_base =
        std::max(1.80, 2.0 * esti_d_cr);  // Smart Fortwo: 1.8
    param.set_width(width);
    param.set_length(length);
    param.set_d_cr(esti_d_cr);
    param.set_wheel_base(esti_wheel_base);
    param.set_front_suspension(
        std::max(0.0, length / 2 - (esti_wheel_base - esti_d_cr)));
    param.set_rear_suspension(std::max(0.0, length / 2 - esti_d_cr));
    vehicle->set_param(param);
    common::State state;
    GetStateFromTransformerState(msg.state, &state);
    // ~ remap
    // RemapUsingQuadraticFuncAroundSmallValue(2.0, state.velocity,
    //                                         &state.velocity);
    state.time_stamp = msg.header.stamp.toSec();
    decimal_t cos_theta = cos(state.angle);
    decimal_t sin_theta = sin(state.angle);
    state.vec_position(0) = state.vec_position(0) - param.d_cr() * cos_theta;
    state.vec_position(1) = state.vec_position(1) - param.d_cr() * sin_theta;
    vehicle->set_state(state);
    return kSuccess;
  }

  static ErrorType GetVehicleSetFromTransformerMotSet(
      const hkust_msg_transformer::MotSet& msg,
      common::VehicleSet* vehicle_set) {
    for (auto& mot : msg.mot_set) {
      common::Vehicle vehicle;
      if (GetVehicleFromTransformerMot(mot, &vehicle) != kSuccess) continue;
      vehicle_set->vehicles.insert(std::make_pair(vehicle.id(), vehicle));
    }
    return kSuccess;
  }

  static ErrorType GetLaneRawFromTransformerLane(
      const hkust_msg_transformer::Lane& msg, common::LaneRaw* lane_raw) {
    lane_raw->id = static_cast<int>(msg.id.data);
    if (msg.left_id.data > 0) {
      lane_raw->l_change_avbl = true;
    } else {
      lane_raw->l_change_avbl = false;
    }
    if (msg.right_id.data > 0) {
      lane_raw->r_change_avbl = true;
    } else {
      lane_raw->r_change_avbl = false;
    }
    lane_raw->l_lane_id = msg.left_id.data;
    lane_raw->r_lane_id = msg.right_id.data;
    decimal_t length = 0.0;
    int num_samples = static_cast<int>(msg.points.size());

    if (num_samples > 2) {
      lane_raw->start_point.x() = msg.points.front().x;
      lane_raw->start_point.y() = msg.points.front().y;
      lane_raw->final_point.x() = msg.points.back().x;
      lane_raw->final_point.y() = msg.points.back().y;
      for (int i = 0; i < num_samples; i++) {
        lane_raw->lane_points.push_back(
            Vec2f(msg.points[i].x, msg.points[i].y));
        if (i >= 1) {
          length += std::hypot(msg.points[i].x - msg.points[i - 1].x,
                               msg.points[i].y - msg.points[i - 1].y);
        }
      }
    }
    lane_raw->length = length;
    return kSuccess;
  }

  static ErrorType GetLaneNetFromTransformerLaneSet(
      const hkust_msg_transformer::LaneSet& msg, common::LaneNet* lane_net) {
    for (auto& lane : msg.lane_set) {
      common::LaneRaw lane_raw;
      GetLaneRawFromTransformerLane(lane, &lane_raw);
      lane_net->lane_set.insert(std::make_pair(lane_raw.id, lane_raw));
    }
    return kSuccess;
  }

  static ErrorType GetGridMap2DFromTransformerFreespace(
      const hkust_msg_transformer::Freespace& msg,
      common::GridMapND<uint8_t, 2>* p_grid) {
    std::array<decimal_t, 2> origin = {
        {msg.origin.position.x, msg.origin.position.y}};
    p_grid->set_origin(origin);
    p_grid->clear_data();
    int width = static_cast<int>(msg.width);
    int height = static_cast<int>(msg.height);
    for (int i = 0; i < width * height; i++) {
      if (msg.data[i] == 0) {
        *(p_grid->get_data_ptr() + i) = common::GridMapND<uint8_t, 2>::FREE;
      } else if (msg.data[i] == 253) {
        *(p_grid->get_data_ptr() + i) = common::GridMapND<uint8_t, 2>::UNKNOWN;
      } else if (msg.data[i] == 200) {
        *(p_grid->get_data_ptr() + i) = common::GridMapND<uint8_t, 2>::OCCUPIED;
      }
    }
    return kSuccess;
  }
};

}  // namespace hkust_msg_transformer

#endif  // _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__