#ifndef _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__
#define _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__

#include <algorithm>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "vehicle_msgs/ArenaInfo.h"
#include "vehicle_msgs/ArenaInfoDynamic.h"
#include "vehicle_msgs/ArenaInfoStatic.h"
#include "vehicle_msgs/Circle.h"
#include "vehicle_msgs/CircleObstacle.h"
#include "vehicle_msgs/ControlSignal.h"
#include "vehicle_msgs/FreeState.h"
#include "vehicle_msgs/Lane.h"
#include "vehicle_msgs/LaneNet.h"
#include "vehicle_msgs/ObstacleSet.h"
#include "vehicle_msgs/OccupancyGridFloat.h"
#include "vehicle_msgs/OccupancyGridUInt8.h"
#include "vehicle_msgs/PolygonObstacle.h"
#include "vehicle_msgs/State.h"
#include "vehicle_msgs/Vehicle.h"
#include "vehicle_msgs/VehicleParam.h"
#include "vehicle_msgs/VehicleSet.h"

namespace vehicle_msgs {
class Encoder {
 public:
  static ErrorType GetRosFreeStateMsgFromFreeState(
      const common::FreeState &in_state, const ros::Time &timestamp,
      vehicle_msgs::FreeState *state) {
    state->header.stamp = timestamp;
    state->header.frame_id = std::string("map");
    state->pos.x = in_state.position[0];
    state->pos.y = in_state.position[1];
    state->vel.x = in_state.velocity[0];
    state->vel.y = in_state.velocity[1];
    state->acc.x = in_state.acceleration[0];
    state->acc.y = in_state.acceleration[1];
    state->angle = in_state.angle;
    return kSuccess;
  }

  static ErrorType GetRosStateMsgFromState(const common::State &in_state,
                                           const ros::Time &timestamp,
                                           vehicle_msgs::State *state) {
    state->header.stamp = timestamp;
    state->header.frame_id = std::string("map");
    state->vec_position.x = in_state.vec_position[0];
    state->vec_position.y = in_state.vec_position[1];
    state->angle = in_state.angle;
    state->curvature = in_state.curvature;
    state->velocity = in_state.velocity;
    state->acceleration = in_state.acceleration;
    state->steer = in_state.steer;
    return kSuccess;
  }

  static ErrorType GetRosVehicleSetFromVehicleSet(
      const common::VehicleSet &vehicle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::VehicleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto &p : vehicle_set.vehicles) {
      vehicle_msgs::Vehicle vehicle_msg;
      GetRosVehicleFromVehicle(p.second, timestamp, frame_id, &vehicle_msg);
      msg->vehicles.push_back(vehicle_msg);
    }
    return kSuccess;
  }

  static ErrorType GetRosVehicleFromVehicle(const common::Vehicle &vehicle,
                                            const ros::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::Vehicle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id.data = vehicle.id();
    msg->subclass.data = vehicle.subclass();
    msg->type.data = vehicle.type();
    vehicle_msgs::VehicleParam param;
    GetRosVehicleParamFromVehicleParam(vehicle.param(), &param);
    msg->param = param;
    vehicle_msgs::State state;
    GetRosStateMsgFromState(vehicle.state(), timestamp, &state);
    msg->state = state;
    return kSuccess;
  }

  static ErrorType GetRosVehicleParamFromVehicleParam(
      const common::VehicleParam &vehicle_param,
      vehicle_msgs::VehicleParam *msg) {
    msg->width = vehicle_param.width();
    msg->length = vehicle_param.length();
    msg->wheel_base = vehicle_param.wheel_base();
    msg->front_suspension = vehicle_param.front_suspension();
    msg->rear_suspension = vehicle_param.rear_suspension();
    msg->max_steering_angle = vehicle_param.max_steering_angle();
    msg->max_longitudinal_acc = vehicle_param.max_longitudinal_acc();
    msg->max_lateral_acc = vehicle_param.max_lateral_acc();
    msg->d_cr = vehicle_param.d_cr();
    return kSuccess;
  }

  static ErrorType GetRosLaneNetFromLaneNet(const common::LaneNet &lane_net,
                                            const ros::Time &timestamp,
                                            const std::string &frame_id,
                                            vehicle_msgs::LaneNet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto lane_raw : lane_net.lane_set) {
      vehicle_msgs::Lane lane;
      GetRosLaneFromLaneRaw(lane_raw.second, timestamp, frame_id, &lane);
      msg->lanes.push_back(lane);
    }
    return kSuccess;
  }

  static ErrorType GetRosLaneFromLaneRaw(const common::LaneRaw &lane,
                                         const ros::Time &timestamp,
                                         const std::string &frame_id,
                                         vehicle_msgs::Lane *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = lane.id;
    msg->dir = lane.dir;
    msg->child_id = lane.child_id;
    msg->father_id = lane.father_id;
    msg->l_lane_id = lane.l_lane_id;
    msg->l_change_avbl = lane.l_change_avbl;
    msg->r_lane_id = lane.r_lane_id;
    msg->r_change_avbl = lane.r_change_avbl;
    msg->behavior = lane.behavior;
    msg->length = lane.length;
    msg->start_point.x = lane.start_point(0);
    msg->start_point.y = lane.start_point(1);
    msg->start_point.z = 0.0;
    msg->final_point.x = lane.final_point(0);
    msg->final_point.y = lane.final_point(1);
    msg->final_point.z = 0.0;
    for (const auto pt : lane.lane_points) {
      geometry_msgs::Point p;
      p.x = pt(0);
      p.y = pt(1);
      p.z = 0.0;
      msg->points.push_back(p);
    }
    return kSuccess;
  }

  static ErrorType GetRosObstacleSetFromObstacleSet(
      const common::ObstacleSet &obstacle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ObstacleSet *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    for (const auto obs : obstacle_set.obs_circle) {
      vehicle_msgs::CircleObstacle obs_temp;
      GetRosCircleObstacleFromCircleObstacle(obs.second, timestamp, frame_id,
                                             &obs_temp);
      msg->obs_circle.push_back(obs_temp);
    }
    for (const auto obs : obstacle_set.obs_polygon) {
      vehicle_msgs::PolygonObstacle obs_temp;
      GetRosPolygonObstacleFromPolygonObstacle(obs.second, timestamp, frame_id,
                                               &obs_temp);
      msg->obs_polygon.push_back(obs_temp);
    }
    return kSuccess;
  }

  static ErrorType GetRosCircleObstacleFromCircleObstacle(
      const common::CircleObstacle &circle, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::CircleObstacle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = circle.id;
    GetRosCircleFromCircle(circle.circle, &msg->circle);
    return kSuccess;
  }

  static ErrorType GetRosPolygonObstacleFromPolygonObstacle(
      const common::PolygonObstacle &poly, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::PolygonObstacle *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->id = poly.id;
    GetRosPolygonFromPolygon(poly.polygon, &msg->polygon);
    return kSuccess;
  }

  static ErrorType GetRosCircleFromCircle(const common::Circle &circle,
                                          vehicle_msgs::Circle *msg) {
    msg->center.x = circle.center.x;
    msg->center.y = circle.center.y;
    msg->center.z = 0.0;
    msg->radius = circle.radius;
    return kSuccess;
  }

  static ErrorType GetRosPolygonFromPolygon(const common::Polygon &poly,
                                            geometry_msgs::Polygon *msg) {
    for (const auto p : poly.points) {
      geometry_msgs::Point32 pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = 0.0;
      msg->points.push_back(pt);
    }
    return kSuccess;
  }

  static ErrorType GetRosArenaInfoFromSimulatorData(
      const common::LaneNet &lane_net, const common::VehicleSet &vehicle_set,
      const common::ObstacleSet &obstacle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ArenaInfo *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosLaneNetFromLaneNet(lane_net, timestamp, frame_id, &msg->lane_net);
    GetRosVehicleSetFromVehicleSet(vehicle_set, timestamp, frame_id,
                                   &msg->vehicle_set);
    GetRosObstacleSetFromObstacleSet(obstacle_set, timestamp, frame_id,
                                     &msg->obstacle_set);
    return kSuccess;
  }

  static ErrorType GetRosArenaInfoStaticFromSimulatorData(
      const common::LaneNet &lane_net, const common::ObstacleSet &obstacle_set,
      const ros::Time &timestamp, const std::string &frame_id,
      vehicle_msgs::ArenaInfoStatic *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosLaneNetFromLaneNet(lane_net, timestamp, frame_id, &msg->lane_net);
    GetRosObstacleSetFromObstacleSet(obstacle_set, timestamp, frame_id,
                                     &msg->obstacle_set);
    return kSuccess;
  }

  static ErrorType GetRosArenaInfoDynamicFromSimulatorData(
      const common::VehicleSet &vehicle_set, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ArenaInfoDynamic *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    GetRosVehicleSetFromVehicleSet(vehicle_set, timestamp, frame_id,
                                   &msg->vehicle_set);
    return kSuccess;
  }

  static ErrorType GetRosControlSignalFromControlSignal(
      const common::VehicleControlSignal &ctrl, const ros::Time &timestamp,
      const std::string &frame_id, vehicle_msgs::ControlSignal *msg) {
    msg->header.frame_id = frame_id;
    msg->header.stamp = timestamp;
    msg->acc = ctrl.acc;
    msg->steer_rate = ctrl.steer_rate;
    msg->is_openloop.data = ctrl.is_openloop;
    GetRosStateMsgFromState(ctrl.state, timestamp, &msg->state);
    return kSuccess;
  }
};  // class Encoder

}  // namespace vehicle_msgs

#endif  // _VEHICLE_MSGS_INC_VEHICLE_MSGS_ENCODER_H__