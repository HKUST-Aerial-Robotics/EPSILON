/**
 * @file arena_loader.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#include "phy_simulator/arena_loader.h"

namespace phy_simulator {

using Json = nlohmann::json;

ArenaLoader::ArenaLoader() {}

ArenaLoader::ArenaLoader(const std::string &vehicle_set_path,
                         const std::string &map_path,
                         const std::string &lane_net_path)
    : vehicle_set_path_(vehicle_set_path),
      map_path_(map_path),
      lane_net_path_(lane_net_path) {}

bool ArenaLoader::ParseVehicleSet(common::VehicleSet *p_vehicle_set) {
  printf("\n[ArenaLoader] Loading vehicle set\n");

  std::fstream fs(vehicle_set_path_);
  Json root;
  fs >> root;

  Json vehicles_json = root["vehicles"];
  Json info_json = vehicles_json["info"];
  // ~ allow loading part of the vehicles for debugging purpose
  for (int i = 0; i < static_cast<int>(info_json.size()); ++i) {
    common::Vehicle vehicle;
    vehicle.set_id(info_json[i]["id"].get<int>());
    vehicle.set_subclass(info_json[i]["subclass"].get<std::string>());
    vehicle.set_type(info_json[i]["type"].get<std::string>());

    Json state_json = info_json[i]["init_state"];
    common::State state;
    state.vec_position(0) = state_json["x"].get<double>();
    state.vec_position(1) = state_json["y"].get<double>();
    state.angle = state_json["angle"].get<double>();
    state.curvature = state_json["curvature"].get<double>();
    state.velocity = state_json["velocity"].get<double>();
    state.acceleration = state_json["acceleration"].get<double>();
    state.steer = state_json["steer"].get<double>();
    vehicle.set_state(state);

    Json params_json = info_json[i]["params"];
    common::VehicleParam param;
    param.set_width(params_json["width"].get<double>());
    param.set_length(params_json["length"].get<double>());
    param.set_wheel_base(params_json["wheel_base"].get<double>());
    param.set_front_suspension(params_json["front_suspension"].get<double>());
    param.set_rear_suspension(params_json["rear_suspension"].get<double>());
    auto max_steering_angle = params_json["max_steering_angle"].get<double>();
    param.set_max_steering_angle(max_steering_angle * kPi / 180.0);
    param.set_max_longitudinal_acc(
        params_json["max_longitudinal_acc"].get<double>());
    param.set_max_lateral_acc(params_json["max_lateral_acc"].get<double>());

    param.set_d_cr(param.length() / 2 - param.rear_suspension());
    vehicle.set_param(param);

    p_vehicle_set->vehicles.insert(
        std::pair<int, common::Vehicle>(vehicle.id(), vehicle));
  }
  p_vehicle_set->print();

  fs.close();
  return true;
}

ErrorType ArenaLoader::ParseMapInfo(common::ObstacleSet *p_obstacle_set) {
  printf("\n[ArenaLoader] Loading map info\n");

  std::fstream fs(map_path_);
  Json root;
  fs >> root;

  Json obstacles_json = root["features"];
  for (int i = 0; i < static_cast<int>(obstacles_json.size()); ++i) {
    Json obs = obstacles_json[i];
    printf("Obstacle id %d.\n", obs["properties"]["id"].get<int>());
    auto is_valid = obs["properties"]["is_valid"].get<int>();
    if (!static_cast<bool>(is_valid)) {
      continue;
    }

    common::PolygonObstacle poly;
    poly.id = obs["properties"]["id"].get<int>();
    poly.type = obs["properties"]["is_spec"].get<int>();
    Json coord = obs["geometry"]["coordinates"][0][0];
    int num_pts = static_cast<int>(coord.size());
    for (int k = 0; k < num_pts; ++k) {
      common::Point point(coord[k][0].get<double>(), coord[k][1].get<double>());
      poly.polygon.points.push_back(point);
    }
    p_obstacle_set->obs_polygon.insert(
        std::pair<int, common::PolygonObstacle>(poly.id, poly));
  }
  p_obstacle_set->print();

  fs.close();
  return kSuccess;
}

ErrorType ArenaLoader::ParseLaneNetInfo(common::LaneNet *p_lane_net) {
  printf("\n[ArenaLoader] Loading lane net info\n");

  std::fstream fs(lane_net_path_);
  Json root;
  fs >> root;

  Json lane_net_json = root["features"];

  for (int i = 0; i < static_cast<int>(lane_net_json.size()); ++i) {
    common::LaneRaw lane_raw;
    Json lane_json = lane_net_json[i];
    Json lane_meta = lane_json["properties"];

    lane_raw.id = lane_meta["id"].get<int>();
    lane_raw.length = lane_meta["length"].get<double>();
    lane_raw.dir = 1;

    printf("-Lane id %d.\n", lane_raw.id);
    std::string str_child_id = lane_meta["child_id"].get<std::string>();
    {
      std::vector<std::string> str_vec;
      common::SplitString(str_child_id, ",", &str_vec);
      for (const auto &str : str_vec) {
        auto lane_id = std::stoi(str);
        if (lane_id != 0) lane_raw.child_id.push_back(lane_id);
      }
    }

    std::string str_father_id = lane_meta["father_id"].get<std::string>();
    {
      std::vector<std::string> str_vec;
      common::SplitString(str_father_id, ",", &str_vec);
      for (const auto &str : str_vec) {
        auto lane_id = std::stoi(str);
        if (lane_id != 0) lane_raw.father_id.push_back(lane_id);
      }
    }

    lane_raw.l_lane_id = lane_meta["left_id"].get<int>();
    lane_raw.r_lane_id = lane_meta["right_id"].get<int>();

    int lchg_vld = lane_meta["lchg_vld"].get<int>();
    int rchg_vld = lane_meta["rchg_vld"].get<int>();
    lane_raw.l_change_avbl = static_cast<bool>(lchg_vld);
    lane_raw.r_change_avbl = static_cast<bool>(rchg_vld);
    lane_raw.behavior = lane_meta["behavior"].get<std::string>();

    Json lane_coordinates = lane_json["geometry"]["coordinates"][0];
    int num_pts = static_cast<int>(lane_coordinates.size());
    for (int k = 0; k < num_pts; ++k) {
      Vec2f pt(lane_coordinates[k][0].get<double>(),
               lane_coordinates[k][1].get<double>());
      lane_raw.lane_points.emplace_back(pt);
    }
    lane_raw.start_point = *(lane_raw.lane_points.begin());
    lane_raw.final_point = *(lane_raw.lane_points.rbegin());
    p_lane_net->lane_set.insert(
        std::pair<int, common::LaneRaw>(lane_raw.id, lane_raw));
  }
  p_lane_net->print();

  fs.close();
  return kSuccess;
}

}  // namespace phy_simulator
