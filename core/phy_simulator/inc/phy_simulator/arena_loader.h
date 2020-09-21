/**
 * @file arena_loader.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

#include "phy_simulator/basics.h"

namespace phy_simulator {

class ArenaLoader {
 public:
  /**
   * @brief Default constructor
   */
  ArenaLoader();

  /**
   * @brief Construct a new ArenaLoader object
   *
   * @param vehicle_set_path
   * @param map_path
   * @param lane_net_path
   */
  ArenaLoader(const std::string &vehicle_set_path, const std::string &map_path,
              const std::string &lane_net_path);

  inline std::string vehicle_set_path() const { return vehicle_set_path_; }
  inline std::string map_path() const { return map_path_; }
  inline std::string lane_net_path() const { return lane_net_path_; }

  inline void set_vehicle_set_path(const std::string &path) {
    vehicle_set_path_ = path;
  }
  inline void set_map_path(const std::string &path) { map_path_ = path; }
  inline void set_lane_net_path(const std::string &path) {
    lane_net_path_ = path;
  }

  /**
   * @brief Parse vehicles info from json
   *
   * @param p_vehicle_set
   * @return true Parsing success
   * @return false Parsing failed
   */
  bool ParseVehicleSet(common::VehicleSet *p_vehicle_set);

  /**
   * @brief Parse obstacle info from json
   *
   * @param p_obstacle_set
   * @return true Parsing success
   * @return false Parsing failed
   */
  ErrorType ParseMapInfo(common::ObstacleSet *p_obstacle_set);

  /**
   * @brief Parse map info from json
   *
   * @param p_lane_net
   * @return true Parsing success
   * @return false Parsing failed
   */
  ErrorType ParseLaneNetInfo(common::LaneNet *p_lane_net);

 private:
  std::string vehicle_set_path_;
  std::string map_path_;
  std::string lane_net_path_;
  std::string pedestrian_set_path_;
};

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_ARENA_LOADER_H_