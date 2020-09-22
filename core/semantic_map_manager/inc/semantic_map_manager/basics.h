/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_

#include <assert.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>
#include <vector>
#include <memory>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"

namespace semantic_map_manager {

struct AgentConfigInfo {
  common::GridMapMetaInfo obstacle_map_meta_info;
  decimal_t surrounding_search_radius;
  bool enable_openloop_prediction{false};
  bool enable_tracking_noise{false};
  bool enable_log{false};
  bool enable_fast_lane_lut{true};
  std::string log_file;

  void PrintInfo() {
    obstacle_map_meta_info.print();
    printf("surrounding_search_radius: %f\n", surrounding_search_radius);
    printf("enable_openloop_prediction: %d\n", enable_openloop_prediction);
    printf("enable_tracking_noise: %d\n", enable_tracking_noise);
    printf("enable_log: %d\n", enable_log);
    printf("enable_fast_lane_lut: %d\n", enable_fast_lane_lut);
    printf("log_file: %s\n", log_file.c_str());
  }
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_BASICS_H_