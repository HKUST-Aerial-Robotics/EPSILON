/**
 * @file visualizer.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-20
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_
#define _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/state.h"
#include "common/visualization/common_visualization_util.h"

#include "phy_simulator/phy_simulator.h"

namespace phy_simulator {

class Visualizer {
 public:
  Visualizer() {}
  Visualizer(ros::NodeHandle nh);
  ~Visualizer() {}

  void set_phy_sim(PhySimulation *p_phy_sim) { p_phy_sim_ = p_phy_sim; }

  void VisualizeData();
  void VisualizeDataWithStamp(const ros::Time &stamp);
  void SendTfWithStamp(const ros::Time &stamp);

 private:
  void VisualizeVehicleSet(const ros::Time &stamp,
                           const common::VehicleSet &vehicle_set);
  void VisualizeLaneNet(const ros::Time &stamp,
                        const common::LaneNet &lane_net);
  void VisualizeObstacleSet(const ros::Time &stamp,
                            const common::ObstacleSet &Obstacle_set);


  ros::NodeHandle nh_;

  ros::Publisher vehicle_set_pub_;
  ros::Publisher lane_net_pub_;
  ros::Publisher obstacle_set_pub_;

  PhySimulation *p_phy_sim_;
};  // Visualizer

}  // namespace phy_simulator

#endif  // _CORE_SEMANTIC_MAP_INC_PHY_SIMULATOR_VISUALIZER_H_