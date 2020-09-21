#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_

#include <random>
#include <assert.h>
#include <iostream>
#include <set>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "nanoflann/include/nanoflann.hpp"
#include "semantic_map_manager/semantic_map_manager.h"

namespace semantic_map_manager {
class DataRenderer {
 public:
  using ObstacleMapType = uint8_t;
  using GridMap2D = common::GridMapND<ObstacleMapType, 2>;

  DataRenderer(SemanticMapManager *smm_ptr);
  ~DataRenderer() {}

  inline void set_ego_id(const int id) { ego_id_ = id; }
  inline void set_obstacle_map_info(const common::GridMapMetaInfo &info) {
    obstacle_map_info_ = info;
  }

  ErrorType Render(const double &time_stamp, const common::LaneNet &lane_net,
                   const common::VehicleSet &vehicle_set,
                   const common::ObstacleSet &obstacle_set);

 private:
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<decimal_t, common::PointVecForKdTree>,
      common::PointVecForKdTree, 2>
      KdTreeFor2dPointVec;

  ErrorType GetEgoVehicle(const common::VehicleSet &vehicle_set);
  ErrorType GetObstacleMap(const common::ObstacleSet &obstacle_set);
  ErrorType GetWholeLaneNet(const common::LaneNet &lane_net);
  ErrorType GetSurroundingLaneNet(const common::LaneNet &lane_net);
  ErrorType GetSurroundingVehicles(const common::VehicleSet &vehicle_set);
  ErrorType GetSurroundingObjects();

  ErrorType RayCastingOnObstacleMap();
  ErrorType FakeMapper();
  ErrorType InjectObservationNoise();

  bool if_kdtree_lane_net_updated_ = false;
  common::PointVecForKdTree lane_net_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_lane_net_;

  bool if_kdtree_obstacle_set_updated_ = false;
  common::PointVecForKdTree obstacle_set_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_obstacle_set_;

  common::PointVecForKdTree vehicle_set_pts_;
  std::shared_ptr<KdTreeFor2dPointVec> kdtree_vehicle_;

  int ego_id_ = 0;

  int ray_casting_num_ = 1440;

  std::set<std::array<decimal_t, 2>> obs_grids_;
  std::set<std::array<decimal_t, 2>> free_grids_;

  double time_stamp_;

  common::Vehicle ego_vehicle_;
  common::VehicleParam ego_param_;
  common::State ego_state_;

  common::VehicleSet surrounding_vehicles_;

  common::GridMapMetaInfo obstacle_map_info_;
  GridMap2D *p_obstacle_grid_;

  decimal_t surrounding_search_radius_;

  common::LaneNet surrounding_lane_net_;
  common::LaneNet whole_lane_net_;

  SemanticMapManager *p_semantic_map_manager_;

  std::vector<int> uncertain_vehicle_ids_;
  std::mt19937 random_engine_;
  int cnt_random_ = 0;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_DATA_RENDERER_H_
