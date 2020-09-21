/**
 * @file ssc_map.h
 * @author HKUST Aerial Robotics Group
 * @brief maintain the spatial temporal map for the planner
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */
#ifndef _UTIL_SSC_PLANNER_INC_SSC_MAP_H_
#define _UTIL_SSC_PLANNER_INC_SSC_MAP_H_

#include <assert.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "common/basics/semantics.h"
#include "common/state/state.h"

namespace planning {

using ObstacleMapType = uint8_t;
using SscMapDataType = uint8_t;

class SscMap {
 public:
  using GridMap3D = common::GridMapND<ObstacleMapType, 3>;

  struct Config {
    std::array<int, 3> map_size = {{1000, 100, 81}};               // s, d, t
    std::array<decimal_t, 3> map_resolution = {{0.25, 0.2, 0.1}};  // m, m, s
    std::array<std::string, 3> axis_name = {{"s", "d", "t"}};
    decimal_t s_back_len = 0.0;
    decimal_t kMaxLongitudinalVel = 50.0;
    decimal_t kMinLongitudinalVel = 0.0;
    decimal_t kMaxLongitudinalAcc = 3.0;
    decimal_t kMaxLongitudinalDecel = -8.0;  // Avg. driver max
    decimal_t kMaxLateralVel = 3.0;
    decimal_t kMaxLateralAcc = 2.5;

    int kMaxNumOfGridAlongTime = 2;

    std::array<int, 6> inflate_steps = {{20, 5, 10, 10, 1, 1}};

    void Print() {
      printf("\nSscMap Config:\n");
      printf(" -- map_size: [%d, %d, %d]\n", map_size[0], map_size[1],
             map_size[2]);
      printf(" -- map_resolution: [%lf, %lf, %lf]\n", map_resolution[0],
             map_resolution[1], map_resolution[2]);
      printf(" -- axis_name: [%s, %s, %s]\n", axis_name[0].c_str(),
             axis_name[1].c_str(), axis_name[2].c_str());
      printf(" -- s_back_len: %lf\n", s_back_len);
      printf(" -- kMaxLongitudinalVel: %lf\n", kMaxLongitudinalVel);
      printf(" -- kMinLongitudinalVel: %lf\n", kMinLongitudinalVel);
      printf(" -- kMaxLongitudinalAcc: %lf\n", kMaxLongitudinalAcc);
      printf(" -- kMaxLongitudinalDecel: %lf\n", kMaxLongitudinalDecel);
      printf(" -- kMaxLateralVel: %lf\n", kMaxLateralVel);
      printf(" -- kMaxLateralAcc: %lf\n", kMaxLateralAcc);
      printf(" -- kMaxNumOfGridAlongTime: %d\n", kMaxNumOfGridAlongTime);
      printf(" -- inflate_steps: [%d, %d, %d, %d, %d, %d]\n", inflate_steps[0],
             inflate_steps[1], inflate_steps[2], inflate_steps[3],
             inflate_steps[4], inflate_steps[5]);
    }
  };

  SscMap() {}
  SscMap(const Config &config);
  ~SscMap() {}

  GridMap3D *p_3d_grid() const { return p_3d_grid_; }
  GridMap3D *p_3d_inflated_grid() const { return p_3d_inflated_grid_; }

  Config config() const { return config_; }

  vec_E<common::DrivingCorridor> driving_corridor_vec() const {
    return driving_corridor_vec_;
  }
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec()
      const {
    return final_corridor_vec_;
  };

  std::vector<int> if_corridor_valid() const { return if_corridor_valid_; }

  void set_start_time(const decimal_t &t) { start_time_ = t; }
  void set_initial_fs(const common::FrenetState &fs) { initial_fs_ = fs; }

  void UpdateMapOrigin(const common::FrenetState &ori_fs);

  ErrorType ConstructSscMap(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs,
      const vec_E<Vec2f> &obstacle_grids);

  ErrorType InflateObstacleGrid(const common::VehicleParam &param);

  ErrorType ConstructCorridorUsingInitialTrajectory(
      GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs);

  ErrorType ClearGridMap();

  ErrorType ClearDrivingCorridor();

  ErrorType GetFinalGlobalMetricCubesList();

  ErrorType ResetSscMap(const common::FrenetState &ini_frenet_state);

 private:
  bool CheckIfCubeIsFree(GridMap3D *p_grid,
                         const common::AxisAlignedCubeNd<int, 3> &cube) const;

  bool CheckIfPlaneIsFreeOnXAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfPlaneIsFreeOnYAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfPlaneIsFreeOnZAxis(GridMap3D *p_grid,
                                 const common::AxisAlignedCubeNd<int, 3> &cube,
                                 const int &z) const;

  bool CheckIfCubeContainsSeed(const common::AxisAlignedCubeNd<int, 3> &cube_a,
                               const Vec3i &seed) const;

  ErrorType GetInitialCubeUsingSeed(
      const Vec3i &seed_0, const Vec3i &seed_1,
      common::AxisAlignedCubeNd<int, 3> *cube) const;

  ErrorType GetTimeCoveredCubeIndices(const common::DrivingCorridor *p_corridor,
                                      const int &start_id, const int &dir,
                                      const int &t_trans,
                                      std::vector<int> *idx_list) const;

  ErrorType CorridorRelaxation(GridMap3D *p_grid,
                               common::DrivingCorridor *p_corridor);

  ErrorType InflateCubeIn3dGrid(GridMap3D *p_grid,
                                const std::array<bool, 6> &dir_disabled,
                                const std::array<int, 6> &dir_step,
                                common::AxisAlignedCubeNd<int, 3> *cube);

  ErrorType GetInflationDirections(const bool &if_first_cube,
                                   std::array<bool, 6> *dirs_disabled);

  bool InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);
  bool InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                             common::AxisAlignedCubeNd<int, 3> *cube);

  ErrorType FillStaticPart(const vec_E<Vec2f> &obs_grid_fs);

  ErrorType FillDynamicPart(
      const std::unordered_map<int, vec_E<common::FsVehicle>>
          &sur_vehicle_trajs_fs);

  ErrorType FillMapWithFsVehicleTraj(const vec_E<common::FsVehicle> traj);

  common::GridMapND<SscMapDataType, 3> *p_3d_grid_;
  common::GridMapND<SscMapDataType, 3> *p_3d_inflated_grid_;

  std::unordered_map<int, std::array<bool, 6>> inters_for_cube_;

  Config config_;

  decimal_t start_time_;

  common::FrenetState initial_fs_;

  bool map_valid_ = false;

  vec_E<common::DrivingCorridor> driving_corridor_vec_;

  std::vector<int> if_corridor_valid_;
  vec_E<vec_E<common::SpatioTemporalSemanticCubeNd<2>>> final_corridor_vec_;
};

}  // namespace planning
#endif  // _UTIL_SSC_PLANNER_INC_SSC_MAP_H_