/**
 * @file ssc_map.cc
 * @author HKUST Aerial Robotics Group
 * @brief implementation for ssc map
 * @version 0.1
 * @date 2019-02
 * @copyright Copyright (c) 2019
 */

#include "ssc_planner/ssc_map.h"

#include <glog/logging.h>

namespace planning {

SscMap::SscMap(const SscMap::Config &config) : config_(config) {
  config_.Print();

  p_3d_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
  p_3d_inflated_grid_ = new common::GridMapND<SscMapDataType, 3>(
      config_.map_size, config_.map_resolution, config_.axis_name);
}

ErrorType SscMap::ResetSscMap(const common::FrenetState &ini_fs) {
  ClearDrivingCorridor();
  ClearGridMap();

  start_time_ = ini_fs.time_stamp;
  UpdateMapOrigin(ini_fs);

  return kSuccess;
}

void SscMap::UpdateMapOrigin(const common::FrenetState &ori_fs) {
  initial_fs_ = ori_fs;

  std::array<decimal_t, 3> map_origin;
  map_origin[0] = ori_fs.vec_s[0] - config_.s_back_len;
  map_origin[1] =
      -1 * (config_.map_size[1] - 1) * config_.map_resolution[1] / 2.0;  // d
  map_origin[2] = ori_fs.time_stamp;                                     // t

  p_3d_grid_->set_origin(map_origin);
  p_3d_inflated_grid_->set_origin(map_origin);
}

ErrorType SscMap::GetInitialCubeUsingSeed(
    const Vec3i &seed_0, const Vec3i &seed_1,
    common::AxisAlignedCubeNd<int, 3> *cube) const {
  std::array<int, 3> lb;
  std::array<int, 3> ub;
  lb[0] = std::min(seed_0(0), seed_1(0));
  lb[1] = std::min(seed_0(1), seed_1(1));
  lb[2] = std::min(seed_0(2), seed_1(2));
  ub[0] = std::max(seed_0(0), seed_1(0));
  ub[1] = std::max(seed_0(1), seed_1(1));
  ub[2] = std::max(seed_0(2), seed_1(2));

  *cube = common::AxisAlignedCubeNd<int, 3>(ub, lb);
  return kSuccess;
}

ErrorType SscMap::ConstructSscMap(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs,
    const vec_E<Vec2f> &obstacle_grids) {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  FillStaticPart(obstacle_grids);
  FillDynamicPart(sur_vehicle_trajs_fs);
  return kSuccess;
}

ErrorType SscMap::GetInflationDirections(const bool &if_first_cube,
                                         std::array<bool, 6> *dirs_disabled) {
  (*dirs_disabled)[0] = false;
  (*dirs_disabled)[1] = false;
  (*dirs_disabled)[2] = false;
  (*dirs_disabled)[3] = false;
  (*dirs_disabled)[4] = false;
  (*dirs_disabled)[5] = !if_first_cube;

  return kSuccess;
}

ErrorType SscMap::ClearGridMap() {
  p_3d_grid_->clear_data();
  p_3d_inflated_grid_->clear_data();
  return kSuccess;
}

ErrorType SscMap::ClearDrivingCorridor() {
  driving_corridor_vec_.clear();
  return kSuccess;
}

ErrorType SscMap::ConstructCorridorUsingInitialTrajectory(
    GridMap3D *p_grid, const vec_E<common::FsVehicle> &trajs) {
  // ~ Stage I: Get seeds
  vec_E<Vec3i> traj_seeds;
  int num_states = static_cast<int>(trajs.size());
  if (num_states > 1) {
    bool first_seed_determined = false;
    for (int k = 0; k < num_states; ++k) {
      std::array<decimal_t, 3> p_w = {};
      if (!first_seed_determined) {
        decimal_t s_0 = initial_fs_.vec_s[0];
        decimal_t d_0 = initial_fs_.vec_dt[0];
        decimal_t t_0 = initial_fs_.time_stamp;
        std::array<decimal_t, 3> p_w_0 = {s_0, d_0, t_0};
        auto coord_0 = p_grid->GetCoordUsingGlobalPosition(p_w_0);

        decimal_t s_1 = trajs[k].frenet_state.vec_s[0];
        decimal_t d_1 = trajs[k].frenet_state.vec_dt[0];
        decimal_t t_1 = trajs[k].frenet_state.time_stamp;
        std::array<decimal_t, 3> p_w_1 = {s_1, d_1, t_1};
        auto coord_1 = p_grid->GetCoordUsingGlobalPosition(p_w_1);
        // * remove the states out of range
        if (!p_grid->CheckCoordInRange(coord_1)) {
          continue;
        }
        // earlier than start time
        if (coord_1[2] <= 0) {
          continue;
        }

        first_seed_determined = true;
        traj_seeds.push_back(Vec3i(coord_0[0], coord_0[1], coord_0[2]));
        traj_seeds.push_back(Vec3i(coord_1[0], coord_1[1], coord_1[2]));
      } else {
        decimal_t s = trajs[k].frenet_state.vec_s[0];
        decimal_t d = trajs[k].frenet_state.vec_dt[0];
        decimal_t t = trajs[k].frenet_state.time_stamp;
        p_w = {s, d, t};
        auto coord = p_grid->GetCoordUsingGlobalPosition(p_w);
        // * remove the states out of range
        if (!p_grid->CheckCoordInRange(coord)) {
          continue;
        }
        traj_seeds.push_back(Vec3i(coord[0], coord[1], coord[2]));
      }
    }
  }

  // ~ Stage II: Inflate cubes
  common::DrivingCorridor driving_corridor;
  bool is_valid = true;
  auto seed_num = static_cast<int>(traj_seeds.size());
  if (seed_num < 2) {
    driving_corridor.is_valid = false;
    driving_corridor_vec_.push_back(driving_corridor);
    is_valid = false;
    return kWrongStatus;
  }
  for (int i = 0; i < seed_num; ++i) {
    if (i == 0) {
      common::AxisAlignedCubeNd<int, 3> cube;
      GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);
      if (!CheckIfCubeIsFree(p_grid, cube)) {
        LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: " << i;

        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_cube.seeds.push_back(traj_seeds[i + 1]);
        driving_corridor.cubes.push_back(driving_cube);

        driving_corridor.is_valid = false;
        driving_corridor_vec_.push_back(driving_corridor);
        is_valid = false;
        break;
      }

      std::array<bool, 6> dirs_disabled = {false, false, false,
                                           false, false, false};
      InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps, &cube);

      common::DrivingCube driving_cube;
      driving_cube.cube = cube;
      driving_cube.seeds.push_back(traj_seeds[i]);
      driving_corridor.cubes.push_back(driving_cube);
    } else {
      if (CheckIfCubeContainsSeed(driving_corridor.cubes.back().cube,
                                  traj_seeds[i])) {
        driving_corridor.cubes.back().seeds.push_back(traj_seeds[i]);
        continue;
      } else {
        // ~ Get the last seed in cube
        Vec3i seed_r = driving_corridor.cubes.back().seeds.back();
        driving_corridor.cubes.back().seeds.pop_back();
        // ~ Cut cube on time axis
        driving_corridor.cubes.back().cube.upper_bound[2] = seed_r(2);
        i = i - 1;

        common::AxisAlignedCubeNd<int, 3> cube;
        GetInitialCubeUsingSeed(traj_seeds[i], traj_seeds[i + 1], &cube);

        if (!CheckIfCubeIsFree(p_grid, cube)) {
          LOG(ERROR) << "[Ssc] SccMap - Initial cube is not free, seed id: "
                     << i;
          common::DrivingCube driving_cube;
          driving_cube.cube = cube;
          driving_cube.seeds.push_back(traj_seeds[i]);
          driving_cube.seeds.push_back(traj_seeds[i + 1]);
          driving_corridor.cubes.push_back(driving_cube);

          driving_corridor.is_valid = false;
          driving_corridor_vec_.push_back(driving_corridor);
          is_valid = false;
          break;
        }

        std::array<bool, 6> dirs_disabled = {false, false, false,
                                             false, false, false};
        InflateCubeIn3dGrid(p_grid, dirs_disabled, config_.inflate_steps,
                            &cube);
        common::DrivingCube driving_cube;
        driving_cube.cube = cube;
        driving_cube.seeds.push_back(traj_seeds[i]);
        driving_corridor.cubes.push_back(driving_cube);
      }
    }
  }
  if (is_valid) {
    // CorridorRelaxation(p_grid, &driving_corridor);
    // ~ Cut cube on time axis
    driving_corridor.cubes.back().cube.upper_bound[2] = traj_seeds.back()(2);
    driving_corridor.is_valid = true;
    driving_corridor_vec_.push_back(driving_corridor);
  }

  return kSuccess;
}

ErrorType SscMap::GetTimeCoveredCubeIndices(
    const common::DrivingCorridor *p_corridor, const int &start_idx,
    const int &dir, const int &t_trans, std::vector<int> *idx_list) const {
  int dt = 0;
  int num_cube = p_corridor->cubes.size();
  int idx = start_idx;
  while (idx < num_cube && idx >= 0) {
    dt += p_corridor->cubes[idx].cube.upper_bound[2] -
          p_corridor->cubes[idx].cube.lower_bound[2];
    idx_list->push_back(idx);
    if (dir == 1) {
      ++idx;
    } else {
      --idx;
    }
    if (dt >= t_trans) {
      break;
    }
  }
  return kSuccess;
}

ErrorType SscMap::CorridorRelaxation(GridMap3D *p_grid,
                                     common::DrivingCorridor *p_corridor) {
  std::array<int, 2> margin = {{50, 10}};
  int t_trans = 7;
  int num_cube = p_corridor->cubes.size();
  for (int i = 0; i < num_cube - 1; ++i) {
    if (1)  // ~ Enable s direction
    {
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[0];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[0];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[0];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[0];

      if (abs(cube_0_ub - cube_1_lb) < margin[0]) {
        int room = margin[0] - abs(cube_0_ub - cube_1_lb);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[0]) {
        int room = margin[0] - abs(cube_0_lb - cube_1_ub);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnXPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnXNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }

    if (1)  // ~ Enable d direction
    {
      int cube_0_lb = p_corridor->cubes[i].cube.lower_bound[1];
      int cube_0_ub = p_corridor->cubes[i].cube.upper_bound[1];

      int cube_1_lb = p_corridor->cubes[i + 1].cube.lower_bound[1];
      int cube_1_ub = p_corridor->cubes[i + 1].cube.upper_bound[1];

      if (abs(cube_0_ub - cube_1_lb) < margin[1]) {
        int room = margin[1] - abs(cube_0_ub - cube_1_lb);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto &idx : up_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto &idx : down_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
      if (abs(cube_0_lb - cube_1_ub) < margin[1]) {
        int room = margin[1] - abs(cube_0_lb - cube_1_ub);
        // ~ Upward
        std::vector<int> up_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i + 1, 1, t_trans, &up_idx_list);
        // ~ Downward
        std::vector<int> down_idx_list;
        GetTimeCoveredCubeIndices(p_corridor, i, 0, t_trans, &down_idx_list);
        for (const auto idx : up_idx_list) {
          InflateCubeOnYPosAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
        for (const auto idx : down_idx_list) {
          InflateCubeOnYNegAxis(p_grid, room, &(p_corridor->cubes[idx].cube));
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::InflateObstacleGrid(const common::VehicleParam &param) {
  decimal_t s_p_inflate_len = param.length() / 2.0 - param.d_cr();
  decimal_t s_n_inflate_len = param.length() - s_p_inflate_len;
  int num_s_p_inflate_grids =
      std::floor(s_p_inflate_len / config_.map_resolution[0]);
  int num_s_n_inflate_grids =
      std::floor(s_n_inflate_len / config_.map_resolution[0]);
  int num_d_inflate_grids =
      std::floor((param.width() - 0.5) / 2.0 / config_.map_resolution[1]);
  bool is_free = false;

  for (int i = 0; i < config_.map_size[0]; ++i) {
    for (int j = 0; j < config_.map_size[1]; ++j) {
      for (int k = 0; k < config_.map_size[2]; ++k) {
        std::array<int, 3> coord = {i, j, k};
        p_3d_grid_->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          for (int s = -num_s_n_inflate_grids; s < num_s_p_inflate_grids; s++) {
            for (int d = -num_d_inflate_grids; d < num_d_inflate_grids; d++) {
              coord = {i + s, j + d, k};
              p_3d_inflated_grid_->SetValueUsingCoordinate(coord, 100);
            }
          }
        }
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::InflateCubeIn3dGrid(GridMap3D *p_grid,
                                      const std::array<bool, 6> &dir_disabled,
                                      const std::array<int, 6> &dir_step,
                                      common::AxisAlignedCubeNd<int, 3> *cube) {
  bool x_p_finish = dir_disabled[0];
  bool x_n_finish = dir_disabled[1];
  bool y_p_finish = dir_disabled[2];
  bool y_n_finish = dir_disabled[3];
  bool z_p_finish = dir_disabled[4];

  int x_p_step = dir_step[0];
  int x_n_step = dir_step[1];
  int y_p_step = dir_step[2];
  int y_n_step = dir_step[3];
  int z_p_step = dir_step[4];

  int t_max_grids = cube->lower_bound[2] + config_.kMaxNumOfGridAlongTime;

  decimal_t t = t_max_grids * p_grid->dims_resolution(2);
  decimal_t a_max = config_.kMaxLongitudinalAcc;
  decimal_t a_min = config_.kMaxLongitudinalDecel;
  decimal_t d_comp = initial_fs_.vec_s[1] * 1;

  decimal_t s_u = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_max * t * t + d_comp;
  decimal_t s_l = initial_fs_.vec_s[0] + initial_fs_.vec_s[1] * t +
                  0.5 * a_min * t * t - d_comp;

  int s_idx_u, s_idx_l;
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_u, 0, &s_idx_u);
  p_grid->GetCoordUsingGlobalMetricOnSingleDim(s_l, 0, &s_idx_l);
  s_idx_l = std::max(s_idx_l, static_cast<int>((config_.s_back_len / 2.0) /
                                               config_.map_resolution[0]));

  while (!(x_p_finish && x_n_finish && y_p_finish && y_n_finish)) {
    if (!x_p_finish) x_p_finish = InflateCubeOnXPosAxis(p_grid, x_p_step, cube);
    if (!x_n_finish) x_n_finish = InflateCubeOnXNegAxis(p_grid, x_n_step, cube);

    if (!y_p_finish) y_p_finish = InflateCubeOnYPosAxis(p_grid, y_p_step, cube);
    if (!y_n_finish) y_n_finish = InflateCubeOnYNegAxis(p_grid, y_n_step, cube);

    if (cube->upper_bound[0] >= s_idx_u) x_p_finish = true;
    if (cube->lower_bound[0] <= s_idx_l) x_n_finish = true;
  }

  // ~ No need to inflate along z-neg
  while (!z_p_finish) {
    if (!z_p_finish) z_p_finish = InflateCubeOnZPosAxis(p_grid, z_p_step, cube);

    if (cube->upper_bound[2] - cube->lower_bound[2] >=
        config_.kMaxNumOfGridAlongTime) {
      z_p_finish = true;
    }
  }

  return kSuccess;
}

bool SscMap::InflateCubeOnXPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->upper_bound[0] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        // The plane in 3D obstacle grid is free
        cube->upper_bound[0] = x;
      } else {
        // The plane in 3D obstacle grid is not free, finish
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnXNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int x = cube->lower_bound[0] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(x, 0)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnXAxis(p_grid, *cube, x)) {
        // The plane in 3D obstacle grid is free
        cube->lower_bound[0] = x;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->upper_bound[1] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        // The plane in 3D obstacle grid is free
        cube->upper_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnYNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int y = cube->lower_bound[1] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(y, 1)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnYAxis(p_grid, *cube, y)) {
        // The plane in 3D obstacle grid is free
        cube->lower_bound[1] = y;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZPosAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->upper_bound[2] + 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        // The plane in 3D obstacle grid is free
        cube->upper_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::InflateCubeOnZNegAxis(GridMap3D *p_grid, const int &n_step,
                                   common::AxisAlignedCubeNd<int, 3> *cube) {
  for (int i = 0; i < n_step; ++i) {
    int z = cube->lower_bound[2] - 1;
    if (!p_grid->CheckCoordInRangeOnSingleDim(z, 2)) {
      return true;
    } else {
      if (CheckIfPlaneIsFreeOnZAxis(p_grid, *cube, z)) {
        // The plane in 3D obstacle grid is free
        cube->lower_bound[2] = z;
      } else {
        return true;
      }
    }
  }
  return false;
}

bool SscMap::CheckIfCubeIsFree(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  int f2_min = cube.lower_bound[2];
  int f2_max = cube.upper_bound[2];

  int i, j, k;
  std::array<int, 3> coord;
  bool is_free;
  for (i = f0_min; i <= f0_max; ++i) {
    for (j = f1_min; j <= f1_max; ++j) {
      for (k = f2_min; k <= f2_max; ++k) {
        coord = {i, j, k};
        p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
        if (!is_free) {
          return false;
        }
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnXAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &x) const {
  int f0_min = cube.lower_bound[1];
  int f0_max = cube.upper_bound[1];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {x, i, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnYAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &y) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[2];
  int f1_max = cube.upper_bound[2];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, y, j};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfPlaneIsFreeOnZAxis(
    GridMap3D *p_grid, const common::AxisAlignedCubeNd<int, 3> &cube,
    const int &z) const {
  int f0_min = cube.lower_bound[0];
  int f0_max = cube.upper_bound[0];
  int f1_min = cube.lower_bound[1];
  int f1_max = cube.upper_bound[1];
  std::array<int, 3> coord;
  bool is_free;
  for (int i = f0_min; i <= f0_max; ++i) {
    for (int j = f1_min; j <= f1_max; ++j) {
      coord = {i, j, z};
      p_grid->CheckIfEqualUsingCoordinate(coord, 0, &is_free);
      if (!is_free) {
        return false;
      }
    }
  }
  return true;
}

bool SscMap::CheckIfCubeContainsSeed(
    const common::AxisAlignedCubeNd<int, 3> &cube_a, const Vec3i &seed) const {
  for (int i = 0; i < 3; ++i) {
    if (cube_a.lower_bound[i] > seed(i) || cube_a.upper_bound[i] < seed(i)) {
      return false;
    }
  }
  return true;
}

ErrorType SscMap::GetFinalGlobalMetricCubesList() {
  final_corridor_vec_.clear();
  if_corridor_valid_.clear();
  for (const auto corridor : driving_corridor_vec_) {
    vec_E<common::SpatioTemporalSemanticCubeNd<2>> cubes;
    if (!corridor.is_valid) {
      if_corridor_valid_.push_back(0);
    } else {
      if_corridor_valid_.push_back(1);
      for (int k = 0; k < static_cast<int>(corridor.cubes.size()); ++k) {
        common::SpatioTemporalSemanticCubeNd<2> cube;
        decimal_t x_lb, x_ub;
        decimal_t y_lb, y_ub;
        decimal_t z_lb, z_ub;

        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[0], 0, &x_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[0], 0, &x_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[1], 1, &y_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[1], 1, &y_ub);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.lower_bound[2], 2, &z_lb);
        p_3d_grid_->GetGlobalMetricUsingCoordOnSingleDim(
            corridor.cubes[k].cube.upper_bound[2], 2, &z_ub);

        cube.t_lb = z_lb;
        cube.t_ub = z_ub;

        cube.p_lb[0] = x_lb;
        cube.p_ub[0] = x_ub;
        cube.v_lb[0] = config_.kMinLongitudinalVel;
        cube.v_ub[0] = config_.kMaxLongitudinalVel;
        cube.a_lb[0] = config_.kMaxLongitudinalDecel;
        cube.a_ub[0] = config_.kMaxLongitudinalAcc;

        cube.p_lb[1] = y_lb;
        cube.p_ub[1] = y_ub;
        cube.v_lb[1] = -config_.kMaxLateralVel;
        cube.v_ub[1] = config_.kMaxLateralVel;
        cube.a_lb[1] = -config_.kMaxLateralAcc;
        cube.a_ub[1] = config_.kMaxLateralAcc;

        if (k == 0) {
          if (y_lb > initial_fs_.vec_dt[0] || y_ub < initial_fs_.vec_dt[0]) {
            LOG(ERROR) << "[Ssc] SscMap - Initial state out of bound d: "
                       << initial_fs_.vec_dt[0] << ", lb: " << y_lb
                       << ", ub: " << y_ub;
            // assert(false);
            return kWrongStatus;
          }
        }

        cubes.push_back(cube);
      }
    }
    final_corridor_vec_.push_back(cubes);
  }

  return kSuccess;
}

ErrorType SscMap::FillStaticPart(const vec_E<Vec2f> &obs_grid_fs) {
  for (int i = 0; i < static_cast<int>(obs_grid_fs.size()); ++i) {
    if (obs_grid_fs[i](0) <= 0) {
      continue;
    }
    for (int k = 0; k < config_.map_size[2]; ++k) {
      std::array<decimal_t, 3> pt = {{obs_grid_fs[i](0), obs_grid_fs[i](1),
                                      (double)k * config_.map_resolution[2]}};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(pt);
      if (p_3d_grid_->CheckCoordInRange(coord)) {
        p_3d_grid_->SetValueUsingCoordinate(coord, 100);
      }
    }
  }
  return kSuccess;
}

ErrorType SscMap::FillDynamicPart(
    const std::unordered_map<int, vec_E<common::FsVehicle>>
        &sur_vehicle_trajs_fs) {
  for (auto it = sur_vehicle_trajs_fs.begin(); it != sur_vehicle_trajs_fs.end();
       ++it) {
    FillMapWithFsVehicleTraj(it->second);
  }
  return kSuccess;
}

ErrorType SscMap::FillMapWithFsVehicleTraj(
    const vec_E<common::FsVehicle> traj) {
  if (traj.size() == 0) {
    LOG(ERROR) << "[Ssc] SscMap - Trajectory is empty.";
    return kWrongStatus;
  }
  for (int i = 0; i < static_cast<int>(traj.size()); ++i) {
    bool is_valid = true;
    for (const auto v : traj[i].vertices) {
      if (v(0) <= 0) {
        is_valid = false;
        break;
      }
    }
    if (!is_valid) {
      continue;
    }
    decimal_t z = traj[i].frenet_state.time_stamp;
    int t_idx = 0;
    std::vector<common::Point2i> v_coord;
    std::array<decimal_t, 3> p_w;
    for (const auto v : traj[i].vertices) {
      p_w = {v(0), v(1), z};
      auto coord = p_3d_grid_->GetCoordUsingGlobalPosition(p_w);
      t_idx = coord[2];
      if (!p_3d_grid_->CheckCoordInRange(coord)) {
        is_valid = false;
        break;
      }
      v_coord.push_back(common::Point2i(coord[0], coord[1]));
    }
    if (!is_valid) {
      continue;
    }
    std::vector<std::vector<cv::Point2i>> vv_coord_cv;
    std::vector<cv::Point2i> v_coord_cv;
    common::ShapeUtils::GetCvPoint2iVecUsingCommonPoint2iVec(v_coord,
                                                             &v_coord_cv);
    vv_coord_cv.push_back(v_coord_cv);
    int w = p_3d_grid_->dims_size()[0];
    int h = p_3d_grid_->dims_size()[1];
    int layer_offset = t_idx * w * h;
    cv::Mat layer_mat =
        cv::Mat(h, w, CV_MAKETYPE(cv::DataType<SscMapDataType>::type, 1),
                p_3d_grid_->get_data_ptr() + layer_offset);
    cv::fillPoly(layer_mat, vv_coord_cv, 100);
  }
  return kSuccess;
}

}  // namespace planning