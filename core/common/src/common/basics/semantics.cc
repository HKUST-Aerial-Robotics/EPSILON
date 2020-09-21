/**
 * @file semantics.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/semantics.h"

namespace common {

void VehicleParam::print() const {
  printf("VehicleParam:\n");
  printf(" -- width:\t %lf.\n", width_);
  printf(" -- length:\t %lf.\n", length_);
  printf(" -- wheel_base:\t %lf.\n", wheel_base_);
  printf(" -- front_suspension:\t %lf.\n", front_suspension_);
  printf(" -- rear_suspension:\t %lf.\n", rear_suspension_);
  printf(" -- d_cr:\t %lf.\n", d_cr_);
  printf(" -- max_steering_angle:\t %lf.\n", max_steering_angle_);
  printf(" -- max_longitudinal_acc:\t %lf.\n", max_longitudinal_acc_);
  printf(" -- max_lateral_acc:\t %lf.\n", max_lateral_acc_);
}

Vehicle::Vehicle() {}

Vehicle::Vehicle(const VehicleParam &param, const State &state)
    : param_(param), state_(state) {}

Vehicle::Vehicle(const int &id, const VehicleParam &param, const State &state)
    : id_(id), param_(param), state_(state) {}

Vehicle::Vehicle(const int &id, const std::string &subclass,
                 const VehicleParam &param, const State &state)
    : id_(id), subclass_(subclass), param_(param), state_(state) {}

Vec3f Vehicle::Ret3DofState() const {
  return Vec3f(state_.vec_position(0), state_.vec_position(1), state_.angle);
}

ErrorType Vehicle::Ret3DofStateAtGeometryCenter(Vec3f *state) const {
  decimal_t cos_theta = cos(state_.angle);
  decimal_t sin_theta = sin(state_.angle);
  decimal_t x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  decimal_t y = state_.vec_position(1) + param_.d_cr() * sin_theta;
  (*state)(0) = x;
  (*state)(1) = y;
  (*state)(2) = state_.angle;
  return kSuccess;
}

void Vehicle::print() const {
  printf("\nVehicle:\n");
  printf(" -- ID:\t%d\n", id_);
  printf(" -- Subclass:\t%s\n", subclass_.c_str());
  param_.print();
  state_.print();
}

OrientedBoundingBox2D Vehicle::RetOrientedBoundingBox() const {
  OrientedBoundingBox2D obb;
  double cos_theta = cos(state_.angle);
  double sin_theta = sin(state_.angle);
  obb.x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  obb.y = state_.vec_position(1) + param_.d_cr() * sin_theta;
  obb.angle = state_.angle;
  obb.width = param_.width();
  obb.length = param_.length();
  return obb;
}

ErrorType Vehicle::RetVehicleVertices(vec_E<Vec2f> *vertices) const {
  SemanticsUtils::GetVehicleVertices(param_, state_, vertices);
  return kSuccess;
}

ErrorType Vehicle::RetBumperVertices(std::array<Vec2f, 2> *vertices) const {
  decimal_t cos_theta = cos(state_.angle);
  decimal_t sin_theta = sin(state_.angle);

  decimal_t c_x = state_.vec_position(0) + param_.d_cr() * cos_theta;
  decimal_t c_y = state_.vec_position(1) + param_.d_cr() * sin_theta;

  decimal_t d_lx = param_.length() / 2.0 * cos_theta;
  decimal_t d_ly = param_.length() / 2.0 * sin_theta;

  (*vertices)[0] = Vec2f(c_x - d_lx, c_y - d_ly);
  (*vertices)[1] = Vec2f(c_x + d_lx, c_y + d_ly);

  return kSuccess;
}

void VehicleSet::print() const {
  printf("Vehicle Set Info:\n");
  for (auto iter = vehicles.begin(); iter != vehicles.end(); ++iter) {
    printf("\n -- ID. %d:\n", iter->first);
    iter->second.print();
  }
  printf("\n");
}

VehicleControlSignal::VehicleControlSignal() {}

VehicleControlSignal::VehicleControlSignal(double acc, double steer_rate)
    : acc(acc), steer_rate(steer_rate), is_openloop(false) {}

VehicleControlSignal::VehicleControlSignal(common::State state)
    : acc(0.0), steer_rate(0.0), is_openloop(true), state(state) {}

GridMapMetaInfo::GridMapMetaInfo() {}

GridMapMetaInfo::GridMapMetaInfo(const int w, const int h, const double res)
    : width(w), height(h), resolution(res) {
  w_metric = w * resolution;
  h_metric = h * resolution;
}

void GridMapMetaInfo::print() const {
  printf("GridMapMetaInfo:\n");
  printf(" -- width:%d\n", width);
  printf(" -- height:%d\n", height);
  printf(" -- resolution:%lf\n", resolution);
  printf(" -- w_metric:%lf\n", w_metric);
  printf(" -- h_metric:%lf\n", h_metric);
  printf("\n");
}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() {}

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(
    const std::array<int, N_DIM> &dims_size,
    const std::array<decimal_t, N_DIM> &dims_resolution,
    const std::array<std::string, N_DIM> &dims_name) {
  dims_size_ = dims_size;
  dims_resolution_ = dims_resolution;
  dims_name_ = dims_name;

  SetNDimSteps(dims_size_);
  SetDataSize(dims_size_);
  data_ = std::vector<T>(data_size_, 0);
  origin_.fill(0);
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, T *val) const {
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  *val = data_[idx];
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, T *val) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  GetValueUsingCoordinate(coord, val);
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val_in, bool *res) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val_in, bool *res) const {
  T val;
  if (GetValueUsingCoordinate(coord, &val) != kSuccess) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM> &coord, const T &val) {
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return kWrongStatus;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  data_[idx] = val;
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w, const T &val) {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  SetValueUsingCoordinate(coord, val);
  return kSuccess;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  return coord;
}

template <typename T, int N_DIM>
std::array<decimal_t, N_DIM>
GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<decimal_t, N_DIM> &p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord[i] = std::round((p_w[i] - origin_[i]) / dims_resolution_[i]);
  }
  std::array<decimal_t, N_DIM> round_pos = {};
  for (int i = 0; i < N_DIM; ++i) {
    round_pos[i] = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return round_pos;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM> &coord,
    std::array<decimal_t, N_DIM> *p_w) const {
  auto ptr = p_w->data();
  for (int i = 0; i < N_DIM; ++i) {
    *(ptr + i) = coord[i] * dims_resolution_[i] + origin_[i];
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const decimal_t &metric, const int &i, int *idx) const {
  *idx = std::round((metric - origin_[i]) / dims_resolution_[i]);
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int &idx, const int &i, decimal_t *metric) const {
  *metric = idx * dims_resolution_[i] + origin_[i];
  return kSuccess;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(
    const std::array<int, N_DIM> &coord) const {
  for (int i = 0; i < N_DIM; ++i) {
    if (coord[i] < 0 || coord[i] >= dims_size_[i]) {
      return false;
    }
  }
  return true;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int &idx,
                                                       const int &i) const {
  return (idx >= 0) && (idx < dims_size_[i]);
}

template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(
    const std::array<int, N_DIM> &idx) const {
  int mono_idx = 0;
  for (int i = 0; i < N_DIM; ++i) {
    mono_idx += dims_step_[i] * idx[i];
  }
  return mono_idx;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(
    const int &idx) const {
  std::array<int, N_DIM> idx_nd = {};
  int tmp = idx;
  for (int i = N_DIM - 1; i >= 0; --i) {
    idx_nd[i] = tmp / dims_step_[i];
    tmp = tmp % dims_step_[i];
  }
  return idx_nd;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM> &dims_size) {
  int step = 1;
  for (int i = 0; i < N_DIM; ++i) {
    dims_step_[i] = step;
    step = step * dims_size[i];
  }
  return kSuccess;
}

template <typename T, int N_DIM>
ErrorType GridMapND<T, N_DIM>::SetDataSize(
    const std::array<int, N_DIM> &dims_size) {
  int total_ele_num = 1;
  for (int i = 0; i < N_DIM; ++i) {
    total_ele_num = total_ele_num * dims_size_[i];
  }
  data_size_ = total_ele_num;
  return kSuccess;
}

template class GridMapND<uint8_t, 2>;
template class GridMapND<uint8_t, 3>;
template class GridMapND<int, 2>;
template class GridMapND<int, 3>;

void LaneRaw::print() const {
  printf("Lane %d:\n", id);
  printf(" -- dir:\t%d\n", dir);

  printf(" -- child_id:\t[");
  for (const auto &id : child_id) {
    printf(" %d ", id);
  }
  printf("]\n");

  printf(" -- father_id:\t[");
  for (const auto &id : father_id) {
    printf(" %d ", id);
  }
  printf("]\n");
  printf(" -- length:\t%f\n", length);
  printf(" -- l_lane_id:\t%d\n", l_lane_id);
  printf(" -- l_change_avbl:\t%d\n", l_change_avbl);
  printf(" -- r_lane_id:\t%d\n", r_lane_id);
  printf(" -- r_change_avbl:\t%d\n", r_change_avbl);

  printf(" -- behavior:\t%s\n", behavior.c_str());

  printf(" -- start_point: (%f, %f)\n", start_point(0), start_point(1));
  printf(" -- final_point: (%f, %f)\n", final_point(0), final_point(1));
  printf(" -- point number:\t%d\n", static_cast<int>(lane_points.size()));
}

void LaneNet::print() const {
  printf("LaneNet:\n");
  printf(" -- Number of lanes:\t%d\n", static_cast<int>(lane_set.size()));
  for (auto it = lane_set.begin(); it != lane_set.end(); ++it) {
    it->second.print();
  }
  printf("\n");
}

void SemanticLaneSet::print() const {
  printf("SemanticLaneSet:\n");
  printf(" -- Number of lanes:\t%d\n", static_cast<int>(semantic_lanes.size()));
}

void CircleObstacle::print() const {
  printf("id: %d\n", id);
  circle.print();
  printf("\n");
}

void PolygonObstacle::print() const {
  printf("id: %d\n", id);
  polygon.print();
  printf("\n");
}

void ObstacleSet::print() const {
  for (const auto &obs : obs_circle) {
    obs.second.print();
  }
  for (const auto &obs : obs_polygon) {
    obs.second.print();
  }
}

TrafficSignal::TrafficSignal()
    : start_point_(Vec2f::Zero()),
      end_point_(Vec2f::Zero()),
      valid_time_(Vec2f(0.0, std::numeric_limits<decimal_t>::max())),
      vel_range_(Vec2f::Zero()),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

TrafficSignal::TrafficSignal(const Vec2f &start_point, const Vec2f &end_point,
                             const Vec2f &valid_time, const Vec2f &vel_range)
    : start_point_(start_point),
      end_point_(end_point),
      valid_time_(valid_time),
      vel_range_(vel_range),
      lateral_range_(Vec2f(-1.75, 1.75)) {}

void TrafficSignal::set_start_point(const Vec2f &start_point) {
  start_point_ = start_point;
}

void TrafficSignal::set_end_point(const Vec2f &end_point) {
  end_point_ = end_point;
}

void TrafficSignal::set_valid_time_til(const decimal_t max_valid_time) {
  valid_time_(1) = max_valid_time;
}

void TrafficSignal::set_valid_time_begin(const decimal_t min_valid_time) {
  valid_time_(0) = min_valid_time;
}

void TrafficSignal::set_valid_time(const Vec2f &valid_time) {
  valid_time_ = valid_time;
}

void TrafficSignal::set_vel_range(const Vec2f &vel_range) {
  vel_range_ = vel_range;
}

void TrafficSignal::set_lateral_range(const Vec2f &lateral_range) {
  lateral_range_ = lateral_range;
}

void TrafficSignal::set_max_velocity(const decimal_t max_velocity) {
  vel_range_(1) = max_velocity;
}

Vec2f TrafficSignal::start_point() const { return start_point_; }
Vec2f TrafficSignal::end_point() const { return end_point_; }
Vec2f TrafficSignal::valid_time() const { return valid_time_; }
Vec2f TrafficSignal::vel_range() const { return vel_range_; }
Vec2f TrafficSignal::lateral_range() const { return lateral_range_; }
decimal_t TrafficSignal::max_velocity() const { return vel_range_(1); }

SpeedLimit::SpeedLimit(const Vec2f &start_point, const Vec2f &end_point,
                       const Vec2f &vel_range)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    vel_range) {}

StoppingSign::StoppingSign(const Vec2f &start_point, const Vec2f &end_point)
    : TrafficSignal(start_point, end_point,
                    Vec2f(0.0, std::numeric_limits<decimal_t>::max()),
                    Vec2f::Zero()) {}

void TrafficLight::set_type(const Type &type) { type_ = type; }

TrafficLight::Type TrafficLight::type() const { return type_; }

ErrorType SemanticsUtils::GetOrientedBoundingBoxForVehicleUsingState(
    const VehicleParam &param, const State &s, OrientedBoundingBox2D *obb) {
  double cos_theta = cos(s.angle);
  double sin_theta = sin(s.angle);
  obb->x = s.vec_position[0] + param.d_cr() * cos_theta;
  obb->y = s.vec_position[1] + param.d_cr() * sin_theta;
  obb->angle = s.angle;
  obb->length = param.length();
  obb->width = param.width();
  return kSuccess;
}

ErrorType SemanticsUtils::GetVehicleVertices(const VehicleParam &param,
                                             const State &state,
                                             vec_E<Vec2f> *vertices) {
  decimal_t angle = state.angle;

  decimal_t cos_theta = cos(angle);
  decimal_t sin_theta = sin(angle);

  decimal_t c_x = state.vec_position(0) + param.d_cr() * cos_theta;
  decimal_t c_y = state.vec_position(1) + param.d_cr() * sin_theta;

  decimal_t d_wx = param.width() / 2 * sin_theta;
  decimal_t d_wy = param.width() / 2 * cos_theta;
  decimal_t d_lx = param.length() / 2 * cos_theta;
  decimal_t d_ly = param.length() / 2 * sin_theta;

  // Counterclockwise from left-front vertex
  vertices->push_back(Vec2f(c_x - d_wx + d_lx, c_y + d_wy + d_ly));
  // vertices->push_back(Vec2f(c_x - d_wx, c_y + d_wy));
  vertices->push_back(Vec2f(c_x - d_wx - d_lx, c_y - d_ly + d_wy));
  // vertices->push_back(Vec2f(c_x - d_lx, c_y - d_ly));
  vertices->push_back(Vec2f(c_x + d_wx - d_lx, c_y - d_wy - d_ly));
  // vertices->push_back(Vec2f(c_x + d_wx, c_y - d_wy));
  vertices->push_back(Vec2f(c_x + d_wx + d_lx, c_y + d_ly - d_wy));
  // vertices->push_back(Vec2f(c_x + d_lx, c_y + d_ly));

  return kSuccess;
}

ErrorType SemanticsUtils::InflateVehicleBySize(const Vehicle &vehicle_in,
                                               const decimal_t delta_w,
                                               const decimal_t delta_l,
                                               Vehicle *vehicle_out) {
  common::Vehicle inflated_vehicle = vehicle_in;
  common::VehicleParam vehicle_param = vehicle_in.param();
  vehicle_param.set_width(vehicle_param.width() + delta_w);
  vehicle_param.set_length(vehicle_param.length() + delta_l);
  inflated_vehicle.set_param(vehicle_param);
  *vehicle_out = inflated_vehicle;
  return kSuccess;
}

}  // namespace common