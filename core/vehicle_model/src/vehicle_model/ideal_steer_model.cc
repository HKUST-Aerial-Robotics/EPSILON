#include "vehicle_model/ideal_steer_model.h"

#include "common/math/calculations.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

namespace simulator {

IdealSteerModel::IdealSteerModel(double wheelbase_len, double max_lon_acc,
                                 double max_lon_dec, double max_lon_acc_jerk,
                                 double max_lon_dec_jerk, double max_lat_acc,
                                 double max_lat_jerk, double max_steering_angle,
                                 double max_steer_rate, double max_curvature)
    : wheelbase_len_(wheelbase_len),
      max_lon_acc_(max_lon_acc),
      max_lon_dec_(max_lon_dec),
      max_lon_acc_jerk_(max_lon_acc_jerk),
      max_lon_dec_jerk_(max_lon_dec_jerk),
      max_lat_acc_(max_lat_acc),
      max_lat_jerk_(max_lat_jerk),
      max_steering_angle_(max_steering_angle),
      max_steer_rate_(max_steer_rate),
      max_curvature_(max_curvature) {
  // printf("[DEBUG]max_lon_acc_ %lf.\n", max_lon_acc_);
  // printf("[DEBUG]max_lon_dec_ %lf.\n", max_lon_dec_);
  // printf("[DEBUG]max_lon_acc_jerk_ %lf.\n", max_lon_acc_jerk_);
  // printf("[DEBUG]max_lon_dec_jerk_ %lf.\n", max_lon_dec_jerk_);
  // printf("[DEBUG]max_lat_acc_ %lf.\n", max_lat_acc_);
  // printf("[DEBUG]max_lat_jerk_ %lf.\n", max_lat_jerk_);
  // printf("[DEBUG]max_steering_angle_ %lf.\n", max_steering_angle_);
  // printf("[DEBUG]max_steer_rate_ %lf.\n", max_steer_rate_);
  // printf("[DEBUG]max_curvature_ %lf.\n", max_curvature_);
  UpdateInternalState();
}

IdealSteerModel::~IdealSteerModel() {}

void IdealSteerModel::TruncateControl(const decimal_t &dt) {
  // truncate velocity
  // decimal_t max_velocity_by_model =
  //     sqrt(max_lat_acc_ / std::min(fabs(state_.curvature), max_curvature_));
  // control_.velocity =
  //     std::min(std::max(0.0, control_.velocity), max_velocity_by_model);
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  decimal_t desired_lon_jerk = (desired_lon_acc_ - state_.acceleration) / dt;
  desired_lon_jerk =
      truncate(desired_lon_jerk, -max_lon_dec_jerk_, max_lon_acc_jerk_);
  desired_lon_acc_ = desired_lon_jerk * dt + state_.acceleration;
  desired_lon_acc_ = truncate(desired_lon_acc_, -max_lon_dec_, max_lon_acc_);
  control_.velocity = std::max(state_.velocity + desired_lon_acc_ * dt, 0.0);

  desired_lat_acc_ =
      pow(control_.velocity, 2) * (tan(control_.steer) / wheelbase_len_);
  decimal_t lat_acc_ori = pow(state_.velocity, 2) * state_.curvature;
  decimal_t lat_jerk_desired = (desired_lat_acc_ - lat_acc_ori) / dt;
  lat_jerk_desired = truncate(lat_jerk_desired, -max_lat_jerk_, max_lat_jerk_);
  desired_lat_acc_ = lat_jerk_desired * dt + lat_acc_ori;
  desired_lat_acc_ = truncate(desired_lat_acc_, -max_lat_acc_, max_lat_acc_);
  control_.steer = atan(desired_lat_acc_ * wheelbase_len_ /
                        std::max(pow(control_.velocity, 2), 0.1 * kBigEPS));
  desired_steer_rate_ = normalize_angle(control_.steer - state_.steer) / dt;
  desired_steer_rate_ =
      truncate(desired_steer_rate_, -max_steer_rate_, max_steer_rate_);
  control_.steer = normalize_angle(state_.steer + desired_steer_rate_ * dt);
}

void IdealSteerModel::Step(double dt) {
  state_.steer = atan(state_.curvature * wheelbase_len_);
  UpdateInternalState();
  control_.velocity = std::max(0.0, control_.velocity);
  control_.steer =
      truncate(control_.steer, -max_steering_angle_, max_steering_angle_);
  TruncateControl(dt);
  desired_lon_acc_ = (control_.velocity - state_.velocity) / dt;
  desired_steer_rate_ = normalize_angle(control_.steer - state_.steer) / dt;

  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  state_.vec_position(0) = internal_state_[0];
  state_.vec_position(1) = internal_state_[1];
  state_.angle = normalize_angle(internal_state_[2]);
  state_.velocity = internal_state_[3];
  state_.steer = normalize_angle(internal_state_[4]);
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = desired_lon_acc_;
  UpdateInternalState();
}

void IdealSteerModel::operator()(const InternalState &x, InternalState &dxdt,
                                 const double /* t */) {
  State cur_state;
  cur_state.vec_position(0) = x[0];
  cur_state.vec_position(1) = x[1];
  cur_state.angle = x[2];
  cur_state.velocity = x[3];
  cur_state.steer = x[4];

  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;
  dxdt[3] = desired_lon_acc_;
  dxdt[4] = desired_steer_rate_;
}

void IdealSteerModel::set_control(const Control &control) {
  control_ = control;
}

const IdealSteerModel::State &IdealSteerModel::state(void) const {
  return state_;
}

void IdealSteerModel::set_state(const IdealSteerModel::State &state) {
  state_ = state;
  UpdateInternalState();
}

void IdealSteerModel::UpdateInternalState(void) {
  internal_state_[0] = state_.vec_position(0);
  internal_state_[1] = state_.vec_position(1);
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.velocity;
  internal_state_[4] = state_.steer;
}

}  // namespace simulator