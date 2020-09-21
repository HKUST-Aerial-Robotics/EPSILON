#include "vehicle_model/vehicle_model.h"

#include "common/math/calculations.h"

#include "odeint-v2/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

namespace simulator {

VehicleModel::VehicleModel() : wheelbase_len_(2.5) { UpdateInternalState(); }

VehicleModel::VehicleModel(double wheelbase_len, double max_steering_angle)
    : wheelbase_len_(wheelbase_len), max_steering_angle_(max_steering_angle) {
  UpdateInternalState();
}

VehicleModel::~VehicleModel() {}

void VehicleModel::Step(double dt) {
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  state_.vec_position(0) = internal_state_[0];
  state_.vec_position(1) = internal_state_[1];
  state_.angle = normalize_angle(internal_state_[2]);
  state_.steer = internal_state_[3];
  if (fabs(state_.steer) >= fabs(max_steering_angle_)) {
    if (state_.steer > 0)
      state_.steer = max_steering_angle_;
    else
      state_.steer = -max_steering_angle_;
  }
  state_.velocity = internal_state_[4];
  state_.curvature = tan(state_.steer) * 1.0 / wheelbase_len_;
  state_.acceleration = control_.acc_long;
  UpdateInternalState();
}

void VehicleModel::operator()(const InternalState &x, InternalState &dxdt,
                              const double /* t */) {
  State cur_state;
  cur_state.vec_position(0) = x[0];
  cur_state.vec_position(1) = x[1];
  cur_state.angle = x[2];
  cur_state.steer = x[3];
  cur_state.velocity = x[4];

  dxdt[0] = cos(cur_state.angle) * cur_state.velocity;
  dxdt[1] = sin(cur_state.angle) * cur_state.velocity;
  dxdt[2] = tan(cur_state.steer) * cur_state.velocity / wheelbase_len_;
  dxdt[3] = control_.steer_rate;
  dxdt[4] = control_.acc_long;
}

void VehicleModel::set_control(const Control &control) {
  control_ = control;
  // TODO: (@denny.ding) add control limit here
}

const VehicleModel::State &VehicleModel::state(void) const { return state_; }

void VehicleModel::set_state(const VehicleModel::State &state) {
  state_ = state;
  UpdateInternalState();
}

void VehicleModel::UpdateInternalState(void) {
  internal_state_[0] = state_.vec_position(0);
  internal_state_[1] = state_.vec_position(1);
  internal_state_[2] = state_.angle;
  internal_state_[3] = state_.steer;
  internal_state_[4] = state_.velocity;
}

}  // namespace simulator