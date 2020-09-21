#include "vehicle_model/ctx_idm_model.h"

#include "common/math/calculations.h"
#include "odeint-v2/boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

namespace simulator {

ContextIntelligentDriverModel::ContextIntelligentDriverModel() {
  UpdateInternalState();
}

ContextIntelligentDriverModel::ContextIntelligentDriverModel(
    const IdmParam &idm_parm, const CtxParam &ctx_param)
    : idm_param_(idm_parm), ctx_param_(ctx_param) {
  UpdateInternalState();
}

ContextIntelligentDriverModel::~ContextIntelligentDriverModel() {}

void ContextIntelligentDriverModel::Step(double dt) {
  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);
  state_.s = internal_state_[0];
  state_.v = internal_state_[1];
  state_.s_front = internal_state_[2];
  state_.v_front = internal_state_[3];
  state_.s_target = internal_state_[4];
  state_.v_target = internal_state_[5];
  UpdateInternalState();
}

void ContextIntelligentDriverModel::operator()(const InternalState &x,
                                               InternalState &dxdt,
                                               const double dt) {
  CtxIdmState cur_state;
  cur_state.s = x[0];
  cur_state.v = x[1];
  cur_state.s_front = x[2];
  cur_state.v_front = x[3];
  cur_state.s_target = x[4];
  cur_state.v_target = x[5];

  IdmState idm_state;

  decimal_t acc_idm;
  common::IntelligentDriverModel::GetAccDesiredAcceleration(
      idm_param_, idm_state, &acc_idm);
  acc_idm = std::max(acc_idm, -std::min(idm_param_.kHardBrakingDeceleration,
                                        cur_state.v / dt));

  decimal_t v_ref =
      cur_state.v_target + ctx_param_.k_s * (cur_state.s_target - cur_state.s);
  decimal_t acc_track = ctx_param_.k_v * (v_ref - cur_state.v);

  acc_track = std::min(std::max(acc_track, -1.0), 1.0);

  decimal_t acc = acc_track;

  dxdt[0] = cur_state.v;
  dxdt[1] = acc;
  dxdt[2] = cur_state.v_front;
  dxdt[3] = 0.0;  // assume other vehicle keep the current velocity
  dxdt[4] = cur_state.v_target;
  dxdt[5] = 0.0;  // assume constant velocity for target state
}

const ContextIntelligentDriverModel::CtxIdmState &
ContextIntelligentDriverModel::state(void) const {
  return state_;
}

void ContextIntelligentDriverModel::set_state(
    const ContextIntelligentDriverModel::CtxIdmState &state) {
  state_ = state;
  UpdateInternalState();
}

void ContextIntelligentDriverModel::UpdateInternalState(void) {
  internal_state_[0] = state_.s;
  internal_state_[1] = state_.v;
  internal_state_[2] = state_.s_front;
  internal_state_[3] = state_.v_front;
  internal_state_[4] = state_.s_target;
  internal_state_[5] = state_.v_target;
}

}  // namespace simulator