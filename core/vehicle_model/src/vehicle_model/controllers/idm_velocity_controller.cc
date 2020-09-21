#include "vehicle_model/controllers/idm_velocity_controller.h"

#include "vehicle_model/idm_model.h"

namespace control {
ErrorType IntelligentVelocityControl::CalculateDesiredVelocity(
    const simulator::IntelligentDriverModel::Param& param, const decimal_t s,
    const decimal_t s_front, const decimal_t v, const decimal_t v_front,
    const decimal_t dt, decimal_t* velocity_at_dt) {
  using simulator::IntelligentDriverModel;
  IntelligentDriverModel model(param);
  IntelligentDriverModel::State state;
  state.s = s;
  state.v =
      std::max(0.0, v);  // negative velocity may cause integration unbounded
  state.s_front = s_front;
  state.v_front = v_front;
  model.set_state(state);
  model.Step(dt);
  // printf("[DEBUG]s %lf, s_front %lf, v %lf, v_front %lf dt %lf.\n", s,
  // s_front,
  //        v, v_front, dt);
  // printf("[DEBUG]desired v %lf, braking acc %lf.\n", param.kDesiredVelocity,
  //        param.kComfortableBrakingDeceleration);

  auto desired_state = model.state();
  // if (std::isnan(desired_state.v)) {
  //   printf("[DEBUGNAN]next s %lf, s_front %lf, v %lf, v_front %lf.\n",
  //          desired_state.s, desired_state.s_front, v, desired_state.v_front);
  //   assert(!std::isnan(desired_state.v));
  // }
  *velocity_at_dt = std::max(0.0, desired_state.v);
  // if (std::isinf(*velocity_at_dt)) {
  //   printf("[DEBUG]idm desired v %lf a %lf, b %lf, T %lf, gap %lf.\n",
  //          param.kDesiredVelocity, param.kAcceleration,
  //          param.kComfortableBrakingDeceleration, param.kDesiredHeadwayTime,
  //          param.kMinimumSpacing);
  //   printf("[DEBUG]s %lf, s_front %lf, v %lf, v_front %lf, dt %lf out%lf.\n",
  //   s,
  //          s_front, v, v_front, dt, *velocity_at_dt);
  //   assert(false);
  // }
  return kSuccess;
}

}  // namespace control