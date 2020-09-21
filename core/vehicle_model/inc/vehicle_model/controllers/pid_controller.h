#ifndef _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_
#define _CORE_VEHICLE_MODEL_INC_CONTROLLERS_PID_CONTROLLER_H_

#include "common/basics/basics.h"

#include <deque>

namespace control {

class PIDControl {
 public:
  struct ControlParam {
    decimal_t kP;
    decimal_t kI;
    decimal_t kD;
    ControlParam() : kP(1.0), kI(1.0), kD(0.5) {}
    ControlParam(const decimal_t p, const decimal_t i, const decimal_t d)
        : kP(p), kI(i), kD(d) {}
  };

  PIDControl(const ControlParam& param);
  PIDControl(const ControlParam& param, const decimal_t dt);
  decimal_t CalculatePIDControl(const decimal_t desired_state, const decimal_t true_state);

 private:
  ControlParam param_;
  decimal_t dt_;
  int max_history_len_;
  std::deque<decimal_t> error_hist_;
};

}  // namespace control

#endif
