#include "vehicle_model/controllers/pid_controller.h"

namespace control {
PIDControl::PIDControl(const ControlParam& param)
    : param_(param), dt_(0.05), max_history_len_(1000) {
  printf("default PID controller with dt: %lf ms.\n", dt_);
}
PIDControl::PIDControl(const ControlParam& param, const decimal_t dt)
    : param_(param), dt_(dt), max_history_len_(1000) {}

decimal_t PIDControl::CalculatePIDControl(const decimal_t desired_state,
                                          const decimal_t true_state) {
  decimal_t et = desired_state - true_state;
  error_hist_.push_back(et);

  decimal_t int_e = 0.0;
  for (auto& err : error_hist_) {
    int_e += err * dt_;
  }

  decimal_t deriv_e = 0.0;
  int num_errors = static_cast<int>(error_hist_.size());
  if (num_errors > 2) {
    deriv_e =
        (error_hist_.at(num_errors - 1) - error_hist_.at(num_errors - 2)) / dt_;
  }

  if (num_errors > max_history_len_) error_hist_.pop_front();

  return param_.kP * et + param_.kI * int_e + param_.kD * deriv_e;
}

}  // namespace control
