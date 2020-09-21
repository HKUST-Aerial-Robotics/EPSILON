#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDEAL_STEER_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDEAL_STEER_MODEL_H__

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "common/basics/basics.h"
#include "common/state/state.h"

namespace simulator {

class IdealSteerModel {
 public:
  using State = common::State;
  struct Control {
    double steer{0.0};     // steer
    double velocity{0.0};  // body velocity
    Control() {}
    Control(const double s, const double v) : steer(s), velocity(v) {}
  };

  IdealSteerModel(double wheelbase_len, double max_lon_acc, double max_lon_dec,
                  double max_lon_acc_jerk, double max_lon_dec_jerk,
                  double max_lat_acc, double max_lat_jerk,
                  double max_steering_angle, double max_steer_rate,
                  double max_curvature);

  ~IdealSteerModel();

  const State &state(void) const;

  void set_state(const State &state);

  void set_control(const Control &control);

  void Step(double dt);

  void TruncateControl(const decimal_t& dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 5> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void UpdateInternalState(void);
  State state_;
  Control control_;
  decimal_t desired_steer_rate_;
  decimal_t desired_lon_acc_;
  decimal_t desired_lat_acc_;
  InternalState internal_state_;
  double wheelbase_len_;
  double max_lon_acc_;
  double max_lon_dec_;
  double max_lon_acc_jerk_;
  double max_lon_dec_jerk_;
  double max_lat_acc_;
  double max_lat_jerk_;
  double max_steering_angle_;
  double max_steer_rate_;
  double max_curvature_;
};
}  // namespace simulator

#endif