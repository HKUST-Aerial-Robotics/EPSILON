#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_VEHICLE_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_VEHICLE_MODEL_H__

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/state/state.h"

namespace simulator {

class VehicleModel {
 public:
  using State = common::State;
  struct Control {
    double steer_rate{0.0};  // steer rate of front wheel
    double acc_long{0.0};    // longitudial acceleration
    Control() {}
    Control(const double s, const double a) : steer_rate(s), acc_long(a) {}
  };

  VehicleModel();

  VehicleModel(double wheelbase_len, double max_steering_angle);

  ~VehicleModel();

  const State &state(void) const;

  void set_state(const State &state);

  void set_control(const Control &control);

  void Step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 5> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void UpdateInternalState(void);
  State state_;
  Control control_;
  InternalState internal_state_;
  double wheelbase_len_;
  double max_steering_angle_;
};
}  // namespace simulator

#endif