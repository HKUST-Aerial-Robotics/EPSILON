#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDM_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_IDM_MODEL_H__

#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "common/basics/basics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/state/state.h"

namespace simulator {

class IntelligentDriverModel {
 public:
  using Param = common::IntelligentDriverModel::Param;
  using State = common::IntelligentDriverModel::State;

  IntelligentDriverModel();

  IntelligentDriverModel(const Param &parm);

  ~IntelligentDriverModel();

  const State &state(void) const;

  void set_state(const State &state);

  void Step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 4> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void UpdateInternalState(void);

  void Linear(const InternalState &x, const double dt, InternalState *x_out);

  InternalState internal_state_;

  Param param_;
  State state_;
};
}  // namespace simulator

#endif