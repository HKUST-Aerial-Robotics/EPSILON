#ifndef _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__
#define _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <iostream>

#include "common/basics/basics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/state/state.h"

namespace simulator {

class ContextIntelligentDriverModel {
 public:
  using IdmParam = common::IntelligentDriverModel::Param;
  using IdmState = common::IntelligentDriverModel::State;

  struct CtxParam {
    decimal_t k_s = 0.5;
    decimal_t k_v = 2.0 * k_s;

    CtxParam() = default;
    CtxParam(const decimal_t &_k_s, const decimal_t &_k_v)
        : k_s(_k_s), k_v(_k_v) {}
  };

  struct CtxIdmState {
    decimal_t s{0.0};        // longitudinal distance
    decimal_t v{0.0};        // longitudinal speed
    decimal_t s_front{0.0};  // leading vehicle
    decimal_t v_front{0.0};
    decimal_t s_target{0.0};
    decimal_t v_target{0.0};
  };

  ContextIntelligentDriverModel();

  ContextIntelligentDriverModel(const IdmParam &idm_parm,
                                const CtxParam &ctx_param);

  ~ContextIntelligentDriverModel();

  const CtxIdmState &state(void) const;

  void set_state(const CtxIdmState &state);

  void Step(double dt);

  // For internal use, but needs to be public for odeint
  typedef boost::array<double, 6> InternalState;
  void operator()(const InternalState &x, InternalState &dxdt,
                  const double /* t */);

 private:
  void UpdateInternalState(void);

  InternalState internal_state_;

  IdmParam idm_param_;
  CtxParam ctx_param_;

  CtxIdmState state_;
};
}  // namespace simulator

#endif  // _VEHICLE_MODEL_INC_VEHIDLE_MODEL_CTX_IDM_MODEL_H__