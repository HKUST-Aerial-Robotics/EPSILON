#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_

#include "common/basics/basics.h"
#include "vehicle_model/ctx_idm_model.h"

namespace control {

class ContextIntelligentVelocityControl {
 public:
  static ErrorType CalculateDesiredVelocity(
      const common::IntelligentDriverModel::Param& idm_param,
      const simulator::ContextIntelligentDriverModel::CtxParam& ctx_param,
      const decimal_t s, const decimal_t s_front, const decimal_t s_target,
      const decimal_t v, const decimal_t v_front, const decimal_t v_target,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif  //_CORE_VEHICLE_MODE_INC_CONTROLLERS_CTX_IDM_VELOCITY_H_