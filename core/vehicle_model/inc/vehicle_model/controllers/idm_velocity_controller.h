#ifndef _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_
#define _CORE_VEHICLE_MODE_INC_CONTROLLERS_IDM_VELOCITY_H_

#include "common/basics/basics.h"

#include "vehicle_model/idm_model.h"

namespace control {

/**
 * @brief Intelligent driver model
 * (https://en.wikipedia.org/wiki/Intelligent_driver_model).
 */
class IntelligentVelocityControl {
 public:
  static ErrorType CalculateDesiredVelocity(
      const simulator::IntelligentDriverModel::Param& param, const decimal_t s,
      const decimal_t s_front, const decimal_t v, const decimal_t v_front,
      const decimal_t dt, decimal_t* velocity_at_dt);
};

}  // namespace control

#endif