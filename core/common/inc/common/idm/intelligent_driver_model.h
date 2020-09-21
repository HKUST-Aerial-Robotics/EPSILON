#ifndef _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__
#define _CORE_COMMON_INC_COMMON_IDM_INTELLIGENT_DRIVER_MODEL_H__

#include "common/basics/basics.h"

namespace common {

class IntelligentDriverModel {
 public:
  struct State {
    decimal_t s{0.0};        // longitudinal distance
    decimal_t v{0.0};        // longitudinal speed
    decimal_t s_front{0.0};  // leading vehicle
    decimal_t v_front{0.0};

    State() {}
    State(const decimal_t &s_, const decimal_t &v_, const decimal_t &s_front_,
          const decimal_t &v_front_)
        : s(s_), v(v_), s_front(s_front_), v_front(v_front_) {}
  };

  struct Param {
    decimal_t kDesiredVelocity = 0.0;
    decimal_t kVehicleLength = 5.0;                   // l_alpha-1
    decimal_t kMinimumSpacing = 2.0;                  // s0
    decimal_t kDesiredHeadwayTime = 1.0;              // T
    decimal_t kAcceleration = 2.0;                    // a
    decimal_t kComfortableBrakingDeceleration = 3.0;  // b
    decimal_t kHardBrakingDeceleration = 5.0;
    int kExponent = 4;  // delta
  };

  static ErrorType GetIdmDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
  static ErrorType GetIIdmDesiredAcceleration(const Param &param,
                                              const State &cur_state,
                                              decimal_t *acc);
  static ErrorType GetAccDesiredAcceleration(const Param &param,
                                             const State &cur_state,
                                             decimal_t *acc);
};

}  // namespace common

#endif