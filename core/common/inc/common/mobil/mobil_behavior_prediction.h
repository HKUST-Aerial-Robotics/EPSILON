#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_BEHAVIOR_PREDICTION_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_BEHAVIOR_PREDICTION_H__

#include <set>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/mobil/mobil_model.h"
#include "common/rss/rss_checker.h"

namespace common {

class MobilBehaviorPrediction {
 public:
  static ErrorType LateralBehaviorPrediction(
      const Vehicle &vehicle, const vec_E<Lane> &lanes,
      const vec_E<common::Vehicle> &leading_vehicles,
      const vec_E<common::FrenetState> &leading_frenet_states,
      const vec_E<common::Vehicle> &following_vehicles,
      const vec_E<common::FrenetState> &follow_frenet_states,
      const common::VehicleSet &nearby_vehicles, ProbDistOfLatBehaviors *res);

  static ErrorType RemapGainsToProb(const bool is_lcl_safe,
                                    const decimal_t mobil_gain_left,
                                    const bool is_lcr_safe,
                                    const decimal_t mobil_gain_right,
                                    ProbDistOfLatBehaviors *res);
};

}  // namespace common

#endif