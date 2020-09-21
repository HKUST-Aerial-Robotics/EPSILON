#ifndef _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__
#define _CORE_COMMON_INC_COMMON_RSS_CHECKER_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"

namespace common {

class RssChecker {
 public:
  enum LongitudinalDirection { Front = 0, Rear };
  enum LateralDirection { Left = 0, Right };
  enum class LongitudinalViolateType { Legal = 0, TooFast, TooSlow };

  struct RssConfig {
    decimal_t response_time = 0.1;
    decimal_t longitudinal_acc_max = 2.0;
    decimal_t longitudinal_brake_min = 4.0;
    decimal_t longitudinal_brake_max = 5.0;
    decimal_t lateral_acc_max = 1.0;
    decimal_t lateral_brake_min = 1.0;
    decimal_t lateral_brake_max = 1.0;
    decimal_t lateral_miu = 0.5;
    RssConfig() {}
    RssConfig(const decimal_t _response_time,
              const decimal_t _longitudinal_acc_max,
              const decimal_t _longitudinal_brake_min,
              const decimal_t _longitudinal_brake_max,
              const decimal_t _lateral_acc_max,
              const decimal_t _lateral_brake_min,
              const decimal_t _lateral_brake_max, const decimal_t _lateral_miu)
        : response_time(_response_time),
          longitudinal_acc_max(_longitudinal_acc_max),
          longitudinal_brake_min(_longitudinal_brake_min),
          longitudinal_brake_max(_longitudinal_brake_max),
          lateral_acc_max(_lateral_acc_max),
          lateral_brake_min(_lateral_brake_min),
          lateral_brake_max(_lateral_brake_max),
          lateral_miu(_lateral_miu) {}
  };

  static ErrorType CalculateSafeLongitudinalDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LongitudinalDirection& direction, const RssConfig& config,
      decimal_t* distance);

  static ErrorType CalculateSafeLateralDistance(
      const decimal_t ego_vel, const decimal_t other_vel,
      const LateralDirection& direction, const RssConfig& config,
      decimal_t* distance);

  static ErrorType CalculateRssSafeDistances(
      const std::vector<decimal_t>& ego_vels,
      const std::vector<decimal_t>& other_vels,
      const LongitudinalDirection& long_direct,
      const LateralDirection& lat_direct, const RssConfig& config,
      std::vector<decimal_t>* safe_distances);

  static ErrorType RssCheck(const FrenetState& ego_fs,
                            const FrenetState& other_fs,
                            const RssConfig& config, bool* is_safe);

  static ErrorType RssCheck(const Vehicle& ego_vehicle,
                            const Vehicle& other_vehicle, const StateTransformer& stf,
                            const RssConfig& config, bool* is_safe,
                            LongitudinalViolateType* lon_type,
                            decimal_t* rss_vel_low, decimal_t* rss_vel_up);

  static ErrorType CalculateSafeLongitudinalVelocity(
      const decimal_t other_vel, const LongitudinalDirection& direction,
      const decimal_t& lon_distance_abs, const RssConfig& config,
      decimal_t* ego_vel_low, decimal_t* ego_vel_upp);
};

}  // namespace common

#endif