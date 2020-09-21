#ifndef _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__
#define _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/idm/intelligent_driver_model.h"
#include "common/rss/rss_checker.h"

namespace common {

class MobilLaneChangingModel {
 public:
  static ErrorType GetMobilAccChangesOnCurrentLane(
      const FrenetState &cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
      decimal_t *acc_c);

  static ErrorType GetMobilAccChangesOnTargetLane(
      const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
      const FrenetState &leading_fs, const Vehicle &following_vehicle,
      const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
      decimal_t *acc_n_tilda, decimal_t *acc_c_tilda);

 private:
  static ErrorType GetDesiredAccelerationUsingIdm(
      const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
      const FrenetState &front_fs, const bool &use_virtual_front,
      decimal_t *acc);
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_MOBIL_MOBIL_MODEL_H__