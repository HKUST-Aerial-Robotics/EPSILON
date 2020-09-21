#include "common/mobil/mobil_model.h"

namespace common {

ErrorType MobilLaneChangingModel::GetMobilAccChangesOnCurrentLane(
    const FrenetState &cur_fs, const Vehicle &leading_vehicle,
    const FrenetState &leading_fs, const Vehicle &following_vehicle,
    const FrenetState &following_fs, decimal_t *acc_o, decimal_t *acc_o_tilda,
    decimal_t *acc_c) {
  bool has_leading_vehicle =
      leading_vehicle.id() == kInvalidAgentId ? false : true;
  bool has_following_vehicle =
      following_vehicle.id() == kInvalidAgentId ? false : true;
  // printf("[Mobil]%d, Cur lane leading: %d, following: %d\n", vehicle.id(),
  //        leading_vehicle.id(), following_vehicle.id());

  decimal_t acc_o_tmp = 0.0, acc_o_tilda_tmp = 0.0, acc_c_tmp = 0.0;

  // ~ IDM desired velocity: current velocity
  IntelligentDriverModel::Param idm_param_o;
  idm_param_o.kDesiredVelocity = following_fs.vec_s(1);
  IntelligentDriverModel::Param idm_param_c;
  idm_param_c.kDesiredVelocity = cur_fs.vec_s(1);

  if ((!has_following_vehicle) || fabs(following_fs.vec_s(1)) < kEPS) {
    // ~ Without following vehicle or following vehicle is stop
    if (!has_leading_vehicle) {
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, common::FrenetState(),
                                     true, &acc_c_tmp);
    } else {
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, leading_fs, false,
                                     &acc_c_tmp);
    }
  } else {
    if (!has_leading_vehicle) {
      // ~ Without leading vehicle, use virtual vehicle
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, cur_fs, false,
                                     &acc_o_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs,
                                     common::FrenetState(), true,
                                     &acc_o_tilda_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, common::FrenetState(),
                                     true, &acc_c_tmp);
    } else {
      // ~ Have leading vehicle, use IDM
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, cur_fs, false,
                                     &acc_o_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_o, following_fs, leading_fs,
                                     false, &acc_o_tilda_tmp);
      GetDesiredAccelerationUsingIdm(idm_param_c, cur_fs, leading_fs, false,
                                     &acc_c_tmp);
    }
  }
  *acc_o = acc_o_tmp;
  *acc_o_tilda = acc_o_tilda_tmp;
  *acc_c = acc_c_tmp;
  // printf("[Mobil]acc_o = %lf, acc_o_tilda = %lf\n", acc_o, acc_o_tilda);
  return kSuccess;
}

ErrorType MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
    const FrenetState &projected_cur_fs, const Vehicle &leading_vehicle,
    const FrenetState &leading_fs, const Vehicle &following_vehicle,
    const FrenetState &following_fs, bool *is_lc_safe, decimal_t *acc_n,
    decimal_t *acc_n_tilda, decimal_t *acc_c_tilda) {
  bool has_leading_vehicle =
      leading_vehicle.id() == kInvalidAgentId ? false : true;
  bool has_following_vehicle =
      following_vehicle.id() == kInvalidAgentId ? false : true;

  decimal_t acc_n_tmp = 0.0, acc_n_tilda_tmp = 0.0, acc_c_tilda_tmp = 0.0;

  *is_lc_safe = false;
  // ~ Safety check
  if (leading_vehicle.id() == following_vehicle.id() &&
      leading_vehicle.id() != kInvalidAgentId) {
    *is_lc_safe = false;
  } else {
    bool is_front_safe = true, is_rear_safe = true;
    RssChecker::RssCheck(projected_cur_fs, leading_fs,
                         common::RssChecker::RssConfig(), &is_front_safe);
    RssChecker::RssCheck(projected_cur_fs, following_fs,
                         common::RssChecker::RssConfig(), &is_rear_safe);
    *is_lc_safe = is_front_safe && is_rear_safe;
  }

  if (*is_lc_safe) {
    IntelligentDriverModel::Param idm_param_n;
    idm_param_n.kDesiredVelocity = following_fs.vec_s(1);
    IntelligentDriverModel::Param idm_param_c;
    idm_param_c.kDesiredVelocity = projected_cur_fs.vec_s(1);

    if ((!has_following_vehicle) || fabs(following_fs.vec_s(1)) < kEPS) {
      // ~ Without following vehicle or following vehicle is stop
      if (!has_leading_vehicle) {
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       common::FrenetState(), true,
                                       &acc_c_tilda_tmp);
      } else {
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       leading_fs, false, &acc_c_tilda_tmp);
      }
    } else {
      if (!has_leading_vehicle) {
        // ~ Without leading vehicle, use virtual vehicle
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       common::FrenetState(), true, &acc_n_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       projected_cur_fs, false,
                                       &acc_n_tilda_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       common::FrenetState(), true,
                                       &acc_c_tilda_tmp);
      } else {
        // ~ Have leading vehicle, use IDM
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs, leading_fs,
                                       false, &acc_n_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_n, following_fs,
                                       projected_cur_fs, false,
                                       &acc_n_tilda_tmp);
        GetDesiredAccelerationUsingIdm(idm_param_c, projected_cur_fs,
                                       leading_fs, false, &acc_c_tilda_tmp);
      }
    }
    *acc_n = acc_n_tmp;
    *acc_n_tilda = acc_n_tilda_tmp;
    *acc_c_tilda = acc_c_tilda_tmp;
  }
  return kSuccess;
}

ErrorType MobilLaneChangingModel::GetDesiredAccelerationUsingIdm(
    const IntelligentDriverModel::Param &param, const FrenetState &rear_fs,
    const FrenetState &front_fs, const bool &use_virtual_front,
    decimal_t *acc) {
  IntelligentDriverModel::State idm_state;
  if (!use_virtual_front) {
    idm_state =
        IntelligentDriverModel::State(rear_fs.vec_s(0), rear_fs.vec_s(1),
                                      front_fs.vec_s(0), front_fs.vec_s(1));
  } else {
    idm_state = IntelligentDriverModel::State(
        0.0, rear_fs.vec_s(1), 100.0 + rear_fs.vec_s(1) * 10, rear_fs.vec_s(1));
  }

  IntelligentDriverModel::GetAccDesiredAcceleration(param, idm_state, acc);
  return kSuccess;
}

}  // namespace common
