#include "common/mobil/mobil_behavior_prediction.h"

namespace common {

ErrorType MobilBehaviorPrediction::RemapGainsToProb(
    const bool is_lcl_safe, const decimal_t mobil_gain_left,
    const bool is_lcr_safe, const decimal_t mobil_gain_right,
    ProbDistOfLatBehaviors *res) {
  decimal_t lower_bound = -1.0;
  // decimal_t upper_bound = 1.0;
  decimal_t upper_bound = 6.0;

  if (is_lcl_safe && is_lcr_safe) {
    decimal_t gain_l = normalize_with_bound(mobil_gain_left, lower_bound,
                                            upper_bound, 0.0, 1.0);
    decimal_t gain_k =
        normalize_with_bound(1.0, lower_bound, upper_bound, 0.0, 1.0);
    decimal_t gain_r = normalize_with_bound(mobil_gain_right, lower_bound,
                                            upper_bound, 0.0, 1.0);

    decimal_t p_l = gain_l / (gain_k + gain_l + gain_r);
    decimal_t p_k = gain_k / (gain_k + gain_l + gain_r);
    decimal_t p_r = gain_r / (gain_k + gain_l + gain_r);

    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, p_l);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, p_r);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, p_k);
    res->is_valid = true;

  } else if (is_lcl_safe) {
    decimal_t gain = normalize_with_bound(mobil_gain_left, lower_bound,
                                          upper_bound, 0.0, 1.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, gain);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0 - gain);
    res->is_valid = true;
  } else if (is_lcr_safe) {
    decimal_t gain = normalize_with_bound(mobil_gain_right, lower_bound,
                                          upper_bound, 0.0, 1.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, gain);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0 - gain);
    res->is_valid = true;
  } else {
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0);
    res->is_valid = true;
  }
  return kSuccess;
}

ErrorType MobilBehaviorPrediction::LateralBehaviorPrediction(
    const Vehicle &vehicle, const vec_E<Lane> &lanes,
    const vec_E<common::Vehicle> &leading_vehicles,
    const vec_E<common::FrenetState> &leading_frenet_states,
    const vec_E<common::Vehicle> &following_vehicles,
    const vec_E<common::FrenetState> &follow_frenet_states,
    const common::VehicleSet &nearby_vehicles, ProbDistOfLatBehaviors *res) {
  if (lanes.size() != 3) {
    printf(
        "[MobilBehaviorPrediction]Must have three lanes (invalid also "
        "acceptable).\n");
    return kWrongStatus;
  }

  // * Notations here are following MOBIL paper
  decimal_t acc_c = 0.0;
  decimal_t acc_o = 0.0, acc_o_tilda = 0.0;
  decimal_t politeness_coeff = 0.0;

  bool is_lcl_safe = false;
  bool is_lcr_safe = false;

  decimal_t mobil_gain_left = -kInf;
  decimal_t mobil_gain_right = -kInf;

  // * When the vehicle is almost stop, we assume the behavior is lane keeping
  decimal_t desired_vel = vehicle.state().velocity;
  if (fabs(desired_vel) < kBigEPS) {
    res->SetEntry(common::LateralBehavior::kLaneChangeLeft, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneChangeRight, 0.0);
    res->SetEntry(common::LateralBehavior::kLaneKeeping, 1.0);
    res->is_valid = true;
    return kSuccess;
  }

  // ~ For current lane
  common::Lane lk_lane = lanes[0];
  if (lk_lane.IsValid()) {
    common::Vehicle leading_vehicle = leading_vehicles[0];
    common::FrenetState leading_fs = leading_frenet_states[0];
    common::Vehicle following_vehicle = following_vehicles[0];
    common::FrenetState following_fs = follow_frenet_states[0];

    common::StateTransformer stf(lk_lane);
    common::FrenetState ego_frenet_state;
    stf.GetFrenetStateFromState(vehicle.state(), &ego_frenet_state);

    if (MobilLaneChangingModel::GetMobilAccChangesOnCurrentLane(
            ego_frenet_state, leading_vehicle, leading_fs, following_vehicle,
            following_fs, &acc_o, &acc_o_tilda, &acc_c) != kSuccess) {
      printf("[MobilBehaviorPrediction]lane-keep lane not valid.\n");
      return kWrongStatus;
    }
  } else {
    return kWrongStatus;
  }

  // ~ For lane change left
  common::Lane lcl_lane = lanes[1];
  if (lcl_lane.IsValid()) {
    common::Vehicle leading_vehicle = leading_vehicles[1];
    common::FrenetState leading_fs = leading_frenet_states[1];
    common::Vehicle following_vehicle = following_vehicles[1];
    common::FrenetState following_fs = follow_frenet_states[1];
    decimal_t acc_n = 0.0, acc_n_tilda = 0.0, acc_c_tilda = 0.0;

    common::StateTransformer stf(lcl_lane);
    common::FrenetState projected_frenet_state;
    stf.GetFrenetStateFromState(vehicle.state(), &projected_frenet_state);

    MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
        projected_frenet_state, leading_vehicle, leading_fs, following_vehicle,
        following_fs, &is_lcl_safe, &acc_n, &acc_n_tilda, &acc_c_tilda);
    if (is_lcl_safe) {
      decimal_t d_a_c = acc_c_tilda - acc_c;
      decimal_t d_a_n = acc_n_tilda - acc_n;
      decimal_t d_a_o = acc_o_tilda - acc_o;
      mobil_gain_left = d_a_c + politeness_coeff * (d_a_n + d_a_o);
    }
  }

  // ~ For lane change right
  common::Lane lcr_lane = lanes[2];
  if (lcr_lane.IsValid()) {
    common::Vehicle leading_vehicle = leading_vehicles[2];
    common::FrenetState leading_fs = leading_frenet_states[2];
    common::Vehicle following_vehicle = following_vehicles[2];
    common::FrenetState following_fs = follow_frenet_states[2];
    decimal_t acc_n = 0.0, acc_n_tilda = 0.0, acc_c_tilda = 0.0;

    common::StateTransformer stf(lcr_lane);
    common::FrenetState projected_frenet_state;
    stf.GetFrenetStateFromState(vehicle.state(), &projected_frenet_state);

    MobilLaneChangingModel::GetMobilAccChangesOnTargetLane(
        projected_frenet_state, leading_vehicle, leading_fs, following_vehicle,
        following_fs, &is_lcr_safe, &acc_n, &acc_n_tilda, &acc_c_tilda);
    if (is_lcr_safe) {
      decimal_t d_a_c = acc_c_tilda - acc_c;
      decimal_t d_a_n = acc_n_tilda - acc_n;
      decimal_t d_a_o = acc_o_tilda - acc_o;
      mobil_gain_right = d_a_c + politeness_coeff * (d_a_n + d_a_o);
    }
  }

  RemapGainsToProb(is_lcl_safe, mobil_gain_left, is_lcr_safe, mobil_gain_right,
                   res);
  printf("[Mobil]%d, Left gain: %.3lf, Right gain: %.3lf\n", vehicle.id(),
         mobil_gain_left, mobil_gain_right);
  return kSuccess;
}

}  // namespace common
