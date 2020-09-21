#ifndef _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_
#define _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_

#include <algorithm>

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "vehicle_model/controllers/ctx_idm_velocity_controller.h"
#include "vehicle_model/controllers/idm_velocity_controller.h"
#include "vehicle_model/controllers/pure_pursuit_controller.h"
#include "vehicle_model/ideal_steer_model.h"

namespace planning {

class OnLaneForwardSimulation {
 public:
  using Lane = common::Lane;
  using State = common::State;
  using FrenetState = common::FrenetState;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;
  using CtxParam = simulator::ContextIntelligentDriverModel::CtxParam;

  struct Param {
    simulator::IntelligentDriverModel::Param idm_param;
    decimal_t steer_control_gain = 1.5;
    decimal_t steer_control_max_lookahead_dist = 50.0;
    decimal_t steer_control_min_lookahead_dist = 3.0;
    decimal_t max_lat_acceleration_abs = 1.5;
    decimal_t max_lat_jerk_abs = 3.0;
    decimal_t max_curvature_abs = 0.33;
    decimal_t max_lon_acc_jerk = 5.0;
    decimal_t max_lon_brake_jerk = 5.0;
    decimal_t max_steer_angle_abs = 45.0 / 180.0 * kPi;
    decimal_t max_steer_rate = 0.39;
    bool auto_decelerate_if_lat_failed = true;
  };

  static ErrorType GetTargetStateOnTargetLane(
      const common::StateTransformer& stf_target,
      const common::Vehicle& ego_vehicle,
      const common::Vehicle& gap_front_vehicle,
      const common::Vehicle& gap_rear_vehicle, const Param& param,
      common::State* target_state) {
    common::FrenetState ego_fs;
    if (kSuccess !=
        stf_target.GetFrenetStateFromState(ego_vehicle.state(), &ego_fs)) {
      return kWrongStatus;
    }

    decimal_t time_headaway = param.idm_param.kDesiredHeadwayTime;
    decimal_t min_spacing = param.idm_param.kMinimumSpacing;

    bool has_front = false;
    common::FrenetState front_fs;
    decimal_t s_ref_front = -1;  // tail of front vehicle
    decimal_t s_thres_front = -1;
    if (gap_front_vehicle.id() != -1 &&
        kSuccess == stf_target.GetFrenetStateFromState(
                        gap_front_vehicle.state(), &front_fs)) {
      has_front = true;
      s_ref_front =
          front_fs.vec_s[0] - (gap_front_vehicle.param().length() / 2.0 -
                               gap_front_vehicle.param().d_cr());
      s_thres_front = s_ref_front - min_spacing -
                      time_headaway * ego_vehicle.state().velocity;
    }

    bool has_rear = false;
    common::FrenetState rear_fs;
    decimal_t s_ref_rear = -1;  // head of rear vehicle
    decimal_t s_thres_rear = -1;
    if (gap_rear_vehicle.id() != -1 &&
        kSuccess == stf_target.GetFrenetStateFromState(gap_rear_vehicle.state(),
                                                       &rear_fs)) {
      has_rear = true;
      s_ref_rear = rear_fs.vec_s[0] + gap_rear_vehicle.param().length() / 2.0 +
                   gap_rear_vehicle.param().d_cr();
      s_thres_rear = s_ref_rear + min_spacing +
                     time_headaway * gap_rear_vehicle.state().velocity;
    }

    decimal_t desired_s = ego_fs.vec_s[0];
    decimal_t desired_v = ego_vehicle.state().velocity;

    // ~ params
    decimal_t k_v = 0.1;      // coeff for dv. k_v * s_err
    decimal_t p_v_ego = 0.1;  // coeff for user preferred vel
    decimal_t dv_lb = -3.0;   // dv lower bound
    decimal_t dv_ub = 5.0;    // dv upper bound

    decimal_t ego_desired_vel =
        ego_vehicle.state().velocity +
        (param.idm_param.kDesiredVelocity - ego_vehicle.state().velocity) *
            p_v_ego;
    if (has_front && has_rear) {
      // * has both front and rear vehicle
      if (s_ref_front < s_ref_rear) {
        return kWrongStatus;
      }

      decimal_t ds = fabs(s_ref_front - s_ref_rear);
      decimal_t s_star = s_ref_rear + ds / 2.0 - ego_vehicle.param().d_cr();

      s_thres_front = std::max(s_star, s_thres_front);
      s_thres_rear = std::min(s_star, s_thres_rear);

      desired_s =
          std::min(std::max(s_thres_rear, ego_fs.vec_s[0]), s_thres_front);

      decimal_t s_err_front = s_thres_front - ego_fs.vec_s[0];
      decimal_t v_ref_front =
          std::max(0.0, gap_front_vehicle.state().velocity +
                            truncate(s_err_front * k_v, dv_lb, dv_ub));

      decimal_t s_err_rear = s_thres_rear - ego_fs.vec_s[0];
      decimal_t v_ref_rear =
          std::max(0.0, gap_rear_vehicle.state().velocity +
                            truncate(s_err_rear * k_v, dv_lb, dv_ub));

      desired_v = std::min(std::max(v_ref_rear, ego_desired_vel), v_ref_front);

    } else if (has_front) {
      // * only has front vehicle
      desired_s = std::min(ego_fs.vec_s[0], s_thres_front);

      decimal_t s_err_front = s_thres_front - ego_fs.vec_s[0];
      decimal_t v_ref_front =
          std::max(0.0, gap_front_vehicle.state().velocity +
                            truncate(s_err_front * k_v, dv_lb, dv_ub));
      desired_v = std::min(ego_desired_vel, v_ref_front);

    } else if (has_rear) {
      // * only has rear vehicle
      desired_s = std::max(ego_fs.vec_s[0], s_thres_rear);

      decimal_t s_err_rear = s_thres_rear - ego_fs.vec_s[0];
      decimal_t v_ref_rear =
          std::max(0.0, gap_rear_vehicle.state().velocity +
                            truncate(s_err_rear * k_v, dv_lb, dv_ub));
      desired_v = std::max(v_ref_rear, ego_desired_vel);
    }

    common::FrenetState target_fs;
    target_fs.Load(Vecf<3>(desired_s, desired_v, 0.0), Vecf<3>(0.0, 0.0, 0.0),
                   common::FrenetState::kInitWithDs);

    if (kSuccess !=
        stf_target.GetStateFromFrenetState(target_fs, target_state)) {
      return kWrongStatus;
    }

    return kSuccess;
  }

  static ErrorType PropagateOnceAdvancedLK(
      const common::StateTransformer& stf, const common::Vehicle& ego_vehicle,
      const Vehicle& leading_vehicle, const decimal_t& lat_track_offset,
      const decimal_t& dt, const Param& param, State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    // * Step I: Calculate steering
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if (stf.GetFrenetStateFromState(current_state, &current_fs) != kSuccess ||
        current_fs.vec_s[1] < -kEPS) {
      // * ego Frenet state invalid or ego vehicle reverse gear
      steer_calculation_failed = true;
    }

    decimal_t steer, velocity;
    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf, current_state, current_fs, wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // * Step II: Calculate velocity
    common::FrenetState leading_fs;
    if (leading_vehicle.id() == kInvalidAgentId ||
        stf.GetFrenetStateFromState(leading_vehicle.state(), &leading_fs) !=
            kSuccess) {
      // ~ Without leading vehicle
      CalcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                 &velocity);
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf, ego_vehicle, leading_vehicle,
                                    leading_fs, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingIdm(
          current_fs.vec_s[0], current_state.velocity, leading_fs.vec_s[0],
          leading_vehicle.state().velocity, dt, sim_param, &velocity);
    }

    // * Step III: Get desired state using vehicle kinematic model
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  static ErrorType PropagateOnceAdvancedLC(
      const common::StateTransformer& stf_current,
      const common::StateTransformer& stf_target,
      const common::Vehicle& ego_vehicle,
      const Vehicle& current_leading_vehicle, const Vehicle& gap_front_vehicle,
      const Vehicle& gap_rear_vehicle, const decimal_t& lat_track_offset,
      const decimal_t& dt, const Param& param, State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    decimal_t steer, velocity;  // values to be calculated

    // * Step I: Calculate steering
    bool steer_calculation_failed = false;
    common::FrenetState ego_on_tarlane_fs;
    if (stf_target.GetFrenetStateFromState(current_state, &ego_on_tarlane_fs) !=
            kSuccess ||
        ego_on_tarlane_fs.vec_s[1] < -kEPS) {
      // * ego Frenet state invalid or ego vehicle reverse gear
      steer_calculation_failed = true;
    }

    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf_target, current_state, ego_on_tarlane_fs,
                          wheelbase_len,
                          Vec2f(approx_lookahead_dist, lat_track_offset),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }
    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // * Step II: Calculate velocity
    // target longitudinal state on target lane
    common::State target_state;
    if (kSuccess != GetTargetStateOnTargetLane(
                        stf_target, ego_vehicle, gap_front_vehicle,
                        gap_rear_vehicle, sim_param, &target_state)) {
      target_state = current_state;
    }
    common::FrenetState target_on_curlane_fs;
    if (stf_current.GetFrenetStateFromState(
            target_state, &target_on_curlane_fs) != kSuccess) {
    }

    common::FrenetState ego_on_curlane_fs;
    if (stf_current.GetFrenetStateFromState(current_state,
                                            &ego_on_curlane_fs) != kSuccess) {
    }

    simulator::ContextIntelligentDriverModel::CtxParam ctx_param(0.4, 0.8);

    common::FrenetState current_leading_fs;
    if (current_leading_vehicle.id() == kInvalidAgentId ||
        stf_current.GetFrenetStateFromState(current_leading_vehicle.state(),
                                            &current_leading_fs) != kSuccess) {
      // ~ Without leading vehicle
      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf_current, ego_vehicle,
                                    current_leading_vehicle, current_leading_fs,
                                    &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingCtxIdm(
          ego_on_tarlane_fs.vec_s[0], current_state.velocity,
          current_leading_fs.vec_s[0], current_leading_vehicle.state().velocity,
          target_on_curlane_fs.vec_s[0], target_state.velocity, dt, sim_param,
          ctx_param, &velocity);
    }

    // * Step: Get desired state using vehicle kinematic model
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  static ErrorType PropagateOnce(const common::StateTransformer& stf,
                                 const common::Vehicle& ego_vehicle,
                                 const Vehicle& leading_vehicle,
                                 const decimal_t& dt, const Param& param,
                                 State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    auto sim_param = param;

    // * Step I: Calculate steering
    bool steer_calculation_failed = false;
    common::FrenetState current_fs;
    if (stf.GetFrenetStateFromState(current_state, &current_fs) != kSuccess ||
        current_fs.vec_s[1] < -kEPS) {
      // * ego Frenet state invalid or ego vehicle reverse gear
      steer_calculation_failed = true;
    }

    decimal_t steer, velocity;
    if (!steer_calculation_failed) {
      decimal_t approx_lookahead_dist =
          std::min(std::max(param.steer_control_min_lookahead_dist,
                            current_state.velocity * param.steer_control_gain),
                   param.steer_control_max_lookahead_dist);
      if (CalcualateSteer(stf, current_state, current_fs, wheelbase_len,
                          Vec2f(approx_lookahead_dist, 0.0),
                          &steer) != kSuccess) {
        steer_calculation_failed = true;
      }
    }

    steer = steer_calculation_failed ? current_state.steer : steer;
    decimal_t sim_vel = param.idm_param.kDesiredVelocity;
    if (param.auto_decelerate_if_lat_failed && steer_calculation_failed) {
      sim_vel = 0.0;
    }
    sim_param.idm_param.kDesiredVelocity = std::max(0.0, sim_vel);

    // * Step II: Calculate velocity
    common::FrenetState leading_fs;
    if (leading_vehicle.id() == kInvalidAgentId ||
        stf.GetFrenetStateFromState(leading_vehicle.state(), &leading_fs) !=
            kSuccess) {
      // ~ Without leading vehicle
      CalcualateVelocityUsingIdm(current_state.velocity, dt, sim_param,
                                 &velocity);
    } else {
      // ~ With leading vehicle
      // * For IDM, vehicle length is subtracted to get the 'net' distance
      // * between ego vehicle and the leading vehicle.
      decimal_t eqv_vehicle_len;
      GetIdmEquivalentVehicleLength(stf, ego_vehicle, leading_vehicle,
                                    leading_fs, &eqv_vehicle_len);
      sim_param.idm_param.kVehicleLength = eqv_vehicle_len;

      CalcualateVelocityUsingIdm(
          current_fs.vec_s[0], current_state.velocity, leading_fs.vec_s[0],
          leading_vehicle.state().velocity, dt, sim_param, &velocity);
    }

    // * Step III: Get desired state using vehicle kinematic model
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          sim_param, desired_state);
    return kSuccess;
  }

  // ~ Propagate using const velocity and steer
  static ErrorType PropagateOnce(const decimal_t& desired_vel,
                                 const common::Vehicle& ego_vehicle,
                                 const decimal_t& dt, const Param& param,
                                 State* desired_state) {
    common::State current_state = ego_vehicle.state();
    decimal_t wheelbase_len = ego_vehicle.param().wheel_base();
    decimal_t steer = current_state.steer;
    decimal_t velocity = current_state.velocity;
    CalculateDesiredState(current_state, steer, velocity, wheelbase_len, dt,
                          param, desired_state);
    return kSuccess;
  }

 private:
  static ErrorType GetIdmEquivalentVehicleLength(
      const common::StateTransformer& stf, const common::Vehicle& ego_vehicle,
      const common::Vehicle& leading_vehicle,
      const common::FrenetState& leading_fs, decimal_t* eqv_vehicle_len) {
    // Different from original IDM, we still use the center of rear axle as the
    // vehicle position, since we need to calculate the 'net' distance between
    // ego vehicle and leading vehicle, here we can get a equivalent leading
    // vehicle's length for IDM

    // In case the leading vehicle has an opposite angle due to MOT error
    std::array<Vec2f, 2> leading_pts;
    leading_vehicle.RetBumperVertices(&leading_pts);

    Vec2f fs_pt1, fs_pt2;
    std::vector<decimal_t> s_vec;
    if (kSuccess == stf.GetFrenetPointFromPoint(leading_pts[0], &fs_pt1)) {
      s_vec.push_back(fs_pt1(0));
    }
    if (kSuccess == stf.GetFrenetPointFromPoint(leading_pts[1], &fs_pt2)) {
      s_vec.push_back(fs_pt2(0));
    }
    s_vec.push_back(leading_fs.vec_s(0));
    decimal_t s_nearest_vtx = *(std::min_element(s_vec.begin(), s_vec.end()));

    decimal_t len_rb2r = fabs(leading_fs.vec_s(0) - s_nearest_vtx);
    // ego rear-axle to front bumper + leading rear bumper to rear-axle
    *eqv_vehicle_len = ego_vehicle.param().length() / 2.0 +
                       ego_vehicle.param().d_cr() + len_rb2r;

    return kSuccess;
  }

  static ErrorType CalcualateSteer(const common::StateTransformer& stf,
                                   const State& current_state,
                                   const FrenetState& current_fs,
                                   const decimal_t& wheelbase_len,
                                   const Vec2f& lookahead_offset,
                                   decimal_t* steer) {
    common::FrenetState dest_fs;
    dest_fs.Load(Vecf<3>(lookahead_offset(0) + current_fs.vec_s[0], 0.0, 0.0),
                 Vecf<3>(lookahead_offset(1), 0.0, 0.0),
                 common::FrenetState::kInitWithDs);

    State dest_state;
    if (stf.GetStateFromFrenetState(dest_fs, &dest_state) != kSuccess) {
      return kWrongStatus;
    }

    decimal_t look_ahead_dist =
        (dest_state.vec_position - current_state.vec_position).norm();
    decimal_t cur_to_dest_angle =
        vec2d_to_angle(dest_state.vec_position - current_state.vec_position);
    decimal_t angle_diff =
        normalize_angle(cur_to_dest_angle - current_state.angle);
    control::PurePursuitControl::CalculateDesiredSteer(
        wheelbase_len, angle_diff, look_ahead_dist, steer);
    return kSuccess;
  }

  // ~ Using leading vehicle
  static ErrorType CalcualateVelocityUsingIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& leading_pos, const decimal_t& leading_vel,
      const decimal_t& dt, const Param& param, decimal_t* velocity) {
    decimal_t leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      leading_vel_fin = 0;
    }
    // ~ note that we cannot use frenet state velocity for idm model, since the
    // ~ velocity in the frenet state may be larger than body velocity if the
    // ~ vehicle is in a highly curvy road (ref to the state transformer) which
    // ~ cannot be directly fed back to body velocity.
    return control::IntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, current_pos, leading_pos, current_vel, leading_vel_fin,
        dt, velocity);
  }

  // ~ Using virtual leading vehicle
  static ErrorType CalcualateVelocityUsingIdm(const decimal_t& current_vel,
                                              const decimal_t& dt,
                                              const Param& param,
                                              decimal_t* velocity) {
    const decimal_t virtual_leading_dist = 100.0 + 100.0 * current_vel;
    return control::IntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, 0.0, 0.0 + virtual_leading_dist, current_vel,
        current_vel, dt, velocity);
  }

  static ErrorType CalcualateVelocityUsingCtxIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& leading_pos, const decimal_t& leading_vel,
      const decimal_t& target_pos, const decimal_t& target_vel,
      const decimal_t& dt, const Param& param, const CtxParam& ctx_param,
      decimal_t* velocity) {
    decimal_t leading_vel_fin = leading_vel;
    if (leading_vel < 0) {
      leading_vel_fin = 0;
    }
    // ~ note that we cannot use frenet state velocity for idm model, since the
    // ~ velocity in the frenet state may be larger than body velocity if the
    // ~ vehicle is in a highly curvy road (ref to the state transformer) which
    // ~ cannot be directly fed back to body velocity.
    return control::ContextIntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, ctx_param, current_pos, leading_pos, target_pos,
        current_vel, leading_vel_fin, target_vel, dt, velocity);
  }

  static ErrorType CalcualateVelocityUsingCtxIdm(
      const decimal_t& current_pos, const decimal_t& current_vel,
      const decimal_t& target_pos, const decimal_t& target_vel,
      const decimal_t& dt, const Param& param, const CtxParam& ctx_param,
      decimal_t* velocity) {
    const decimal_t virtual_leading_pos =
        current_pos + 100.0 + 100.0 * current_vel;
    return control::ContextIntelligentVelocityControl::CalculateDesiredVelocity(
        param.idm_param, ctx_param, current_pos, virtual_leading_pos,
        target_pos, current_vel, current_vel, target_vel, dt, velocity);
  }

  static ErrorType CalculateDesiredState(const State& current_state,
                                         const decimal_t steer,
                                         const decimal_t velocity,
                                         const decimal_t wheelbase_len,
                                         const decimal_t dt, const Param& param,
                                         State* state) {
    simulator::IdealSteerModel model(
        wheelbase_len, param.idm_param.kAcceleration,
        param.idm_param.kHardBrakingDeceleration, param.max_lon_acc_jerk,
        param.max_lon_brake_jerk, param.max_lat_acceleration_abs,
        param.max_lat_jerk_abs, param.max_steer_angle_abs, param.max_steer_rate,
        param.max_curvature_abs);
    model.set_state(current_state);
    model.set_control(simulator::IdealSteerModel::Control(steer, velocity));
    model.Step(dt);
    *state = model.state();
    state->time_stamp = current_state.time_stamp + dt;
    // if (std::isnan(state->velocity)) {
    //   printf(
    //       "[DEBUG]state velocity %lf steer: %lf, velocity %lf output velocity
    //       "
    //       "%lf.\n",
    //       current_state.velocity, steer, velocity, state->velocity);
    // }
    return kSuccess;
  }
};

}  // namespace planning

#endif  // _CORE_FORWARD_SIMULATOR_INC_ONLANE_FORWARD_SIMULATOR_H_