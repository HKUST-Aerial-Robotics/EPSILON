#ifndef _CORE_FORWARD_SIMULATOR_INC_SOCIAL_FORCE_MODEL_H_
#define _CORE_FORWARD_SIMULATOR_INC_SOCIAL_FORCE_MODEL_H_

// Reference:
// Helbing D, Molnar P. Social force model for pedestrian dynamics[J]. Physical
// review E, 1995, 51(5): 4282.

#include <algorithm>

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"

namespace planning {

class SocialForceModel {
 public:
  using State = common::State;
  using Vehicle = common::Vehicle;

  struct Param {
    decimal_t dt = 2.0;
    decimal_t c = 10;
    decimal_t lambda = 3.0;
  };

  struct Ellipse {
    Vec2f f1, f2;
    decimal_t a, b, c;
  };

  struct SocialForceInfo {
    std::unordered_map<int, Vec2f> force_set;
    Vec2f resultant_force;
    Vec2f decomp_force;
  };

  static ErrorType GetSocialForceInfo(const common::Vehicle& ego_vehicle,
                                      const common::VehicleSet& other_vehicles,
                                      SocialForceInfo* force_info) {
    decimal_t dt = 1.0;
    decimal_t c = 10.0;
    decimal_t lambda = 2.0;
    // * for each other vehicle, calculate force
    std::unordered_map<int, Vec2f> force_set;
    for (const auto& pv : other_vehicles.vehicles) {
      if (pv.second.id() == kInvalidAgentId) continue;

      Vec2f f1;
      GetSocialForceForTargetVehicle(ego_vehicle, pv.second, dt, c, lambda,
                                     &f1);
      Vec2f f2;
      GetSocialForceForTargetVehicle(pv.second, ego_vehicle, dt, c, lambda,
                                     &f2);
      Vec2f f = f1 - f2;

      if (f.norm() < 0.1) continue;

      force_set.insert(std::make_pair(pv.first, f));
    }

    // * calculate the sum of forces
    Vec2f resultant_force = Vec2f(0.0, 0.0);
    for (const auto& entry : force_set) {
      resultant_force = resultant_force + entry.second;
    }

    // * decompose
    decimal_t cos_theta = cos(ego_vehicle.state().angle);
    decimal_t sin_theta = sin(ego_vehicle.state().angle);
    Mat2f Rot;
    Rot << cos_theta, -sin_theta, sin_theta, cos_theta;
    Vec2f ego_force = Rot.transpose() * resultant_force;

    // printf("[XXX]res_force = [%lf, %lf], decomp_force = [%lf, %lf]\n",
    //        resultant_force(0), resultant_force(1), ego_force(0),
    //        ego_force(1));

    force_info->force_set = force_set;
    force_info->resultant_force = resultant_force;
    force_info->decomp_force = ego_force;

    return kSuccess;
  }

 private:
  static ErrorType GetSocialForceForTargetVehicle(
      const common::Vehicle& ego_vehicle, const common::Vehicle& tar_vehicle,
      const decimal_t& dt, const decimal_t& c, const decimal_t& lambda,
      Vec2f* force_out) {
    auto ego_position = ego_vehicle.state().vec_position;

    Vec2f normal_vec;
    Ellipse ellipse;
    GetNormalVectorOnEllipsePotential(ego_position, tar_vehicle.state(), dt,
                                      &normal_vec, &ellipse);

    decimal_t x = ellipse.a * 2 / (tar_vehicle.state().velocity + kBigEPS);
    decimal_t val = NegExpPotential(x, c, lambda);

    *force_out = normal_vec * val;

    return kSuccess;
  }

  static ErrorType GetNormalVectorOnEllipsePotential(
      const Vec2f& pos, const common::State& tar_state, const decimal_t& dt,
      Vec2f* normal_vec, Ellipse* ellipse) {
    decimal_t cos_theta = cos(tar_state.angle);
    decimal_t sin_theta = sin(tar_state.angle);

    decimal_t c = 0.5 * (dt * tar_state.velocity) + kBigEPS;  // focal distance

    Vec2f f1 = tar_state.vec_position;
    Vec2f f2 = f1 + Vec2f(cos_theta, sin_theta) * 2 * c;

    Vec2f r_vec = pos - f1;

    decimal_t dist_ef2 = (r_vec - (f2 - f1)).norm();

    decimal_t b = 0.5 * sqrt(pow((r_vec.norm() + dist_ef2), 2) - pow(2 * c, 2));
    decimal_t a = 0.5 * (r_vec.norm() + dist_ef2);

    Vec2f origin_w = (f1 + f2) / 2.0;

    Mat2f Rot;
    Rot << cos_theta, -sin_theta, sin_theta, cos_theta;

    // normal vector
    Vec2f pos_local = Rot.transpose() * (pos - origin_w);
    Vec2f grad_local = Vec2f(b * pos_local(0) / a, a * pos_local(1) / b);
    Vec2f n_local = grad_local / grad_local.norm();

    *normal_vec = Rot * n_local;

    ellipse->f1 = f1;
    ellipse->f2 = f2;
    ellipse->a = a;
    ellipse->b = b;
    ellipse->c = c;

    return kSuccess;
  }

  static decimal_t NegExpPotential(const decimal_t& x, const decimal_t& a,
                                   const decimal_t& b) {
    return a * exp(-x / b);
  }
};

}  // namespace planning

#endif  // _CORE_FORWARD_SIMULATOR_INC_SOCIAL_FORCE_MODEL_H_