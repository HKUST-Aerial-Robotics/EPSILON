#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_BEZIER_TRAJ_H__

#include "common/basics/config.h"
#include "common/spline/bezier.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"

namespace common {

class FrenetBezierTrajectory : public FrenetTrajectory {
 public:
  using BezierTrajectory = BezierSpline<TrajectoryDegree, TrajectoryDim>;

  FrenetBezierTrajectory() {}
  FrenetBezierTrajectory(const BezierTrajectory& bezier_spline,
                         const StateTransformer& stf)
      : bezier_spline_(bezier_spline), stf_(stf), is_valid_(true) {}

  decimal_t begin() const override { return bezier_spline_.begin(); }
  decimal_t end() const override { return bezier_spline_.end(); }
  bool IsValid() const override { return is_valid_; }
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    common::FrenetState fs;
    if (GetFrenetState(t, &fs) != kSuccess) {
      return kWrongStatus;
    }
    if (stf_.GetStateFromFrenetState(fs, state) != kSuccess) {
      return kWrongStatus;
    }
    state->velocity = std::max(0.0, state->velocity);
    return kSuccess;
  }

  ErrorType GetFrenetState(const decimal_t& t, FrenetState* fs) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    Vecf<2> pos, vel, acc;
    bezier_spline_.evaluate(t, 0, &pos);
    bezier_spline_.evaluate(t, 1, &vel);
    bezier_spline_.evaluate(t, 2, &acc);
    // ~ frenet bezier by default works lateral independent mode
    fs->time_stamp = t;
    fs->Load(Vec3f(pos[0], vel[0], acc[0]), Vec3f(pos[1], vel[1], acc[1]),
             common::FrenetState::kInitWithDt);
    if (!fs->is_ds_usable) {
      fs->Load(Vec3f(pos[0], 0.0, 0.0), Vec3f(pos[1], 0.0, 0.0),
               FrenetState::kInitWithDs);
    }
    return kSuccess;
  }

  std::vector<decimal_t> variables() const override {
    // TODO
    return std::vector<decimal_t>();
  }

  void set_variables(const std::vector<decimal_t>& variables) override {
    // TODO
  }

  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    // TODO
  }

 private:
  BezierTrajectory bezier_spline_;
  StateTransformer stf_;
  bool is_valid_ = false;
};

}  // namespace common

#endif