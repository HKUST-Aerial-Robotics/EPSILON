#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_PRIMITIVE_TRAJ_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_PRIMITIVE_TRAJ_H__

#include "common/basics/config.h"
#include "common/primitive/frenet_primitive.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/state/state_transformer.h"
#include "common/trajectory/frenet_traj.h"
namespace common {

class FrenetPrimitiveTrajectory : public FrenetTrajectory {
 public:
  FrenetPrimitiveTrajectory() {}
  FrenetPrimitiveTrajectory(const FrenetPrimitive& primitive,
                            const StateTransformer& stf)
      : primitive_(primitive), stf_(stf), is_valid_(true) {}

  decimal_t begin() const override { return primitive_.begin(); }
  decimal_t end() const override { return primitive_.end(); }
  bool IsValid() const override { return is_valid_; }
  ErrorType GetState(const decimal_t& t, State* state) const override {
    if (t < begin() - kEPS || t > end() + kEPS) return kWrongStatus;
    FrenetState fs;
    if (primitive_.GetFrenetState(t, &fs) != kSuccess) {
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
    if (primitive_.GetFrenetState(t, fs) != kSuccess) {
      return kWrongStatus;
    }
    return kSuccess;
  }

  std::vector<decimal_t> variables() const override {
    std::vector<decimal_t> variables(12);
    Vecf<6> coeff_s = primitive_.poly_s().coeff();
    Vecf<6> coeff_d = primitive_.poly_d().coeff();
    for (int i = 0; i < 6; i++) {
      variables[i] = coeff_s[i];
      variables[i + 6] = coeff_d[i];
    }
    return variables;
  }

  void set_variables(const std::vector<decimal_t>& variables) override {
    assert(variables.size() == 12);
    Vecf<6> coeff_s, coeff_d;
    for (int i = 0; i < 6; i++) {
      coeff_s[i] = variables[i];
      coeff_d[i] = variables[6 + i];
    }
    Polynomial<5> poly_s, poly_d;
    poly_s.set_coeff(coeff_s);
    poly_d.set_coeff(coeff_d);
    primitive_.set_poly_s(poly_s);
    primitive_.set_poly_d(poly_d);
  }

  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const override {
    primitive_.GetJ(j_lon, j_lat);
  }

 private:
  FrenetPrimitive primitive_;
  StateTransformer stf_;
  bool is_valid_ = false;
};

}  // namespace common

#endif