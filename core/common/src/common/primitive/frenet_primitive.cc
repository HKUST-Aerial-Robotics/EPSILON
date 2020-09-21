#include "common/primitive/frenet_primitive.h"

namespace common {

ErrorType FrenetPrimitive::Connect(const FrenetState& fs0,
                                   const FrenetState& fs1,
                                   const decimal_t stamp, const decimal_t T,
                                   bool is_lateral_independent) {
  is_lateral_independent_ = is_lateral_independent;

  poly_s_.GetJerkOptimalConnection(fs0.vec_s(0), fs0.vec_s(1), fs0.vec_s(2),
                                   fs1.vec_s(0), fs1.vec_s(1), fs1.vec_s(2), T);

  if (is_lateral_independent) {
    poly_d_.GetJerkOptimalConnection(fs0.vec_dt(0), fs0.vec_dt(1),
                                     fs0.vec_dt(2), fs1.vec_dt(0),
                                     fs1.vec_dt(1), fs1.vec_dt(2), T);
  } else {
    if (fs1.vec_s[0] - fs0.vec_s[0] < kSmallDistanceThreshold_) {
      const decimal_t virtual_large_distance = 100.0;
      poly_d_.GetJerkOptimalConnection(
          fs0.vec_ds(0), fs0.vec_ds(1), fs0.vec_ds(2), fs1.vec_ds(0),
          fs1.vec_ds(1), fs1.vec_ds(2), virtual_large_distance);
    } else {
      poly_d_.GetJerkOptimalConnection(
          fs0.vec_ds(0), fs0.vec_ds(1), fs0.vec_ds(2), fs1.vec_ds(0),
          fs1.vec_ds(1), fs1.vec_ds(2), fs1.vec_s[0] - fs0.vec_s[0]);
    }
  }

  stamp_ = stamp;
  fs0_ = fs0;
  duration_ = T;
  fs1_ = fs1;
  // if (!is_lateral_independent_) {
  //   printf("stamp %lf, T %lf, s0 %lf s1 %lf.\n", stamp, T, fs0.vec_s[0],
  //          fs1.vec_s[0]);
  //   printf("state0 - d : %lf, %lf, %lf.\n", fs0.vec_ds(0), fs0.vec_ds(1),
  //          fs0.vec_ds(2));
  //   printf("state1 - d : %lf, %lf, %lf.\n", fs1.vec_ds(0), fs1.vec_ds(1),
  //          fs1.vec_ds(2));
  //   printf("lon coeff: .\n");
  //   poly_s_.print();
  //   printf("lateral coeff: .\n");
  //   poly_d_.print();
  // }
  return kSuccess;
}

ErrorType FrenetPrimitive::Propagate(const FrenetState& fs0, const Vecf<2>& u,
                                     const decimal_t stamp, const decimal_t T) {
  is_lateral_independent_ = true;
  Vecf<6> coeff;
  coeff << 0.0, 0.0, 0.0, u[0], fs0.vec_s[1], fs0.vec_s[0];
  poly_s_.set_coeff(coeff);
  coeff << 0.0, 0.0, 0.0, u[1], fs0.vec_dt[1], fs0.vec_dt[0];
  poly_d_.set_coeff(coeff);
  duration_ = T;
  stamp_ = stamp;
  fs0_ = fs0;
  GetFrenetState(stamp + T, &fs1_);
  return kSuccess;
}

ErrorType FrenetPrimitive::GetFrenetState(const decimal_t t_global,
                                          FrenetState* fs) const {
  if (duration_ < kEPS) return kWrongStatus;
  auto t = t_global - stamp_;
  if (is_lateral_independent_) {
    fs->Load(Vecf<3>(poly_s_.evaluate(t, 0), poly_s_.evaluate(t, 1),
                     poly_s_.evaluate(t, 2)),
             Vecf<3>(poly_d_.evaluate(t, 0), poly_d_.evaluate(t, 1),
                     poly_d_.evaluate(t, 2)),
             FrenetState::kInitWithDt);
    fs->time_stamp = t_global;
  } else {
    auto s = poly_s_.evaluate(t, 0);
    auto st = s - fs0_.vec_s[0];
    fs->Load(Vecf<3>(s, poly_s_.evaluate(t, 1), poly_s_.evaluate(t, 2)),
             Vecf<3>(poly_d_.evaluate(st, 0), poly_d_.evaluate(st, 1),
                     poly_d_.evaluate(st, 2)),
             FrenetState::kInitWithDs);
    fs->time_stamp = t_global;
  }
  return kSuccess;
}

ErrorType FrenetPrimitive::GetFrenetStateSamples(
    const decimal_t step, const decimal_t offset,
    vec_E<FrenetState>* fs_vec) const {
  FrenetState fs;
  fs_vec->clear();
  int num_samples_esti =
      static_cast<int>((end() - begin() - offset) / step) + 10;
  fs_vec->reserve(num_samples_esti);
  for (decimal_t t = begin() + offset; t < end(); t += step) {
    if (GetFrenetState(t, &fs) == kSuccess) {
      fs_vec->push_back(fs);
    }
  }
  return kSuccess;
}

ErrorType FrenetPrimitive::GetJ(decimal_t* c_s, decimal_t* c_d) const {
  *c_s = poly_s_.J(duration_, 3);
  if (is_lateral_independent_) {
    *c_d = poly_d_.J(duration_, 3);
  } else {
    *c_d = poly_d_.J(fs1_.vec_s[0] - fs0_.vec_s[0], 3);
  }
  return kSuccess;
}

decimal_t FrenetPrimitive::lateral_T() const {
  if (is_lateral_independent_) {
    return duration_;
  } else {
    return fs1_.vec_s[0] - fs0_.vec_s[0];
  }
}

decimal_t FrenetPrimitive::longitudial_T() const { return duration_; }

FrenetState FrenetPrimitive::fs1() const { return fs1_; }

FrenetState FrenetPrimitive::fs0() const { return fs0_; }

}  // namespace common