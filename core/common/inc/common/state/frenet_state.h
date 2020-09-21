#ifndef _COMMON_INC_COMMON_STATE_FRENET_STATE_H__
#define _COMMON_INC_COMMON_STATE_FRENET_STATE_H__

namespace common {
/**
 * @brief Definition of frenet state
 * @param vec_s, s coordinate
 * @param vec_d, d coordinate (derivative may vary)
 * @param is_lateral_independent, if true, vec_d is w.r.t t, if false, vec_d is
 * w.r.t s, default to be true (high speed mode)
 */
struct FrenetState {
  enum InitType { kInitWithDt, kInitWithDs };
  decimal_t time_stamp{0.0};
  Vecf<3> vec_s{Vecf<3>::Zero()};
  Vecf<3> vec_dt{Vecf<3>::Zero()};
  Vecf<3> vec_ds{Vecf<3>::Zero()};
  bool is_ds_usable = true;

  FrenetState() {}
  void Load(const Vecf<3>& s, const Vecf<3>& d, const InitType& type) {
    vec_s = s;
    if (type == kInitWithDt) {
      vec_dt = d;
      vec_ds[0] = vec_dt[0];
      if (fabs(vec_s[1]) > kEPS) {
        vec_ds[1] = vec_dt[1] / vec_s[1];
        vec_ds[2] = (vec_dt[2] - vec_ds[1] * vec_s[2]) / (vec_s[1] * vec_s[1]);
        is_ds_usable = true;
      } else {
        vec_ds[1] = 0.0;
        vec_ds[2] = 0.0;
        is_ds_usable = false;
      }
    } else if (type == kInitWithDs) {
      vec_ds = d;
      vec_dt[0] = vec_ds[0];
      vec_dt[1] = vec_s[1] * vec_ds[1];
      vec_dt[2] = vec_ds[2] * vec_s[1] * vec_s[1] + vec_ds[1] * vec_s[2];
      is_ds_usable = true;
    } else {
      assert(false);
    }
  }

  FrenetState(const Vecf<3>& s, const Vecf<3>& dt, const Vecf<3>& ds) {
    vec_s = s;
    vec_dt = dt;
    vec_ds = ds;
  }

  void print() const {
    printf("frenet state stamp: %lf.\n", time_stamp);
    printf("-- vec_s: (%lf, %lf, %lf).\n", vec_s[0], vec_s[1], vec_s[2]);
    printf("-- vec_dt: (%lf, %lf, %lf).\n", vec_dt[0], vec_dt[1], vec_dt[2]);
    printf("-- vec_ds: (%lf, %lf, %lf).\n", vec_ds[0], vec_ds[1], vec_ds[2]);
    if (!is_ds_usable) printf("-- warning: ds not usable.\n");
  }
};

}  // namespace common

#endif