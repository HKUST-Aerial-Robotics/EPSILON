#include "common/math/calculations.h"

long long fac(int n) {
  if (n == 0) return 1;
  if (n == 1) return 1;
  if (n == 2) return 2;
  if (n == 3) return 6;
  if (n == 4) return 24;
  if (n == 5) return 120;

  long long ans = 1;
  for (int i = 1; i <= n; i++) ans *= i;
  return ans;
}

long long nchoosek(int n, int k) { return fac(n) / fac(k) / fac(n - k); }

decimal_t normalize_angle(const decimal_t& theta) {
  decimal_t theta_tmp = theta;
  theta_tmp -= (theta >= kPi) * 2 * kPi;
  theta_tmp += (theta < -kPi) * 2 * kPi;
  return theta_tmp;
}

Vecf<2> rotate_vector_2d(const Vecf<2>& v, const decimal_t angle) {
  return Vecf<2>(v[0] * cos(angle) - v[1] * sin(angle),
                 v[0] * sin(angle) + v[1] * cos(angle));
}

decimal_t vec2d_to_angle(const Vecf<2>& v) { return atan2(v[1], v[0]); }

decimal_t truncate(const decimal_t& val_in, const decimal_t& lower,
                   const decimal_t& upper) {
  if (lower > upper) {
    printf("[Calculations]Invalid input!\n");
    assert(false);
  }
  decimal_t res = val_in;
  res = std::max(res, lower);
  res = std::min(res, upper);
  return res;
}

decimal_t normalize_with_bound(const decimal_t& val_in, const decimal_t& lower,
                               const decimal_t& upper,
                               const decimal_t& new_lower,
                               const decimal_t& new_upper) {
  if (new_lower > new_upper) {
    printf("[Calculations]Invalid input!\n");
    assert(false);
  }
  decimal_t val_bounded = truncate(val_in, lower, upper);
  decimal_t ratio = (val_bounded - lower) / (upper - lower);
  decimal_t res = new_lower + (new_upper - new_lower) * ratio;
  return res;
}

ErrorType RemapUsingQuadraticFuncAroundSmallValue(const decimal_t& th,
                                                  const decimal_t& val_in,
                                                  decimal_t* val_out) {
  decimal_t c = 1.0 / th;
  if (fabs(val_in) <= fabs(th)) {
    // quadratic, y = c * x ^ 2
    *val_out = sgn(val_in) * c * val_in * val_in;
  } else {
    // linear, y = x
    *val_out = val_in;
  }
  return kSuccess;
}