#include "common/spline/lookup_table.h"

namespace common {

MatNf<6> GetAInverse(decimal_t t) {
  MatNf<6> A;
  //   A << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
  //   0.0,
  //       0.0, 1.0, 0.0, 0.0, pow(t, 5) / 120.0, pow(t, 4) / 24.0, pow(t, 3)
  //       / 6.0, t * t / 2.0, t, 1, pow(t, 4) / 24.0, pow(t, 3) / 6.0, t * t
  //       / 2.0, t, 1.0, 0.0, pow(t, 3) / 6.0, t * t / 2.0, t, 1.0, 0.0, 0.0;
  A << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
      0.0, 2.0, 0.0, 0.0, pow(t, 5), pow(t, 4), pow(t, 3), t * t, t, 1,
      5.0 * pow(t, 4), 4.0 * pow(t, 3), 3.0 * t * t, 2.0 * t, 1.0, 0.0,
      20.0 * pow(t, 3), 12.0 * t * t, 6.0 * t, 2.0, 0.0, 0.0;
  return A.inverse();
}

std::map<decimal_t, MatNf<6>, std::less<decimal_t>,
         Eigen::aligned_allocator<std::pair<const decimal_t, MatNf<6>>>>
    kTableAInverse{
        {3.0, GetAInverse(3.0)}, {3.5, GetAInverse(3.5)},
        {4.0, GetAInverse(4.0)}, {4.5, GetAInverse(4.5)},
        {5.0, GetAInverse(5.0)},
    };

}  // namespace common
