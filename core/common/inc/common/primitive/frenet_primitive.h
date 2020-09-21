#ifndef _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__
#define _CORE_COMMON_INC_COMMON_FRENET_PRIMITIVE_H__

#include "common/spline/polynomial.h"
#include "common/state/frenet_state.h"

namespace common {

class FrenetPrimitive {
 public:
  FrenetPrimitive() {}
  /**
   * @brief Construct primitive in the frenet frame with two different modes
   * @param fs0, start frenet state
   * @param fs1, end frenet state
   * @param is_lateral_independent, whether high speed mode enabled
   */
  ErrorType Connect(const FrenetState& fs0, const FrenetState& fs1,
                    const decimal_t stamp, const decimal_t T,
                    bool is_lateral_independent);
  /**
   * @brief Construct primitive in the frenet frame with two different modes
   * @param fs0, start frenet state
   * @param u, longitudal and lateral control
   * @param stamp, start time stamp of the primitive
   * @param T, primitive duration
   */
  ErrorType Propagate(const FrenetState& fs0, const Vecf<2>& u,
                      const decimal_t stamp, const decimal_t T);

  /**
   * @brief Get the start point of the parameterization
   */
  decimal_t begin() const { return stamp_; }

  /**
   * @brief Get the end point of the parameterization
   */
  decimal_t end() const { return stamp_ + duration_; }

  /**
   * @brief Get the frenet state from the primitive
   * @note  when t is outside of the parameterization, extrapolation will be
   * applied
   */
  ErrorType GetFrenetState(const decimal_t t_global, FrenetState* fs) const;

  ErrorType GetFrenetStateSamples(const decimal_t step, const decimal_t offset,
                                  vec_E<FrenetState>* fs_vec) const;

  ErrorType GetJ(decimal_t* c_s, decimal_t* c_d) const;

  decimal_t lateral_T() const;
  decimal_t longitudial_T() const;

  FrenetState fs1() const;
  FrenetState fs0() const;

  Polynomial<5> poly_s() const { return poly_s_; }
  Polynomial<5> poly_d() const { return poly_d_; }

  void set_poly_s(const Polynomial<5>& poly) { poly_s_ = poly; }
  void set_poly_d(const Polynomial<5>& poly) { poly_d_ = poly; }
  /**
   * @brief Debug print
   */
  void print() const {
    printf("frenet primitive in duration [%lf, %lf].\n", begin(), end());
    poly_s_.print();
    poly_d_.print();
    fs0_.print();
    fs1_.print();
  }

  bool is_lateral_independent_ = false;

 private:
  Polynomial<5> poly_s_;
  Polynomial<5> poly_d_;
  decimal_t stamp_{0.0};
  decimal_t duration_{0.0};
  FrenetState fs0_;
  FrenetState fs1_;
  decimal_t kSmallDistanceThreshold_ = 2.0;
};

}  // namespace common

#endif
