#ifndef _CORE_COMMON_INC_COMMON_SPLINE_POLYNOMIAL_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_POLYNOMIAL_H__

#include "common/basics/basics.h"
#include "common/math/calculations.h"
#include "common/spline/lookup_table.h"

#include <assert.h>

namespace common {

/**
 * @brief Polynomial class, coeffs are stored in reversed order
 * The parameterization is given by
 * f(s) = coeff_(n) + coeff_(n-1)/1!^s + ... + coeff_(0)/(n!)*s^n
 * f(s) = coeff_normal_order_(0) + coeff_normal_order_(1)^s + ... +
 * coeff_normal_order_(n)*s^n
 * coeffs are scaled to have straightforward physical
 * meaning, also improve the efficiency when evaluating the derivatives.
 */
template <int N_DEG>
class Polynomial {
 public:
  typedef Vecf<N_DEG + 1> VecNf;
  enum { NeedsToAlign = (sizeof(VecNf) % 16) == 0 };

  Polynomial() { set_zero(); }
  Polynomial(const VecNf& coeff) : coeff_(coeff) { update(); }

  /**
   * @brief Return coefficients of the polynomial
   */
  VecNf coeff() const { return coeff_; }

  /**
   * @brief Set coefficients of the polynomial
   */
  void set_coeff(const VecNf& coeff) {
    coeff_ = coeff;
    update();
  }

  void update() {
    for (int i = 0; i < N_DEG + 1; i++) {
      coeff_normal_order_[i] = coeff_[N_DEG - i] / fac(i);
    }
  }

  /**
   * @brief Set coefficients of the polynomial to zero
   */
  void set_zero() {
    coeff_.setZero();
    coeff_normal_order_.setZero();
  }

  /**
   * @brief Return position of polynomial with respect to parameterization
   * @param s value of polynomial parameterization
   */
  inline decimal_t evaluate(const decimal_t& s, const int& d) const {
    // Use horner's rule for quick evaluation
    decimal_t p = coeff_(0) / fac(N_DEG - d);
    for (int i = 1; i <= N_DEG - d; i++) {
      p = (p * s + coeff_(i) / fac(N_DEG - i - d));
    }
    return p;
    // After testing, above is generally faster than below (obvious when d = 1,
    // less obvious as d increases)
    // decimal_t p = coeff_normal_order_[N_DEG] * fac(N_DEG) / fac(N_DEG - d);
    // for (int i = 1; i <= N_DEG - d; i++) {
    //   p = (p * s + coeff_normal_order_[N_DEG - i] * fac(N_DEG - i) /
    //                    fac(N_DEG - i - d));
    // }
    // return p;
  }

  inline decimal_t evaluate(const decimal_t& s) const {
    // Use horner's rule for quick evaluation
    // note that this function is much faster than evaluate(s, 0);
    decimal_t p = coeff_normal_order_[N_DEG];
    for (int i = 1; i <= N_DEG; i++) {
      p = (p * s + coeff_normal_order_[N_DEG - i]);
    }
    return p;
  }

  inline decimal_t J(decimal_t s, int d) const {
    if (d == 3) {
      // integration of squared jerk
      return coeff_(0) * coeff_(0) / 20.0 * pow(s, 5) +
             coeff_(0) * coeff_(1) / 4 * pow(s, 4) +
             (coeff_(1) * coeff_(1) + coeff_(0) * coeff_(2)) / 3 * pow(s, 3) +
             coeff_(1) * coeff_(2) * s * s + coeff_(2) * coeff_(2) * s;
    } else if (d == 2) {
      // integration of squared accleration
      return coeff_(0) * coeff_(0) / 252 * pow(s, 7) +
             coeff_(0) * coeff_(1) / 36 * pow(s, 6) +
             (coeff_(1) * coeff_(1) / 20 + coeff_(0) * coeff_(2) / 15) *
                 pow(s, 5) +
             (coeff_(0) * coeff_(3) / 12 + coeff_(1) * coeff_(2) / 4) *
                 pow(s, 4) +
             (coeff_(2) * coeff_(2) / 3 + coeff_(1) * coeff_(3) / 3) *
                 pow(s, 3) +
             coeff_(2) * coeff_(3) * s * s + coeff_(3) * coeff_(3) * s;
    } else {
      assert(false);
    }
    return 0.0;
  }

  /**
   * @brief Generate jerk-optimal primitive with boundary condition
   * @note The polynomial should have degree >= quintic (5)
   * @param p1, x0 position
   * @param dp1, x0 velocity
   * @param ddp1, x0 acceleration
   * @param p2, x1 position
   * @param dp2, x1 velocity
   * @param ddp2, x1 acceleration
   * @param S, duration
   */
  void GetJerkOptimalConnection(const decimal_t p1, const decimal_t dp1,
                                const decimal_t ddp1, const decimal_t p2,
                                const decimal_t dp2, const decimal_t ddp2,
                                const decimal_t S) {
    assert(N_DEG >= 5);
    Vecf<6> b;
    b << p1, dp1, ddp1, p2, dp2, ddp2;

    coeff_.setZero();

    // NOTE: fix the singularity of S=0 caused
    // by stopping traj
    if (S < kEPS) {
      Vecf<6> c;
      c << 0.0, 0.0, 0.0, ddp1, dp1, p1;
      coeff_.template segment<6>(N_DEG - 5) = c;
      return;
    }

    MatNf<6> A_inverse;
    if (!LookUpCache(S, &A_inverse)) {
      A_inverse = GetAInverse(S);
    }

    auto coeff = A_inverse * b;
    coeff_[N_DEG - 5] = coeff(0) * fac(5);
    coeff_[N_DEG - 4] = coeff(1) * fac(4);
    coeff_[N_DEG - 3] = coeff(2) * fac(3);
    coeff_[N_DEG - 2] = coeff(3) * fac(2);
    coeff_[N_DEG - 1] = coeff(4) * fac(1);
    coeff_[N_DEG - 0] = coeff(5);
    update();
    // coeff_.template segment<6>(N_DEG - 5) = A_inverse * b;;
  }

  /**
   * @Get look up cached A inverse
   */
  bool LookUpCache(const decimal_t S, MatNf<6>* A_inverse) {
    auto it = kTableAInverse.find(S);
    if (it != kTableAInverse.end()) {
      *A_inverse = it->second;
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief Debug function
   */
  void print() const {
    std::cout << std::fixed << std::setprecision(7) << coeff_.transpose()
              << std::endl;
  }

 private:
  VecNf coeff_;
  VecNf coeff_normal_order_;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

template <int N_DEG, int N_DIM>
class PolynomialND {
 public:
  PolynomialND() {}
  PolynomialND(const std::array<Polynomial<N_DEG>, N_DIM>& polys)
      : polys_(polys) {}

  /**
   * @brief Return a reference to a certain 1D polynomial
   * @param j index of the dimension
   */
  Polynomial<N_DEG>& operator[](int j) {
    assert(j < N_DIM);
    return polys_[j];
  }

  /**
   * @brief Return evaluated vector
   * @param s evaluation point
   * @param d derivative to take
   */
  inline void evaluate(const decimal_t s, int d, Vecf<N_DIM>* vec) const {
    for (int i = 0; i < N_DIM; i++) {
      (*vec)[i] = polys_[i].evaluate(s, d);
    }
  }

  inline void evaluate(const decimal_t s, Vecf<N_DIM>* vec) const {
    for (int i = 0; i < N_DIM; i++) {
      (*vec)[i] = polys_[i].evaluate(s);
    }
  }

  void print() const {
    for (int i = 0; i < N_DIM; i++) polys_[i].print();
  }

 private:
  std::array<Polynomial<N_DEG>, N_DIM> polys_;
};

}  // namespace common

#endif