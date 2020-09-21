#ifndef _CORE_COMMON_INC_COMMON_SOLVER_OOQP_INTERFACE_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_OOQP_INTERFACE_H__

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

#include <algorithm>  // std::max
#include <cmath>      // std::abs
#include <limits>     // std::numeric_limits

namespace common {

class OoQpItf {
 public:
  /*!
   * Solve min 1/2 x' Q x + c' x, such that A x = b, d <= Cx <= f, and l <= x <=
   * u.
   * @param [in] Q a symmetric positive semidefinite matrix (nxn)
   * @param [in] c a vector (nx1)
   * @param [in] A a (possibly null) matrices (m_axn)
   * @param [in] b a vector (m_ax1)
   * @param [in] C a (possibly null) matrices (m_cxn)
   * @param [in] d a vector (m_cx1)
   * @param [in] f a vector (m_cx1)
   * @param [in] l a vector (nx1)
   * @param [in] u a vector (nx1)
   * @param [out] x a vector of variables (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x, const bool ignoreUnknownError = false,
                    const bool verbose = false);
 private:
  /*!
   * Determine which limits are active and which are not.
   * @param [in]  l
   * @param [in]  u
   * @param [out] useLowerLimit
   * @param [out] useUpperLimit
   * @param [out] lowerLimit
   * @param [out] upperLimit
   */
  static void generateLimits(
      const Eigen::VectorXd& l, const Eigen::VectorXd& u,
      Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
      Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
      Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit);

  static void printProblemFormulation(
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
      const Eigen::VectorXd& c,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
      const Eigen::VectorXd& b,
      const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
      const Eigen::VectorXd& d, const Eigen::VectorXd& f,
      const Eigen::VectorXd& l, const Eigen::VectorXd& u);

  static void printLimits(
      const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
      const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
      const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit);

  static void printSolution(const int status, const Eigen::VectorXd& x);
};

class NumericalUtil {
 public:
  /*!
   * Takes max(|a|, |b|) and multiplies it with epsilon.
   * @param a the first number.
   * @param b the second number.
   * @param epsilon the precision (epsilon > 0).
   * @return the result of max(|a|, |b|) * epsilon.
   */
  template <typename ValueType_>
  static inline ValueType_ maxTimesEpsilon(const ValueType_ a,
                                           const ValueType_ b,
                                           const ValueType_ epsilon) {
    return std::max(std::abs(a), std::abs(b)) * epsilon;
  }
  /*!
   * Checks if two numbers a and b are equal within a relative error.
   * @param[in] a the first number to compare.
   * @param[in] b the second number to compare.
   * @param[in] epsilon the relative error (optional, if not declared the
   * precision of the datatype).
   * @return true if a and b are approximately equal, false otherwise.
   */
  template <typename ValueType_>
  static bool ApproximatelyEqual(
      const ValueType_ a, const ValueType_ b,
      ValueType_ epsilon = std::numeric_limits<ValueType_>::epsilon()) {
    return std::abs(a - b) <= maxTimesEpsilon(a, b, epsilon);
  }
};

}  // namespace common

#endif