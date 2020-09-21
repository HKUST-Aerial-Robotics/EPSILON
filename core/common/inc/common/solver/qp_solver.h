#ifndef _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__
#define _CORE_COMMON_INC_COMMON_SOLVER_QP_SOLVER_H__

#include "common/basics/basics.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/SparseCore>

namespace common {

/**
 * @brief Using quadratic programming to find the optimal polynomial coeffs for
 * the spline
 * @note Heavily brought from ooqp_eigen_interface's implementation, thanks!
 * @note https://github.com/ethz-asl/ooqp_eigen_interface
 */

class QuadraticProblem {
 public:
  /*!
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c and
   * d <= Dx <= f
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] C a (possibly null) matrix (m_cxn)
   * @param [in] c a vector (m_cx1)
   * @param [in] D a (possibly null) matrix (m_dxn)
   * @param [in] d a vector (m_dx1)
   * @param [in] f a vector (m_dx1)
   * @param [in] l a vector (m_dx1)
   * @param [in] u a vector (m_dx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x);
  /*!
   * Finds x that minimizes f = (Ax-b)' S (Ax-b) + x' W x, such that Cx = c
   * @param [in] A a matrix (mxn)
   * @param [in] S a diagonal weighting matrix (mxm)
   * @param [in] b a vector (mx1)
   * @param [in] W a diagonal weighting matrix (nxn)
   * @param [in] C a (possibly null) matrix (m_cxn)
   * @param [in] c a vector (m_cx1)
   * @param [out] x a vector (nx1)
   * @return true if successful
   */
  static bool solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
                    const Eigen::VectorXd& b,
                    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& c, Eigen::VectorXd& x);
};

}  // namespace common

#endif
