#include "common/solver/qp_solver.h"

#include <stdexcept>

#include "common/solver/ooqp_interface.h"

namespace common {

bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& D,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u, Eigen::VectorXd& x) {
  // f = (Ax-b)' S (Ax-b) + x' W x = x'A'SAx - 2x'A'Sb + b'Sb + x'Wx.
  // This means minimizing f is equivalent to minimizing: 1/2 x'Qx + c'x,
  // where Q = A'SA + W and c = -A'Sb.
  // Adapted from 'simulationandcontrol' by Stelian Coros.
  int m = A.rows();
  int n = A.cols();
  x.setZero(n);
  assert(static_cast<int>(b.size()) == m);
  assert(static_cast<int>(S.rows()) == m);
  assert(static_cast<int>(W.rows()) == n);
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q_temp;
  Q_temp = A.transpose() * S * A +
           (Eigen::SparseMatrix<double, Eigen::RowMajor>)W.toDenseMatrix()
               .sparseView();
  // Q_temp = A.transpose() * S * A;
  Eigen::VectorXd c_temp = -A.transpose() * S * b;
  return OoQpItf::solve(Q_temp, c_temp, C, c, D, d, f, l, u, x);
}

bool QuadraticProblem::solve(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& S,
    const Eigen::VectorXd& b,
    const Eigen::DiagonalMatrix<double, Eigen::Dynamic>& W,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& c, Eigen::VectorXd& x) {
  // limit x is infinite
  int nx = A.cols();
  Eigen::VectorXd u =
      std::numeric_limits<double>::max() * Eigen::VectorXd::Ones(nx);
  Eigen::VectorXd l = (-u.array()).matrix();
  // null D matrix
  Eigen::SparseMatrix<double, Eigen::RowMajor> D;
  Eigen::VectorXd d, f;
  return solve(A, S, b, W, C, c, D, d, f, l, u, x);

}

}  // namespace common