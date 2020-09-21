#include "common/solver/ooqp_interface.h"

#include <stdexcept>

#include "GondzioSolver.h"
#include "QpGenData.h"
#include "QpGenResiduals.h"
#include "QpGenSparseMa27.h"
#include "QpGenVars.h"
#include "Status.h"

namespace common {

using namespace Eigen;
using namespace std;

bool OoQpItf::solve(const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
                    const Eigen::VectorXd& c,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
                    const Eigen::VectorXd& b,
                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
                    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
                    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                    Eigen::VectorXd& x, const bool ignoreUnknownError,
                    const bool verbose) {
  int nx = Q.rows();  // nx is the number of primal variables (x).
  // OOQPEI_ASSERT_GT(range_error, nx, 0, "Matrix Q has size 0.");
  x.setZero(nx);

  // Make copies of variables that are changed.
  auto ccopy(c);
  auto Acopy(A);
  auto bcopy(b);
  auto Ccopy(C);

  // Make sure Q is in lower triangular form (Q is symmetric).
  // Refer to OOQP user guide section 2.2 (p. 11).
  // TODO Check if Q is really symmetric.
  SparseMatrix<double, Eigen::RowMajor> Q_triangular =
      Q.triangularView<Lower>();

  if (verbose) {
    printProblemFormulation(Q_triangular, ccopy, Acopy, bcopy, Ccopy, d, f, l,
                            u);
  }

  // Compress sparse Eigen matrices (refer to Eigen Sparse Matrix user manual).
  Q_triangular.makeCompressed();
  Acopy.makeCompressed();
  Ccopy.makeCompressed();

  assert(Ccopy.rows() == d.size());
  assert(Ccopy.rows() == f.size());
  // Determine which limits are active and which are not.
  // Refer to OOQP user guide section 2.2 (p. 10).
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForX;
  VectorXd lowerLimitForX;
  VectorXd upperLimitForX;
  Matrix<char, Eigen::Dynamic, 1> useLowerLimitForInequalityConstraints;
  Matrix<char, Eigen::Dynamic, 1> useUpperLimitForInequalityConstraints;
  VectorXd lowerLimitForInequalityConstraints;
  VectorXd upperLimitForInequalityConstraints;

  generateLimits(l, u, useLowerLimitForX, useUpperLimitForX, lowerLimitForX,
                 upperLimitForX);
  generateLimits(d, f, useLowerLimitForInequalityConstraints,
                 useUpperLimitForInequalityConstraints,
                 lowerLimitForInequalityConstraints,
                 upperLimitForInequalityConstraints);
  if (verbose) {
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR X" << endl;
    printLimits(useLowerLimitForX, useUpperLimitForX, lowerLimitForX,
                upperLimitForX);
    cout << "-------------------------------" << endl;
    cout << "LIMITS FOR INEQUALITY CONSTRAINTS" << endl;
    printLimits(useLowerLimitForInequalityConstraints,
                useUpperLimitForInequalityConstraints,
                lowerLimitForInequalityConstraints,
                upperLimitForInequalityConstraints);
  }

  // Setting up OOQP solver
  // Refer to OOQP user guide section 2.3 (p. 14).

  // Initialize new problem formulation.
  int my = bcopy.size();
  int mz = lowerLimitForInequalityConstraints.size();
  int nnzQ = Q_triangular.nonZeros();
  int nnzA = Acopy.nonZeros();
  int nnzC = Ccopy.nonZeros();

  QpGenSparseMa27* qp = new QpGenSparseMa27(nx, my, mz, nnzQ, nnzA, nnzC);
  // Fill in problem data.
  double* cp = &ccopy.coeffRef(0);
  int* krowQ = Q_triangular.outerIndexPtr();
  int* jcolQ = Q_triangular.innerIndexPtr();
  double* dQ = Q_triangular.valuePtr();
  double* xlow = &lowerLimitForX.coeffRef(0);
  char* ixlow = &useLowerLimitForX.coeffRef(0);
  double* xupp = &upperLimitForX.coeffRef(0);
  char* ixupp = &useUpperLimitForX.coeffRef(0);
  int* krowA = Acopy.outerIndexPtr();
  int* jcolA = Acopy.innerIndexPtr();
  double* dA = Acopy.valuePtr();
  double* bA = &bcopy.coeffRef(0);
  int* krowC = Ccopy.outerIndexPtr();
  int* jcolC = Ccopy.innerIndexPtr();
  double* dC = Ccopy.valuePtr();
  double* clow = &lowerLimitForInequalityConstraints.coeffRef(0);
  char* iclow = &useLowerLimitForInequalityConstraints.coeffRef(0);
  double* cupp = &upperLimitForInequalityConstraints.coeffRef(0);
  char* icupp = &useUpperLimitForInequalityConstraints.coeffRef(0);

  QpGenData* prob = (QpGenData*)qp->makeData(
      cp, krowQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp, krowA, jcolA, dA, bA,
      krowC, jcolC, dC, clow, iclow, cupp, icupp);

  // Create object to store problem variables.
  QpGenVars* vars = (QpGenVars*)qp->makeVariables(prob);
  //  if (isInDebugMode()) prob->print(); // Matrices are printed as [index_x,
  //  index_y, value]

  // Create object to store problem residual data.
  QpGenResiduals* resid = (QpGenResiduals*)qp->makeResiduals(prob);

  // Create solver object.
  GondzioSolver* s = new GondzioSolver(qp, prob);

  if (verbose) {
    s->monitorSelf();
  }

  // Solve.
  int status = s->solve(prob, vars, resid);

  if ((status == SUCCESSFUL_TERMINATION) ||
      (ignoreUnknownError && (status == UNKNOWN)))
    vars->x->copyIntoArray(&x.coeffRef(0));

  if (verbose) {
    printSolution(status, x);
  }
  // vars->x->writefToStream( cout, "x[%{index}] = %{value}" );

  delete s;
  delete resid;
  delete vars;
  delete prob;
  delete qp;

  return ((status == SUCCESSFUL_TERMINATION) ||
          (ignoreUnknownError && (status == UNKNOWN)));
}

void OoQpItf::generateLimits(
    const Eigen::VectorXd& l, const Eigen::VectorXd& u,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    Eigen::VectorXd& lowerLimit, Eigen::VectorXd& upperLimit) {
  int n = l.size();
  useLowerLimit.setConstant(n, 1);
  useUpperLimit.setConstant(n, 1);
  lowerLimit = l;
  upperLimit = u;

  for (int i = 0; i < n; i++) {
    if (NumericalUtil::ApproximatelyEqual(
            l(i), -std::numeric_limits<double>::max())) {
      useLowerLimit(i) = 0;
      lowerLimit(i) = 0.0;
    }
    if (NumericalUtil::ApproximatelyEqual(u(i),
                                          std::numeric_limits<double>::max())) {
      useUpperLimit(i) = 0;
      upperLimit(i) = 0.0;
    }
  }
}

void OoQpItf::printProblemFormulation(
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Q,
    const Eigen::VectorXd& c,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A,
    const Eigen::VectorXd& b,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& C,
    const Eigen::VectorXd& d, const Eigen::VectorXd& f,
    const Eigen::VectorXd& l, const Eigen::VectorXd& u) {
  cout << "-------------------------------" << endl;
  cout << "Find x: min 1/2 x' Q x + c' x such that A x = b, d <= Cx <= f, and "
          "l <= x <= u"
       << endl
       << endl;
  cout << "Q (triangular) << " << endl << MatrixXd(Q) << endl;
  cout << "c << " << c.transpose() << endl;
  cout << "A << " << endl << MatrixXd(A) << endl;
  cout << "b << " << b.transpose() << endl;
  cout << "C << " << endl << MatrixXd(C) << endl;
  cout << "d << " << d.transpose() << endl;
  cout << "f << " << f.transpose() << endl;
  cout << "l << " << l.transpose() << endl;
  cout << "u << " << u.transpose() << endl;
}

void OoQpItf::printLimits(
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useLowerLimit,
    const Eigen::Matrix<char, Eigen::Dynamic, 1>& useUpperLimit,
    const Eigen::VectorXd& lowerLimit, const Eigen::VectorXd& upperLimit) {
  cout << "useLowerLimit << " << std::boolalpha
       << useLowerLimit.cast<bool>().transpose() << endl;
  cout << "lowerLimit << " << lowerLimit.transpose() << endl;
  cout << "useUpperLimit << " << std::boolalpha
       << useUpperLimit.cast<bool>().transpose() << endl;
  cout << "upperLimit << " << upperLimit.transpose() << endl;
}

void OoQpItf::printSolution(const int status, const Eigen::VectorXd& x) {
  if (status == 0) {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Ok, ended with status " << status << "." << endl;
    cout << "x << " << x.transpose() << endl;
  } else {
    cout << "-------------------------------" << endl;
    cout << "SOLUTION" << endl;
    cout << "Error, ended with status " << status << "." << endl;
  }
}

}  // namespace common