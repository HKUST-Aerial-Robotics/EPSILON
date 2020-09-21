#ifndef _COMMON_INC_COMMON_SOLVER_SPLINE_SOLVER_H__
#define _COMMON_INC_COMMON_SOLVER_SPLINE_SOLVER_H__

#include "common/basics/basics.h"
#include "common/spline/spline.h"
#include "common/state/waypoint.h"

namespace common {

/**
 * @brief Using quadratic programming to find the optimal polynomial coeffs for
 * the spline
 * @note Heavily brought from Sikang Liu's implementation, thanks!
 * @note https://github.com/sikang/motion_primitive_library
 */
template <int N_DEG, int N_DIM>
class SplineUnQP {
 public:
  typedef Spline<N_DEG, N_DIM> SplineType;
  static ErrorType GetSplineByWaypoints(const vec_E<Waypoint<N_DIM>>& waypoints,
                                        const int minimize_derivative,
                                        SplineType* spline) {
    int num_waypoints = static_cast<int>(waypoints.size());
    int num_segments = num_waypoints - 1;
    int N = N_DEG + 1;
    int smooth_derivative_order = N / 2 - 1;
    int QR = minimize_derivative;

    if (num_waypoints < 2) return kIllegalInput;

    std::vector<decimal_t> dts;
    if (GetSegmentDurationsFromWaypoints(waypoints, &dts) != kSuccess) {
      return kWrongStatus;
    }

    MatDf A = MatDf::Zero(num_segments * N, num_segments * N);
    MatDf Q = MatDf::Zero(num_segments * N, num_segments * N);

    for (int i = 0; i < num_segments; i++) {
      decimal_t seg_time = dts[i];
      // n column
      for (int n = 0; n < N; n++) {
        // A_0
        if (n < N / 2) {
          int val = 1;
          for (int m = 0; m < n; m++) val *= (n - m);
          A(i * N + n, i * N + n) = val;
        }
        // A_T
        for (int r = 0; r < N / 2; r++) {
          if (r <= n) {
            int val = 1;
            for (int m = 0; m < r; m++) val *= (n - m);
            A(i * N + N / 2 + r, i * N + n) = val * pow(seg_time, n - r);
          }
        }
        // Q
        for (int r = 0; r < N; r++) {
          if (r >= QR && n >= QR) {
            int val = 1;
            for (int m = 0; m < QR; m++) val *= (r - m) * (n - m);
            Q(i * N + r, i * N + n) =
                val * pow(seg_time, r + n - 2 * QR + 1) / (r + n - 2 * QR + 1);
          }
        }
      }
    }

    const int num_total_derivatives = num_waypoints * N / 2;
    int num_fixed_derivatives = 0;
    for (const auto& it : waypoints) {
      if (it.fix_pos && smooth_derivative_order >= 0) num_fixed_derivatives++;
      if (it.fix_vel && smooth_derivative_order >= 1) num_fixed_derivatives++;
      if (it.fix_acc && smooth_derivative_order >= 2) num_fixed_derivatives++;
      if (it.fix_jrk && smooth_derivative_order >= 3) num_fixed_derivatives++;
    }
    const int num_free_derivatives =
        num_total_derivatives - num_fixed_derivatives;

    // printf(
    //     "[SplineByWaypoints] smooth order: %d,num_fixed ds : %d, num_free ds:
    //     "
    //     "%d.\n",
    //     smooth_derivative_order, num_fixed_derivatives,
    //     num_free_derivatives);
    std::vector<std::pair<int, int>> permutation_table;
    int raw_cnt = 0;
    int fix_cnt = 0;
    int free_cnt = 0;
    int id = 0;
    for (const auto& it : waypoints) {
      if (it.fix_pos && smooth_derivative_order >= 0) {
        permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(raw_cnt + N / 2, fix_cnt));
        raw_cnt++;
        fix_cnt++;
      } else if (!it.fix_pos && smooth_derivative_order >= 0) {
        permutation_table.push_back(
            std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(
              raw_cnt + N / 2, num_fixed_derivatives + free_cnt));
        raw_cnt++;
        free_cnt++;
      }
      if (it.fix_vel && smooth_derivative_order >= 1) {
        permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(raw_cnt + N / 2, fix_cnt));
        raw_cnt++;
        fix_cnt++;
      } else if (!it.fix_vel && smooth_derivative_order >= 1) {
        permutation_table.push_back(
            std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(
              raw_cnt + N / 2, num_fixed_derivatives + free_cnt));
        raw_cnt++;
        free_cnt++;
      }
      if (it.fix_acc && smooth_derivative_order >= 2) {
        permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(raw_cnt + N / 2, fix_cnt));
        raw_cnt++;
        fix_cnt++;
      } else if (!it.fix_acc && smooth_derivative_order >= 2) {
        permutation_table.push_back(
            std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(
              raw_cnt + N / 2, num_fixed_derivatives + free_cnt));
        raw_cnt++;
        free_cnt++;
      }
      if (it.fix_jrk && smooth_derivative_order >= 3) {
        permutation_table.push_back(std::make_pair(raw_cnt, fix_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(raw_cnt + N / 2, fix_cnt));
        raw_cnt++;
        fix_cnt++;
      } else if (!it.fix_jrk && smooth_derivative_order >= 3) {
        permutation_table.push_back(
            std::make_pair(raw_cnt, num_fixed_derivatives + free_cnt));
        if (id > 0 && id < num_waypoints - 1)
          permutation_table.push_back(std::make_pair(
              raw_cnt + N / 2, num_fixed_derivatives + free_cnt));
        raw_cnt++;
        free_cnt++;
      }

      if (id > 0 && id < num_waypoints - 1) raw_cnt += N / 2;
      id++;
    }

    // M
    MatDf M = MatDf::Zero(num_segments * N, num_waypoints * N / 2);
    for (const auto& it : permutation_table) M(it.first, it.second) = 1;

    // Eigen::MatrixXf A_inv = A.inverse();
    MatDf A_inv_M = A.partialPivLu().solve(M);
    MatDf R = A_inv_M.transpose() * Q * A_inv_M;
    MatDf Rpp = R.block(num_fixed_derivatives, num_fixed_derivatives,
                        num_free_derivatives, num_free_derivatives);
    MatDf Rpf = R.block(num_fixed_derivatives, 0, num_free_derivatives,
                        num_fixed_derivatives);

    // Fixed derivatives
    MatDNf<N_DIM> Df = MatDNf<N_DIM>(num_fixed_derivatives, N_DIM);
    for (const auto& it : permutation_table) {
      if (it.second < num_fixed_derivatives) {
        int id = std::floor((it.first + N / 2) / N);
        int derivative = it.first % (N / 2);
        if (derivative == 0)
          Df.row(it.second) = waypoints[id].pos.transpose();
        else if (derivative == 1)
          Df.row(it.second) = waypoints[id].vel.transpose();
        else if (derivative == 2)
          Df.row(it.second) = waypoints[id].acc.transpose();
        else if (derivative == 3)
          Df.row(it.second) = waypoints[id].jrk.transpose();
      }
    }

    MatDNf<N_DIM> D = MatDNf<N_DIM>(num_waypoints * N / 2, N_DIM);
    D.topRows(num_fixed_derivatives) = Df;

    if (num_waypoints > 2 && num_free_derivatives > 0) {
      MatDNf<N_DIM> Dp = -Rpp.partialPivLu().solve(Rpf * Df);
      D.bottomRows(num_free_derivatives) = Dp;
    }

    MatDNf<N_DIM> d = M * D;

    std::vector<decimal_t> vec_domain;
    vec_domain.reserve(num_waypoints);
    for (int i = 0; i < num_waypoints; i++) {
      vec_domain.push_back(waypoints[i].t);
    }
    spline->set_vec_domain(vec_domain);
    for (int i = 0; i < num_segments; i++) {
      const MatDNf<N_DIM> p = A.block(i * N, i * N, N, N)
                                  .partialPivLu()
                                  .solve(d.block(i * N, 0, N, N_DIM));
      for (int j = 0; j < N_DIM; j++) {
        Vecf<N_DEG + 1> coeff = Vecf<N_DEG + 1>::Zero();
        auto v = p.col(j);
        for (int t = 0; t < N_DEG + 1; t++) coeff[N_DEG - t] = v[t] * fac(t);
        (*spline)(i, j).set_coeff(coeff);
      }
    }

    return kSuccess;
  }

  static ErrorType GetSegmentDurationsFromWaypoints(
      const vec_E<Waypoint<N_DIM>>& waypoints, std::vector<decimal_t>* dts) {
    int num_waypoints = static_cast<int>(waypoints.size());
    if (num_waypoints < 1) return kIllegalInput;
    for (auto& pt : waypoints) {
      if (!pt.stamped) return kIllegalInput;
    }

    dts->clear();
    dts->reserve(num_waypoints - 1);
    for (int i = 0; i < num_waypoints - 1; i++) {
      dts->push_back(waypoints[i + 1].t - waypoints[i].t);
    }
    return kSuccess;
  }
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_SOLVER_SPLINE_SOLVER_H__