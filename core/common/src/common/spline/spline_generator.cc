#include "common/spline/spline_generator.h"

#include "common/solver/ooqp_interface.h"

namespace common {

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetCubicSplineBySampleInterpolation(
    const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
    SplineType* spline) {
  if (samples.size() < 3) {
    // printf("[SampleInterpolation]input sample size: %d.\n",
    //        static_cast<int>(samples.size()));
    return kWrongStatus;
  }
  if (samples.size() != para.size()) return kIllegalInput;
  if (N_DEG < 3) {
    printf(
        "[CubicSplineBySampleInterpolation]Currently we just support "
        "interpolation >= cubic(3).\n");
    return kWrongStatus;
  }

  SplineType cubic_spline;
  // interpolate parameterization if too sparse
  // NOTE: the issue is caused by GetArcLengthByVecPosition
  // the current implementation is a fast version and reduces
  // the evaluation of spline as much as possible, but suffers
  // from the local minimum issue. To avoid the local minimum in practice
  // it is recommended to have a relatively dense parameterization
  // e.g, delta para < 3.5 * 2 = 7.0 m
  const decimal_t para_delta_threshold = 7.0;
  int num_samples = static_cast<int>(samples.size());
  vec_Vecf<N_DIM> interpolated_samples{samples[0]};
  std::vector<decimal_t> interpolated_para{para[0]};

  for (int i = 1; i < num_samples; i++) {
    decimal_t dis = para[i] - para[i - 1];
    if (dis > para_delta_threshold) {
      int n_inserted = static_cast<int>(dis / para_delta_threshold) + 1;
      decimal_t delta_para = dis / n_inserted;
      for (int n = 0; n < n_inserted; n++) {
        decimal_t para_insert = para[i - 1] + (n + 1) * delta_para;
        decimal_t partion = (n + 1) * delta_para / dis;
        if (para_insert < para[i]) {
          interpolated_para.push_back(para_insert);
          Vecf<N_DIM> sample_insert;
          for (int d = 0; d < N_DIM; d++) {
            sample_insert[d] = (samples[i][d] - samples[i - 1][d]) * partion +
                               samples[i - 1][d];
          }
          interpolated_samples.push_back(sample_insert);
        }
      }
    }
    interpolated_para.push_back(para[i]);
    interpolated_samples.push_back(samples[i]);
  }

  cubic_spline.set_vec_domain(interpolated_para);
  for (int i = 0; i < N_DIM; i++) {
    std::vector<decimal_t> X, Y;
    for (int s = 0; s < static_cast<int>(interpolated_samples.size()); s++) {
      X.push_back(interpolated_para[s]);
      Y.push_back(interpolated_samples[s][i]);
    }

    tk::spline cubic_fitting;
    cubic_fitting.set_points(X, Y);

    for (int n = 0; n < cubic_fitting.num_pts() - 1; n++) {
      Vecf<N_DEG + 1> coeff = Vecf<N_DEG + 1>::Zero();
      for (int d = 0; d <= N_DEG; d++) {
        if (d <= 3) {
          coeff[N_DEG - d] = cubic_fitting.get_coeff(n, d) * fac(d);
        } else {
          coeff[N_DEG - d] = 0.0;
        }
      }
      cubic_spline(n, i).set_coeff(coeff);
    }
  }
  (*spline) = std::move(cubic_spline);
  return kSuccess;
}

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetQuinticSplineBySampleFitting(
    const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
    const Eigen::ArrayXf& breaks, const decimal_t regulator,
    SplineType* spline) {
  // ~ the polynomial order follows the definition in polynomial.h
  // printf("number of samples: %d.\n", static_cast<int>(samples.size()));
  // std::cout << "breaks: " << breaks.transpose() << std::endl;

  if (N_DEG < 5) {
    printf("[QuinticSpline]Degree not support.\n");
    return kWrongStatus;
  }
  if (breaks.size() < 2) {
    printf("[QuinticSpline]Cannot support less than two segment.\n");
    return kWrongStatus;
  }

  if (samples.size() < 3) {
    // printf("[QuinticSpline]Cannot support less than three samples.\n");
    return kWrongStatus;
  }

  // TicToc timer;
  int num_segments = breaks.size() - 1;
  int num_samples = static_cast<int>(samples.size());
  int num_order = N_DEG + 1;

  // re-organize samples and compute relative t
  // vec_E<vec_E<std::pair<double, Vecf<N_DIM>>>> all_samples;
  std::vector<std::vector<double>> all_samples;
  std::vector<decimal_t> durations;
  std::vector<int> num_samples_hist{0};
  {
    int idx_low = 0;
    for (int n = 0; n < num_segments; n++) {
      std::vector<double> sub_samples;
      auto lbd = breaks[n];
      auto ubd = breaks[n + 1];
      // auto lower_it = std::lower_bound(para.begin(), para.end(), lbd);
      auto upper_it = std::upper_bound(para.begin(), para.end(), ubd);
      // int idx_low = static_cast<int>(lower_it - para.begin());
      int idx_up = static_cast<int>(upper_it - para.begin());
      // printf("idx_low: %d, idx_up: %d.\n", idx_low, idx_up);
      for (int i = idx_low; i < idx_up; i++) {
        sub_samples.push_back(para[i] - lbd);
      }
      all_samples.push_back(sub_samples);
      durations.push_back(ubd - lbd);
      num_samples_hist.push_back(idx_up);
      idx_low = idx_up;
    }
  }

  // timer.tic();
  // prepare A
  Eigen::SparseMatrix<double, Eigen::RowMajor> A(
      N_DIM * num_samples, N_DIM * num_segments * num_order);
  A.reserve(Eigen::VectorXi::Constant(N_DIM * num_samples, num_order));
  {
    int idx, idy;
    double val;
    for (int n = 0; n < num_segments; n++) {
      int num_sub_samples = static_cast<int>(all_samples[n].size());
      for (int i = 0; i < num_sub_samples; i++) {
        auto deltat = all_samples[n][i];
        for (int j = 0; j < num_order; j++) {
          val = pow(deltat, num_order - j - 1) / fac(num_order - j - 1);
          for (int d = 0; d < N_DIM; d++) {
            idx = d * num_samples + num_samples_hist[n] + i;
            idy = d * num_segments * num_order + n * num_order + j;
            A.insert(idx, idy) = val;
          }
        }
      }
    }
  }

  // prepare b
  Eigen::VectorXd b;
  b.resize(N_DIM * num_samples);
  {
    for (int i = 0; i < num_samples; i++) {
      for (int d = 0; d < N_DIM; d++) {
        b(d * num_samples + i) = samples[i][d];
      }
    }
  }

  // prepare S
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> S(N_DIM * num_samples);
  S.diagonal() = Eigen::VectorXd::Ones(N_DIM * num_samples);

  // prepare W
  Eigen::DiagonalMatrix<double, Eigen::Dynamic> W(N_DIM * num_segments *
                                                  num_order);
  Eigen::VectorXd weight(N_DIM * num_segments * num_order);
  weight.setZero();
  int regulate_top_degree = 3;
  for (int n = 0; n < num_segments; n++) {
    for (int j = 0; j < num_order; j++) {
      for (int d = 0; d < N_DIM; d++) {
        if (j < regulate_top_degree)
          weight(d * num_segments * num_order + n * num_order + j) =
              regulator / pow(10, j);
      }
    }
  }
  W.diagonal() = weight;

  // prepare C
  int num_connections = num_segments - 1;
  int num_continuity = 4;  // continuity up to jerk should be enough
  Eigen::SparseMatrix<double, Eigen::RowMajor> C(
      N_DIM * num_connections * num_continuity,
      N_DIM * num_segments * num_order);
  C.reserve(Eigen::VectorXi::Constant(N_DIM * num_connections * num_continuity,
                                      2 * num_order));
  {
    int idx, idy_l, idy_r;
    double val_l, val_r;
    for (int n = 0; n < num_connections; n++) {
      for (int c = 0; c < num_continuity; c++) {
        for (int j = 0; j < num_order; j++) {
          if (j <= num_order - 1 - c) {
            auto T = durations[n];
            val_l = pow(T, num_order - j - 1 - c) / fac(num_order - j - 1 - c);
            val_r =
                -pow(0.0, num_order - j - 1 - c) / fac(num_order - j - 1 - c);
            for (int d = 0; d < N_DIM; d++) {
              idx =
                  d * num_connections * num_continuity + n * num_continuity + c;

              idy_l = d * num_segments * num_order + n * num_order + j;
              idy_r = d * num_segments * num_order + (n + 1) * num_order + j;
              C.insert(idx, idy_l) = val_l;
              C.insert(idx, idy_r) = val_r;
            }
          }
        }
      }
    }
  }

  // prepare c
  Eigen::VectorXd c;
  c.resize(N_DIM * num_connections * num_continuity);
  c.setZero();

  // double t_elapsed = timer.toc();
  // printf("time consumed in wrap constraints: %lf.\n", t_elapsed);

  Eigen::VectorXd x(N_DIM * num_segments * num_order);
  if (QuadraticProblem::solve(A, S, b, W, C, c, x)) {
    SplineType quintic_spline;
    std::vector<double> vec_domain;
    for (int i = 0; i < breaks.size(); i++) vec_domain.push_back(breaks[i]);
    quintic_spline.set_vec_domain(vec_domain);
    for (int d = 0; d < N_DIM; d++) {
      for (int n = 0; n < num_segments; n++) {
        Vecf<N_DEG + 1> coeff = Vecf<N_DEG + 1>::Zero();
        for (int i = 0; i <= N_DEG; i++) {
          coeff(i) = x[d * num_segments * num_order + n * num_order + i];
        }
        quintic_spline(n, d).set_coeff(coeff);
      }
    }
    (*spline) = std::move(quintic_spline);
  } else {
    printf("solver failed.\n");
    return kWrongStatus;
  }

  return kSuccess;
}

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetWaypointsFromPositionSamples(
    const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
    vec_E<Waypoint<N_DIM>>* waypoints) {
  int num_samples = static_cast<int>(samples.size());
  int num_paras = static_cast<int>(para.size());
  if (num_samples != num_paras) {
    return kIllegalInput;
  }

  waypoints->clear();
  waypoints->reserve(num_samples);
  for (int i = 0; i < num_samples; i++) {
    Waypoint<N_DIM> wp;
    wp.pos = samples[i];
    wp.fix_pos = true;
    wp.t = para[i];
    wp.stamped = true;
    waypoints->push_back(wp);
  }

  return kSuccess;
}

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetSplineFromStateVec(
    const std::vector<decimal_t>& para, const vec_E<State>& state_vec,
    SplineType* spline) {
  if (N_DEG < 5) {
    printf(
        "[GetSplineFromStateVec]To fit state vec we need at least 5 "
        "degree.\n");
    return kWrongStatus;
  }
  if (para.size() < 2) {
    printf("[GetSplineFromStateVec]failed to get spline from state vec.\n");
    return kWrongStatus;
  }

  assert(para.size() == state_vec.size());
  spline->set_vec_domain(para);
  int num_pts = static_cast<int>(state_vec.size());
  for (int i = 0; i < num_pts - 1; i++) {
    FreeState s0, s1;
    GetFreeStateFromState(state_vec[i], &s0);
    GetFreeStateFromState(state_vec[i + 1], &s1);
    for (int d = 0; d < N_DIM; d++) {
      if (d <= 1) {
        (*spline)(i, d).GetJerkOptimalConnection(
            s0.position[d], s0.velocity[d], s0.acceleration[d], s1.position[d],
            s1.velocity[d], s1.acceleration[d], para[i + 1] - para[i]);
      } else {
        (*spline)(i, d).set_zero();
      }
    }
  }
  return kSuccess;
}

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetSplineFromFreeStateVec(
    const std::vector<decimal_t>& para, const vec_E<FreeState>& free_state_vec,
    SplineType* spline) {
  if (N_DEG < 5) {
    printf(
        "[GetSplineFromStateVec]To fit state vec we need at least 5 "
        "degree.\n");
    return kWrongStatus;
  }
  if (para.size() < 2) {
    printf("[GetSplineFromStateVec]failed to get spline from state vec.\n");
    return kWrongStatus;
  }
  assert(para.size() == free_state_vec.size());
  spline->set_vec_domain(para);
  int num_pts = static_cast<int>(free_state_vec.size());

  for (int i = 0; i < num_pts - 1; i++) {
    FreeState s0 = free_state_vec[i];
    FreeState s1 = free_state_vec[i + 1];
    for (int d = 0; d < N_DIM; d++) {
      if (d <= 1) {
        (*spline)(i, d).GetJerkOptimalConnection(
            s0.position[d], s0.velocity[d], s0.acceleration[d], s1.position[d],
            s1.velocity[d], s1.acceleration[d], para[i + 1] - para[i]);
      } else {
        (*spline)(i, d).set_zero();
      }
    }
  }
  return kSuccess;
}

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetBezierSplineUsingCorridor(
    const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const vec_E<Vecf<N_DIM>>& start_constraints,
    const vec_E<Vecf<N_DIM>>& end_constraints,
    const std::vector<decimal_t>& ref_stamps,
    const vec_E<Vecf<N_DIM>>& ref_points, const decimal_t& weight_proximity,
    BezierSplineType* bezier_spline) {
  int num_segments = static_cast<int>(cubes.size());
  int num_order = N_DEG + 1;
  int derivative_degree = 3;

  // ~ Stage I: stack objective
  int total_num_vals = N_DIM * num_segments * num_order;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q(total_num_vals,
                                                 total_num_vals);
  Q.reserve(
      Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
  {
    MatNf<N_DEG + 1> hessian =
        BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
    int idx, idy;
    decimal_t val;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          for (int k = 0; k < num_order; k++) {
            idx = d * num_segments * num_order + n * num_order + j;
            idy = d * num_segments * num_order + n * num_order + k;
            val = hessian(j, k) / pow(duration, 2 * derivative_degree - 3);
            Q.insert(idx, idy) = val;
          }
        }
      }
    }
  }

  Eigen::VectorXd c;
  c.resize(total_num_vals);
  c.setZero();

  Eigen::SparseMatrix<double, Eigen::RowMajor> P(total_num_vals,
                                                 total_num_vals);
  P.reserve(
      Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));

  // * Only position difference is considered
  if (!ref_stamps.empty()) {
    int idx, idy;
    int num_ref_samples = ref_stamps.size();
    for (int i = 0; i < num_ref_samples; i++) {
      if (ref_stamps[i] < cubes[0].t_lb ||
          ref_stamps[i] > cubes[num_segments - 1].t_ub)
        continue;

      int n;
      for (n = 0; n < num_segments; n++) {
        if (cubes[n].t_ub > ref_stamps[i]) {
          break;
        }
      }
      n = std::min(num_segments - 1, n);
      decimal_t s = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t t = ref_stamps[i] - cubes[n].t_lb;
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          idx = d * num_segments * num_order + n * num_order + j;
          c[idx] += -2 * ref_points[i][d] * s * nchoosek(N_DEG, j) *
                    pow(t / s, j) * pow(1 - t / s, N_DEG - j);
          for (int k = 0; k < num_order; k++) {
            idy = d * num_segments * num_order + n * num_order + k;
            P.coeffRef(idx, idy) += s * s * nchoosek(N_DEG, j) *
                                    nchoosek(N_DEG, k) * pow(t / s, j + k) *
                                    pow(1 - t / s, 2 * N_DEG - j - k);
          }
        }
      }
    }
  }
  // std::cout << "Q = \n" << Q << "\n" << std::endl;
  // std::cout << "P = \n" << P << "\n" << std::endl;
  // std::cout << "c = \n" << c << "\n" << std::endl;

  P = P * weight_proximity;
  c = c * weight_proximity;

  Q = 2 * (Q + P);  // 0.5 * x' * Q * x

  // ~ Stage II: stack equality constraints
  int num_continuity = 3;  // continuity up to jerk
  int num_connections = num_segments - 1;
  // printf("num conenctions: %d.\n", num_connections);
  int num_continuity_constraints = N_DIM * num_connections * num_continuity;
  int num_start_eq_constraints =
      static_cast<int>(start_constraints.size()) * N_DIM;
  int num_end_eq_constraints = static_cast<int>(end_constraints.size()) *
                               N_DIM;  // exclude s position constraint
  int total_num_eq_constraints = num_continuity_constraints +
                                 num_start_eq_constraints +
                                 num_end_eq_constraints;
  Eigen::SparseMatrix<double, Eigen::RowMajor> A(
      total_num_eq_constraints, N_DIM * num_segments * num_order);
  A.reserve(Eigen::VectorXi::Constant(total_num_eq_constraints, 2 * num_order));

  Eigen::VectorXd b;
  b.resize(total_num_eq_constraints);
  b.setZero();

  int idx, idy;
  decimal_t val;
  {
    // ~ continuity constraints
    for (int n = 0; n < num_connections; n++) {
      decimal_t duration_l = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t duration_r = cubes[n + 1].t_ub - cubes[n + 1].t_lb;
      for (int c = 0; c < num_continuity; c++) {
        decimal_t scale_l = pow(duration_l, 1 - c);
        decimal_t scale_r = pow(duration_r, 1 - c);
        for (int d = 0; d < N_DIM; d++) {
          idx = d * num_connections * num_continuity + n * num_continuity + c;
          if (c == 0) {
            // ~ position end
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ position begin
            idy = d * num_segments * num_order + (n + 1) * num_order + 0;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 1) {
            // ~ velocity end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ velocity begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 2) {
            // ~ acceleration end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ acceleration begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = -2.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 3) {
            // ~ jerk end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 3;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ jerk begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = -3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 3;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          }
        }
      }
    }

    // ~ start state constraints
    {
      int num_order_constraint_start =
          static_cast<int>(start_constraints.size());
      decimal_t duration = cubes[0].t_ub - cubes[0].t_lb;
      decimal_t scale;
      int n = 0;
      for (int j = 0; j < num_order_constraint_start; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          idx = num_continuity_constraints + d * num_order_constraint_start + j;
          if (j == 0) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;

            b[idx] = start_constraints[j][d];
            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
          } else if (j == 1) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 1;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;

            b[idx] = start_constraints[j][d];
            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
          } else if (j == 2) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            b[idx] = start_constraints[j][d];
            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
          }
        }
      }
    }
    // ~ end state constraints
    {
      int num_order_constraint_end = static_cast<int>(end_constraints.size());
      decimal_t duration =
          cubes[num_segments - 1].t_ub - cubes[num_segments - 1].t_lb;
      decimal_t scale;
      int n = num_segments - 1;
      int accu_eq_cons_idx =
          num_continuity_constraints + num_start_eq_constraints;
      for (int j = 0; j < num_order_constraint_end; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          // if (j == 0 && d == 0) continue;
          idx = accu_eq_cons_idx++;
          if (j == 0) {
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 1) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 2) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          }
        }
      }
    }
  }

  // ~ Stage III: stack inequality constraints
  int total_num_ineq = 0;
  for (int i = 0; i < num_segments; i++) {
    total_num_ineq += (static_cast<int>(cubes[i].p_ub.size())) * num_order;
    total_num_ineq +=
        (static_cast<int>(cubes[i].v_ub.size())) * (num_order - 1);
    total_num_ineq +=
        (static_cast<int>(cubes[i].a_ub.size())) * (num_order - 2);
  }
  // Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  Eigen::VectorXd lbd;
  Eigen::VectorXd ubd;
  Eigen::SparseMatrix<double, Eigen::RowMajor> C(total_num_ineq,
                                                 total_num_vals);
  C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3));
  lbd.setZero(total_num_ineq);
  ubd.setZero(total_num_ineq);
  {
    int accu_num_ineq = 0;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t scale;
      for (int d = 0; d < N_DIM; d++) {
        // ~ enforce position bounds
        scale = pow(duration, 1 - 0);
        for (int j = 0; j < num_order; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].p_lb[d];
          ubd[idx] = cubes[n].p_ub[d];
          accu_num_ineq++;
        }
        // ~ enforce velocity bounds
        scale = pow(duration, 1 - 1);
        for (int j = 0; j < num_order - 1; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = -N_DEG * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = N_DEG * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].v_lb[d];
          ubd[idx] = cubes[n].v_ub[d];
          accu_num_ineq++;
        }
        // ~ enforce acceleration bounds
        scale = pow(duration, 1 - 2);
        for (int j = 0; j < num_order - 2; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = -2.0 * N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 2);
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].a_lb[d];
          ubd[idx] = cubes[n].a_ub[d];
          accu_num_ineq++;
        }
      }
    }
  }

  // dummy constraints
  Eigen::VectorXd u = std::numeric_limits<double>::max() *
                      Eigen::VectorXd::Ones(total_num_vals);
  Eigen::VectorXd l = (-u.array()).matrix();

  // ~ Stage IV: solve
  Eigen::VectorXd x;
  x.setZero(total_num_vals);
  if (!OoQpItf::solve(Q, c, A, b, C, lbd, ubd, l, u, x, true, false)) {
    printf("[GetBezierSplineUsingCorridor]Solver error.\n");
    return kWrongStatus;
  }

  // std::cout << "solver result: " << x.transpose() << std::endl;
  // ~ Stage V: set back to bezier struct
  std::vector<decimal_t> vec_domain;
  vec_domain.push_back(cubes.front().t_lb);
  for (int n = 0; n < num_segments; n++) {
    vec_domain.push_back(cubes[n].t_ub);
  }
  bezier_spline->set_vec_domain(vec_domain);
  for (int n = 0; n < num_segments; n++) {
    for (int j = 0; j < num_order; j++) {
      Vecf<N_DIM> coeff;
      for (int d = 0; d < N_DIM; d++)
        coeff[d] = x[d * num_segments * num_order + n * num_order + j];
      bezier_spline->set_coeff(n, j, coeff);
    }
  }
  return kSuccess;
}

template <int N_DEG, int N_DIM>
ErrorType SplineGenerator<N_DEG, N_DIM>::GetBezierSplineUsingCorridor(
    const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
    const vec_E<Vecf<N_DIM>>& start_constraints,
    const vec_E<Vecf<N_DIM>>& end_constraints,
    BezierSplineType* bezier_spline) {
  int num_segments = static_cast<int>(cubes.size());
  int num_order = N_DEG + 1;
  int derivative_degree = 3;

  // ~ Stage I: stack objective
  int total_num_vals = N_DIM * num_segments * num_order;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Q(total_num_vals,
                                                 total_num_vals);
  Q.reserve(
      Eigen::VectorXi::Constant(N_DIM * num_segments * num_order, num_order));
  {
    MatNf<N_DEG + 1> hessian =
        BezierUtils<N_DEG>::GetBezierHessianMat(derivative_degree);
    int idx, idy;
    decimal_t val;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      for (int d = 0; d < N_DIM; d++) {
        for (int j = 0; j < num_order; j++) {
          for (int k = 0; k < num_order; k++) {
            idx = d * num_segments * num_order + n * num_order + j;
            idy = d * num_segments * num_order + n * num_order + k;
            val = hessian(j, k) / pow(duration, 2 * derivative_degree - 3);
            Q.insert(idx, idy) = val;
          }
        }
      }
    }
  }

  Eigen::VectorXd c;
  c.resize(total_num_vals);
  c.setZero();

  // ~ Stage II: stack equality constraints
  int num_continuity = 3;  // continuity up to jerk
  int num_connections = num_segments - 1;
  // printf("num conenctions: %d.\n", num_connections);
  int num_continuity_constraints = N_DIM * num_connections * num_continuity;
  int num_start_eq_constraints =
      static_cast<int>(start_constraints.size()) * N_DIM;
  int num_end_eq_constraints = static_cast<int>(end_constraints.size()) *
                               N_DIM;  // exclude s position constraint
  int total_num_eq_constraints = num_continuity_constraints +
                                 num_start_eq_constraints +
                                 num_end_eq_constraints;
  Eigen::SparseMatrix<double, Eigen::RowMajor> A(
      total_num_eq_constraints, N_DIM * num_segments * num_order);
  A.reserve(Eigen::VectorXi::Constant(total_num_eq_constraints, 2 * num_order));

  Eigen::VectorXd b;
  b.resize(total_num_eq_constraints);
  b.setZero();

  int idx, idy;
  decimal_t val;
  {
    // ~ continuity constraints
    for (int n = 0; n < num_connections; n++) {
      decimal_t duration_l = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t duration_r = cubes[n + 1].t_ub - cubes[n + 1].t_lb;
      for (int c = 0; c < num_continuity; c++) {
        decimal_t scale_l = pow(duration_l, 1 - c);
        decimal_t scale_r = pow(duration_r, 1 - c);
        for (int d = 0; d < N_DIM; d++) {
          idx = d * num_connections * num_continuity + n * num_continuity + c;
          if (c == 0) {
            // ~ position end
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ position begin
            idy = d * num_segments * num_order + (n + 1) * num_order + 0;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 1) {
            // ~ velocity end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ velocity begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 2) {
            // ~ acceleration end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ acceleration begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = -2.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          } else if (c == 3) {
            // ~ jerk end
            idy = d * num_segments * num_order + n * num_order + N_DEG - 3;
            val = -1.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -3.0 * scale_l;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale_l;
            A.insert(idx, idy) = val;
            // ~ jerk begin
            idy = d * num_segments * num_order + (n + 1) * num_order;
            val = -1.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 1;
            val = 3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 2;
            val = -3.0 * scale_r;
            A.insert(idx, idy) = -val;
            idy = d * num_segments * num_order + (n + 1) * num_order + 3;
            val = 1.0 * scale_r;
            A.insert(idx, idy) = -val;
          }
        }
      }
    }

    // ~ start state constraints
    {
      int num_order_constraint_start =
          static_cast<int>(start_constraints.size());
      decimal_t duration = cubes[0].t_ub - cubes[0].t_lb;
      decimal_t scale;
      int n = 0;
      for (int j = 0; j < num_order_constraint_start; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          idx = num_continuity_constraints + d * num_order_constraint_start + j;
          if (j == 0) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;

            b[idx] = start_constraints[j][d];
            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
          } else if (j == 1) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + 1;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;

            b[idx] = start_constraints[j][d];
            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
          } else if (j == 2) {
            idy = d * num_segments * num_order + n * num_order + 0;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            b[idx] = start_constraints[j][d];
            // printf("d: %d, j: %d, idx: %d, b: %lf.\n", d, j, idx, b[idx]);
          }
        }
      }
    }
    // ~ end state constraints
    {
      int num_order_constraint_end = static_cast<int>(end_constraints.size());
      decimal_t duration =
          cubes[num_segments - 1].t_ub - cubes[num_segments - 1].t_lb;
      decimal_t scale;
      int n = num_segments - 1;
      int accu_eq_cons_idx =
          num_continuity_constraints + num_start_eq_constraints;
      for (int j = 0; j < num_order_constraint_end; j++) {
        scale = pow(duration, 1 - j);
        for (int d = 0; d < N_DIM; d++) {
          // if (j == 0 && d == 0) continue;
          idx = accu_eq_cons_idx++;
          if (j == 0) {
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 1) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          } else if (j == 2) {
            idy = d * num_segments * num_order + n * num_order + N_DEG - 2;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + N_DEG - 1;
            val = -2.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;

            idy = d * num_segments * num_order + n * num_order + N_DEG;
            val = 1.0 * N_DEG * (N_DEG - 1) * scale;
            A.insert(idx, idy) = val;
            b[idx] = end_constraints[j][d];
          }
        }
      }
    }
  }

  // ~ Stage III: stack inequality constraints
  int total_num_ineq = 0;
  for (int i = 0; i < num_segments; i++) {
    total_num_ineq += (static_cast<int>(cubes[i].p_ub.size())) * num_order;
    total_num_ineq +=
        (static_cast<int>(cubes[i].v_ub.size())) * (num_order - 1);
    total_num_ineq +=
        (static_cast<int>(cubes[i].a_ub.size())) * (num_order - 2);
  }
  // Eigen::SparseMatrix<double, Eigen::RowMajor> C;
  Eigen::VectorXd lbd;
  Eigen::VectorXd ubd;
  Eigen::SparseMatrix<double, Eigen::RowMajor> C(total_num_ineq,
                                                 total_num_vals);
  C.reserve(Eigen::VectorXi::Constant(total_num_ineq, 3));
  lbd.setZero(total_num_ineq);
  ubd.setZero(total_num_ineq);
  {
    int accu_num_ineq = 0;
    for (int n = 0; n < num_segments; n++) {
      decimal_t duration = cubes[n].t_ub - cubes[n].t_lb;
      decimal_t scale;
      for (int d = 0; d < N_DIM; d++) {
        // ~ enforce position bounds
        scale = pow(duration, 1 - 0);
        for (int j = 0; j < num_order; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].p_lb[d];
          ubd[idx] = cubes[n].p_ub[d];
          accu_num_ineq++;
        }
        // ~ enforce velocity bounds
        scale = pow(duration, 1 - 1);
        for (int j = 0; j < num_order - 1; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = -N_DEG * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = N_DEG * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].v_lb[d];
          ubd[idx] = cubes[n].v_ub[d];
          accu_num_ineq++;
        }
        // ~ enforce acceleration bounds
        scale = pow(duration, 1 - 2);
        for (int j = 0; j < num_order - 2; j++) {
          idx = accu_num_ineq;
          idy = d * num_segments * num_order + n * num_order + j;
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 1);
          val = -2.0 * N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          idy = d * num_segments * num_order + n * num_order + (j + 2);
          val = N_DEG * (N_DEG - 1) * scale;
          C.insert(idx, idy) = val;
          lbd[idx] = cubes[n].a_lb[d];
          ubd[idx] = cubes[n].a_ub[d];
          accu_num_ineq++;
        }
      }
    }
  }

  // dummy constraints
  Eigen::VectorXd u = std::numeric_limits<double>::max() *
                      Eigen::VectorXd::Ones(total_num_vals);
  Eigen::VectorXd l = (-u.array()).matrix();

  // ~ Stage IV: solve
  Eigen::VectorXd x;
  x.setZero(total_num_vals);
  if (!OoQpItf::solve(Q, c, A, b, C, lbd, ubd, l, u, x, true, false)) {
    printf("[GetBezierSplineUsingCorridor]Solver error.\n");
    return kWrongStatus;
  }

  // std::cout << "solver result: " << x.transpose() << std::endl;
  // ~ Stage V: set back to bezier struct
  std::vector<decimal_t> vec_domain;
  vec_domain.push_back(cubes.front().t_lb);
  for (int n = 0; n < num_segments; n++) {
    vec_domain.push_back(cubes[n].t_ub);
  }
  bezier_spline->set_vec_domain(vec_domain);
  for (int n = 0; n < num_segments; n++) {
    for (int j = 0; j < num_order; j++) {
      Vecf<N_DIM> coeff;
      for (int d = 0; d < N_DIM; d++)
        coeff[d] = x[d * num_segments * num_order + n * num_order + j];
      bezier_spline->set_coeff(n, j, coeff);
    }
  }
  return kSuccess;
}

template class SplineGenerator<5, 2>;
template class SplineGenerator<5, 1>;

}  // namespace common