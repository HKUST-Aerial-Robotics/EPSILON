#include "common/lane/lane.h"

namespace common {

ErrorType Lane::GetCurvatureByArcLength(const decimal_t& arc_length,
                                        decimal_t* curvature,
                                        decimal_t* curvature_derivative) const {
  if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel, acc, jrk;
  GetDerivativeByArcLength(arc_length, 1, &vel);
  // printf("vel: (%lf, %lf) at arc length %lf.\n", vel[0], vel[1], arc_length);
  GetDerivativeByArcLength(arc_length, 2, &acc);
  // printf("acc: (%lf, %lf) at arc length %lf.\n", acc[0], acc[1], arc_length);
  GetDerivativeByArcLength(arc_length, 3, &jrk);
  // printf("jrk: (%lf, %lf) at arc length %lf.\n", jrk[0], jrk[1], arc_length);

  decimal_t c0 = vel[0] * acc[1] - vel[1] * acc[0];
  decimal_t c1 = vel.norm();
  *curvature = c0 / (c1 * c1 * c1);
  *curvature_derivative =
      ((acc[0] * acc[1] + vel[0] * jrk[1] - acc[1] * acc[0] - vel[1] * jrk[0]) /
           c1 * c1 * c1 -
       3 * c0 * (vel[0] * acc[0] + vel[1] * acc[1]) / (c1 * c1 * c1 * c1 * c1));
  return kSuccess;
}

ErrorType Lane::GetCurvatureByArcLength(const decimal_t& arc_length,
                                        decimal_t* curvature) const {
  if (CheckInputArcLength(arc_length) != kSuccess || LaneDim != 2) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel, acc;
  GetDerivativeByArcLength(arc_length, 1, &vel);
  GetDerivativeByArcLength(arc_length, 2, &acc);

  decimal_t c0 = vel[0] * acc[1] - vel[1] * acc[0];
  decimal_t c1 = vel.norm();
  *curvature = c0 / (c1 * c1 * c1);
  return kSuccess;
}

ErrorType Lane::GetDerivativeByArcLength(const decimal_t arc_length,
                                         const int d,
                                         Vecf<LaneDim>* derivative) const {
  return position_spline_.evaluate(arc_length, d, derivative);
}

ErrorType Lane::GetPositionByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* derivative) const {
  return position_spline_.evaluate(arc_length, derivative);
}

ErrorType Lane::GetTangentVectorByArcLength(
    const decimal_t arc_length, Vecf<LaneDim>* tangent_vector) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  *tangent_vector = vel / vel.norm();
  return kSuccess;
}

ErrorType Lane::GetNormalVectorByArcLength(const decimal_t arc_length,
                                           Vecf<LaneDim>* normal_vector) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  Vecf<LaneDim> tangent_vector = vel / vel.norm();
  *normal_vector = rotate_vector_2d(tangent_vector, M_PI / 2.0);
  return kSuccess;
}

ErrorType Lane::GetOrientationByArcLength(const decimal_t arc_length,
                                          decimal_t* angle) const {
  if (CheckInputArcLength(arc_length) != kSuccess) {
    return kIllegalInput;
  }

  Vecf<LaneDim> vel;
  GetDerivativeByArcLength(arc_length, 1, &vel);

  if (vel.norm() < kEPS) {
    return kWrongStatus;
  }

  Vecf<LaneDim> tangent_vector = vel / vel.norm();
  *angle = vec2d_to_angle(tangent_vector);
  return kSuccess;
}

ErrorType Lane::GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                          decimal_t* arc_length) const {
  if (!IsValid()) {
    return kWrongStatus;
  }

  static constexpr int kMaxCnt = 4;
  static constexpr decimal_t kMaxDistSquare = 900.0;

  const decimal_t val_lb = position_spline_.begin();
  const decimal_t val_ub = position_spline_.end();
  decimal_t step = (val_ub - val_lb) * 0.5;

  decimal_t s1 = val_lb;
  decimal_t s2 = val_lb + step;
  decimal_t s3 = val_ub;
  decimal_t initial_guess = s2;

  // printf("[XXX]s1 = %lf, s2 = %lf, s3 = %lf\n", s1, s2, s3);

  Vecf<LaneDim> start_pos, mid_pos, final_pos;
  position_spline_.evaluate(s1, &start_pos);
  position_spline_.evaluate(s2, &mid_pos);
  position_spline_.evaluate(s3, &final_pos);

  // ~ Step I: use binary search to find a initial guess
  decimal_t d1 = (start_pos - vec_position).squaredNorm();
  decimal_t d2 = (mid_pos - vec_position).squaredNorm();
  decimal_t d3 = (final_pos - vec_position).squaredNorm();

  for (int i = 0; i < kMaxCnt; ++i) {
    // printf("[XXX] - it = %d - s1 = %lf, s2 = %lf, s3 = %lf\n", i, s1, s2,
    // s3);
    decimal_t min_dis = std::min(std::min(d1, d2), d3);
    if (min_dis < kMaxDistSquare) {
      if (min_dis == d1) {
        initial_guess = s1;
      } else if (min_dis == d2) {
        initial_guess = s2;
      } else if (min_dis == d3) {
        initial_guess = s3;
      } else {
        assert(false);
      }
      break;
    }
    step *= 0.5;
    if (min_dis == d1) {
      initial_guess = s1;
      s3 = s2;
      s2 = s1 + step;
      position_spline_.evaluate(s2, &mid_pos);
      position_spline_.evaluate(s3, &final_pos);
      d2 = (mid_pos - vec_position).squaredNorm();
      d3 = (final_pos - vec_position).squaredNorm();
    } else if (min_dis == d2) {
      initial_guess = s2;
      s1 = s2 - step;
      s3 = s2 + step;
      position_spline_.evaluate(s1, &start_pos);
      position_spline_.evaluate(s3, &final_pos);
      d1 = (start_pos - vec_position).squaredNorm();
      d3 = (final_pos - vec_position).squaredNorm();
    } else if (min_dis == d3) {
      initial_guess = s3;
      s1 = s2;
      s2 = s3 - step;
      position_spline_.evaluate(s1, &start_pos);
      position_spline_.evaluate(s2, &mid_pos);
      d1 = (start_pos - vec_position).squaredNorm();
      d2 = (mid_pos - vec_position).squaredNorm();
    } else {
      printf(
          "[Lane]GetArcLengthByVecPosition - d1: %lf, d2: %lf, d3: %lf, "
          "min_dis: %lf\n",
          d1, d2, d3, min_dis);
      assert(false);
    }
  }

  // printf("[XXX]initial_guess = %lf\n", initial_guess);

  // ~ Step II: use Newton's method to find the local minimum
  GetArcLengthByVecPositionWithInitialGuess(vec_position, initial_guess,
                                            arc_length);

  return kSuccess;
}

ErrorType Lane::GetArcLengthByVecPositionWithInitialGuess(
    const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
    decimal_t* arc_length) const {
  if (!IsValid()) {
    return kWrongStatus;
  }

  const decimal_t val_lb = position_spline_.begin();
  const decimal_t val_ub = position_spline_.end();

  // ~ use Newton's method to find the local minimum
  static constexpr decimal_t epsilon = 1e-3;
  static constexpr int kMaxIter = 8;
  decimal_t x = std::min(std::max(initial_guess, val_lb), val_ub);
  Vecf<LaneDim> p, dp, ddp, tmp_vec;

  for (int i = 0; i < kMaxIter; ++i) {
    position_spline_.evaluate(x, 0, &p);
    position_spline_.evaluate(x, 1, &dp);
    position_spline_.evaluate(x, 2, &ddp);

    tmp_vec = p - vec_position;
    double f_1 = tmp_vec.dot(dp);
    double f_2 = dp.dot(dp) + tmp_vec.dot(ddp);
    double dx = -f_1 / f_2;

    if (std::fabs(dx) < epsilon) {
      break;
    }

    if (x + dx > val_ub) {
      x = val_ub;
      // printf(
      //     "[Lane]GetArcLengthByVecPosition - Out of range, use upper
      //     bound\n");
      break;
    } else if (x + dx < val_lb) {
      x = val_lb;
      // printf(
      //     "[Lane]GetArcLengthByVecPosition - Out of range, use lower
      //     bound\n");
      break;
    }

    x += dx;
  }

  *arc_length = x;

  return kSuccess;
}

// ErrorType Lane::GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
//                                           decimal_t* arc_length) const {
//   if (!IsValid()) {
//     return kWrongStatus;
//   }

//   std::vector<decimal_t> vec_domain = position_spline_.vec_domain();
//   int num_pts = static_cast<int>(vec_domain.size());

//   decimal_t min_dis = kInf;
//   decimal_t min_s = 0.0;
//   int idx = 0;
//   decimal_t dis;
//   for (int i = 0; i < num_pts; i++) {
//     Vecf<LaneDim> vec{Vecf<LaneDim>::Zero()};
//     position_spline_.evaluate(vec_domain[i], &vec);
//     dis = (vec - vec_position).norm();
//     if (dis < min_dis) {
//       idx = i;
//       min_dis = dis;
//       min_s = vec_domain[i];
//     }
//   }

//   int low_idx = std::max(0, idx - 1);
//   int up_idx = std::min(idx + 1, num_pts - 1);

//   const decimal_t step = 0.05;  // 0.05-->2cm error, 0.01-->less than 1 cm
//   error

//   // ~ search forward
//   decimal_t min_dis_forward = min_dis;
//   decimal_t min_s_forward = min_s;
//   for (decimal_t s = vec_domain[idx]; s < vec_domain[up_idx]; s += step) {
//     Vecf<LaneDim> vec{Vecf<LaneDim>::Zero()};
//     position_spline_.evaluate(s, &vec);
//     dis = (vec - vec_position).norm();
//     if (dis < min_dis_forward + kEPS) {
//       min_dis_forward = dis;
//       min_s_forward = s;
//     } else {
//       break;
//     }
//   }

//   decimal_t min_dis_backward = min_dis;
//   decimal_t min_s_backward = min_s;
//   for (decimal_t s = vec_domain[idx]; s > vec_domain[low_idx]; s -= step) {
//     Vecf<LaneDim> vec{Vecf<LaneDim>::Zero()};
//     position_spline_.evaluate(s, &vec);
//     dis = (vec - vec_position).norm();
//     if (dis < min_dis_backward + kEPS) {
//       min_dis_backward = dis;
//       min_s_backward = s;
//     } else {
//       // not make progress any more, break
//       break;
//     }
//   }

//   *arc_length =
//       min_dis_forward < min_dis_backward ? min_s_forward : min_s_backward;
//   return kSuccess;
// }

ErrorType Lane::CheckInputArcLength(const decimal_t arc_length) const {
  if (!IsValid()) {
    printf("[CheckInputArcLength]Quering invalid lane.\n");
    return kWrongStatus;
  }
  if (arc_length < position_spline_.begin() - kEPS ||
      arc_length > position_spline_.end() + kEPS) {
    // printf("[CheckInputArcLength]Out of domain error: %lf --> (%lf, %lf).\n",
    //        arc_length, position_spline_.begin(), position_spline_.end());
    return kIllegalInput;
  }
  return kSuccess;
}

}  // namespace common