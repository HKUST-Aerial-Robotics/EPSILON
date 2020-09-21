#ifndef _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__
#define _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__

#include <vector>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/math/calculations.h"
#include "common/solver/qp_solver.h"
#include "common/spline/bezier.h"
#include "common/spline/lookup_table.h"
#include "common/spline/polynomial.h"
#include "common/spline/spline.h"
#include "common/state/state.h"
#include "common/state/waypoint.h"
#include "tk_spline/spline.h"

namespace common {

template <int N_DEG, int N_DIM>
class SplineGenerator {
 public:
  typedef Spline<N_DEG, N_DIM> SplineType;
  typedef BezierSpline<N_DEG, N_DIM> BezierSplineType;

  /**
   * @brief Return natural (cubic) spline fitting with respect to
   * parameterization
   * @param samples, the points you want to interterpolate
   * @param parameterization, the parameterization used to evaluate the spline
   * @param spline, the spline returned (wrap in the general N_DEG spline)
   */
  static ErrorType GetCubicSplineBySampleInterpolation(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      SplineType* spline);

  /**
   * @brief Return optimal spline fitting with continuity constraints
   * @param samples, the points you want to fit
   * @param para, the parameterization, typically arc length
   * @param breaks, break points (represent the segments of the spline)
   * @param spline, the spline returned (wrap in the general N_DEG spline)
   * @note this may be an expensive function depending on the number of breaks
   */
  static ErrorType GetQuinticSplineBySampleFitting(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      const Eigen::ArrayXf& breaks, const decimal_t regulator,
      SplineType* spline);

  /**
   * @brief Return waypoints from position samples
   * @param samples, the points you want to interterpolate
   * @param parameterization, the parameterization used to evaluate the spline
   * @param waypoints, output the waypoints
   */
  static ErrorType GetWaypointsFromPositionSamples(
      const vec_Vecf<N_DIM>& samples, const std::vector<decimal_t>& para,
      vec_E<Waypoint<N_DIM>>* waypoints);

  /**
   * @brief Return spline from state vec
   * @param state_vec, the vector of states
   * @param para, the parameterization set
   * @param spline, spline returned
   */
  static ErrorType GetSplineFromStateVec(const std::vector<decimal_t>& para,
                                         const vec_E<State>& state_vec,
                                         SplineType* spline);

  /**
   * @brief Return spline from state vec
   * @param state_vec, the vector of free states
   * @param para, the parameterization set
   * @param spline, spline returned
   */
  static ErrorType GetSplineFromFreeStateVec(
      const std::vector<decimal_t>& para,
      const vec_E<FreeState>& free_state_vec, SplineType* spline);

  /**
   * @brief Return the optimal bezier curve in corridor
   * @note typical usage parameterization is 2D-t, the corridor is 3D (2D+t)
   * @param start_constraints, pos, vel, acc, etc constraints at start point
   * @param end_constraints, pos, vel, acc, etc constrainst at end point
   */

  static ErrorType GetBezierSplineUsingCorridor(
      const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const vec_E<Vecf<N_DIM>>& start_constraints,
      const vec_E<Vecf<N_DIM>>& end_constraints,
      const std::vector<decimal_t>& ref_stamps,
      const vec_E<Vecf<N_DIM>>& ref_points, const decimal_t& weight_proximity,
      BezierSplineType* bezier_spline);

  static ErrorType GetBezierSplineUsingCorridor(
      const vec_E<SpatioTemporalSemanticCubeNd<N_DIM>>& cubes,
      const vec_E<Vecf<N_DIM>>& start_constraints,
      const vec_E<Vecf<N_DIM>>& end_constraints,
      BezierSplineType* bezier_spline);

};  // class spline generator

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_SPLINE_GENERATOR_H__