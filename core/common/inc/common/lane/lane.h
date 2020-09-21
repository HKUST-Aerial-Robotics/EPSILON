#ifndef _CORE_COMMON_INC_COMMON_LANE_LANE_H__
#define _CORE_COMMON_INC_COMMON_LANE_LANE_H__

#include "common/basics/config.h"
#include "common/spline/spline.h"
#include "common/state/state.h"

namespace common {
class Lane {
 public:
  typedef Spline<LaneDegree, LaneDim> SplineType;
  // typedef SplineGenerator<LaneDegree, LaneDim> SplineGeneratorType;

  Lane() {}
  Lane(const SplineType& position_spline)
      : position_spline_(position_spline), is_valid_(true) {}
  bool IsValid() const { return is_valid_; }

  /**
   * @brief Set the parameterization of the lane
   * @param position spline, position spline with required degree
   */
  void set_position_spline(const SplineType& position_spline) {
    if (position_spline.vec_domain().empty()) return;
    position_spline_ = position_spline;
    is_valid_ = true;
  }

  /**
   * @brief Get curvature by arc length
   * @param arc_length, evaluation arc length
   * @param curvature, curvature returned
   * @param curvature, curvature derivative
   */
  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature,
                                    decimal_t* curvature_derivative) const;

  ErrorType GetCurvatureByArcLength(const decimal_t& arc_length,
                                    decimal_t* curvature) const;

  /**
   * @brief Get derivative by arc length (d = 0 means position evaluation)
   * @param d, derivative to take
   * @param derivative, derivative returned
   */
  ErrorType GetDerivativeByArcLength(const decimal_t arc_length, const int d,
                                     Vecf<LaneDim>* derivative) const;

  ErrorType GetPositionByArcLength(const decimal_t arc_length,
                                   Vecf<LaneDim>* derivative) const;

  ErrorType GetTangentVectorByArcLength(const decimal_t arc_length,
                                        Vecf<LaneDim>* tangent_vector) const;

  ErrorType GetNormalVectorByArcLength(const decimal_t arc_length,
                                       Vecf<LaneDim>* normal_vector) const;

  ErrorType GetOrientationByArcLength(const decimal_t arc_length,
                                      decimal_t* angle) const;

  ErrorType GetArcLengthByVecPosition(const Vecf<LaneDim>& vec_position,
                                      decimal_t* arc_length) const;

  ErrorType GetArcLengthByVecPositionWithInitialGuess(
      const Vecf<LaneDim>& vec_position, const decimal_t& initial_guess,
      decimal_t* arc_length) const;

  // ErrorType GetArcLengthByVecPositionUsingBinarySearch(
  //     const Vecf<LaneDim>& vec_position, decimal_t* arc_length) const;

  ErrorType CheckInputArcLength(const decimal_t arc_length) const;

  SplineType position_spline() const { return position_spline_; }

  decimal_t begin() const { return position_spline_.begin(); }

  decimal_t end() const { return position_spline_.end(); }

  void print() const { position_spline_.print(); }

 private:
  SplineType position_spline_;
  bool is_valid_ = false;
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_LANE_LANE_H__