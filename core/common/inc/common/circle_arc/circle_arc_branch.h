/*
 * File: circle_arc_branch.h
 * Author: ZHANG Lu (lzhangbz@connect.ust.hk)
 * Description:
 */

#ifndef _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_
#define _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"
#include "common/circle_arc/circle_arc.h"

namespace common {

class CircleArcBranch {
 public:
  CircleArcBranch();

  CircleArcBranch(const Vec3f &start_state,
                  const std::vector<double> &curvature_vec,
                  const std::vector<double> length_vec)
      : start_state_(start_state),
        curvature_vec_(curvature_vec),
        length_vec_(length_vec) {
    CalculateCircleArcBranch();
  }

  ~CircleArcBranch() {}

  inline Vec3f start_state() const { return start_state_; }
  inline std::vector<double> curvature_vec() const { return curvature_vec_; }
  inline std::vector<double> length_vec() const { return length_vec_; }
  inline std::vector<CircleArc> circle_arc_vec() const {
    return circle_arc_vec_;
  }

  /**
   * @brief Return all final states of circle arcs
   * @param states All final states
   */
  void RetFinalStates(std::vector<Vec3f> *p_states) const;

  /**
   * @brief Return all sampled states in circle arc branch
   * @param states All sampled states
   */
  void RetAllSampledStates(std::vector<Vec3f> *p_states) const;

 private:
  /**
   * @brief Calculate circle arc branch using a initial state
   * @param state Initial state
   */
  void CalculateCircleArcBranch();

  Vec3f start_state_;
  std::vector<double> curvature_vec_;
  std::vector<double> length_vec_;
  std::vector<CircleArc> circle_arc_vec_;
};

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_CIRCLE_ARC_BRANCH_H_