/*
 * File: circle_arc.h
 * Author: ZHANG Lu (lzhangbz@connect.ust.hk)
 * Description:
 */

#ifndef _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_
#define _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include "common/basics/basics.h"

namespace common {

class CircleArc {
 public:
  CircleArc() {}
  CircleArc(const Vec3f &start_state, const double &curvature,
            const double &arc_length);
  ~CircleArc() {}

  inline double curvature() const { return curvature_; }
  inline double arc_length() const { return arc_length_; }
  inline double central_angle() const { return central_angle_; }

  inline Vec3f start_state() const { return start_state_; }
  inline Vec3f final_state() const { return final_state_; }
  inline Vec2f center() const { return center_; }
  // inline std::vector<Vec3f> sampled_states() const { return sampled_states_;
  // }

  double x_d0(const double s) const;
  double x_d1(const double s) const;
  double x_d2(const double s) const;

  double y_d0(const double s) const;
  double y_d1(const double s) const;
  double y_d2(const double s) const;

  double theta_d0(const double s) const;
  double theta_d1(const double s) const;

  // * Tangent
  double tx_d0(const double s) const { return x_d1(s); }
  double ty_d0(const double s) const { return y_d1(s); }

  // * Normal Vector (Left)
  double nx_d0(const double s) const { return -this->ty_d0(s); }
  double ny_d0(const double s) const { return this->tx_d0(s); }

  double x_offs_d0(const double s, const double offs) const;
  double x_offs_d1(const double s, const double offs) const;

  double y_offs_d0(const double s, const double offs) const;
  double y_offs_d1(const double s, const double offs) const;

  double theta_offs_d0(const double s, const double offs) const;

  void GetSampledStates(const double s_step,
                        std::vector<Vec3f> *p_sampled_states) const;

 private:
  Vec3f start_state_;
  Vec3f final_state_;

  double curvature_;
  double arc_length_;

  bool is_arc_ = true;
  double central_angle_;
  Vec2f center_;

  // for sampling
  // double sampled_arc_length_step_ = 0.2;
  // double sampled_central_angle_step_;
  // std::vector<Vec3f> sampled_states_;
};  // CircleArc

}  // namespace common

#endif  // _CORE_COMMON_INC_COMMON_CIRCLE_ARC_H_