#include "common/circle_arc/circle_arc.h"

namespace common {
CircleArc::CircleArc(const Vec3f &start_state, const double &curvature,
                     const double &arc_length)
    : start_state_(start_state),
      curvature_(curvature),
      arc_length_(arc_length) {
  if (curvature_ != 0.0) {
    is_arc_ = true;

    central_angle_ = curvature_ * arc_length_;
    double r = 1.0 / curvature_;

    // Calculate center point
    center_(0) = start_state_(0) - r * sin(start_state_(2));
    center_(1) = start_state_(1) + r * cos(start_state_(2));

    // Calculate final state
    final_state_(0) = center_(0) + r * sin(start_state_(2) + central_angle_);
    final_state_(1) = center_(1) - r * cos(start_state_(2) + central_angle_);
    final_state_(2) = start_state_(2) + central_angle_;

  } else {
    is_arc_ = false;
    // Calculate final state
    final_state_(0) = start_state_(0) + arc_length_ * cos(start_state_(2));
    final_state_(1) = start_state_(1) + arc_length_ * sin(start_state_(2));
    final_state_(2) = start_state_(2);
  }
}

double CircleArc::x_d0(const double s) const {
  if (is_arc_) {
    double r = 1 / curvature_;
    return center_(0) + r * sin(start_state_(2) + s / r);
  } else {
    return start_state_(0) + s * cos(start_state_(2));
  }
}

double CircleArc::x_d1(const double s) const {
  if (is_arc_) {
    return cos(start_state_(2) + s * curvature_);
  } else {
    return cos(start_state_(2));
  }
}

double CircleArc::x_d2(const double s) const {
  if (is_arc_) {
    return -curvature_ * sin(start_state_(2) + s * curvature_);
  } else {
    return 0;
  }
}

double CircleArc::y_d0(const double s) const {
  if (is_arc_) {
    double r = 1 / curvature_;
    return center_(1) - r * cos(start_state_(2) + s / r);
  } else {
    return start_state_(1) + s * sin(start_state_(2));
  }
}

double CircleArc::y_d1(const double s) const {
  if (is_arc_) {
    return sin(start_state_(2) + s * curvature_);
  } else {
    return sin(start_state_(2));
  }
}

double CircleArc::y_d2(const double s) const {
  if (is_arc_) {
    return curvature_ * cos(start_state_(2) + s * curvature_);
  } else {
    return 0;
  }
}

double CircleArc::theta_d0(const double s) const {
  if (is_arc_) {
    return start_state_(2) + s * curvature_;
  } else {
    return start_state_(2);
  }
}

double CircleArc::theta_d1(const double s) const {
  if (is_arc_) {
    return curvature_;
  } else {
    return 0;
  }
}

double CircleArc::x_offs_d0(const double s, const double offs) const {
  return x_d0(s) + offs * nx_d0(s);
}

double CircleArc::y_offs_d0(const double s, const double offs) const {
  return y_d0(s) + offs * ny_d0(s);
}

double CircleArc::theta_offs_d0(const double s, const double offs) const {
  return theta_d0(s);
}

void CircleArc::GetSampledStates(const double s_step,
                                 std::vector<Vec3f> *p_sampled_states) const {
  // Calculate sampled state for straight line
  for (double length = 0.0; fabs(length) < fabs(arc_length_);
       length += s_step) {
    Vec3f sampled_state;
    sampled_state(0) = x_d0(length);
    sampled_state(1) = y_d0(length);
    sampled_state(2) = theta_d0(length);
    p_sampled_states->emplace_back(sampled_state);
  }
}

}  // namespace common
