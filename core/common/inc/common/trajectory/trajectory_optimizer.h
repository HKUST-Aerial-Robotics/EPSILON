#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_OPTIMIZER_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_OPTIMIZER_H__

#include "common/state/state.h"
#include "common/trajectory/frenet_traj.h"
#include "common/trajectory/trajectory.h"

namespace common {

class TrajectoryOptimizer {
 public:
  struct objectiveFunc1Data {
    FrenetTrajectory *traj_ptr;
    common::State init_state;
    common::State final_state;
    std::vector<decimal_t> t_samples;
  };

  static ErrorType Optimize(FrenetTrajectory *traj_in);

  static double objectiveFunc1(const std::vector<double> &x,
                               std::vector<double> &grad, void *func_data);

 private:
};

}  // namespace common

#endif