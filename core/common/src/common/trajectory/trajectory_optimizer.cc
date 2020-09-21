#include "common/trajectory/trajectory_optimizer.h"

#include "common/basics/tool_func.h"
#include "common/state/state.h"
// use ceres auto diff to optimize the trajectory
#include <nlopt.hpp>

namespace common {

double TrajectoryOptimizer::objectiveFunc1(const std::vector<double> &x,
                                           std::vector<double> &grad,
                                           void *func_data) {
  objectiveFunc1Data *data_ptr =
      reinterpret_cast<objectiveFunc1Data *>(func_data);
  double cost = 0.;
  data_ptr->traj_ptr->set_variables(x);

  // optimize frenet jerk
  decimal_t j_lon = 0.0, j_lat = 0.0;
  data_ptr->traj_ptr->Jerk(&j_lon, &j_lat);
  cost += (j_lon + j_lat);
  // enforce start state constraint
  common::State state;
  data_ptr->traj_ptr->GetState(data_ptr->init_state.time_stamp, &state);
  cost +=
      10.0 * ((data_ptr->init_state.vec_position - state.vec_position).norm() +
              pow(data_ptr->init_state.velocity - state.velocity, 2) +
              pow(data_ptr->init_state.acceleration - state.acceleration, 2) +
              pow(data_ptr->init_state.curvature - state.curvature, 2));
  data_ptr->traj_ptr->GetState(data_ptr->final_state.time_stamp, &state);
  cost +=
      0.1 * ((data_ptr->final_state.vec_position - state.vec_position).norm() +
             pow(data_ptr->final_state.velocity - state.velocity, 2));
  // optimize lateral acceleration
  for (auto &t : data_ptr->t_samples) {
    data_ptr->traj_ptr->GetState(t, &state);
    cost += 100.0 * pow(state.curvature * state.velocity * state.velocity, 2);
  }

  return cost;
}

ErrorType TrajectoryOptimizer::Optimize(FrenetTrajectory *traj_in) {
  if (traj_in == nullptr) return kWrongStatus;
  std::vector<decimal_t> variables = traj_in->variables();
  std::vector<decimal_t> variables_backup = traj_in->variables();

  for (auto &x : variables) {
    printf("x %lf.\n", x);
  }

  /* ---------------------  Optimizer ----------------------------*/
  std::vector<decimal_t> t_samples;
  objectiveFunc1Data data;
  data.traj_ptr = traj_in;
  traj_in->GetState(traj_in->begin(), &data.init_state);
  traj_in->GetState(traj_in->end(), &data.final_state);
  const decimal_t delta_t = 0.1;
  common::GetRangeVector<decimal_t>(traj_in->begin(),
                                    std::min(traj_in->end(), 2.0), delta_t,
                                    true, &data.t_samples);

  nlopt::opt nlopt_optimizer(nlopt::algorithm(nlopt::LN_COBYLA),
                             variables.size());
  nlopt_optimizer.set_min_objective(TrajectoryOptimizer::objectiveFunc1, &data);
  nlopt_optimizer.set_maxtime(0.02);
  double min_f;
  nlopt::result result = nlopt_optimizer.optimize(variables, min_f);
  /* ---------------------- END ------------------------------ */
  traj_in->set_variables(variables);
  return kSuccess;
}

}  // namespace common