#ifndef _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_
#define _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_

#include "common/basics/semantics.h"
#include "common/lane/lane.h"
#include "common/state/state.h"
#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

class OnLaneFsPredictor {
 public:
  using Lane = common::Lane;
  using State = common::State;
  using VehicleControlSignal = common::VehicleControlSignal;
  using Vehicle = common::Vehicle;

  OnLaneFsPredictor() {}
  ~OnLaneFsPredictor();

  static ErrorType GetPredictedTrajectory(const Lane& lane,
                                          const Vehicle& vehicle,
                                          const decimal_t& t_pred,
                                          const decimal_t& t_step,
                                          vec_E<State>* pred_states) {
    pred_states->clear();
    int num_step = std::round(t_pred / t_step);
    State desired_state;
    decimal_t desired_vel = vehicle.state().velocity;
    planning::OnLaneForwardSimulation::Param sim_param;
    sim_param.idm_param.kDesiredVelocity = desired_vel;
    pred_states->push_back(vehicle.state());  // insert the first state
    common::Vehicle v_in = vehicle;
    common::StateTransformer stf = common::StateTransformer(lane);
    for (int i = 0; i < num_step; ++i) {
      if (lane.IsValid()) {
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                stf, v_in, common::Vehicle(), t_step, sim_param,
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      } else {
        if (planning::OnLaneForwardSimulation::PropagateOnce(
                desired_vel, v_in, t_step,
                planning::OnLaneForwardSimulation::Param(),
                &desired_state) != kSuccess) {
          return kWrongStatus;
        }
      }
      pred_states->push_back(desired_state);
      v_in.set_state(desired_state);
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif  // _CORE_MOTION_PREDICTOR_INC_ONLANE_FORWARD_SIMULATION_PREDICTOR_H_