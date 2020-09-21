#ifndef _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__
#define _CORE_FORWARD_SIMULATOR_MULTIMODAL_FORWARD_H__

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/lane/lane.h"

#include "forward_simulator/onlane_forward_simulation.h"

namespace planning {

class MultiModalForward {
 public:
  using Lane = common::Lane;
  using VehicleSet = common::VehicleSet;
  using GridMap = common::GridMapND<uint8_t, 2>;
  using State = common::State;
  typedef int AggressivenessLevel;

  static ErrorType ParamLookUp(const AggressivenessLevel& agg_level,
                               OnLaneForwardSimulation::Param* param) {
    switch (agg_level) {
      case 1:
        param->idm_param.kDesiredHeadwayTime = 2.0;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.0;
        param->steer_control_gain = 2.0;
        break;
      case 2:
        param->idm_param.kDesiredHeadwayTime = 1.7;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 1.0;
        param->idm_param.kComfortableBrakingDeceleration = 1.67;
        param->steer_control_gain = 2.0;
        break;
      case 3:
        param->idm_param.kDesiredHeadwayTime = 1.5;
        param->idm_param.kMinimumSpacing = 2.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 4:
        param->idm_param.kDesiredHeadwayTime = 1.0;
        param->idm_param.kMinimumSpacing = 1.5;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      case 5:
        param->idm_param.kDesiredHeadwayTime = 0.5;
        param->idm_param.kMinimumSpacing = 1.0;
        param->idm_param.kAcceleration = 2.0;
        param->idm_param.kComfortableBrakingDeceleration = 3.0;
        param->steer_control_gain = 2.0;
        break;
      default:
        assert(false);
        break;
    }
    return kSuccess;
  }

 private:
};

}  // namespace planning

#endif