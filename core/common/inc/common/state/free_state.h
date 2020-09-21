#ifndef _COMMON_INC_COMMON_STATE_FREE_STATE_H__
#define _COMMON_INC_COMMON_STATE_FREE_STATE_H__

#include "common/basics/basics.h"
#include "common/state/state.h"

#include <math.h>
namespace common {
struct FreeState {
  decimal_t time_stamp{0.0};
  Vecf<2> position{Vecf<2>::Zero()};
  Vecf<2> velocity{Vecf<2>::Zero()};
  Vecf<2> acceleration{Vecf<2>::Zero()};
  decimal_t angle{0.0};

  void print() const {
    printf("position: (%lf, %lf).\n", position[0], position[1]);
    printf("velocity: (%lf, %lf).\n", velocity[0], velocity[1]);
    printf("acceleration: (%lf, %lf).\n", acceleration[0], acceleration[1]);
    printf("angle: %lf.\n", angle);
  }
};

void GetFreeStateFromState(const State& state, FreeState* free_state);
void GetStateFromFreeState(const FreeState& free_state, State* state);
}  // namespace common

#endif  // _COMMON_INC_COMMON_STATE_FREE_STATE_H__