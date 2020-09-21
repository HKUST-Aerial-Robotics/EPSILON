#ifndef _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__
#define _COMMON_INC_COMMON_STATE_STATE_TRANSFORMER_H__

#include "common/basics/basics.h"
#include "common/basics/config.h"
#include "common/lane/lane.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"

namespace common {
class StateTransformer {
 public:
  StateTransformer() {}
  StateTransformer(const Lane& lane) { lane_ = lane; }

  ErrorType GetStateFromFrenetState(const FrenetState& fs, State* s) const;

  /**
   *  ~ note that this function may introduce approx 1cm error in vec position
   *  ~ due to the finite sampling strategy. the time consumed is about 0.03ms.
   */
  ErrorType GetFrenetStateFromState(const State& s, FrenetState* fs) const;

  ErrorType GetFrenetStateVectorFromStates(const vec_E<State> state_vec,
                                           vec_E<FrenetState>* fs_vec) const;

  ErrorType GetStateVectorFromFrenetStates(const vec_E<FrenetState>& fs_vec,
                                           vec_E<State>* state_vec) const;

  ErrorType GetFrenetPointFromPoint(const Vec2f& s, Vec2f* fs) const;

  ErrorType GetFrenetPointVectorFromPoints(const vec_E<Vec2f>& s,
                                           vec_E<Vec2f>* fs) const;

  bool IsValid() const { return lane_.IsValid(); }

  void print() {}

 private:
  Lane lane_;
};

}  // namespace common

#endif
