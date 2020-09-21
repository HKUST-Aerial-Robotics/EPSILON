#include "common/state/free_state.h"

#include <math.h>
namespace common {

void GetFreeStateFromState(const State& state, FreeState* free_state) {
  free_state->position = state.vec_position;
  decimal_t cn = cos(state.angle);
  decimal_t sn = sin(state.angle);
  free_state->velocity[0] = state.velocity * cn;
  free_state->velocity[1] = state.velocity * sn;
  decimal_t normal_acc = state.velocity * state.velocity * state.curvature;
  free_state->acceleration[0] = state.acceleration * cn - normal_acc * sn;
  free_state->acceleration[1] = state.acceleration * sn + normal_acc * cn;
  free_state->angle = state.angle;
  free_state->time_stamp = state.time_stamp;
}

void GetStateFromFreeState(const FreeState& free_state, State* state) {
  state->angle = free_state.angle;
  state->vec_position = free_state.position;
  state->velocity = free_state.velocity.norm();
  decimal_t cn = cos(state->angle);
  decimal_t sn = sin(state->angle);
  Vecf<2> tangent_vec{Vecf<2>(cn, sn)};
  Vecf<2> normal_vec{Vecf<2>(-sn, cn)};
  auto a_tangent = free_state.acceleration.dot(tangent_vec);
  auto a_normal = free_state.acceleration.dot(normal_vec);
  state->acceleration = a_tangent;
  if (fabs(state->velocity) > kBigEPS) {
    state->curvature = a_normal / pow(state->velocity, 2);
  } else {
    state->curvature = 0.0;
  }
  state->time_stamp = free_state.time_stamp;
}

}  // namespace common