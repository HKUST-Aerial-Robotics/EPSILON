#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_FRENET_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/frenet_state.h"
#include "common/state/state.h"
#include "common/trajectory/trajectory.h"

namespace common {

class FrenetTrajectory : public Trajectory {
 public:
  virtual ~FrenetTrajectory() = default;
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;
  virtual ErrorType GetFrenetState(const decimal_t& t,
                                   FrenetState* fs) const = 0;
  virtual decimal_t begin() const = 0;
  virtual decimal_t end() const = 0;
  virtual bool IsValid() const = 0;
  virtual std::vector<decimal_t> variables()
      const = 0;  // return a copy of variables
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
  // * Frenet interfaces
  virtual void Jerk(decimal_t* j_lon, decimal_t* j_lat) const = 0;
};

}  // namespace common

#endif