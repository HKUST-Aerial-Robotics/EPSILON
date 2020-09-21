#ifndef _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__
#define _CORE_COMMON_INC_COMMON_TRAJECTORY_TRAJECTORY_H__

#include "common/basics/config.h"
#include "common/state/state.h"

namespace common {

class Trajectory {
 public:
  virtual ~Trajectory() = default;
  virtual ErrorType GetState(const decimal_t& t, State* state) const = 0;
  virtual decimal_t begin() const = 0;
  virtual decimal_t end() const = 0;
  virtual bool IsValid() const = 0;
  // optimization-related interface
  virtual std::vector<decimal_t> variables() const = 0;  // return a copy of variables
  virtual void set_variables(const std::vector<decimal_t>& variables) = 0;
};

}  // namespace common

#endif