#ifndef _COMMON_INC_COMMON_INTERFACE_PLANNER_H__
#define _COMMON_INC_COMMON_INTERFACE_PLANNER_H__

#include <string>

#include "common/basics/basics.h"

namespace planning {
/**
 * @brief A general base class for different planners including
 * path/motion/behavior planners
 */
class Planner {
 public:
  Planner() = default;

  virtual ~Planner() = default;

  virtual std::string Name() = 0;

  virtual ErrorType Init(const std::string config) = 0;

  virtual ErrorType RunOnce() = 0;
};

}  // namespace planning

#endif