#ifndef _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_
#define _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_

#include <assert.h>
#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <json/json.hpp>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"
#include "common/state/free_state.h"
#include "common/state/state.h"
#include "semantic_map_manager/basics.h"

namespace semantic_map_manager {

class ConfigLoader {
 public:
  ConfigLoader() {}
  ConfigLoader(const std::string &agent_config_path)
      : agent_config_path_(agent_config_path) {}
  ~ConfigLoader() {}

  inline std::string agent_config_path() const { return agent_config_path_; }
  inline void set_agent_config_path(const std::string &path) {
    agent_config_path_ = path;
  };

  inline int ego_id() const { return ego_id_; }
  inline void set_ego_id(const int &id) { ego_id_ = id; }

  ErrorType ParseAgentConfig(AgentConfigInfo *p_agent_config);

 private:
  int ego_id_;
  std::string agent_config_path_;
};

}  // namespace semantic_map_manager

#endif  // _CORE_SEMANTIC_MAP_INC_SEMANTIC_MAP_MANAGER_CONFIG_LOADER_H_