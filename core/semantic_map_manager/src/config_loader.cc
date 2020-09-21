#include "semantic_map_manager/config_loader.h"

namespace semantic_map_manager {

using Json = nlohmann::json;

ErrorType ConfigLoader::ParseAgentConfig(AgentConfigInfo *p_agent_config) {
  printf("\n[ConfigLoader] Loading vehicle set\n");

  std::fstream fs(agent_config_path_);
  Json root;
  fs >> root;

  Json agent_config_json = root["agent_config"];
  int num = static_cast<int>(agent_config_json["info"].size());
  for (int i = 0; i < num; ++i) {
    Json agent = agent_config_json["info"][i];
    if (agent["id"].get<int>() != ego_id_) continue;
    p_agent_config->obstacle_map_meta_info = common::GridMapMetaInfo(
        agent["obstacle_map_meta_info"]["width"].get<double>(),
        agent["obstacle_map_meta_info"]["height"].get<double>(),
        agent["obstacle_map_meta_info"]["resolution"].get<double>());
    p_agent_config->surrounding_search_radius =
        agent["surrounding_search_radius"].get<double>();
    p_agent_config->enable_openloop_prediction =
        agent["enable_openloop_prediction"].get<bool>();
    if (agent.count("enable_tracking_noise")) {
      p_agent_config->enable_tracking_noise =
          agent["enable_tracking_noise"].get<bool>();
    }
    if (agent.count("enable_log")) {
      p_agent_config->enable_log = agent["enable_log"].get<bool>();
      p_agent_config->log_file = agent["log_file"].get<std::string>();
    }
  }

  p_agent_config->PrintInfo();
  fs.close();
  return kSuccess;
}

}  // namespace semantic_map_manager