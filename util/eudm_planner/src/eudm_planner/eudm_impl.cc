// * externals
#include "eudm_planner/eudm_itf.h"
// * internals
#include <glog/logging.h>
#include <ros/ros.h>

#include <vector>

#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "hkust_msg_transformer/hkust_decoder.h"
#include "semantic_map_manager/semantic_map_manager.h"

namespace planning {

namespace eudm {

class EudmImpl : public EudmItf {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;
  using DcpLatAction = planning::DcpTree::DcpLatAction;
  using DcpLonAction = planning::DcpTree::DcpLonAction;
  using DcpAction = planning::DcpTree::DcpAction;

  EudmImpl() {}

  void Init(const std::string config_path, double work_rate) override {
    // init map configurations;
    {
      p_smm_ =
          new semantic_map_manager::SemanticMapManager(0, 120.0, true, false);
      std::array<int, 2> map_size = {{1280, 1280}};
      std::array<decimal_t, 2> map_resl = {{0.2, 0.2}};
      std::array<std::string, 2> map_name = {{"height", "width"}};
      p_free_space_grid_ = new GridMap2D(map_size, map_resl, map_name);
    }
    bp_manager_.Init(config_path, work_rate);
  }

  void Clean() override {
    // reset the context state will force from scratch
    bp_manager_.Reset();
  }

  bool Plan(const RawInput &raw, const Task &task,
            PlainOutput *output) override {
    // * Phase I: Process raw input
    TicToc dec_timer;
    DecodeMessage(raw);
    auto map_ptr =
        std::make_shared<semantic_map_manager::SemanticMapManager>(*p_smm_);
    LOG(WARNING) << "[Eudm]Time - Decode message: " << dec_timer.toc() << " ms";
    // * Phase II: Run
    if (bp_manager_.Run(raw.stamp, map_ptr, task) != kSuccess) {
      return false;
    }
    // * Phase III: Construct output
    TicToc con_timer;
    bp_manager_.ConstructPlainOutput(output);
    LOG(WARNING) << "[Eudm]Time - Construct output: " << con_timer.toc()
                 << " ms";
    return true;
  }
  void Publish() override {}

 private:
  bool DecodeMessage(const RawInput &raw) {
    // * decode ego vehicle info
    common::Vehicle ego_vehicle;
    {
      hkust_msg_transformer::Decoder::GetVehicleFromTransformerVins(
          raw.state, &ego_vehicle);
    }
    // * decode lane net info
    common::LaneNet lane_net;
    {
      hkust_msg_transformer::Decoder::GetLaneNetFromTransformerLaneSet(
          raw.lane_set, &lane_net);
    }
    // * decode freespace info
    {
      hkust_msg_transformer::Decoder::GetGridMap2DFromTransformerFreespace(
          raw.freespace, p_free_space_grid_);
    }
    // * decode vehicle set info
    common::VehicleSet surrounding_vehicles;
    {
      hkust_msg_transformer::Decoder::GetVehicleSetFromTransformerMotSet(
          raw.mot_set, &surrounding_vehicles);
    }

    const std::set<std::array<decimal_t, 2>> obstacle_grids;
    p_smm_->UpdateSemanticMap(raw.stamp, ego_vehicle, lane_net, lane_net,
                              *p_free_space_grid_, obstacle_grids,
                              surrounding_vehicles);

    return true;
  }

  // ***** planner states ****
  // * map
  GridMap2D *p_free_space_grid_;
  semantic_map_manager::SemanticMapManager *p_smm_;
  // * planner
  EudmManager bp_manager_;
};

EudmItf *Create() { return new EudmImpl(); }

}  // namespace eudm
}  // namespace planning
