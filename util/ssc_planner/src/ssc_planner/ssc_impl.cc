#include <glog/logging.h>

#include <vector>

#include "hkust_msg_transformer/hkust_decoder.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "ssc_planner/map_adapter.h"
#include "ssc_planner/ssc_itf.h"
#include "ssc_planner/ssc_planner.h"
#include "ssc_planner/ssc_utils.h"

namespace planning {

namespace ssc {

class SscItfUtils {
 public:
  static void GetPlainStateUsingState(const common::State &state,
                                      PlainState *plain) {
    plain->timestamp = state.time_stamp;
    plain->x = state.vec_position.x();
    plain->y = state.vec_position.y();
    plain->angle = state.angle;
    plain->vel = state.velocity;
    plain->acc = state.acceleration;
    plain->curvature = state.curvature;
  }

  static void GetStateUsingPlainState(const PlainState &plain,
                                      common::State *state) {
    state->time_stamp = plain.timestamp;
    state->vec_position = Vec2f(plain.x, plain.y);
    state->angle = plain.angle;
    state->curvature = plain.curvature;
    state->velocity = plain.vel;
    state->acceleration = plain.acc;
  }

  static void GetPlainFrenetStateUsingFrenetState(const common::FrenetState &fs,
                                                  PlainFrenetState *plain_fs) {
    plain_fs->time_stamp = fs.time_stamp;
    plain_fs->vec_s = {fs.vec_s[0], fs.vec_s[1], fs.vec_s[2]};
    plain_fs->vec_dt = {fs.vec_dt[0], fs.vec_dt[1], fs.vec_dt[2]};
    plain_fs->vec_ds = {fs.vec_ds[0], fs.vec_ds[1], fs.vec_ds[2]};
  }
};

class SscFrenetTraj : public SscTrajItf {
 public:
  SscFrenetTraj() = default;

  void GetState(const double &t, PlainState *plain_state) override {
    common::State state;
    if (kSuccess != p_traj_->GetState(t, &state)) {
      printf("[XXX]SscFrenetTraj get state error\n");
      return;
    }
    SscItfUtils::GetPlainStateUsingState(state, plain_state);
  }

  void GetFrenetState(const double &t, PlainFrenetState *plain_fs) override {
    common::FrenetState fs;
    if (kSuccess != p_traj_->GetFrenetState(t, &fs)) {
      printf("[XXX]SscFrenetTraj get frenet state error\n");
      return;
    }
    SscItfUtils::GetPlainFrenetStateUsingFrenetState(fs, plain_fs);
  }

  double GetBegin() override { return p_traj_->begin(); }

  double GetEnd() override { return p_traj_->end(); }

  double GetTimeStamp() override { return p_traj_->begin(); }

  void set_traj(std::unique_ptr<common::FrenetTrajectory> p_traj) {
    p_traj_ = std::move(p_traj);
  }

 private:
  std::unique_ptr<common::FrenetTrajectory> p_traj_;
};

class SscImpl : public SscItf {
 public:
  using GridMap2D = common::GridMapND<uint8_t, 2>;

  SscImpl() = default;

  void Init(const std::string config_path, double work_rate) override {
    // init map configurations;
    {
      p_smm_ =
          new semantic_map_manager::SemanticMapManager(0, 120.0, false, false);
      std::array<int, 2> map_size = {{1280, 1280}};
      std::array<decimal_t, 2> map_resl = {{0.2, 0.2}};
      std::array<std::string, 2> map_name = {{"height", "width"}};
      p_free_space_grid_ = new GridMap2D(map_size, map_resl, map_name);
    }

    planner_.Init(config_path);
    planner_.set_map_interface(&map_adapter_);
  }

  void Clean() override {
    // reset the context state will force from scratch
  }

  bool Plan(const RawInput &raw, const BpResult &bp_result,
            SscOutput *ssc_output) override {
    // * Phase I: Process raw input
    TicToc pre_timer;
    DecodeMessage(raw);
    LOG(WARNING) << "[Ssc]Time - Decode message: " << pre_timer.toc() << " ms";

    // * Phase II: Convert bp_result to semantic_behavior
    pre_timer.tic();
    common::SemanticBehavior behavior;
    // PlainOutput -> SemanticBehavior
    if (!GetSemanticBehaviorUsingBpResult(bp_result, &behavior)) {
      printf("[SscImpl] BpResult is not valid\n");
      return false;
    }
    p_smm_->set_ego_behavior(behavior);

    auto map_ptr =
        std::make_shared<semantic_map_manager::SemanticMapManager>(*p_smm_);
    map_adapter_.set_map(map_ptr);
    LOG(WARNING) << "[Ssc]Time - Convert BpResult: " << pre_timer.toc()
                 << " ms";

    // * Phase III: Run
    if (planner_.RunOnce() != kSuccess) {
      printf("[SscImpl]planning failed.\n");
      return false;
    }

    // * Phase IV: Generate output
    TicToc gen_timer;
    planner_.GetSscOutput(ssc_output);
    LOG(WARNING) << "[Ssc]Time - Construct output: " << gen_timer.toc()
                 << " ms";
    return true;
  }

  void Publish() override {}

  SscTrajItf *RetTraj() override {
    auto traj = new SscFrenetTraj();
    traj->set_traj(planner_.trajectory());
    return traj;
  }

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

  bool GetSemanticBehaviorUsingBpResult(const BpResult &bp_result,
                                        common::SemanticBehavior *behavior) {
    if (!bp_result.valid) {
      return false;
    }

    auto winner_elem = bp_result.elements[bp_result.winner_element_id];

    // * behaviors
    behavior->lat_behavior =
        static_cast<common::LateralBehavior>(winner_elem.lat_behaviors.front());
    behavior->lon_behavior = static_cast<common::LongitudinalBehavior>(
        winner_elem.lon_behaviors.front());
    behavior->forward_behaviors = std::vector<common::LateralBehavior>{
        static_cast<common::LateralBehavior>(
            winner_elem.lat_behaviors.front())};

    // * state
    behavior->state = p_smm_->ego_vehicle().state();

    // * ref lane
    common::Lane ref_lane;
    p_smm_->GetRefLaneForStateByBehavior(behavior->state, std::vector<int>(),
                                         behavior->lat_behavior, 250.0, 20.0,
                                         true, &ref_lane);
    behavior->ref_lane = ref_lane;

    // * trajs
    vec_E<common::Vehicle> ego_traj;
    GetForwardTrajectoryUsingPlainStates(
        winner_elem.ego_traj, p_smm_->ego_vehicle().param(), 0, &ego_traj);
    behavior->forward_trajs.push_back(ego_traj);

    std::unordered_map<int, vec_E<common::Vehicle>> surround_trajs;
    auto surrounding_vehicles = p_smm_->surrounding_vehicles();
    for (const auto &p_sur_traj : winner_elem.surround_trajs) {
      int id = p_sur_traj.first;
      vec_E<common::Vehicle> traj;
      GetForwardTrajectoryUsingPlainStates(
          p_sur_traj.second, surrounding_vehicles.vehicles.at(id).param(), id,
          &traj);
      surround_trajs.insert(std::pair<int, vec_E<common::Vehicle>>(id, traj));
    }
    behavior->surround_trajs.push_back(surround_trajs);

    return true;
  }

  void GetForwardTrajectoryUsingPlainStates(
      const std::vector<PlainState> &plain_states,
      const common::VehicleParam &param, const int id,
      vec_E<common::Vehicle> *traj) {
    common::Vehicle v;
    v.set_id(id);
    v.set_param(param);

    for (const auto &plain_state : plain_states) {
      common::State state;
      SscItfUtils::GetStateUsingPlainState(plain_state, &state);
      state.steer = atan(param.wheel_base() * state.curvature);

      v.set_state(state);
      traj->push_back(v);
    }
  }

  GridMap2D *p_free_space_grid_;
  semantic_map_manager::SemanticMapManager *p_smm_;

  planning::SscPlanner planner_;
  planning::SscPlannerAdapter map_adapter_;
};

SscItf *Create() { return new SscImpl(); }

}  // namespace ssc
}  // namespace planning
