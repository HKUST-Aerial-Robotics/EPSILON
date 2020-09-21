#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_EUDM_MANAGER_H_

#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
/**
 * @brief since eudm itself is completely stateless, we use a manager to
 *        track the input and output of the eudm, to satisfy task level
 *        constraints.
 */
namespace planning {
class EudmManager {
 public:
  using DcpLatAction = planning::DcpTree::DcpLatAction;
  using DcpLonAction = planning::DcpTree::DcpLonAction;
  using DcpAction = planning::DcpTree::DcpAction;
  using LateralBehavior = common::LateralBehavior;
  using LongitudinalBehavior = common::LongitudinalBehavior;
  using CostStructure = planning::EudmPlanner::CostStructure;

  enum class LaneChangeTriggerType { kStick = 0, kActive };

  struct ReplanningContext {
    bool is_valid = false;
    decimal_t seq_start_time;
    std::vector<DcpAction> action_seq;
  };

  struct ActivateLaneChangeRequest {
    decimal_t trigger_time;
    decimal_t desired_operation_time;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeProposal {
    bool valid = false;
    decimal_t trigger_time = 0.0;
    decimal_t operation_at_seconds = 0.0;
    int ego_lane_id;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
  };

  struct LaneChangeContext {
    bool completed = true;
    bool trigger_when_appropriate = false;
    decimal_t trigger_time = 0.0;
    decimal_t desired_operation_time = 0.0;
    int ego_lane_id = 0;
    LateralBehavior lat = LateralBehavior::kLaneKeeping;
    LaneChangeTriggerType type;
  };

  struct Snapshot {
    bool valid = false;
    int original_winner_id;
    int processed_winner_id;
    common::State plan_state;
    std::vector<std::vector<DcpAction>> action_script;
    std::vector<bool> sim_res;
    std::vector<bool> risky_res;
    std::vector<std::string> sim_info;
    std::vector<decimal_t> final_cost;
    std::vector<std::vector<CostStructure>> progress_cost;
    std::vector<CostStructure> tail_cost;
    vec_E<vec_E<common::Vehicle>> forward_trajs;
    std::vector<std::vector<LateralBehavior>> forward_lat_behaviors;
    std::vector<std::vector<LongitudinalBehavior>> forward_lon_behaviors;
    vec_E<std::unordered_map<int, vec_E<common::Vehicle>>> surround_trajs;
    common::Lane ref_lane;

    double plan_stamp = 0.0;
    double time_cost = 0.0;
  };

  EudmManager() {}

  void Init(const std::string& config_path, const decimal_t work_rate);

  ErrorType Run(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  void Reset();

  void ConstructBehavior(common::SemanticBehavior* behavior);

  EudmPlanner& planner();

  int original_winner_id() const { return last_snapshot_.original_winner_id; }
  int processed_winner_id() const { return last_snapshot_.processed_winner_id; }
  std::shared_ptr<semantic_map_manager::SemanticMapManager> map() {
    return map_adapter_.map();
  }

 private:
  decimal_t GetNearestFutureDecisionPoint(const decimal_t& stamp,
                                          const decimal_t& delta);

  bool IsTriggerAppropriate(const LateralBehavior& lat);

  ErrorType Prepare(
      const decimal_t stamp,
      const std::shared_ptr<semantic_map_manager::SemanticMapManager>& map_ptr,
      const planning::eudm::Task& task);

  ErrorType EvaluateReferenceVelocity(const planning::eudm::Task& task,
                                      decimal_t* ref_vel);

  bool GetReplanDesiredAction(const decimal_t current_time,
                              DcpAction* desired_action);

  void SaveSnapshot(Snapshot* snapshot);

  ErrorType ReselectByContext(const decimal_t stamp, const Snapshot& snapshot,
                              int* new_seq_id);

  void UpdateLaneChangeContextByTask(const decimal_t stamp,
                                     const planning::eudm::Task& task);

  ErrorType GenerateLaneChangeProposal(const decimal_t& stamp,
                                       const planning::eudm::Task& task);

  EudmPlanner bp_;
  EudmPlannerMapAdapter map_adapter_;
  decimal_t work_rate_{20.0};

  int ego_lane_id_;
  ReplanningContext context_;
  Snapshot last_snapshot_;
  planning::eudm::Task last_task_;
  LaneChangeContext lc_context_;
  LaneChangeProposal last_lc_proposal_;
  std::vector<ActivateLaneChangeRequest> preliminary_active_requests_;
};

}  // namespace planning

#endif