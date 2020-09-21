#include "behavior_planner/behavior_server_ros.h"

namespace planning {

BehaviorPlannerServer::BehaviorPlannerServer(ros::NodeHandle nh, int ego_id)
    : nh_(nh), work_rate_(20.0), ego_id_(ego_id) {
  p_visualizer_ = new BehaviorPlannerVisualizer(nh, &bp_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
}

BehaviorPlannerServer::BehaviorPlannerServer(ros::NodeHandle nh,
                                             double work_rate, int ego_id)
    : nh_(nh), work_rate_(work_rate), ego_id_(ego_id) {
  p_visualizer_ = new BehaviorPlannerVisualizer(nh, &bp_, ego_id);
  p_input_smm_buff_ = new moodycamel::ReaderWriterQueue<SemanticMapManager>(
      config_.kInputBufferSize);
}

void BehaviorPlannerServer::PushSemanticMap(const SemanticMapManager& smm) {
  if (p_input_smm_buff_) p_input_smm_buff_->try_enqueue(smm);
}

void BehaviorPlannerServer::PublishData() {
  p_visualizer_->PublishDataWithStamp(ros::Time::now());
}

void BehaviorPlannerServer::Init() {
  bp_.Init("bp");
  if (bp_.autonomous_level() >= 2) {
    joy_sub_ =
        nh_.subscribe("/joy", 10, &BehaviorPlannerServer::JoyCallback, this);
  }
  bool use_sim_state = true;
  nh_.param("use_sim_state", use_sim_state, true);
  bp_.set_use_sim_state(use_sim_state);
  p_visualizer_->Init();
}

void BehaviorPlannerServer::JoyCallback(const sensor_msgs::Joy::ConstPtr& msg) {
  if (bp_.autonomous_level() < 2) return;
  if (!is_hmi_enabled_) return;
  int msg_id;
  if (std::string("").compare(msg->header.frame_id) == 0) {
    msg_id = 0;
  } else {
    msg_id = std::stoi(msg->header.frame_id);
  }
  if (msg_id != ego_id_) return;
  // ~ buttons[2] --> 1 -->  lcl
  // ~ buttons[1] --> 1 -->  lcr
  // ~ buttons[3] --> 1 -->  +1m/s
  // ~ buttons[0] --> 1 -->  -1m/s
  if (msg->buttons[0] == 0 && msg->buttons[1] == 0 && msg->buttons[2] == 0 &&
      msg->buttons[3] == 0)
    return;

  if (msg->buttons[2] == 1) {
    bp_.set_hmi_behavior(common::LateralBehavior::kLaneChangeLeft);
  } else if (msg->buttons[1] == 1) {
    bp_.set_hmi_behavior(common::LateralBehavior::kLaneChangeRight);
  } else if (msg->buttons[3] == 1) {
    bp_.set_user_desired_velocity(bp_.user_desired_velocity() + 1.0);
  } else if (msg->buttons[0] == 1) {
    bp_.set_user_desired_velocity(bp_.user_desired_velocity() - 1.0);
  }
}

void BehaviorPlannerServer::Start() {
  bp_.set_map_interface(&map_adapter_);
  std::thread(&BehaviorPlannerServer::MainThread, this).detach();
}

void BehaviorPlannerServer::MainThread() {
  using namespace std::chrono;
  system_clock::time_point current_start_time{system_clock::now()};
  system_clock::time_point next_start_time{current_start_time};
  const milliseconds interval{static_cast<int>(1000.0 / work_rate_)};
  while (true) {
    current_start_time = system_clock::now();
    next_start_time = current_start_time + interval;
    PlanCycleCallback();
    std::this_thread::sleep_until(next_start_time);
  }
}

void BehaviorPlannerServer::PlanCycleCallback() {
  if (p_input_smm_buff_ == nullptr) return;

  SemanticMapManager smm;
  bool has_updated_map = false;
  while (p_input_smm_buff_->try_dequeue(smm)) {
    has_updated_map = true;
  }

  if (has_updated_map) {
    auto map_ptr =
        std::make_shared<semantic_map_manager::SemanticMapManager>(smm);
    map_adapter_.set_map(map_ptr);

    TicToc timer;
    if (bp_.RunOnce() == kSuccess) {
      smm.set_ego_behavior(bp_.behavior());
    }

    if (has_callback_binded_) {
      private_callback_fn_(smm);
    }

    PublishData();
  }
}

void BehaviorPlannerServer::BindBehaviorUpdateCallback(
    std::function<int(const SemanticMapManager&)> fn) {
  private_callback_fn_ = std::bind(fn, std::placeholders::_1);
  has_callback_binded_ = true;
}

void BehaviorPlannerServer::Replan() {}

void BehaviorPlannerServer::set_autonomous_level(int level) {
  bp_.set_autonomous_level(level);
}

void BehaviorPlannerServer::set_aggressive_level(int level) {
  bp_.set_aggressive_level(level);
}

void BehaviorPlannerServer::set_user_desired_velocity(
    const decimal_t desired_vel) {
  bp_.set_user_desired_velocity(desired_vel);
}

decimal_t BehaviorPlannerServer::user_desired_velocity() const {
  return bp_.user_desired_velocity();
}

decimal_t BehaviorPlannerServer::reference_desired_velocity() const {
  return bp_.reference_desired_velocity();
}

void BehaviorPlannerServer::enable_hmi_interface() { is_hmi_enabled_ = true; }

}  // namespace planning
