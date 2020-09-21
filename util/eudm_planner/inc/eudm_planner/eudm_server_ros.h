#ifndef _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#define _CORE_EUDM_PLANNER_INC_EUDM_SERVER_ROS_H__
#include <sensor_msgs/Joy.h>

#include <chrono>
#include <functional>
#include <numeric>
#include <thread>

#include "common/basics/tic_toc.h"
#include "common/visualization/common_visualization_util.h"
#include "eudm_planner/dcp_tree.h"
#include "eudm_planner/eudm_itf.h"
#include "eudm_planner/eudm_manager.h"
#include "eudm_planner/eudm_planner.h"
#include "eudm_planner/map_adapter.h"
#include "eudm_planner/visualizer.h"
#include "moodycamel/atomicops.h"
#include "moodycamel/readerwriterqueue.h"
#include "ros/ros.h"
#include "semantic_map_manager/semantic_map_manager.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "vehicle_msgs/encoder.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {
class EudmPlannerServer {
 public:
  using SemanticMapManager = semantic_map_manager::SemanticMapManager;
  using DcpAction = DcpTree::DcpAction;
  using DcpLonAction = DcpTree::DcpLonAction;
  using DcpLatAction = DcpTree::DcpLatAction;

  struct Config {
    int kInputBufferSize{100};
  };

  EudmPlannerServer(ros::NodeHandle nh, int ego_id);

  EudmPlannerServer(ros::NodeHandle nh, double work_rate, int ego_id);

  void PushSemanticMap(const SemanticMapManager &smm);

  void BindBehaviorUpdateCallback(
      std::function<int(const SemanticMapManager &)> fn);
  /**
   * @brief set desired velocity
   */
  void set_user_desired_velocity(const decimal_t desired_vel);

  decimal_t user_desired_velocity() const;

  void Init(const std::string &bp_config_path);

  void Start();

 private:
  void PlanCycleCallback();

  void JoyCallback(const sensor_msgs::Joy::ConstPtr &msg);

  void Replan();

  void PublishData();

  void MainThread();

  ErrorType GetCorrespondingActionInActionSequence(
      const decimal_t &t, const std::vector<DcpAction> &action_seq,
      DcpAction *a) const;

  Config config_;

  EudmManager bp_manager_;
  EudmPlannerVisualizer *p_visualizer_;

  SemanticMapManager smm_;

  planning::eudm::Task task_;
  bool use_sim_state_ = true;
  // ros related
  ros::NodeHandle nh_;
  ros::Subscriber joy_sub_;

  double work_rate_{20.0};
  int ego_id_;

  // input buffer
  moodycamel::ReaderWriterQueue<SemanticMapManager> *p_input_smm_buff_;

  bool has_callback_binded_ = false;
  std::function<int(const SemanticMapManager &)> private_callback_fn_;
};

}  // namespace planning

#endif