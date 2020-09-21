#ifndef _UTIL_SSC_PLANNER_INC_SSC_PLANNER_SSC_ITF_H_
#define _UTIL_SSC_PLANNER_INC_SSC_PLANNER_SSC_ITF_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "hkust_msg_transformer/Freespace.h"
#include "hkust_msg_transformer/FrenetState.h"
#include "hkust_msg_transformer/LaneSet.h"
#include "hkust_msg_transformer/MotSet.h"
#include "hkust_msg_transformer/PlainOutput.h"
#include "hkust_msg_transformer/PlainSscOutput.h"
#include "hkust_msg_transformer/PlainDrivingCube.h"
#include "hkust_msg_transformer/Vins.h"

namespace planning {

namespace ssc {

struct RawInput {
  double stamp;
  hkust_msg_transformer::Vins state;
  hkust_msg_transformer::MotSet mot_set;
  hkust_msg_transformer::LaneSet lane_set;
  hkust_msg_transformer::Freespace freespace;
};

struct PlainState {
  double timestamp;
  double x;
  double y;
  double angle;
  double vel;
  double acc;
  double curvature;

  PlainState() {}
  PlainState(double _timestamp, double _x, double _y, double _angle,
             double _vel, double _acc, double _curvature)
      : timestamp(_timestamp),
        x(_x),
        y(_y),
        angle(_angle),
        vel(_vel),
        acc(_acc),
        curvature(_curvature) {}
};

struct PlainCost {
  int valid_sample_index_ub;
  double efficiency = 0.0;
  double safety = 0.0;
  double navigation = 0.0;
  double weight = 0.0;

  PlainCost() {}
  PlainCost(int _valid_sample_index_ub, double _efficiency, double _safety,
            double _navigation, double _weight)
      : valid_sample_index_ub(_valid_sample_index_ub),
        efficiency(_efficiency),
        safety(_safety),
        navigation(_navigation),
        weight(_weight) {}
};

struct OutputElement {
  // * total states
  bool valid = false;
  bool risky = false;
  double final_cost = 0.0;
  std::string sim_info;
  // * macro states
  std::vector<int> lat_behaviors;
  std::vector<int> lon_behaviors;
  // * macro cmd
  std::vector<int> lat_cmds;
  std::vector<int> lon_cmds;

  std::vector<PlainCost> stage_costs;
  // * micro states
  std::vector<PlainState> ego_traj;
  std::unordered_map<int, std::vector<PlainState>> surround_trajs;
};

struct BpResult {
  bool valid = false;
  int winner_element_id;
  int original_winner_id;
  double best_cost;
  double ongoing_action_duration;
  std::vector<OutputElement> elements;
};

struct PlainFrenetState {
  double time_stamp{0.0};
  std::array<double, 3> vec_s;
  std::array<double, 3> vec_dt;
  std::array<double, 3> vec_ds;
};

struct PlainDrivingCube {
  double t_lb, t_ub;
  std::array<double, 2> p_lb, p_ub;
  std::array<double, 2> v_lb, v_ub;
  std::array<double, 2> a_lb, a_ub;
};

struct SscOutput {
  bool valid = false;

  // traj for vis (x-y)
  std::vector<PlainState> sampled_traj;

  // qp traj (s-d)
  std::vector<PlainFrenetState> ref_ff_states;
  std::vector<PlainFrenetState> qp_traj;

  // corridor
  std::vector<PlainDrivingCube> corridor;

  double plan_stamp = 0.0;
  double time_cost = 0.0;
};

class SscTrajItf {
 public:
  virtual ~SscTrajItf() = default;

  virtual void GetState(const double& t, PlainState* state) = 0;
  virtual void GetFrenetState(const double& t, PlainFrenetState* fs) = 0;
  virtual double GetBegin() = 0;
  virtual double GetEnd() = 0;
  virtual double GetTimeStamp() = 0;
};

class SscItf {
 public:
  virtual ~SscItf() = default;
  virtual void Init(const std::string config_path, double work_rate) = 0;
  virtual bool Plan(const RawInput& raw, const BpResult& bp_result,
                    SscOutput* ssc_output) = 0;
  virtual void Publish() = 0;
  virtual void Clean() = 0;

  virtual SscTrajItf* RetTraj() = 0;
};

SscItf* Create();

}  // namespace ssc

}  // namespace planning

#endif