/**
 * @file behavior_tree.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-07-07
 *
 * @copyright Copyright (c) 2019
 *
 */

#ifndef _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_
#define _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_

#include <map>
#include <memory>
#include <string>

#include "common/basics/basics.h"
#include "common/basics/semantics.h"

namespace planning {

class DcpTree {
 public:
  using LateralBehavior = common::LateralBehavior;

  enum class DcpLonAction {
    kMaintain = 0,
    kAccelerate,
    kDecelerate,
    MAX_COUNT = 3
  };

  enum class DcpLatAction {
    kLaneKeeping = 0,
    kLaneChangeLeft,
    kLaneChangeRight,
    MAX_COUNT = 3
  };

  struct DcpAction {
    DcpLonAction lon = DcpLonAction::kMaintain;
    DcpLatAction lat = DcpLatAction::kLaneKeeping;

    decimal_t t = 0.0;

    friend std::ostream& operator<<(std::ostream& os, const DcpAction& action) {
      os << "(lon: " << static_cast<int>(action.lon)
         << ", lat: " << static_cast<int>(action.lat) << ", t: " << action.t
         << ")";
      return os;
    }

    DcpAction() {}
    DcpAction(const DcpLonAction& lon_, const DcpLatAction& lat_,
              const decimal_t& t_)
        : lon(lon_), lat(lat_), t(t_) {}
  };

  DcpTree(const int& tree_height, const decimal_t& layer_time);
  DcpTree(const int& tree_height, const decimal_t& layer_time,
          const decimal_t& last_layer_time);
  ~DcpTree() = default;

  void set_ongoing_action(const DcpAction& a) { ongoing_action_ = a; }

  std::vector<std::vector<DcpAction>> action_script() const {
    return action_script_;
  }

  decimal_t planning_horizon() const;

  int tree_height() const { return tree_height_; }

  decimal_t sim_time_per_layer() const { return layer_time_; }

  ErrorType UpdateScript();

  static std::string RetLonActionName(const DcpLonAction a) {
    std::string a_str;
    switch (a) {
      case DcpLonAction::kMaintain: {
        a_str = std::string("M");
        break;
      }
      case DcpLonAction::kAccelerate: {
        a_str = std::string("A");
        break;
      }
      case DcpLonAction::kDecelerate: {
        a_str = std::string("D");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

  static std::string RetLatActionName(const DcpLatAction a) {
    std::string a_str;
    switch (a) {
      case DcpLatAction::kLaneKeeping: {
        a_str = std::string("K");
        break;
      }
      case DcpLatAction::kLaneChangeLeft: {
        a_str = std::string("L");
        break;
      }
      case DcpLatAction::kLaneChangeRight: {
        a_str = std::string("R");
        break;
      }
      default: {
        a_str = std::string("Null");
        break;
      }
    }
    return a_str;
  }

 private:
  ErrorType GenerateActionScript();

  std::vector<DcpAction> AppendActionSequence(
      const std::vector<DcpAction>& seq_in, const DcpAction& a,
      const int& n) const;

  int tree_height_ = 5;
  decimal_t layer_time_ = 1.0;
  decimal_t last_layer_time_ = 1.0;
  DcpAction ongoing_action_;
  std::vector<std::vector<DcpAction>> action_script_;
};
}  // namespace planning

#endif  //  _CORE_EUDM_PLANNER_INC_EUDM_PLANNER_BEHAVIOR_TREE_H_