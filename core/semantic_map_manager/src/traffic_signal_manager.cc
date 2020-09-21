#include "semantic_map_manager/traffic_signal_manager.h"

namespace semantic_map_manager {

TrafficSignalManager::TrafficSignalManager() { LoadSignals(); }

ErrorType TrafficSignalManager::Init() { return kSuccess; }

ErrorType TrafficSignalManager::LoadSignals() {
  // TODO: (@denny.ding) traffic signal should be controlled by phy sim
  // ~ hard code some
  // * Speed limit
  // * For google_urban
  {
      // common::SpeedLimit speed_limit1(Vec2f(549.927, 2.587),
      //                                 Vec2f(545.641, -96.123),
      //                                 Vec2f(0.0, 8.0));
      // speed_limit1.set_lateral_range(Vec2f(-1.75, 5.25));
      // speed_limit1.set_start_angle(1.530);
      // speed_limit1.set_end_angle(1.616);
      // speed_limit_list_.push_back(speed_limit1);
  } {
      // common::SpeedLimit speed_limit1(Vec2f(552.916, -218.287),
      //                                 Vec2f(541.683, -238.290),
      //                                 Vec2f(0.0, 6.0));
      // speed_limit1.set_start_angle(1.610);
      // speed_limit1.set_end_angle(0.0);
      // speed_limit1.set_lateral_range(Vec2f(-1.75, 5.25));
      // speed_limit_list_.push_back(speed_limit1);
  } {
    // ~ for left turn
    // common::SpeedLimit speed_limit1(Vec2f(532.304, -257.094),
    //                                 Vec2f(572.222, -223.729),
    //                                 Vec2f(0.0, 5.0));
    // speed_limit1.set_lateral_range(Vec2f(-20, 20));
    // speed_limit1.set_start_angle(3.046);
    // speed_limit1.set_end_angle(-1.571);
    // speed_limit_list_.push_back(speed_limit1);
  }  // {
  // common::SpeedLimit speed_limit1(Vec2f(547.783, -219.819),
  //                                 Vec2f(530.753, -232.968), Vec2f(0.0, 5.0));
  // speed_limit1.set_lateral_range(Vec2f(-20, 20));
  // speed_limit1.set_start_angle(1.603);
  // speed_limit1.set_end_angle(-0.042);
  // speed_limit_list_.push_back(speed_limit1);
  // }
  // {
  //   common::SpeedLimit speed_limit1(Vec2f(501.245, -235.298),
  //                                   Vec2f(489.041, -234.299), Vec2f(0.0,
  //                                   0.5));
  //   speed_limit1.set_lateral_range(Vec2f(-1.75, 5.25));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  // * For highway
  // {
  //   Benchmark
  //   common::SpeedLimit speed_limit1(Vec2f(193.181, -396.072),
  //                                   Vec2f(227.831, -465.545),
  //                                   Vec2f(0.0, 4.0));
  //   speed_limit1.set_start_angle(2.034);
  //   speed_limit1.set_end_angle(2.034);
  //   speed_limit1.set_lateral_range(Vec2f(-10, 10));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  // {
  //   Benchmark
  //   common::SpeedLimit speed_limit1(Vec2f(153.855, -315.488),
  //                                   Vec2f(161.858, -332.227), Vec2f(0.0,
  //                                   0.0));
  //   speed_limit1.set_start_angle(2.034);
  //   speed_limit1.set_end_angle(2.034);
  //   speed_limit1.set_lateral_range(Vec2f(-10, 10));
  //   speed_limit1.set_valid_time(Vec2f(0.0, 65.0));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  // {
  //   common::SpeedLimit speed_limit1(Vec2f(615.498, -1137.748),
  //                                   Vec2f(682.836, -1359.883),
  //                                   Vec2f(0.0, 15.0));
  //   speed_limit1.set_lateral_range(Vec2f(-10, 10));
  //   speed_limit_list_.push_back(speed_limit1);
  // }

  return kSuccess;
}

ErrorType TrafficSignalManager::UpdateSignals(const decimal_t time_elapsed) {
  // printf("[xx]time elapsed: %lf.\n", time_elapsed);
  for (auto it = speed_limit_list_.begin(); it < speed_limit_list_.end();) {
    Vec2f valid_time = it->valid_time();
    if (time_elapsed < valid_time[0] || time_elapsed > valid_time[1]) {
      printf("[xxxxx]signal erased at %lf.\n", time_elapsed);
      it = speed_limit_list_.erase(it);
    } else {
      ++it;
    }
  }
  return kSuccess;
}

ErrorType TrafficSignalManager::GetSpeedLimit(const State& state,
                                              const Lane& lane,
                                              decimal_t* speed_limit) const {
  common::StateTransformer stf(lane);
  common::FrenetState ref_fs;
  if (stf.GetFrenetStateFromState(state, &ref_fs) != kSuccess) {
    // printf("[GetSpeedLimit]Cannot get ref state frenet state.\n");
    return kWrongStatus;
  }

  const decimal_t acc_esti = 1.0;
  decimal_t limit = kInf;
  for (auto& speed_limit : speed_limit_list_) {
    IntersectionType intersection_type;
    decimal_t dist_to_startpt;
    decimal_t dist_to_endpt;
    if (CheckIntersectionTypeWithSignal(ref_fs, lane, speed_limit,
                                        &intersection_type, &dist_to_startpt,
                                        &dist_to_endpt) != kSuccess) {
      continue;
    }
    if (intersection_type != kNotIntersect) {
      decimal_t effect_speed_limit_dist =
          state.velocity > speed_limit.max_velocity()
              ? fabs(speed_limit.max_velocity() * speed_limit.max_velocity() -
                     state.velocity * state.velocity) /
                    (2.0 * acc_esti)
              : 0.0;
      // printf("[denny]cur vel %lf limit max vel %lf, effect dist %lf.\n",
      //        state.velocity, speed_limit.max_velocity(),
      //        effect_speed_limit_dist);
      // if (intersection_type == kSignalAhead)
      //   printf("[denny]Speed limit ahead.\n");
      // if (intersection_type == kSignalControlled)
      //   printf("[denny]under speed limit control.\n");
      if ((intersection_type == kSignalAhead &&
           dist_to_startpt < effect_speed_limit_dist) ||
          intersection_type == kSignalControlled) {
        limit = limit > speed_limit.max_velocity() ? speed_limit.max_velocity()
                                                   : limit;
      }
    }
  }
  *speed_limit = limit;
  return kSuccess;
}

ErrorType TrafficSignalManager::CheckIntersectionTypeWithSignal(
    const common::FrenetState& fs, const Lane& lane,
    const common::TrafficSignal& signal, IntersectionType* intersection_type,
    decimal_t* dist_to_startpt, decimal_t* dist_to_endpt) const {
  // ~ NOTICE: use some naive solution to determine whether
  // ~ a speed limit is effective for current <state, lane>
  common::StateTransformer stf(lane);
  Vec2f start_pt_fs, end_pt_fs;
  if (stf.GetFrenetPointFromPoint(signal.start_point(), &start_pt_fs) !=
      kSuccess) {
    return kWrongStatus;
  }
  if (stf.GetFrenetPointFromPoint(signal.end_point(), &end_pt_fs) != kSuccess) {
    return kWrongStatus;
  }

  Vec2f lateral_range = signal.lateral_range();
  IntersectionType int_type = kNotIntersect;
  decimal_t dist_to_start = 0.0;
  decimal_t dist_to_end = 0.0;
  // * regard (projection onlane is reversed, start pt and end pt is too
  // * far away from lateral range) as not intersect
  // std::cout << "[denny]start pt fs" << start_pt_fs.transpose() << std::endl;
  // std::cout << "[denny]end pt fs" << end_pt_fs.transpose() << std::endl;
  if (start_pt_fs[0] < end_pt_fs[0] + kEPS) {
    if (start_pt_fs[1] + lateral_range(1) > 0.0 &&
        start_pt_fs[1] + lateral_range(0) < 0.0 &&
        end_pt_fs[1] + lateral_range(1) > 0.0 &&
        end_pt_fs[1] + lateral_range(0) < 0.0) {
      if (fs.vec_s[0] < start_pt_fs[0]) {
        int_type = kSignalAhead;
        dist_to_start = start_pt_fs[0] - fs.vec_s[0];
        dist_to_end = end_pt_fs[0] - fs.vec_s[0];
      } else if (fs.vec_s[0] >= start_pt_fs[0] && fs.vec_s[0] < end_pt_fs[0]) {
        int_type = kSignalControlled;
        dist_to_start = start_pt_fs[0] - fs.vec_s[0];
        dist_to_end = end_pt_fs[0] - fs.vec_s[0];
      } else {
        int_type = kNotIntersect;
      }
    }
  }
  *intersection_type = int_type;
  *dist_to_startpt = dist_to_start;
  *dist_to_endpt = dist_to_end;
  return kSuccess;
}

ErrorType TrafficSignalManager::GetTrafficStoppingState(
    const State& state, const Lane& lane, State* stopping_state) const {
  return kSuccess;
}

}  // namespace semantic_map_manager