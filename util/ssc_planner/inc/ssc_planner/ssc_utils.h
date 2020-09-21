/**
 * @file ssc_utils.h
 * @author HKUST Aerial Robotics Group (lzhangbz@ust.hk)
 * @brief
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "ssc_planner/ssc_itf.h"
#include "visualization_msgs/MarkerArray.h"

namespace planning {

namespace ssc {
class SscUtils {
 public:
  static void ConvertSscOutputToMsg(
      const SscOutput& data, hkust_msg_transformer::PlainSscOutput* msg) {
    msg->header.stamp = ros::Time(data.plan_stamp);
    msg->valid = data.valid;
    msg->time_cost = data.time_cost;
    for (const auto& state : data.sampled_traj) {
      hkust_msg_transformer::State s_msg;
      s_msg.header.stamp = ros::Time(state.timestamp);
      s_msg.vec_position.x = state.x;
      s_msg.vec_position.y = state.y;
      s_msg.angle = state.angle;
      s_msg.velocity = state.vel;
      s_msg.acceleration = state.acc;
      s_msg.curvature = state.curvature;
      msg->sampled_traj.push_back(s_msg);
    }

    for (const auto& plain_fs : data.ref_ff_states) {
      hkust_msg_transformer::FrenetState fs_msg;
      fs_msg.header.stamp = ros::Time(plain_fs.time_stamp);
      fs_msg.vec_s.push_back(plain_fs.vec_s[0]);
      fs_msg.vec_s.push_back(plain_fs.vec_s[1]);
      fs_msg.vec_s.push_back(plain_fs.vec_s[2]);

      fs_msg.vec_dt.push_back(plain_fs.vec_dt[0]);
      fs_msg.vec_dt.push_back(plain_fs.vec_dt[1]);
      fs_msg.vec_dt.push_back(plain_fs.vec_dt[2]);

      fs_msg.vec_ds.push_back(plain_fs.vec_ds[0]);
      fs_msg.vec_ds.push_back(plain_fs.vec_ds[1]);
      fs_msg.vec_ds.push_back(plain_fs.vec_ds[2]);
      msg->ref_ff_states.push_back(fs_msg);
    }

    for (const auto& plain_fs : data.qp_traj) {
      hkust_msg_transformer::FrenetState fs_msg;
      fs_msg.header.stamp = ros::Time(plain_fs.time_stamp);
      fs_msg.vec_s.push_back(plain_fs.vec_s[0]);
      fs_msg.vec_s.push_back(plain_fs.vec_s[1]);
      fs_msg.vec_s.push_back(plain_fs.vec_s[2]);

      fs_msg.vec_dt.push_back(plain_fs.vec_dt[0]);
      fs_msg.vec_dt.push_back(plain_fs.vec_dt[1]);
      fs_msg.vec_dt.push_back(plain_fs.vec_dt[2]);

      fs_msg.vec_ds.push_back(plain_fs.vec_ds[0]);
      fs_msg.vec_ds.push_back(plain_fs.vec_ds[1]);
      fs_msg.vec_ds.push_back(plain_fs.vec_ds[2]);
      msg->qp_traj.push_back(fs_msg);
    }

    for (const auto& plain_cube : data.corridor) {
      hkust_msg_transformer::PlainDrivingCube cube_msg;
      cube_msg.t_lb = plain_cube.t_lb;
      cube_msg.t_ub = plain_cube.t_ub;

      cube_msg.p_lb.assign(plain_cube.p_lb.begin(), plain_cube.p_lb.end());
      cube_msg.p_ub.assign(plain_cube.p_ub.begin(), plain_cube.p_ub.end());

      cube_msg.v_lb.assign(plain_cube.v_lb.begin(), plain_cube.v_lb.end());
      cube_msg.v_ub.assign(plain_cube.v_ub.begin(), plain_cube.v_ub.end());

      cube_msg.a_lb.assign(plain_cube.a_lb.begin(), plain_cube.a_lb.end());
      cube_msg.a_ub.assign(plain_cube.a_ub.begin(), plain_cube.a_ub.end());

      msg->corridor.push_back(cube_msg);
    }
  }
};

}  // namespace ssc
}  // namespace planning