/**
 * @file common_visualization_util.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-18
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__
#define _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__

#include "common/basics/basics.h"
#include "common/basics/colormap.h"
#include "common/basics/semantics.h"
#include "common/basics/shapes.h"
#include "common/basics/tool_func.h"
#include "common/circle_arc/circle_arc.h"
#include "common/circle_arc/circle_arc_branch.h"
#include "common/lane/lane.h"
#include "common/math/calculations.h"
#include "common/spline/polynomial.h"
#include "common/spline/spline.h"
#include "common/state/state.h"
#include "common/state/waypoint.h"
#include "common/trajectory/trajectory.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace common {

class VisualizationUtil {
 public:
  /**
   * @brief Get the marker from polynomial parameterization
   * @notice this function do not take care of the header (incl. stamp, frame,
   * and marker id)
   *
   * @tparam N_DEG
   * @tparam N_DIM
   * @param poly input polynomial
   * @param s0 evaluation start
   * @param s1 evaluation end
   * @param step evaluation step
   * @param scale scale for the three dimension
   * @param color color in the order of r, g, b, a
   * @param marker output
   * @return ErrorType
   */
  template <int N_DEG, int N_DIM>
  static ErrorType GetMarkerByPolynomial(const PolynomialND<N_DEG, N_DIM>& poly,
                                         const decimal_t s0, const decimal_t s1,
                                         const decimal_t step,
                                         const Vec3f scale,
                                         const ColorARGB color,
                                         visualization_msgs::Marker* marker) {
    marker->type = visualization_msgs::Marker::LINE_STRIP;
    marker->action = visualization_msgs::Marker::MODIFY;
    FillScaleColorInMarker(scale, color, marker);
    for (decimal_t s = s0; s < s1; s += step) {
      auto v = poly.evaluate(s);
      geometry_msgs::Point point;
      ConvertVectorToPoint<N_DIM>(v, &point);
      marker->points.push_back(point);
    }
    return kSuccess;
  }

  /**
   * @brief Get the Marker By Spline object
   *
   * @tparam N_DEG
   * @tparam N_DIM
   * @param spline
   * @param step
   * @param scale
   * @param color
   * @param offset_z
   * @param marker
   * @return ErrorType
   */
  template <int N_DEG, int N_DIM>
  static ErrorType GetMarkerBySpline(const Spline<N_DEG, N_DIM>& spline,
                                     const decimal_t step, const Vec3f& scale,
                                     const ColorARGB& color,
                                     const decimal_t offset_z,
                                     visualization_msgs::Marker* marker) {
    marker->type = visualization_msgs::Marker::LINE_STRIP;
    marker->action = visualization_msgs::Marker::MODIFY;
    FillScaleColorInMarker(scale, color, marker);
    for (decimal_t s = spline.begin(); s < spline.end(); s += step) {
      Vecf<N_DIM> ret;
      if (spline.evaluate(s, 0, &ret) == kSuccess) {
        geometry_msgs::Point point;
        ConvertVectorToPoint<N_DIM>(ret, &point);
        point.z = offset_z;
        marker->points.push_back(point);
      }
    }

    return kSuccess;
  }

  /**
   * @brief Get the Marker By Lane object
   *
   * @param lane
   * @param step
   * @param scale
   * @param color
   * @param offset_z
   * @param marker
   * @return ErrorType
   */
  static ErrorType GetMarkerByLane(const Lane& lane, const decimal_t step,
                                   const Vec3f& scale, const ColorARGB& color,
                                   const decimal_t offset_z,
                                   visualization_msgs::Marker* marker) {
    // unwrap the parameterization

    if (!lane.IsValid()) return kIllegalInput;
    GetMarkerBySpline<LaneDegree, LaneDim>(lane.position_spline(), step, scale,
                                           color, offset_z, marker);
    return kSuccess;
  }

  /**
   * @brief Get the Marker Array By Trajectory object
   *
   * @param traj
   * @param step
   * @param scale
   * @param color
   * @param offset_z
   * @param marker_arr
   * @return ErrorType
   */
  static ErrorType GetMarkerArrayByTrajectory(
      const Trajectory& traj, const decimal_t step, const Vec3f& scale,
      const ColorARGB& color, const decimal_t offset_z,
      visualization_msgs::MarkerArray* marker_arr) {
    if (!traj.IsValid()) return kIllegalInput;
    visualization_msgs::Marker traj_mk;
    traj_mk.type = visualization_msgs::Marker::LINE_STRIP;
    traj_mk.action = visualization_msgs::Marker::MODIFY;
    FillScaleColorInMarker(scale, color, &traj_mk);
    for (decimal_t s = traj.begin(); s < traj.end(); s += step) {
      common::State state;
      if (traj.GetState(s, &state) == kSuccess) {
        geometry_msgs::Point point;
        point.x = state.vec_position[0];
        point.y = state.vec_position[1];
        point.z = offset_z;
        traj_mk.points.push_back(point);
      }
    }
    marker_arr->markers.push_back(traj_mk);
    return kSuccess;
  }

  /**
   * @brief Convert vector to ROS Point
   *
   * @tparam N_DIM
   * @param vec
   * @param point
   * @return ErrorType
   */
  template <int N_DIM>
  static ErrorType ConvertVectorToPoint(const Vecf<N_DIM> vec,
                                        geometry_msgs::Point* point) {
    point->x = 0.0;
    point->y = 0.0;
    point->z = 0.0;

    if (N_DIM >= 1) {
      point->x = vec[0];
    }
    if (N_DIM >= 2) {
      point->y = vec[1];
    }
    if (N_DIM >= 3) {
      point->z = vec[2];
    }
    return kSuccess;
  }

  /**
   * @brief Convert vector to ROS Point32
   *
   * @tparam N_DIM
   * @param vec
   * @param point
   * @return ErrorType
   */
  template <int N_DIM>
  static ErrorType ConvertVectorToPoint32(const Vecf<N_DIM>& vec,
                                          geometry_msgs::Point32* point) {
    point->x = 0.0;
    point->y = 0.0;
    point->z = 0.0;

    if (N_DIM >= 1) {
      point->x = vec[0];
    }
    if (N_DIM >= 2) {
      point->y = vec[1];
    }
    if (N_DIM >= 3) {
      point->z = vec[2];
    }
    return kSuccess;
  }

  /**
   * @brief Get the marker array from state vec
   *
   * @param state_vec a vector of states (probably from one trajectory)
   * @param color color
   * @param marker_arr returned marker array
   * @return ErrorType
   */
  static ErrorType GetMarkerArrayByStateVector(
      const vec_E<State>& state_vec, const ColorARGB& color,
      visualization_msgs::MarkerArray* marker_arr) {
    visualization_msgs::Marker traj_mk;
    traj_mk.type = visualization_msgs::Marker::LINE_STRIP;
    traj_mk.action = visualization_msgs::Marker::MODIFY;
    FillScaleColorInMarker(Vec3f(0.05, 0.05, 0.05), color, &traj_mk);
    for (auto& state : state_vec) {
      geometry_msgs::Point point;
      ConvertVectorToPoint<2>(state.vec_position, &point);
      traj_mk.points.push_back(point);
    }
    marker_arr->markers.push_back(traj_mk);

    return kSuccess;
  }

  /**
   * @brief
   *
   * @param scale
   * @param color
   * @param marker
   * @return ErrorType
   */
  static ErrorType FillScaleColorInMarker(const Vec3f scale,
                                          const ColorARGB color,
                                          visualization_msgs::Marker* marker) {
    // default pose at origin
    marker->pose.position.x = 0.0;
    marker->pose.position.y = 0.0;
    marker->pose.position.z = 0.0;
    marker->pose.orientation.w = 1.0;
    marker->pose.orientation.x = 0.0;
    marker->pose.orientation.y = 0.0;
    marker->pose.orientation.z = 0.0;

    FillColorInMarker(color, marker);
    FillScaleInMarker(scale, marker);
    return kSuccess;
  }

  /**
   * @brief
   *
   * @param color
   * @param marker
   * @return ErrorType
   */
  static ErrorType FillColorInMarker(const ColorARGB& color,
                                     visualization_msgs::Marker* marker) {
    marker->color.a = color.a;
    marker->color.r = color.r;
    marker->color.g = color.g;
    marker->color.b = color.b;
    return kSuccess;
  }

  /**
   * @brief
   *
   * @param scale
   * @param marker
   * @return ErrorType
   */
  static ErrorType FillScaleInMarker(const Vec3f scale,
                                     visualization_msgs::Marker* marker) {
    marker->scale.x = scale(0);
    marker->scale.y = scale(1);
    marker->scale.z = scale(2);
    return kSuccess;
  }

  /**
   * @brief
   *
   * @param if_ascending
   * @param marker
   * @return ErrorType
   */
  static ErrorType FillGradientColorInMarker(
      const bool if_ascending, visualization_msgs::Marker* marker) {
    int num = marker->points.size();
    for (int i = 0; i < num; ++i) {
      double k = (double)i / (double)num;
      common::ColorARGB c = common::GetJetColorByValue(k, 1.0, 0.0);
      std_msgs::ColorRGBA c_ros;
      c_ros.a = c.a;
      c_ros.r = c.r;
      c_ros.g = c.g;
      c_ros.b = c.b;
      marker->colors.push_back(c_ros);
    }
    return kSuccess;
  }

  /**
   * @brief
   *
   * @param time_stamp
   * @param frame_id
   * @param last_array_size
   * @param marker_arr
   * @return ErrorType
   */
  static ErrorType FillHeaderIdInMarkerArray(
      const ros::Time time_stamp, const std::string frame_id,
      const int last_array_size, visualization_msgs::MarkerArray* marker_arr) {
    int marker_id = 0;
    for (auto& mk : marker_arr->markers) {
      mk.id = marker_id;
      mk.header.stamp = time_stamp;
      mk.header.frame_id = frame_id;
      marker_id++;
    }

    visualization_msgs::Marker delete_mk;
    delete_mk.header.stamp = time_stamp;
    delete_mk.header.frame_id = frame_id;
    delete_mk.action = visualization_msgs::Marker::DELETE;
    for (int i = marker_id; i < last_array_size; i++) {
      delete_mk.id = i;
      marker_arr->markers.push_back(delete_mk);
    }
    return kSuccess;
  }

  /**
   * @brief
   *
   * @param time_stamp
   * @param marker_arr
   * @return ErrorType
   */
  static ErrorType FillStampInMarkerArray(
      const ros::Time time_stamp, visualization_msgs::MarkerArray* marker_arr) {
    for (auto& mk : marker_arr->markers) {
      mk.header.stamp = time_stamp;
    }
    return kSuccess;
  }

  /**
   * @brief
   *
   * @param duration
   * @param marker_arr
   * @return ErrorType
   */
  static ErrorType FillLifeTimeInMarkerArray(
      const ros::Duration duration,
      visualization_msgs::MarkerArray* marker_arr) {
    for (auto& mk : marker_arr->markers) {
      mk.lifetime = duration;
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Pose From 3 Dof State object
   *
   * @param state
   * @param p_pose
   * @return ErrorType
   */
  static ErrorType GetRosPoseFrom3DofState(const Vec3f& state,
                                           geometry_msgs::Pose* p_pose) {
    p_pose->position.x = state(0);
    p_pose->position.y = state(1);
    p_pose->position.z = 0.0;

    tf::Quaternion q;
    q = tf::createQuaternionFromRPY(0.0, 0.0, state(2));

    p_pose->orientation.x = q.x();
    p_pose->orientation.y = q.y();
    p_pose->orientation.z = q.z();
    p_pose->orientation.w = q.w();
    return kSuccess;
  }

  /**
   * @brief Get the Ros Pose Stamped From Vec 3d object
   *
   * @param state
   * @param p_pose_stamped
   * @return ErrorType
   */
  static ErrorType GetRosPoseStampedFromVec3d(
      const Vec3f& state, geometry_msgs::PoseStamped* p_pose_stamped) {
    p_pose_stamped->header.frame_id = "map";
    p_pose_stamped->header.stamp = ros::Time::now();
    geometry_msgs::Pose pose;
    GetRosPoseFrom3DofState(state, &pose);
    p_pose_stamped->pose = pose;
    return kSuccess;
  }

  /**
   * @brief Get the Ros Pose Array From State 3d Vector object
   *
   * @param states
   * @param p_pose_array
   * @return ErrorType
   */
  static ErrorType GetRosPoseArrayFromState3dVector(
      const std::vector<Vec3f>& states,
      geometry_msgs::PoseArray* p_pose_array) {
    p_pose_array->header.frame_id = "map";
    p_pose_array->header.stamp = ros::Time::now();
    for (const auto& state : states) {
      geometry_msgs::Pose pose;
      GetRosPoseFrom3DofState(state, &pose);
      p_pose_array->poses.push_back(pose);
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Point Cloud From 3 Dof State Vector object
   *
   * @param states
   * @param p_pc
   * @return ErrorType
   */
  static ErrorType GetRosPointCloudFrom3DofStateVector(
      const std::vector<Vec3f>& states, sensor_msgs::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = ros::Time::now();
    for (const auto& state : states) {
      Vec2f vec(state(0), state(1));
      geometry_msgs::Point32 pt;
      ConvertVectorToPoint32<2>(vec, &pt);
      p_pc->points.push_back(pt);
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Point Cloud From Circle Arc object
   *
   * @param arc
   * @param p_pc
   * @return ErrorType
   */
  static ErrorType GetRosPointCloudFromCircleArc(
      const CircleArc& arc, sensor_msgs::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = ros::Time::now();
    std::vector<Vec3f> states;
    arc.GetSampledStates(0.2, &states);
    GetRosPointCloudFrom3DofStateVector(states, p_pc);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Point Cloud From Circle Arc Branch object
   *
   * @param arc_branch
   * @param p_pc
   * @return ErrorType
   */
  static ErrorType GetRosPointCloudFromCircleArcBranch(
      const CircleArcBranch& arc_branch, sensor_msgs::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = ros::Time::now();
    for (const auto& arc : arc_branch.circle_arc_vec()) {
      GetRosPointCloudFromCircleArc(arc, p_pc);
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Point Cloud From Point List object
   *
   * @param point_list
   * @param p_pc
   * @return ErrorType
   */
  static ErrorType GetRosPointCloudFromPointList(
      const std::vector<common::Point>& point_list,
      sensor_msgs::PointCloud* p_pc) {
    p_pc->header.frame_id = "map";
    p_pc->header.stamp = ros::Time::now();

    for (const auto& p : point_list) {
      geometry_msgs::Point32 pt;
      pt.x = p.x;
      pt.y = p.y;
      pt.z = p.z;
      p_pc->points.push_back(pt);
    }

    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Sphere Using Point object
   *
   * @param pt
   * @param color
   * @param scale
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerSphereUsingPoint(
      const Vec3f& pt, const ColorARGB& color, const Vec3f& scale,
      const int& id, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::SPHERE;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    FillColorInMarker(color, p_marker);
    FillScaleInMarker(scale, p_marker);
    p_marker->pose.position.x = pt(0);
    p_marker->pose.position.y = pt(1);
    p_marker->pose.position.z = pt(2);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cylinder Using Circle object
   *
   * @param circle
   * @param color
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerCylinderUsingCircle(
      const Circle& circle, const ColorARGB& color, const int& id,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::CYLINDER;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = circle.radius * 2;
    p_marker->scale.y = circle.radius * 2;
    p_marker->scale.z = 1;
    p_marker->pose.position.x = circle.center.x;
    p_marker->pose.position.y = circle.center.y;
    p_marker->pose.position.z = 0;
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cylinder Using Point object
   *
   * @param pt
   * @param scale
   * @param color
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerCylinderUsingPoint(
      const Point& pt, const Vec3f& scale, const ColorARGB& color,
      const int& id, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::CYLINDER;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    FillColorInMarker(color, p_marker);
    FillScaleInMarker(scale, p_marker);
    p_marker->pose.position.x = pt.x;
    p_marker->pose.position.y = pt.y;
    p_marker->pose.position.z = pt.z;
    return kSuccess;
  }

  /**
   * @brief Get 3-Dof State From Ros Pose
   *
   * @param pose
   * @param p_state
   * @return ErrorType
   */
  static ErrorType Get3DofStateFromRosPose(const geometry_msgs::Pose& pose,
                                           Vec3f* p_state) {
    (*p_state)(0) = pose.position.x;
    (*p_state)(1) = pose.position.y;
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.orientation, q);
    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    (*p_state)(2) = yaw;
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cube Using Oriented Bounding Box 2D object
   *
   * @param obb
   * @param color
   * @param scale_z
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerCubeUsingOrientedBoundingBox2D(
      const OrientedBoundingBox2D& obb, const ColorARGB& color,
      const decimal_t& scale_z, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::CUBE;
    p_marker->action = visualization_msgs::Marker::MODIFY;

    Vec3f scale(obb.length, obb.width, scale_z);
    FillScaleColorInMarker(scale, color, p_marker);
    geometry_msgs::Pose obb_pose;
    GetRosPoseFrom3DofState(Vec3f(obb.x, obb.y, obb.angle), &obb_pose);
    p_marker->pose = obb_pose;
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cube Using Oriented Bounding Box 2 D With Offset
   * Z object
   *
   * @param obb
   * @param offset_z
   * @param color
   * @param scale_z
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerCubeUsingOrientedBoundingBox2DWithOffsetZ(
      const OrientedBoundingBox2D& obb, const decimal_t offset_z,
      const ColorARGB& color, const decimal_t& scale_z,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::CUBE;
    p_marker->action = visualization_msgs::Marker::MODIFY;

    Vec3f scale(obb.length, obb.width, scale_z);
    FillScaleColorInMarker(scale, color, p_marker);
    geometry_msgs::Pose obb_pose;
    GetRosPoseFrom3DofState(Vec3f(obb.x, obb.y, obb.angle), &obb_pose);
    obb_pose.position.z = offset_z;
    p_marker->pose = obb_pose;
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cube Using Axis Aligned Bounding Box 3 D object
   *
   * @param aabb
   * @param color
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerCubeUsingAxisAlignedBoundingBox3D(
      const AxisAlignedBoundingBoxND<3>& aabb, const ColorARGB& color,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::CUBE;
    p_marker->action = visualization_msgs::Marker::MODIFY;

    Vec3f scale(aabb.len[0], aabb.len[1], aabb.len[2]);
    FillScaleColorInMarker(scale, color, p_marker);

    p_marker->pose.position.x = aabb.coord[0];
    p_marker->pose.position.y = aabb.coord[1];
    p_marker->pose.position.z = aabb.coord[2];
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cube Using Axis Aligned Cube 3 D object
   *
   * @param aabb
   * @param color
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerCubeUsingAxisAlignedCube3D(
      const AxisAlignedCubeNd<int, 3>& aabb, const ColorARGB& color,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::CUBE;
    p_marker->action = visualization_msgs::Marker::MODIFY;

    decimal_t x_len = aabb.upper_bound[0] - aabb.lower_bound[0];
    decimal_t y_len = aabb.upper_bound[1] - aabb.lower_bound[1];
    decimal_t z_len = aabb.upper_bound[2] - aabb.lower_bound[2];

    Vec3f scale(x_len, y_len, z_len);
    FillScaleColorInMarker(scale, color, p_marker);

    p_marker->pose.position.x =
        (aabb.upper_bound[0] + aabb.lower_bound[0]) / 2.0;
    p_marker->pose.position.y =
        (aabb.upper_bound[1] + aabb.lower_bound[1]) / 2.0;
    p_marker->pose.position.z =
        (aabb.upper_bound[2] + aabb.lower_bound[2]) / 2.0;
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Mesh Using Oriented Bounding Box 2 D object
   *
   * @param obb
   * @param color
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerMeshUsingOrientedBoundingBox2D(
      const OrientedBoundingBox2D& obb, const ColorARGB& color,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->mesh_resource = "package://common/materials/bmw_x5.dae";
    p_marker->mesh_use_embedded_materials = true;
    FillScaleInMarker(Vec3f(1.0, 1.0, 1.0), p_marker);
    FillColorInMarker(color, p_marker);
    geometry_msgs::Pose obb_pose;
    GetRosPoseFrom3DofState(Vec3f(obb.x, obb.y, obb.angle), &obb_pose);
    // obb_pose.position.z = -0.4;
    p_marker->pose = obb_pose;
    tf::Quaternion q(0.0, -0.7071, -0.7071, 0.0);
    quaternionTFToMsg(
        tf::Quaternion(obb_pose.orientation.x, obb_pose.orientation.y,
                       obb_pose.orientation.z, obb_pose.orientation.w) *
            q,
        p_marker->pose.orientation);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Mesh Cone Using Position object
   *
   * @param pos
   * @param color
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerMeshConeUsingPosition(
      const Vec3f& pos, const ColorARGB& color, const int& id,
      visualization_msgs::Marker* p_marker) {
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = ros::Time::now();
    p_marker->id = id;
    p_marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->mesh_resource = "package://common/materials/traffic_cone.dae";
    // p_marker->mesh_use_embedded_materials = true;
    FillScaleInMarker(Vec3f(2, 2, 2), p_marker);
    FillColorInMarker(color, p_marker);
    p_marker->pose.position.x = pos(0);
    p_marker->pose.position.y = pos(1);
    p_marker->pose.position.z = pos(2);
    tf::Quaternion q(0.0, -0.7071, -0.7071, 0.0);
    quaternionTFToMsg(q, p_marker->pose.orientation);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Mesh Hexagon Sign Using Position object
   *
   * @param state
   * @param z_offset
   * @param color
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerMeshHexagonSignUsingPosition(
      const Vec3f& state, const decimal_t& z_offset, const ColorARGB& color,
      const int& id, visualization_msgs::Marker* p_marker) {
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = ros::Time::now();
    p_marker->id = id;
    p_marker->type = visualization_msgs::Marker::MESH_RESOURCE;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->mesh_resource = "package://common/materials/hexagon_sign.dae";
    // p_marker->mesh_use_embedded_materials = true;
    FillScaleInMarker(Vec3f(0.5, 0.5, 0.5), p_marker);
    FillColorInMarker(color, p_marker);
    geometry_msgs::Pose pose;
    GetRosPoseFrom3DofState(state, &pose);
    pose.position.z = z_offset;
    p_marker->pose = pose;
    // tf::Quaternion q(0.0, -0.7071, -0.7071, 0.0);
    // quaternionTFToMsg(q, p_marker->pose.orientation);
    return kSuccess;
  }

  static ErrorType GetRosMarkerArrowUsingOriginAndVector(
      const Vec3f& origin, const Vec3f& vec, visualization_msgs::Marker* p_mk) {
    p_mk->type = visualization_msgs::Marker::ARROW;
    p_mk->action = visualization_msgs::Marker::MODIFY;

    geometry_msgs::Point pt0, pt1;
    pt0.x = origin(0);
    pt0.y = origin(1);
    pt0.z = origin(2);
    pt1.x = origin(0) + vec(0);
    pt1.y = origin(1) + vec(1);
    pt1.z = origin(2) + vec(2);

    p_mk->points.push_back(pt0);
    p_mk->points.push_back(pt1);

    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Arrow Using Pose And Norm object
   *
   * @param pose
   * @param norm
   * @param color
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerArrowUsingPoseAndNorm(
      const geometry_msgs::Pose& pose, const double& norm,
      const ColorARGB& color, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::ARROW;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->pose = pose;
    p_marker->scale.x = std::max(0.15, norm);  // 0.0 will cause rviz warning
    p_marker->scale.y = 0.15;
    p_marker->scale.z = 0.15;
    FillColorInMarker(color, p_marker);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Line Strip Using 2 Dof Vec object
   *
   * @param path
   * @param color
   * @param scale
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerLineStripUsing2DofVec(
      const vec_E<Vec2f>& path, const ColorARGB& color, const Vec3f& scale,
      const int& id, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = 0.0;
      p_marker->points.push_back(pt);
    }
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Line Strip Using 2 Dof Vec With Offset Z object
   *
   * @param path
   * @param color
   * @param scale
   * @param z
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerLineStripUsing2DofVecWithOffsetZ(
      const vec_E<Vec2f>& path, const ColorARGB& color, const Vec3f& scale,
      const decimal_t& z, const int& id, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = z;
      p_marker->points.push_back(pt);
    }
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Line Strip Using 3 Dof State Vec object
   *
   * @param path
   * @param z_offset
   * @param color
   * @param scale
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerLineStripUsing3DofStateVec(
      const std::vector<Vec3f>& path, const decimal_t z_offset,
      const ColorARGB& color, const Vec3f& scale, const int& id,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = z_offset;
      p_marker->points.push_back(pt);
    }
    FillColorInMarker(color, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Text Using Position And String object
   *
   * @param pos
   * @param str
   * @param color
   * @param scale
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerTextUsingPositionAndString(
      const Vec3f& pos, const std::string& str, const ColorARGB& color,
      const Vec3f& scale, const int& id, visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    p_marker->pose.position.x = pos(0);
    p_marker->pose.position.y = pos(1);
    p_marker->pose.position.z = pos(2);
    p_marker->text = str;
    FillColorInMarker(color, p_marker);
    FillScaleInMarker(scale, p_marker);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Line Strip Gradient Color Using 3 Dof State Vec
   * object
   *
   * @param path
   * @param scale
   * @param offset_z
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerLineStripGradientColorUsing3DofStateVec(
      const std::vector<Vec3f>& path, const Vec3f& scale,
      const decimal_t& offset_z, const int& id,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    for (const auto& state : path) {
      geometry_msgs::Point pt;
      pt.x = state(0);
      pt.y = state(1);
      pt.z = offset_z;
      p_marker->points.push_back(pt);
    }
    FillGradientColorInMarker(0, p_marker);
    p_marker->scale.x = scale(0);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Line Strip Using Points object
   *
   * @param points
   * @param scale
   * @param color
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerLineStripUsingPoints(
      const std::vector<Point> points, const Vec3f& scale,
      const ColorARGB& color, const int& id,
      visualization_msgs::Marker* p_marker) {
    p_marker->type = visualization_msgs::Marker::LINE_STRIP;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = id;
    FillScaleColorInMarker(scale, color, p_marker);
    for (const auto& p : points) {
      geometry_msgs::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      p_marker->points.push_back(point);
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Array Using Vehicle object
   *
   * @param vehicle
   * @param color_obb
   * @param color_vel_vec
   * @param color_steer
   * @param id
   * @param p_marker_array
   * @return ErrorType
   */
  static ErrorType GetRosMarkerArrayUsingVehicle(
      const Vehicle& vehicle, const ColorARGB& color_obb,
      const ColorARGB& color_vel_vec, const ColorARGB& color_steer,
      const int& id, visualization_msgs::MarkerArray* p_marker_array) {
    ros::Time ros_time = ros::Time::now();
    // OBB
    visualization_msgs::Marker obb_marker;
    obb_marker.header.frame_id = "map";
    obb_marker.header.stamp = ros_time;
    obb_marker.id = id;
    // obb_marker.ns = std::string("obb");
    OrientedBoundingBox2D obb = vehicle.RetOrientedBoundingBox();
    GetRosMarkerCubeUsingOrientedBoundingBox2D(obb, color_obb, 1.7,
                                               &obb_marker);
    obb_marker.pose.position.z = 0.75;

    // Vehicle model
    visualization_msgs::Marker mesh_marker;
    mesh_marker.header.frame_id = "map";
    mesh_marker.header.stamp = ros_time;
    mesh_marker.id = id + 1;
    // mesh_marker.ns = std::string("model");
    GetRosMarkerMeshUsingOrientedBoundingBox2D(obb, color_obb, &mesh_marker);

    // Velocity vector
    visualization_msgs::Marker vel_vec_marker;
    vel_vec_marker.header.frame_id = "map";
    vel_vec_marker.header.stamp = ros_time;
    vel_vec_marker.id = id + 2;
    // vel_vec_marker.ns = std::string("vel_vec");
    geometry_msgs::Pose pose;
    GetRosPoseFrom3DofState(vehicle.Ret3DofState(), &pose);
    pose.position.z = 0.8;
    GetRosMarkerArrowUsingPoseAndNorm(pose, vehicle.state().velocity,
                                      color_vel_vec, &vel_vec_marker);

    // Velocity text
    visualization_msgs::Marker vel_text_marker;
    vel_text_marker.header.frame_id = "map";
    vel_text_marker.header.stamp = ros_time;
    auto pos = vehicle.Ret3DofState();
    pos(2) = 3.0;
    std::string str;
    str += std::string(std::to_string(vehicle.id()) + "_");
    // TODO: (@denny.ding) remove this code!
    decimal_t visualized_vel = vehicle.state().velocity;
    // if (visualized_vel < 0.1) visualized_vel = 0.0;
    str += std::string(
        GetStringByValueWithPrecision<decimal_t>(visualized_vel, 3) + " m/s\n");
    // str += std::string("lon_acc: " +
    // GetStringByValueWithPrecision<decimal_t>(
    //                                      vehicle.state().acceleration, 3));
    // vel_text_marker.ns = std::string("txt");
    GetRosMarkerTextUsingPositionAndString(pos, str, cmap.at("black"),
                                           Vec3f(0.75, 0.75, 0.75), id + 3,
                                           &vel_text_marker);

    // Steering angle
    double arc_length = 10;
    visualization_msgs::Marker steering_angle_marker;
    steering_angle_marker.header.frame_id = "map";
    steering_angle_marker.header.stamp = ros_time;
    Vec3f state(vehicle.state().vec_position(0),
                vehicle.state().vec_position(1), vehicle.state().angle);

    common::CircleArc arc(state, vehicle.state().curvature, arc_length);
    std::vector<Vec3f> arc_samples;
    arc.GetSampledStates(0.2, &arc_samples);
    GetRosMarkerLineStripUsing3DofStateVec(arc_samples, +0.2, color_steer,
                                           Vec3f(0.1, 0, 0), id + 4,
                                           &steering_angle_marker);
    // steering_angle_marker.ns = std::string("steer_p");

    visualization_msgs::Marker steering_angle_marker_reverse;
    steering_angle_marker_reverse.header.frame_id = "map";
    steering_angle_marker_reverse.header.stamp = ros_time;
    Vec3f state_reverse = state;
    state_reverse(2) = normalize_angle(kPi + state_reverse(2));
    common::CircleArc arc2(state_reverse, -1.0 * vehicle.state().curvature,
                           arc_length);
    std::vector<Vec3f> arc_samples2;
    arc2.GetSampledStates(0.2, &arc_samples2);
    GetRosMarkerLineStripUsing3DofStateVec(arc_samples2, +0.2, color_steer,
                                           Vec3f(0.1, 0, 0), id + 5,
                                           &steering_angle_marker_reverse);
    // steering_angle_marker_reverse.ns = std::string("steer_n");

    visualization_msgs::Marker horizontal_marker;
    horizontal_marker.header.frame_id = "map";
    horizontal_marker.header.stamp = ros_time;
    auto state3df = vehicle.Ret3DofState();
    std::vector<Vecf<3>> horizontal_line;
    const decimal_t line_width = 1.7;
    horizontal_line.emplace_back(state3df[0] + line_width * sin(state3df[2]),
                                 state3df[1] - line_width * cos(state3df[2]),
                                 -0.2);
    horizontal_line.emplace_back(state3df[0] - line_width * sin(state3df[2]),
                                 state3df[1] + line_width * cos(state3df[2]),
                                 -0.2);
    GetRosMarkerLineStripUsing3DofStateVec(horizontal_line, -0.4, color_steer,
                                           Vec3f(0.1, 0, 0), id + 6,
                                           &horizontal_marker);

    if (vehicle.id() == 0) {
      p_marker_array->markers.push_back(mesh_marker);
    } else {
      p_marker_array->markers.push_back(obb_marker);
    }
    p_marker_array->markers.push_back(vel_vec_marker);
    p_marker_array->markers.push_back(vel_text_marker);
    // p_marker_array->markers.push_back(steering_angle_marker);
    // p_marker_array->markers.push_back(steering_angle_marker_reverse);
    // p_marker_array->markers.push_back(horizontal_marker);

    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Using Circle Obstacle object
   *
   * @param obs
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerUsingCircleObstacle(
      const CircleObstacle& obs, visualization_msgs::Marker* p_marker) {
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = ros::Time::now();
    GetRosMarkerCylinderUsingCircle(obs.circle, ColorARGB(0.5, 1.0, 1.0, 1.0),
                                    obs.id, p_marker);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Using Polygon Obstacle object
   *
   * @param obs
   * @param id
   * @param p_marker
   * @return ErrorType
   */
  static ErrorType GetRosMarkerUsingPolygonObstacle(
      const PolygonObstacle& obs, const int& id,
      visualization_msgs::Marker* p_marker) {
    std::vector<Point> points = obs.polygon.points;
    points.push_back(*(points.begin()));
    for (auto& p : points) {
      p.z = -0.2;
    }
    p_marker->header.frame_id = "map";
    p_marker->header.stamp = ros::Time::now();
    GetRosMarkerLineStripUsingPoints(
        points, Vec3f(0.2, 0, 0), ColorARGB(1.0, 0.7, 0.7, 0.7), id, p_marker);
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Using Obstacle Set object
   *
   * @param obstacles
   * @param p_marker_array
   * @return ErrorType
   */
  static ErrorType GetRosMarkerUsingObstacleSet(
      const ObstacleSet& obstacles,
      visualization_msgs::MarkerArray* p_marker_array) {
    for (const auto& p_obs : obstacles.obs_circle) {
      visualization_msgs::Marker obs_marker;
      GetRosMarkerUsingCircleObstacle(p_obs.second, &obs_marker);
      p_marker_array->markers.push_back(obs_marker);
    }
    int id_cnt = 0;
    for (const auto& p_obs : obstacles.obs_polygon) {
      switch (p_obs.second.type) {
        case 0: {
          visualization_msgs::Marker obs_marker;
          GetRosMarkerUsingPolygonObstacle(p_obs.second, id_cnt, &obs_marker);
          p_marker_array->markers.push_back(obs_marker);
          ++id_cnt;
          break;
        }
        case 1: {
          for (const auto& pt : p_obs.second.polygon.points) {
            visualization_msgs::Marker obs_marker;
            Vec3f pos(pt.x, pt.y, 0.4);
            GetRosMarkerMeshConeUsingPosition(pos, cmap.at("yellow"), id_cnt,
                                              &obs_marker);
            p_marker_array->markers.push_back(obs_marker);
            ++id_cnt;
          }
          break;
        }
        default:
          break;
      }
    }
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Arr Using Semantic Behavior object
   *
   * @param behavior
   * @param p_marker_array
   * @return ErrorType
   */
  static ErrorType GetRosMarkerArrUsingSemanticBehavior(
      const SemanticBehavior& behavior,
      visualization_msgs::MarkerArray* p_marker_array) {
    // lane direction marker
    {
      if (behavior.ref_lane.IsValid()) {
        visualization_msgs::Marker direction_mk;
        direction_mk.header.stamp = ros::Time::now();
        direction_mk.header.frame_id = std::string("map");
        direction_mk.id = 0;
        direction_mk.type = visualization_msgs::Marker::LINE_LIST;
        direction_mk.action = visualization_msgs::Marker::MODIFY;
        decimal_t angle = 0.0;  // angle between horizontal line & direction
        if (behavior.lat_behavior == common::LateralBehavior::kLaneChangeLeft ||
            behavior.lat_behavior ==
                common::LateralBehavior::kLaneChangeRight) {
          FillScaleColorInMarker(Vec3f(0.4, 0.0, 0.0),
                                 ColorARGB(1.0, 1.0, 1.0, 0.0), &direction_mk);
          angle = kPi / 4.0;
        } else if (behavior.lon_behavior ==
                   common::LongitudinalBehavior::kDecelerate) {
          FillScaleColorInMarker(Vec3f(0.4, 0.0, 0.0),
                                 ColorARGB(1.0, 1.0, 0.0, 0.0), &direction_mk);
          angle = 0.0;
        } else {
          FillScaleColorInMarker(Vec3f(0.4, 0.0, 0.0),
                                 ColorARGB(1.0, 0.1, 0.8, 0.1), &direction_mk);
          angle = kPi / 4.0;
        }

        const decimal_t sample_step = 5.0;
        const decimal_t arrow_width = 0.75;
        for (decimal_t s = behavior.ref_lane.begin();
             s < behavior.ref_lane.end(); s += sample_step) {
          Vecf<2> pos;
          behavior.ref_lane.GetPositionByArcLength(s, &pos);
          Vecf<2> normal_vec;
          behavior.ref_lane.GetNormalVectorByArcLength(s, &normal_vec);

          geometry_msgs::Point origin;
          ConvertVectorToPoint<2>(pos, &origin);
          {
            geometry_msgs::Point left_arrow;
            Vecf<2> left = pos + arrow_width / acos(angle) *
                                     rotate_vector_2d(normal_vec, angle);
            ConvertVectorToPoint<2>(left, &left_arrow);
            direction_mk.points.push_back(origin);
            direction_mk.points.push_back(left_arrow);
          }
          {
            geometry_msgs::Point right_arrow;
            Vecf<2> right = pos + arrow_width / acos(angle) *
                                      rotate_vector_2d(-normal_vec, -angle);
            ConvertVectorToPoint<2>(right, &right_arrow);
            direction_mk.points.push_back(origin);
            direction_mk.points.push_back(right_arrow);
          }
        }
        p_marker_array->markers.push_back(direction_mk);
      }  // end if valid
    }

    // visualize curvature
    {
      decimal_t sample_step = 1.0;
      if (behavior.ref_lane.IsValid()) {
        visualization_msgs::Marker curvature_mk;
        curvature_mk.header.stamp = ros::Time::now();
        curvature_mk.header.frame_id = std::string("map");
        curvature_mk.id = 1;
        curvature_mk.type = visualization_msgs::Marker::LINE_STRIP;
        curvature_mk.action = visualization_msgs::Marker::MODIFY;
        curvature_mk.scale.x = 0.2;
        for (decimal_t s = behavior.ref_lane.begin();
             s < behavior.ref_lane.end(); s += sample_step) {
          Vecf<2> pos;
          behavior.ref_lane.GetPositionByArcLength(s, &pos);
          geometry_msgs::Point origin;
          ConvertVectorToPoint<2>(pos, &origin);
          curvature_mk.points.push_back(origin);

          decimal_t c, cc;
          behavior.ref_lane.GetCurvatureByArcLength(s, &c, &cc);
          common::ColorARGB color =
              common::GetJetColorByValue(fabs(c), 0.4, 0.0);
          std_msgs::ColorRGBA c_ros;
          c_ros.a = color.a;
          c_ros.r = color.r;
          c_ros.g = color.g;
          c_ros.b = color.b;
          curvature_mk.colors.push_back(c_ros);
        }
        p_marker_array->markers.push_back(curvature_mk);
      }
    }

    // visualize longitudinal behavior
    {
      if (behavior.ref_lane.IsValid()) {
        common::ColorARGB clr(1.0, 1.0, 0, 0);
        decimal_t length = 0.0;
        switch (behavior.lon_behavior) {
          case common::LongitudinalBehavior::kMaintain: {
            length = 0.0;
            clr = common::ColorARGB(1.0, 0.0, 1.0, 0.0);
            break;
          }
          case common::LongitudinalBehavior::kAccelerate: {
            length = 0.75;
            clr = common::ColorARGB(1.0, 1.0, 1.0, 0);
            break;
          }
          case common::LongitudinalBehavior::kDecelerate: {
            length = -0.75;
            clr = common::ColorARGB(1.0, 1.0, 0, 0);
            break;
          }
          default:
            break;
        }

        visualization_msgs::Marker lon_mk;
        lon_mk.header.stamp = ros::Time::now();
        lon_mk.header.frame_id = std::string("map");
        lon_mk.id = 3;
        lon_mk.type = visualization_msgs::Marker::ARROW;
        lon_mk.action = visualization_msgs::Marker::MODIFY;
        geometry_msgs::Point pt0, pt1;
        pt0.x = behavior.state.vec_position(0);
        pt0.y = behavior.state.vec_position(1);
        pt0.z = 2.5;
        pt1.x = behavior.state.vec_position(0);
        pt1.y = behavior.state.vec_position(1);
        pt1.z = pt0.z + length;
        lon_mk.points.push_back(pt0);
        lon_mk.points.push_back(pt1);
        lon_mk.scale.x = 0.2;
        lon_mk.scale.y = 0.4;
        FillColorInMarker(clr, &lon_mk);
        p_marker_array->markers.push_back(lon_mk);
      }
    }

    // forward trajs
    {
      auto surround_trajs_set = behavior.surround_trajs;
      common::ColorARGB traj_color = cmap.at("sky blue");
      traj_color.a = 0.8;
      double traj_z = 0.3;
      int cnt = 100;

      for (const auto& surround_trajs : surround_trajs_set) {
        for (const auto& p_traj : surround_trajs) {
          std::vector<common::Point> points;
          for (const auto& v : p_traj.second) {
            common::Point pt(v.state().vec_position(0),
                             v.state().vec_position(1));
            pt.z = traj_z;
            points.push_back(pt);
            visualization_msgs::Marker point_marker;

            point_marker.header.stamp = ros::Time::now();
            point_marker.header.frame_id = std::string("map");

            common::VisualizationUtil::GetRosMarkerCylinderUsingPoint(
                common::Point(pt), Vec3f(0.5, 0.5, 0.1), traj_color, ++cnt,
                &point_marker);
            p_marker_array->markers.push_back(point_marker);
          }
          visualization_msgs::Marker line_marker;
          line_marker.header.stamp = ros::Time::now();
          line_marker.header.frame_id = std::string("map");
          common::VisualizationUtil::GetRosMarkerLineStripUsingPoints(
              points, Vec3f(0.1, 0.1, 0.1), traj_color, ++cnt, &line_marker);
          p_marker_array->markers.push_back(line_marker);
        }
      }
    }

    return kSuccess;
  }

  /**
   * @brief Convert GridMapND<T, 2> to nav_msgs::OccupancyGrid
   *
   * @tparam T Data type
   * @param GridMapND in type T
   * @param time_stamp ROS timestamp
   * @param p_occ_grid Pointer of ROS nav_msgs::OccupancyGrid
   * @return ErrorType
   */
  template <typename T>
  static ErrorType GetRosOccupancyGridUsingGripMap2D(
      const GridMapND<T, 2>& grid_map, const ros::Time& time_stamp,
      nav_msgs::OccupancyGrid* p_occ_grid) {
    p_occ_grid->header.frame_id = "map";
    p_occ_grid->header.stamp = time_stamp;
    p_occ_grid->info.height = grid_map.dims_size(0);
    p_occ_grid->info.width = grid_map.dims_size(1);
    p_occ_grid->info.resolution = grid_map.dims_resolution(0);
    geometry_msgs::Pose origin;
    Vec3f origin_pose(grid_map.origin()[0], grid_map.origin()[1], 0.0);
    GetRosPoseFrom3DofState(origin_pose, &origin);
    p_occ_grid->info.origin = origin;
    p_occ_grid->info.map_load_time = time_stamp;

    p_occ_grid->data.resize(grid_map.data_size());
    std::copy(grid_map.data()->begin(), grid_map.data()->end(),
              p_occ_grid->data.begin());
    return kSuccess;
  }

  /**
   * @brief Get the Ros Marker Cube List Using Grip Map 3 D object
   *
   * @tparam T
   * @param p_grid_map
   * @param time_stamp
   * @param frame_id
   * @param pose
   * @param p_marker
   * @return ErrorType
   */
  template <typename T>
  static ErrorType GetRosMarkerCubeListUsingGripMap3D(
      const GridMapND<T, 3>* p_grid_map, const ros::Time& time_stamp,
      const std::string& frame_id, const Vec3f& pose,
      visualization_msgs::Marker* p_marker) {
    p_marker->header.frame_id = frame_id;
    p_marker->header.stamp = time_stamp;
    p_marker->type = visualization_msgs::Marker::CUBE_LIST;
    p_marker->action = visualization_msgs::Marker::MODIFY;
    p_marker->id = 0;

    geometry_msgs::Pose pose_origin;
    GetRosPoseFrom3DofState(pose, &pose_origin);
    p_marker->pose = pose_origin;

    p_marker->scale.x = p_grid_map->dims_resolution(0);
    p_marker->scale.y = p_grid_map->dims_resolution(1);
    p_marker->scale.z = p_grid_map->dims_resolution(2);

    auto origin = p_grid_map->origin();

    int ele_num = p_grid_map->data_size();
    p_marker->points.reserve(ele_num);
    p_marker->colors.reserve(ele_num);

    const T* map_ptr = p_grid_map->data_ptr();
    // int z_max = p_grid_map->dims_size(2);
    std::array<int, 3> idx;
    std::array<decimal_t, 3> p_w;
    for (int i = 0; i < p_grid_map->data_size(); ++i) {
      if (i > p_grid_map->dims_step(2)) {
        break;
      }

      if (*(map_ptr + i) == 0) continue;

      idx = p_grid_map->GetNDimIdxUsingMonoIdx(i);
      p_grid_map->GetGlobalPositionUsingCoordinate(idx, &p_w);

      geometry_msgs::Point pt;
      pt.x = p_w[0];
      pt.y = p_w[1];
      pt.z = p_w[2] - origin[2];
      p_marker->points.push_back(pt);

      std_msgs::ColorRGBA clr_ros;
      // ColorARGB clr = GetJetColorByValue(idx[2], z_max, 0);
      // clr_ros.a = 0.95;
      // clr_ros.r = clr.r;
      // clr_ros.g = clr.g;
      // clr_ros.b = clr.b;

      clr_ros.a = 0.5;
      clr_ros.r = 1.0;
      clr_ros.g = 0.0;
      clr_ros.b = 0.0;
      p_marker->colors.push_back(clr_ros);
    }
    return kSuccess;
  }
};

}  // namespace common

#endif  // _COMMON_INC_COMMON_VISUALIZATION_VISUALIZATION_UTIL_H__
