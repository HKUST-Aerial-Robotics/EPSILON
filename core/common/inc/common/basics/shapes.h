/**
 * @file shapes.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_SHAPES_H__
#define _COMMON_INC_COMMON_BASICS_SHAPES_H__

#include <map>
#include <opencv2/core/core.hpp>

#include "common/basics/basics.h"
#include "common/basics/colormap.h"
#include "common/math/calculations.h"

namespace common {

struct Point {
  decimal_t x = 0.0;
  decimal_t y = 0.0;
  decimal_t z = 0.0;

  /**
   * @brief Default constructor
   */
  Point();

  /**
   * @brief Construct a new Point object
   *
   * @param _x
   * @param _y
   */
  Point(decimal_t _x, decimal_t _y);

  /**
   * @brief Construct a new Point object
   *
   * @param _x
   * @param _y
   * @param _z
   */
  Point(decimal_t _x, decimal_t _y, decimal_t _z);

  /**
   * @brief Print info
   */
  void print() const;
};

struct Point2i {
  int x = 0;
  int y = 0;

  /**
   * @brief  Default constructor
   */
  Point2i();

  /**
   * @brief Construct a new Point 2i object
   *
   * @param _x
   * @param _y
   */
  Point2i(int _x, int _y);

  /**
   * @brief Print info
   */
  void print() const;
};

template <typename T>
struct PointWithValue {
  Point pt;
  std::vector<T> values;
};

struct OrientedBoundingBox2D {
  decimal_t x;
  decimal_t y;
  decimal_t angle;
  decimal_t width;
  decimal_t length;

  /**
   * @brief Default constructor
   */
  OrientedBoundingBox2D();

  /**
   * @brief Construct a new Oriented Bounding Box 2 D object
   *
   * @param x_ x of center of obb
   * @param y_ y of center of obb
   * @param angle_ angle between x-positive and longitudinal axis of obb
   * @param width_ width of obb
   * @param length_ length of obb
   */
  OrientedBoundingBox2D(const decimal_t x_, const decimal_t y_,
                        const decimal_t angle_, const decimal_t width_,
                        const decimal_t length_);
};

template <int N>
struct AxisAlignedBoundingBoxND {
  std::array<decimal_t, N> coord;
  std::array<decimal_t, N> len;

  /**
   * @brief Default constructor
   */
  AxisAlignedBoundingBoxND() {}

  /**
   * @brief Construct a new AxisAlignedBoundingBoxND object
   *
   * @param coord_ Center of AABB
   * @param len_ Length of AABB
   */
  AxisAlignedBoundingBoxND(const std::array<decimal_t, N> coord_,
                           const std::array<decimal_t, N> len_)
      : coord(coord_), len(len_) {}
};

template <typename T, int N_DIM>
struct AxisAlignedCubeNd {
  std::array<T, N_DIM> upper_bound;
  std::array<T, N_DIM> lower_bound;

  AxisAlignedCubeNd() = default;
  AxisAlignedCubeNd(const std::array<T, N_DIM> ub,
                    const std::array<T, N_DIM> lb)
      : upper_bound(ub), lower_bound(lb) {}
};

struct Circle {
  Point center;
  decimal_t radius;

  /**
   * @brief Print info
   */
  void print() const;
};

struct PolyLine {
  int dir;
  std::vector<Point> points;

  /**
   * @brief Print info
   */
  void print() const;
};

struct Polygon {
  std::vector<Point> points;

  /**
   * @brief Print info
   */
  void print() const;
};

class ShapeUtils {
 public:
  /**
   * @brief Check if 2 OBBs intersect
   *
   * @param obb_a OBB A
   * @param obb_b OBB B
   * @return true Intersect
   * @return false No intersect
   */
  static bool CheckIfOrientedBoundingBoxIntersect(
      const OrientedBoundingBox2D& obb_a, const OrientedBoundingBox2D& obb_b);

  /**
   * @brief Get the Vertices Of OrientedBoundingBox object
   *
   * @param obb Input OBB
   * @param vertices Output vertice vector
   * @return ErrorType
   */
  static ErrorType GetVerticesOfOrientedBoundingBox(
      const OrientedBoundingBox2D& obb, vec_E<Vecf<2>>* vertices);

  /**
   * @brief Get the Perpendicular Axes Of Oriented Bounding Box
   *
   * @param vertices
   * @param axes
   * @return ErrorType
   */
  static ErrorType GetPerpendicularAxesOfOrientedBoundingBox(
      const vec_E<Vecf<2>>& vertices, vec_E<Vecf<2>>* axes);

  /**
   * @brief Get the Projection On Axis object
   *
   * @param vertices
   * @param axis
   * @param proj
   * @return ErrorType
   */
  static ErrorType GetProjectionOnAxis(const vec_E<Vecf<2>>& vertices,
                                       const Vecf<2>& axis, Vecf<2>* proj);

  /**
   * @brief Get the Perpendicular Axis Of Oriented Bounding Box object
   *
   * @param vertices
   * @param index
   * @param axis
   * @return ErrorType
   */
  static ErrorType GetPerpendicularAxisOfOrientedBoundingBox(
      const vec_E<Vecf<2>>& vertices, const int index, Vecf<2>* axis);

  /**
   * @brief Get the Overlap Length
   *
   * @param a
   * @param b
   * @param len
   * @return ErrorType
   */
  static ErrorType GetOverlapLength(const Vecf<2> a, const Vecf<2> b,
                                    decimal_t* len);

  /**
   * @brief Get the cv::Point2i Vector Using Common::Point2i Vector
   *
   * @param pts_in Input common::Point2i vector
   * @param pts_out Output cv::Point2i vector
   * @return ErrorType
   */
  static ErrorType GetCvPoint2iVecUsingCommonPoint2iVec(
      const std::vector<Point2i>& pts_in, std::vector<cv::Point2i>* pts_out);

  /**
   * @brief Get the cv::Point2i Using Common::Point2i
   *
   * @param pt_in Input Common::Point2i
   * @param pt_out Output cv::Point2i
   * @return ErrorType
   */
  static ErrorType GetCvPoint2iUsingCommonPoint2i(const Point2i& pt_in,
                                                  cv::Point2i* pt_out);

  /**
   * @brief
   *
   * @tparam T
   * @tparam N_DIM
   * @param cube_a
   * @param cube_b
   * @return true
   * @return false
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(
      const common::AxisAlignedCubeNd<T, N_DIM>& cube_a,
      const common::AxisAlignedCubeNd<T, N_DIM>& cube_b) {
    for (int i = 0; i < N_DIM; ++i) {
      if (cube_a.lower_bound[i] > cube_b.lower_bound[i] ||
          cube_a.upper_bound[i] < cube_b.upper_bound[i]) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief
   *
   * @tparam T
   * @tparam N_DIM
   * @param aabb_a
   * @param aabb_b
   * @return true
   * @return false
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeCollide(
      const common::AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const common::AxisAlignedCubeNd<T, N_DIM>& aabb_b) {
    for (int i = 0; i < N_DIM; ++i) {
      decimal_t half_len_a = fabs(
          static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
          2.0);
      decimal_t half_len_b = fabs(
          static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
          2.0);

      decimal_t center_a =
          static_cast<double>(aabb_a.upper_bound[i] + aabb_a.lower_bound[i]) /
          2.0;
      decimal_t center_b =
          static_cast<double>(aabb_b.upper_bound[i] + aabb_b.lower_bound[i]) /
          2.0;

      decimal_t len_c = fabs(center_a - center_b);

      // ~ Seperated
      if (fabs(half_len_a + half_len_b) <= len_c) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief
   *
   * @tparam T
   * @tparam N_DIM
   * @param aabb_a
   * @param aabb_b
   * @param inter_dim_a
   * @param inter_dim_b
   * @return true If two AABBs intersect
   * @return false If two AABBs is seperated or one AABB contains another
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdIntersect(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b,
      std::array<bool, N_DIM * 2>* inter_dim_a,
      std::array<bool, N_DIM * 2>* inter_dim_b) {
    inter_dim_a->fill(false);
    inter_dim_b->fill(false);
    // ~ A contains B or B contains A
    if (CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(aabb_a, aabb_b) ||
        CheckIfAxisAlignedCubeAContainsAxisAlignedCubeB(aabb_b, aabb_a)) {
      return false;
    }
    // ~ Check if collide
    if (!CheckIfAxisAlignedCubeCollide(aabb_a, aabb_b)) {
      return false;
    }
    // ~ Calculate on which A's surface collision happened
    for (int i = 0; i < N_DIM; ++i) {
      if (CheckIfAxisAlignedCubeNdCollideOnOneDim(aabb_a, aabb_b, i)) {
        if (aabb_a.upper_bound[i] < aabb_b.upper_bound[i] &&
            aabb_a.upper_bound[i] > aabb_b.lower_bound[i]) {
          (*inter_dim_a)[2 * i] = true;
        }
        if (aabb_a.lower_bound[i] < aabb_b.upper_bound[i] &&
            aabb_a.lower_bound[i] > aabb_b.lower_bound[i]) {
          (*inter_dim_a)[2 * i + 1] = true;
        }
      }
    }
    // ~ Calculate on which B's surface collision happened
    for (int i = 0; i < N_DIM; ++i) {
      if (CheckIfAxisAlignedCubeNdCollideOnOneDim(aabb_a, aabb_b, i)) {
        if (aabb_b.upper_bound[i] > aabb_a.lower_bound[i] &&
            aabb_b.upper_bound[i] < aabb_a.upper_bound[i]) {
          (*inter_dim_b)[2 * i] = true;
        }
        if (aabb_b.lower_bound[i] > aabb_a.lower_bound[i] &&
            aabb_b.lower_bound[i] < aabb_a.upper_bound[i]) {
          (*inter_dim_b)[2 * i + 1] = true;
        }
      }
    }
    return true;
  }

  /**
   * @brief
   * @notice This function only check on dim intersection
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdIntersectionOnOneDim(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b, const int& i) {
    decimal_t half_len_a = fabs(
        static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
        2.0);
    decimal_t half_len_b = fabs(
        static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
        2.0);

    decimal_t center_a = aabb_a.lower_bound[i] + half_len_a;
    decimal_t center_b = aabb_b.lower_bound[i] + half_len_b;
    decimal_t len_c = fabs(center_a - center_b);

    // ~ Seperated or Contained
    if (fabs(half_len_a + half_len_b) <= len_c ||
        fabs(half_len_a - half_len_b) >= len_c) {
      return false;
    }
    return true;
  };

  /**
   * @brief
   * @notice This function only check on dim collision
   *
   * @tparam T
   * @tparam N_DIM
   * @param aabb_a
   * @param aabb_b
   * @param i
   * @return true
   * @return false
   */
  template <typename T, int N_DIM>
  static bool CheckIfAxisAlignedCubeNdCollideOnOneDim(
      const AxisAlignedCubeNd<T, N_DIM>& aabb_a,
      const AxisAlignedCubeNd<T, N_DIM>& aabb_b, const int& i) {
    decimal_t half_len_a = fabs(
        static_cast<double>(aabb_a.upper_bound[i] - aabb_a.lower_bound[i]) /
        2.0);
    decimal_t half_len_b = fabs(
        static_cast<double>(aabb_b.upper_bound[i] - aabb_b.lower_bound[i]) /
        2.0);

    decimal_t center_a = aabb_a.lower_bound[i] + half_len_a;
    decimal_t center_b = aabb_b.lower_bound[i] + half_len_b;
    decimal_t len_c = fabs(center_a - center_b);

    // ~ Seperated or Contained
    if (fabs(half_len_a + half_len_b) <= len_c) {
      return false;
    }
    return true;
  };

};  // class ShapeUtils

}  // namespace common

#endif  //_COMMON_INC_COMMON_BASICS_SHAPES_H__