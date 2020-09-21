/**
 * @file basics.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_BASICS_H_
#define _CORE_COMMON_INC_BASICS_BASICS_H_

#include "common/basics/macros.h"
#include "common/basics/tic_toc.h"

#include <math.h>
#include <stdio.h>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>

#define BACKWARD_HAS_UNWIND 1
#define BACKWARD_HAS_DW 1
#include "backward.hpp"

enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };

using decimal_t = double;

// Eigen aliasing
template <int N>
using Vecf = Eigen::Matrix<decimal_t, N, 1>;

template <int N>
using Veci = Eigen::Matrix<int, N, 1>;

template <int M, int N>
using Matf = Eigen::Matrix<decimal_t, M, N>;

template <int M, int N>
using Mati8 = Eigen::Matrix<uint8_t, M, N>;

template <int M, int N>
using Mati = Eigen::Matrix<int, M, N>;

template <int N>
using MatNf = Matf<N, N>;

template <int N>
using MatDNf = Eigen::Matrix<decimal_t, Eigen::Dynamic, N>;

using MatDf = Matf<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi = Mati<Eigen::Dynamic, Eigen::Dynamic>;
using MatDi8 = Mati8<Eigen::Dynamic, Eigen::Dynamic>;

using Mat2f = Matf<2, 2>;
using Mat3f = Matf<3, 3>;
using Mat4f = Matf<4, 4>;

using Vec2f = Vecf<2>;
using Vec3f = Vecf<3>;
using Vec4f = Vecf<4>;

using Vec2i = Veci<2>;
using Vec3i = Veci<3>;
using Vec4i = Veci<4>;

// Workaround with STL container with eigen with fixed size eigen vector
template <typename T>
using vec_E = std::vector<T, Eigen::aligned_allocator<T>>;

template <int N>
using vec_Vecf = vec_E<Vecf<N>>;

const decimal_t kBigEPS = 1e-1;

const decimal_t kEPS = 1e-6;

const decimal_t kSmallEPS = 1e-10;

const decimal_t kPi = acos(-1.0);

const decimal_t kInf = 1e20;

const int kInvalidAgentId = -1;
const int kInvalidLaneId = -1;

#endif  // CORE_COMMON_INC_BASICS_BASICS_H