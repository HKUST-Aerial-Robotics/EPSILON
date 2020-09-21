#ifndef _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__
#define _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__

#include "common/basics/basics.h"

/**
 * @brief Sign function
 *
 * @tparam T
 * @param val
 * @return int
 */
template <typename T>
int sgn(const T val) {
  return (T(0) < val) - (val < T(0));
}

/**
 * @brief Calculate factorial
 *
 * @param n Input value
 * @return long long Result
 */
long long fac(int n);

/**
 * @brief Calculate num of combinations
 *
 * @param n, k Input value
 * @return long long Result
 */
long long nchoosek(int n, int k);

/**
 * @brief Normalize angle to [-pi, pi)
 *
 * @param theta Input value
 * @return decimal_t Result
 */
decimal_t normalize_angle(const decimal_t& theta);

/**
 * @brief Shift the vector with anti-clock-wise degree
 *
 * @param v Input vector
 * @param angle Rotation angle
 * @return Vecf<2> Output vector
 */
Vecf<2> rotate_vector_2d(const Vecf<2>& v, const decimal_t angle);

/**
 * @brief Return normalized angle by vec2d
 *
 * @param v Input vector
 * @return decimal_t Output angle
 */
decimal_t vec2d_to_angle(const Vecf<2>& v);

/**
 * @brief Return truncated value
 *
 * @param val_in
 * @param upper
 * @param lower
 * @return decimal_t
 */
decimal_t truncate(const decimal_t& val_in, const decimal_t& lower,
                   const decimal_t& upper);

/**
 * @brief Return normalize value
 *
 * @param val_in
 * @param lower
 * @param upper
 * @param new_lower
 * @param new_upper
 * @return decimal_t
 */
decimal_t normalize_with_bound(const decimal_t& val_in, const decimal_t& lower,
                               const decimal_t& upper,
                               const decimal_t& new_lower,
                               const decimal_t& new_upper);

/**
 * @brief
 *
 * @param th
 * @param val_in
 * @param val_out
 * @return ErrorType
 */
ErrorType RemapUsingQuadraticFuncAroundSmallValue(const decimal_t& th,
                                                  const decimal_t& val_in,
                                                  decimal_t* val_out);

#endif  // _CORE_COMMON_INC_COMMON_MATH_CALCULATIONS_H__