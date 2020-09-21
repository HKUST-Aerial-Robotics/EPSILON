/**
 * @file tool_func.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__
#define _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "common/basics/basics.h"

namespace common {

/**
 * @brief Split string
 *
 * @param s Input string
 * @param c Separator
 * @param v Result
 */
void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v);

/**
 * @brief Get the Result In Vector object
 *
 * @param vec
 * @param N
 * @param tmp
 * @param tmp_result
 */
void GetResultInVector(const std::vector<std::vector<int>>& vec, const int& N,
                       std::vector<int>* tmp,
                       std::vector<std::vector<int>>* tmp_result);
/**
 * @brief Get the all combinations among input vectors
 *
 * @param vec_in
 * @param res
 */
void GetAllCombinations(const std::vector<std::vector<int>>& vec_in,
                        std::vector<std::vector<int>>* res);

/**
 * @brief Get the String By Value With Precision object
 *
 * @tparam T Data typename
 * @param val Data value
 * @param pre Precision
 * @return std::string Output string
 */
template <typename T>
std::string GetStringByValueWithPrecision(const T& val, const int& pre) {
  std::ostringstream os;
  os << std::fixed;
  os << std::setprecision(pre);
  os << val;
  return os.str();
}

template <typename T>
void GetRangeVector(const T& lb, const T& ub, const T& step,
                    const bool& if_inc_tail, std::vector<T>* vec) {
  vec->clear();
  int num = std::ceil((ub - lb - kEPS) / step);
  for (int i = 0; i < num; ++i) {
    vec->push_back(lb + i * step);
  }
  if (if_inc_tail) {
    vec->push_back(ub);
  }
}

}  // namespace common

#endif  // _COMMON_INC_COMMON_BASICS_TOOL_FUNC_H__