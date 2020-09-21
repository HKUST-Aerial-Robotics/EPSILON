/**
 * @file ma_filter.h
 * @author HKUST Aerial Robotics Group
 * @brief moving average filter
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_MA_FILTER_H_
#define _COMMON_INC_COMMON_BASICS_MA_FILTER_H_

#include <vector>
#include "common/basics/basics.h"

class MAFilter {
 public:
  MAFilter() {}
  MAFilter(int L) : L_(L) {}

  void set_L(const int L) { L_ = L; }

  void add(const decimal_t new_obs) {
    observations_.push_back(new_obs);
    if (static_cast<int>(observations_.size()) > kMaxSize_) {
      observations_.erase(observations_.begin());
    }
  }

  void reset() { observations_.clear(); }

  decimal_t get() {
    int num_elements = static_cast<int>(observations_.size());
    if (num_elements == 0) return 0.0;
    int cnt = 0;
    decimal_t accumulate = 0.0;
    for (int i = num_elements - 1; i >= 0; i--) {
      accumulate += observations_[i];
      cnt++;
      if (cnt == L_) break;
    }
    return accumulate / cnt;
  }

 private:
  std::vector<decimal_t> observations_;
  const int kMaxSize_ = 10;
  int L_{3};
};

#endif