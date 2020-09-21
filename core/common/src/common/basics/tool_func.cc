/**
 * @file tool_func.cc
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#include "common/basics/tool_func.h"

namespace common {
void SplitString(const std::string& s, const std::string& c,
                 std::vector<std::string>* v) {
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  while (std::string::npos != pos2) {
    v->push_back(s.substr(pos1, pos2 - pos1));

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length()) v->push_back(s.substr(pos1));
}

void GetResultInVector(const std::vector<std::vector<int>>& vec, const int& N,
                       std::vector<int>* tmp,
                       std::vector<std::vector<int>>* tmp_result) {
  for (int i = 0; i < vec[N].size(); ++i) {
    tmp->push_back(vec[N][i]);
    if (N < vec.size() - 1) {
      GetResultInVector(vec, N + 1, tmp, tmp_result);
    } else {
      std::vector<int> one_result;
      for (int i = 0; i < tmp->size(); ++i) {
        one_result.push_back(tmp->at(i));
      }
      tmp_result->push_back(one_result);
    }
    tmp->pop_back();
  }
}

void GetAllCombinations(const std::vector<std::vector<int>>& vec_in,
                        std::vector<std::vector<int>>* res) {
  std::vector<int> tmp_vec;
  GetResultInVector(vec_in, 0, &tmp_vec, res);
}

}  // namespace common