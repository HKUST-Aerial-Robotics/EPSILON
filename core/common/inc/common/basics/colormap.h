/**
 * @file colormap.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_BASICS_COLORMAP_H__
#define _COMMON_INC_COMMON_BASICS_COLORMAP_H__

#include <map>

#include "common/basics/basics.h"

namespace common {
struct ColorARGB {
  decimal_t a;
  decimal_t r;
  decimal_t g;
  decimal_t b;
  ColorARGB(decimal_t A, decimal_t R, decimal_t G, decimal_t B)
      : a(A), r(R), g(G), b(B) {}
  ColorARGB() : a(0.0), r(0.0), g(0.0), b(0.0) {}

  ColorARGB set_a(const decimal_t _a) { return ColorARGB(_a, r, g, b); }
};

extern std::map<std::string, ColorARGB> cmap;
extern std::map<decimal_t, ColorARGB> jet_map;
extern std::map<decimal_t, ColorARGB> autumn_map;

/**
 * @brief Get the color in colormap by value using LUT
 *
 * @param val input value
 * @param m color map
 * @return ColorARGB color
 */
ColorARGB GetColorByValue(const decimal_t val,
                          const std::map<decimal_t, ColorARGB>& m);

/**
 * @brief Get the Jet colormap By value using mapping function
 *
 * @param val input value
 * @param max maximum value
 * @param min minimum value
 * @return ColorARGB color
 */
ColorARGB GetJetColorByValue(const decimal_t val, const decimal_t max,
                             const decimal_t min);
}  // namespace common

#endif