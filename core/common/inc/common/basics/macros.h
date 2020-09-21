/**
 * @file marcos.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _COMMON_INC_COMMON_MACROS_H__
#define _COMMON_INC_COMMON_MACROS_H__
#define DECLARE_BACKWARD       \
  namespace backward {         \
  backward::SignalHandling sh; \
  }
#endif