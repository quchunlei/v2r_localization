/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Localization Group
 * Version: 0.2.0
 * Date: 2018.05
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#ifndef PROJECT_COMPUTE_H
#define PROJECT_COMPUTE_H

#include <math.h>
#include <Eigen/Core>

namespace robosense
{
namespace localization
{
// 平方运算
#define SQUARE(x) ((x) * (x))
// 将角度值归一化在(-PI, PI)范围内
#define ATAN2F(x) atan2f(sinf(x), cosf(x))

inline float getCrossAngle(const Eigen::Vector3f& src, const Eigen::Vector3f& tar)
{
  return acosf(src.normalized().dot(tar.normalized()));
}

inline float distanceTwoPoint(float x_1, float y_1, float x_2, float y_2)
{
  return sqrtf(SQUARE(x_1 - x_2) + SQUARE(y_1 - y_2));
}

inline float disOfTwoPoint(const float& x_1, const float& y_1, const float& x_2, const float& y_2)
{
  return sqrtf(SQUARE(x_1 - x_2) + SQUARE(y_1 - y_2));
}

inline float disOfTwoVector2f(const Eigen::Vector2f& one, const Eigen::Vector2f& two)
{
  return disOfTwoPoint(one.x(), one.y(), two.x(), two.y());
}

}  // namespace localization
}  // namespace robosense

#endif  // PROJECT_COMPUTE_H
