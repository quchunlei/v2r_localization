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

#ifndef MSF_CONST_H
#define MSF_CONST_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <msf/core/msf_state.h>

namespace robosense
{
namespace localization
{
namespace msf
{
// covar
Eigen::Matrix<float, 16, 16> Qi;
inline Eigen::Matrix4f getOmegaMat(const Eigen::Vector3f& vec)
{
  return (Eigen::Matrix4f() << 0, vec[2], -vec[1], vec[0], -vec[2], 0, vec[0], vec[1], vec[1], -vec[0], 0, vec[2],
          -vec[0], -vec[1], -vec[2], 0)
      .finished();
}

inline Eigen::Quaterniond smallAngleQuat(const Eigen::Vector3f& theta)
{
  Eigen::Quaterniond ret(1, theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  ret.normalize();
  return std::move(ret);
}

}  // namespace msf

}  // namespace localization
}  // namespace robosense

#endif /*MSF_CONST_H*/