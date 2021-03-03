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

#ifndef PROJECT_DISPLAY_H
#define PROJECT_DISPLAY_H

#include <math.h>
#include <vector>
#include <Eigen/Core>
#include <iostream>

namespace robosense
{
namespace localization
{
inline void display(const Vector3f& res, const std::string& name)
{
  std::cout << name << ": " << res(0) << " " << res(1) << " " << res(2) * 180 / M_PI << "deg" << std::endl;
}

template <typename T>
inline void display(const std::vector<T>& res, const std::string& name)
{
  std::cout << name << ": ";
  for (int i = 0; i < res.size(); ++i)
  {
    std::cout << res[i] << ",";
  }
  std::cout << std::endl;
}

}  // namespace localization
}  // namespace robosense

#endif  // PROJECT_DISPLAY_H
