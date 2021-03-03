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
#ifndef RS_LOCALIZATION_OBSERVATION_H
#define RS_LOCALIZATION_OBSERVATION_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <Eigen/Core>
#include <functional>

#include "common/common.h"
#include "msf/core/msf_state.h"

namespace robosense
{
namespace localization
{
namespace msf
{
struct Observation
{
  uint64_t seq = 0;
  double timestamp = 0.0;
  std::string obs_source;

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Matrix3f pos_cov = Eigen::Matrix3f::Zero();

  Eigen::Vector3f linear_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_vel_cov = Eigen::Matrix3f::Zero();

  Eigen::Vector3f linear_acc = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_acc_cov = Eigen::Matrix3f::Zero();

  Eigen::Vector4f angle = Eigen::Vector4f::Zero();
  Eigen::Matrix4f angle_cov = Eigen::Matrix4f::Zero();

  Eigen::Vector3f angular_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angular_vel_cov = Eigen::Matrix3f::Zero();

  std::function<bool(const Observation&, const MsfState&, MsfState&)> apply = [this](const Observation&,
                                                                                     const MsfState&, MsfState&) {
    std::cout << "Running into a undefined observation " << obs_source << " this shoud never happen " << std::endl;
    return false;
  };

  int type;
};
}  // namespace msf

}  // namespace localization
}  // namespace robosense

#endif /*RS_LOCALIZATION_OBSERVATION_H*/
