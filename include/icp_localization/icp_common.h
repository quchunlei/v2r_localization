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

#ifndef RS_LOCALIZATION_ICPCOMMON_H
#define RS_LOCALIZATION_ICPCOMMON_H

#include <iostream>
#include <cmath>
#include <list>
#include <math.h>
#include <string>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <chrono>

#include "map_server/map_server.h"
#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include "yaml/yaml.h"

namespace robosense
{
namespace localization
{
struct ICPRotatedSamples
{
  DP map_;
  DP scan_;
  std::list<std::tuple<float, Eigen::Matrix4f, Eigen::Matrix<float, 6, 6>>> samples_;  // score, tf
  Eigen::Matrix4f map_tf_;
};

struct ICPSamples
{
  std::list<ICPRotatedSamples> rotated_samples_;
};

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_ICPCOMMON_H */