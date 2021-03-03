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

#ifndef RS_LOCALIZATION_ICPLOCALIZATION_H
#define RS_LOCALIZATION_ICPLOCALIZATION_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <memory>

#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"

#include "icp_localization/icp_point_filter.h"
#include "icp_localization/icp_sampler.h"
#include "icp_localization/icp_pre_matcher.h"
#include "icp_localization/icp_matcher.h"
#include "icp_localization/icp_evaluator.h"
#include "map_server/map_server.h"
#include "map_server/celled_map.h"

namespace robosense
{
namespace localization
{
class ICPLocalization
{
public:
  explicit ICPLocalization(const std::shared_ptr<MapServer> map_obj, const YAML::Node& param);
  bool loadParams(void);
  // void setLocalMap(const std::shared_ptr<DP>& local_map);
  // TODO: DP scan is not a ref now maybe change it back later
  bool locate(DP scan, const Eigen::Matrix4f& init_pose, Eigen::Matrix4f& output_pose);
  bool locate(DP scan, const Eigen::Matrix4f& init_pose, Eigen::Matrix4f& output_pose,
              Eigen::Matrix<float, 6, 6>& output_cov);
  const std::string name() const
  {
    return "LidarRegistraLocalization";
  }

private:
  YAML::Node yaml_param_;
  bool for_init_;

  std::unique_ptr<ICPPointFilter> filter_;
  std::unique_ptr<ICPSampler> sampler_;
  std::unique_ptr<ICPPreMatcher> pre_matcher_;
  std::unique_ptr<ICPMatcher> matcher_;
  std::unique_ptr<ICPEvaluator> evaluator_;
};
}  // namespace localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_ICPLOCALIZATION_H