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

#ifndef RS_LOCALIZATION_SAMPLER_H
#define RS_LOCALIZATION_SAMPLER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <list>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include "icp_localization/icp_common.h"

namespace robosense
{
namespace localization
{
struct ICPMatcher
{
  explicit ICPMatcher(const YAML::Node& param);
  virtual ~ICPMatcher() = default;

  virtual bool run(ICPSamples& samples) = 0;

protected:
  std::string name_;
  bool for_init_;
  YAML::Node yaml_param_;

  const std::string& name(void)
  {
    return name_;
  }
};

struct PMICPMatcher : public ICPMatcher
{
  explicit PMICPMatcher(const YAML::Node& param);

  bool run(ICPSamples& samples);

private:
  bool config(void);
  bool configMatcher(void);
  bool configMatcherInit(void);

  PM::ICPSequence icp_;
};

// struct ICPPointToPlaneMatcher : public ICPMatcher
// {
//   struct Options
//   {
//     Options()
//     {
//       name = "ICPPointToPlaneMatcher";
//     }
//     std::string name;
//   };

//   ICPPointToPlaneMatcher(Options option = Options());

//   bool match(const std::list<boost::shared_ptr<DP>>& cloud, std::list<Eigen::Matrix4f>& trans,
//   std::list<float>& score);

// private:
//   boost::shared_ptr<PM::ICP> matcher_;
// };

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_SAMPLER_H */