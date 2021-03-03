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

#include "icp_localization/icp_pre_matcher.h"

namespace robosense
{
namespace localization
{
/************************************ ICPPreMatcher ***********************************/
ICPPreMatcher::ICPPreMatcher(const YAML::Node& param) : yaml_param_(param)
{
  common::yamlRead(param, "name", name_, "ICPPreMatcher");

  if (!common::yamlRead(param, "init", for_init_))
  {
    ERROR << name() << " init not set" << RESET << END;
    exit(-1);
  }
}

/************************************ ICPNullPreMatcher ***********************************/
ICPNullPreMatcher::ICPNullPreMatcher(const YAML::Node& param) : ICPPreMatcher(param)
{
}

bool ICPNullPreMatcher::run(ICPSamples& samples)
{
  return true;
}

}  // namespace localization
}  // namespace robosense