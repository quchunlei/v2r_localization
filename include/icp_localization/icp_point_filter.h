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

#ifndef RS_LOCALIZATION_ICPPOINTFILTER_H
#define RS_LOCALIZATION_ICPPOINTFILTER_H

#include <iostream>
#include <cmath>
#include <list>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "icp_localization/icp_common.h"
#include "map_server/map_server.h"
#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"

namespace robosense
{
namespace localization
{
// TODO : Add try catch
class ICPPointFilter
{
public:
  //   ICPPointFilter() = delete;
  explicit ICPPointFilter(const YAML::Node& param);
  virtual ~ICPPointFilter() = default;

  virtual bool run(DP& scan) = 0;

protected:
  std::string name_;
  bool for_init_;
  YAML::Node yaml_param_;

  PM::DataPointsFilters filter_;
  const std::string& name(void)
  {
    return name_;
  }
};

class ICPCustomPointFilter : public ICPPointFilter
{
public:
  //   ICPCustomPointFilter() = delete;
  explicit ICPCustomPointFilter(const YAML::Node& param);
  bool run(DP& scan);

protected:
  bool config(void);
  bool configFilter(void);
  bool configFilterInit(void);
  bool getDefaultConfig(void);
  bool getDefaultConfigInit(void);

  bool use_default_;
};

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_ICPPOINTFILTER_H */