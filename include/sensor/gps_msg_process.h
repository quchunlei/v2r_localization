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

#ifndef RS_LOCALIZATION_GPSMSG_H
#define RS_LOCALIZATION_GPSMSG_H

#include <iostream>
#include <stdio.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "common/prompt.h"

namespace robosense
{
namespace localization
{
class GpsMsgProcess
{
public:
  explicit GpsMsgProcess();
  void setGpsOrigin(const Eigen::Vector3d& gps_origin_lat_lon_alt);
  bool gps2xyz(const Eigen::Vector3d& lon_lat_alt, Eigen::Vector3d& xyz);
  bool gps2xyz(const double& longitude, const double& latitude, const double& altitude, Eigen::Vector3d& xyz);
  bool xyz2gps(const Eigen::Vector3d& xyz, double& longitude, double& latitude, double& altitude);
  bool xyz2gps(const Eigen::Vector3d& xyz, Eigen::Vector3d& lon_lat_alt);

  const std::string name() const
  {
    return "GpsMsgProcess";
  }

private:
  Eigen::Vector3d WGS84toECEF(const Eigen::Vector3d& gps);
  Eigen::Vector3d ECEFtoWGS84(const Eigen::Vector3d& xyz);

private:
  bool is_set_gps_origin_;
  Eigen::Vector3d gps_origin_;
  Eigen::Vector3d origin_ECEF_;
};

}  // localization
}  // robosense
#endif  // RS_LOCALIZATION_GPSMSG_H
