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
#ifndef RS_LOCALIZATION_SENSOR_DATA_H
#define RS_LOCALIZATION_SENSOR_DATA_H

#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <Eigen/Core>
#include <functional>

#include "common/common.h"

namespace robosense
{
namespace localization
{
namespace msf
{
struct LidarData
{
  uint64_t seq = 0;
  double timestamp = 0.0;
  DP scan;
};

struct ImuData
{
  uint64_t seq = 0;
  double timestamp = 0.0;

  Eigen::Vector4f angle = Eigen::Vector4f::Zero();
  Eigen::Matrix3f angle_cov = Eigen::Matrix3f::Identity();  //　Covariance should be 3X3 since quaternion is 3-DOF

  Eigen::Vector3f angular_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angular_vel_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f linear_acc = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_acc_cov = Eigen::Matrix3f::Identity();
};

struct GnssData
{
  uint64_t seq = 0;
  double timestamp = 0.0;

  int status = 0;

  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Matrix3f pos_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector4f angle = Eigen::Vector4f::Zero();  // quaternion: x, y, z, w
  Eigen::Matrix4f angle_cov = Eigen::Matrix4f::Identity();

  Eigen::Vector3f linear_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_vel_cov = Eigen::Matrix3f::Identity();
  enum GnssStatus
  {
    STATUS_NO_FIX = -1,   // unable to fix position
    STATUS_FIX = 0,       // unaugmented fix
    STATUS_SBAS_FIX = 1,  // with satellite-based augmentation
    STATUS_GBAS_FIX = 2,  // with ground-based augmentation
  } gnss_status;

  enum GnssService
  {
    SERVICE_GPS = 0,
    SERVICE_GLONASS = 1,
    SERVICE_BEIDOU = 2,
    SERVICE_GALILEO = 8
  } gnss_service;
};

struct OdomData
{
  uint64_t seq = 0;
  double timestamp = 0.0;

  Eigen::Vector3f angular_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angular_vel_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f linear_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_vel_cov = Eigen::Matrix3f::Identity();
};

struct ImuRtkData
{
  uint64_t seq = 0;
  double timestamp = 0.0;

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Matrix3f pos_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f linear_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_vel_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f linear_acc = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_acc_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f angle = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angle_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f angular_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angular_vel_cov = Eigen::Matrix3f::Identity();
};

}  // namespace msf

}  // namespace localization
}  // namespace robosense

#endif /*RS_LOCALIZATION_SENSOR_DATA_H*/
