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

#ifndef PROJECT_COMMON_H
#define PROJECT_COMMON_H

#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include <pcl/io/io.h>
#include <memory>

namespace robosense
{
namespace localization
{
typedef struct KeyFrame
{
  explicit KeyFrame()
  {
    frame_num = 0;
    trans_pose = Eigen::Matrix4f::Identity();
    cloud_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    trans_x = 0;
    trans_y = 0;
    trans_z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
  }
  int frame_num;
  Eigen::Matrix4f trans_pose;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_points;
  float trans_x;
  float trans_y;
  float trans_z;
  float roll;
  float pitch;
  float yaw;
} KeyFrame;
typedef std::shared_ptr<KeyFrame> KeyFramePtr;
typedef std::shared_ptr<const KeyFrame> KeyFrameConstPtr;

inline std::ostream& operator<<(std::ostream& o, KeyFrame const& obj)
{
  return o << "PointCloud Size: " << obj.cloud_points->size() << '\n' << "Transformation: " << '\n' << obj.trans_pose;
}

// map load
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;
typedef PM::ICP PMICP;
typedef std::shared_ptr<PMICP> PMICPPtr;
typedef std::shared_ptr<const PMICP> PMICPConstPtr;
typedef PM::DataPointsFilters PMDPF;
typedef std::shared_ptr<PMDPF> PMDPFPtr;
typedef std::shared_ptr<const PMDPF> PMDPFConstPtr;
typedef typename PM::DataPoints::View View;

}  // namespace common
}  // namespace robosense

#endif  // PROJECT_COMMON_H
