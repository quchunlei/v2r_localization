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

#ifndef PROJECT_SAVE_POINTCLOUD_H
#define PROJECT_SAVE_POINTCLOUD_H

#include <string>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace robosense
{
namespace localization
{
template <typename PointT>
void savePointCloud(const std::string& path, typename pcl::PointCloud<PointT>::ConstPtr pointcloud);

}  // namespace common
}  // namespace robosense

#endif  // PROJECT_SAVE_POINTCLOUD_H
