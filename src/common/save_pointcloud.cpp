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

#include "common/save_pointcloud.h"

namespace robosense
{
namespace localization
{
template <typename PointT>
void savePointCloud(const std::string& path, typename pcl::PointCloud<PointT>::ConstPtr pointcloud)
{
  if (!pointcloud->empty())
  {
    pcl::io::savePCDFileASCII(path, *pointcloud);
    std::cout << "Save in: " << path << std::endl;
  }
  else
  {
    std::cerr << "The input pointcloud is empty!" << std::endl;
  }
}

}  // namespace localization
}  // namespace robosense
