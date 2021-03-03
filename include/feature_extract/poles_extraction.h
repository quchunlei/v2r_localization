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

#ifndef RS_LOCALIZATION_POLESEXTRACTION_H
#define RS_LOCALIZATION_POLESEXTRACTION_H

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <yaml/yaml.h>
#include "feature_extract/poles_extract.h"

namespace robosense
{
namespace localization
{
class Extraction
{
public:
  Extraction(const YAML::Node& param);
  bool extractPoles(pcl::PointCloud<pcl::PointXYZI>::ConstPtr input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position, const Eigen::Vector3f& lidar_pose,
                    float dis_lidar_to_ground);
  bool extractPoles(pcl::PointCloud<pcl::PointXYZI>::ConstPtr input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr poles_position, float dis_lidar_to_ground);

  bool extractPoles(pcl::PointCloud<pcl::PointXYZI>::ConstPtr input_cloud, std::vector<Eigen::Vector3f>& poles_points,
                    const Eigen::Vector3f& lidar_pose, float dis_lidar_to_ground);
  bool extractPoles(pcl::PointCloud<pcl::PointXYZI>::ConstPtr input_cloud, std::vector<Eigen::Vector3f>& poles_points,
                    float dis_lidar_to_ground);

private:
  // for new poles extract method
  std::unique_ptr<PolesExtract> poles_extract_obj_;
};
}  // namespace rs_localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_POLESEXTRACTION_H
