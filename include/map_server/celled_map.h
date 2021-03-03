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

#ifndef RS_LOCALIZATION_CELLED_GLOBAL_MAP_H
#define RS_LOCALIZATION_CELLED_GLOBAL_MAP_H

#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <Eigen/Core>

#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"

namespace robosense
{
namespace localization
{
class CelledMap
{
public:
  CelledMap();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void loadConfigration(const int cell_length, const int local_map_radius);
  void calcMapSize(const DP& point_cloud);
  void setGlobalMap(const DP& point_cloud);

  float distance2Cell(const Eigen::Vector3f pose, const Eigen::Vector3i cell);
  void getLocalMap(const Eigen::Vector3f pose, DP& local_map);

  inline int getSize(void)
  {
    return cell_count_[0] * cell_count_[1];
  }

  inline float getCellLength(void)
  {
    return cell_length_;
  }

private:
  inline std::string name()
  {
    return "CelledMap";
  }

  Eigen::Matrix<DP, Eigen::Dynamic, Eigen::Dynamic> map_;
  Eigen::Vector3i cell_count_;
  Eigen::Vector3f cell_min_;
  Eigen::Vector3f cell_max_;
  float cell_length_;
  // in num of cells row and col ie: local_map_radius = 10 map_size = 10*cell_length_
  float local_map_radius_;

  bool isIdxValid(const int& i, const int& j);
};

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_CELLED_GLOBAL_MAP_H */
