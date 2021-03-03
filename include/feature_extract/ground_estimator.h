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

#ifndef PROJECT_GROUND_ESTIMATOR_H
#define PROJECT_GROUND_ESTIMATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

namespace robosense
{
namespace localization
{
class GroundEstimator
{
public:
  GroundEstimator();
  ~GroundEstimator()
  {
  }

  /**
   * @brief ground remove use linefit method
   * @param[in] in_cloud_ptr input cloud for ground remove
   * @param[out] objects output objects
   */
  void groundEstimator(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr, const float& estimate_lidar_height,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud_ptr);
  /**
   * @brief get ground points
   * @param[in,out] ground_cloud_ptr ground point cloud of ground
   */
  void getGroundPts(pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr);

  void setGndEstConstrainParams(const float& height_thre = 0.2, const float& angle_thre = 5.);
  void setGndEstParams(const float& x_min = -50.f, const float& x_max = 50.f, const float& y_min = -50.f,
                       const float& y_max = 50.f, const float& angle_step = 0.5, const float& range_step = 0.2);
  void setGndConstrainParams(const float& thre_ = 0.2, const float& abs_thre = 0.5f);

protected:
  inline float comBeta(const pcl::PointXYZI& pt);
  inline bool isNanPt(const pcl::PointXYZI& pt);
  float max_radius_, rad_step_, range_step_;
  int rows_, cols_;
  float height_thre_, rad_thre_;
  float ground_plane_thre_;
  float abs_thre_;
  float x_max_, x_min_, y_max_, y_min_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr_;
};

}  // namespace localization
}  // namespace robosense
#endif  // PROJECT_GROUND_ESTIMATOR_H
