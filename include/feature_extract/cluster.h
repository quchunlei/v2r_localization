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

#ifndef PROJECT_CLUSTER_H
#define PROJECT_CLUSTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stack>

namespace robosense
{
namespace localization
{
class Cluster
{
public:
  Cluster();
  ~Cluster()
  {
  }

  void cluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr,
               std::vector<pcl::PointCloud<pcl::PointXYZI> >& objects);
  // not use
  void cluster(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr in_cloud_ptr, const std::vector<float>& offset_x,
               const std::vector<float>& offset_y, std::vector<pcl::PointCloud<pcl::PointXYZI> >& objects);

  void setClusterParams(const float& x_max = 30.f, const float& x_min = -30.f, const float& y_max = 30.f,
                        const float& y_min = -30.f, const float& grid_size = 0.1875f, const int& scan_range = 1,
                        const float& clutser_z = 2.f, const float& growth_z = 5.f, const float& growth_step = 0.25f);

  void enableGrowth(const bool& growth = false);

protected:
  void seedFillAlgorithm(const Eigen::MatrixXi& bin_mat, Eigen::MatrixXi& label_mat, int& cluster_num);
  void seedFillWithOffsetAlgorithm(const Eigen::MatrixXi& bin_mat, const Eigen::MatrixXi& offset_x_mat,
                                   const Eigen::MatrixXi& offset_y_mat, Eigen::MatrixXi& label_mat, int& cluster_num);
  void growthAlgorithm(const Eigen::MatrixXi& label_mat, const std::vector<Eigen::MatrixXi>& growth_bin_mat,
                       std::vector<Eigen::MatrixXi>& growth_label_mat);
  float x_max_, x_min_, y_max_, y_min_;
  float grid_size_, growth_z_, growth_step_, cluster_z_;
  int height_, width_, growth_slice_, scan_range_;
  bool growth_;

private:
};

}  // namespace localization
}  // namespace robosense

#endif  // PROJECT_CLUSTER_H
