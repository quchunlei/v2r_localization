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

#ifndef __POINT_CLOUD_VIEWER__
#define __POINT_CLOUD_VIEWER__

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <thread>

#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include "common/common.h"

namespace robosense
{
namespace localization
{
class PointCloudViewer
{
public:
  // Only support 1,2,4 widows don't use 3
  PointCloudViewer();
  PointCloudViewer(std::string name);
  PointCloudViewer(int window_count_, std::string name);

  void setCurrWindow(int curr_window);

  void setPointCloud(DP point_cloud, Eigen::Vector3i color, std::string id);
  void setPointCloud(DP point_cloud, Eigen::Matrix4f trans, Eigen::Vector3i color, std::string id);
  void setPointCloud(pcl::PointCloud<pcl::PointXYZI> point_cloud, Eigen::Vector3i color, std::string id);
  void setPointCloud(sensor_msgs::PointCloud2 point_cloud, Eigen::Vector3i color, std::string id);
  void setPointCloudWithNormal(DP point_cloud, Eigen::Vector3i color, std::string id);
  void setPointCloudWithNormal(DP point_cloud, Eigen::Vector3i color, std::string id, int level, float normal_length);

private:
  struct frame
  {
  public:
    frame() : new_(false)
    {
    }
    frame(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, int red, int green, int blue)
    {
      update(point_cloud, red, green, blue);
      new_ = true;
      withNormal_ = false;
    }
    frame(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int red,
          int green, int blue)
    {
      update(point_cloud, normals, red, green, blue);
      new_ = true;
      withNormal_ = false;
    }

    inline void update(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,
                       int red, int green, int blue)
    {
      point_cloud_ = point_cloud;
      normals_ = normals;
      red_ = red;
      green_ = green;
      blue_ = blue;
    }

    inline void update(pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud, int red, int green, int blue)
    {
      point_cloud_ = point_cloud;
      red_ = red;
      green_ = green;
      blue_ = blue;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;
    pcl::PointCloud<pcl::Normal>::Ptr normals_;
    int red_;
    int green_;
    int blue_;
    bool new_;
    bool withNormal_;
    int normal_disp_level_;
    float normal_disp_length_;
  };

  void viewThread(void);

  DP transform(const DP&, Eigen::Matrix4f);
  inline void getLock(void)
  {
    while (lock_)
      usleep(1000);
    lock_ = 1;
  }
  inline void releaseLock(void)
  {
    lock_ = 0;
  }
  volatile int lock_;

  bool new_data_;
  const int window_count_;
  const std::thread thd_;
  int curr_window_;
  std::map<std::string, frame> view_buffer_[4];
  std::string name_;
};
}  // namespace localization
}  // namespace robosense

#endif /* __POINT_CLOUD_VIEWER__ */