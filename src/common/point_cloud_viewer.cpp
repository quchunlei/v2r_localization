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

#include "common/point_cloud_viewer.h"
#include "common/common.h"

namespace robosense
{
namespace localization
{
PointCloudViewer::PointCloudViewer() : window_count_(1), thd_(&PointCloudViewer::viewThread, this)
{
  curr_window_ = 1;
  new_data_ = false;
  releaseLock();
}

PointCloudViewer::PointCloudViewer(std::string name)
  : window_count_(1), thd_(&PointCloudViewer::viewThread, this), name_(name)
{
  curr_window_ = 1;
  new_data_ = false;
  releaseLock();
}

PointCloudViewer::PointCloudViewer(int window_count_, std::string name)
  : window_count_(window_count_), thd_(&PointCloudViewer::viewThread, this), name_(name)
{
  curr_window_ = 1;
  new_data_ = false;
  releaseLock();
}

void PointCloudViewer::setCurrWindow(int curr_window)
{
  assert(curr_window <= window_count_);
  curr_window_ = curr_window;
}

void PointCloudViewer::setPointCloud(pcl::PointCloud<pcl::PointXYZI> point_cloud, Eigen::Vector3i color, std::string id)
{
  getLock();
  if (view_buffer_[curr_window_].find(id) == view_buffer_[curr_window_].end())
  {
    view_buffer_[curr_window_][id] = frame(point_cloud.makeShared(), color[0], color[1], color[2]);
  }
  else
  {
    view_buffer_[curr_window_][id].update(point_cloud.makeShared(), color[0], color[1], color[2]);
  }
  new_data_ = true;
  releaseLock();
}

void PointCloudViewer::setPointCloud(DP point_cloud, Eigen::Vector3i color, std::string id)
{
  sensor_msgs::PointCloud2 pc_ros =
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(point_cloud, "/rs_odom", ros::Time::now());
  pcl::PointCloud<pcl::PointXYZI> pc_pcl;
  pcl::moveFromROSMsg(pc_ros, pc_pcl);
  setPointCloud(std::move(pc_pcl), color, id);
}

void PointCloudViewer::setPointCloud(DP point_cloud, Eigen::Matrix4f eigenT, Eigen::Vector3i color, std::string id)
{
  // transform the pointcloud
  DP tranformed_pc = transform(point_cloud, eigenT);
  setPointCloud(tranformed_pc, color, id);
}

void PointCloudViewer::setPointCloud(sensor_msgs::PointCloud2 point_cloud, Eigen::Vector3i color, std::string id)
{
  pcl::PointCloud<pcl::PointXYZI> pc_pcl;
  pcl::moveFromROSMsg(point_cloud, pc_pcl);
  setPointCloud(std::move(pc_pcl), color, id);
}

void PointCloudViewer::setPointCloudWithNormal(DP point_cloud, Eigen::Vector3i color, std::string id, int level,
                                               float normal_length)
{
  sensor_msgs::PointCloud2 pc_ros =
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(point_cloud, "/rs_odom", ros::Time::now());
  pcl::PointCloud<pcl::PointXYZI> pc_pcl;
  pcl::moveFromROSMsg(pc_ros, pc_pcl);

  if (!point_cloud.descriptorExists("normals"))
  {
    std::cout << "No normals in current point cloud" << std::endl;
    return;
  }

  View normal_view = point_cloud.getDescriptorViewByName("normals");
  pcl::PointCloud<pcl::Normal> normals;
  for (int i; i < point_cloud.getNbPoints(); i++)
  {
    pcl::Normal temp(normal_view(0, i), normal_view(1, i), normal_view(2, i));
    normals.push_back(temp);
  }

  getLock();
  if (view_buffer_[curr_window_].find(id) == view_buffer_[curr_window_].end())
  {
    view_buffer_[curr_window_][id] = frame(pc_pcl.makeShared(), normals.makeShared(), color[0], color[1], color[2]);
  }
  else
  {
    view_buffer_[curr_window_][id].update(pc_pcl.makeShared(), normals.makeShared(), color[0], color[1], color[2]);
  }
  view_buffer_[curr_window_][id].withNormal_ = true;
  view_buffer_[curr_window_][id].normal_disp_level_ = level;
  view_buffer_[curr_window_][id].normal_disp_length_ = normal_length;

  new_data_ = true;
  releaseLock();

  // setPointCloud(std::move(pc_pcl), color, id);
}

DP PointCloudViewer::transform(const DP& point_cloud, Eigen::Matrix4f eigen_T)
{
  PM::TransformationParameters T(eigen_T);
  std::shared_ptr<PM::Transformation> rigidTrans;
  rigidTrans = PM::get().REG(Transformation).create("RigidTransformation");

  if (!rigidTrans->checkParameters(T))
  {
    std::cout << "WARNING: T does not represent a valid rigid transformation\nProjecting onto an orthogonal basis"
              << std::endl;
    T = rigidTrans->correctParameters(T);
  }

  return rigidTrans->compute(point_cloud, T);
}

void PointCloudViewer::viewThread(void)
{
  std::cout << "\n\r\n\n\r viewer thread created" << name_ << "\n\r\n\r\n\r";
  pcl::visualization::PCLVisualizer view(name_);
  std::cout << "\n\r\n\n\r viewer thread created!\n\r\n\r\n\r";
  view.initCameraParameters();
  view.setBackgroundColor(0.0, 0.05, 0.05);
  while (1)
  {
    if (new_data_ == true)
    {
      new_data_ = false;  // This is not thread safe but doesnt matter here
      getLock();
      for (std::map<std::string, frame>::iterator it = view_buffer_[curr_window_].begin();
           it != view_buffer_[curr_window_].end(); ++it)
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> sources_cloud_color(
            it->second.point_cloud_, it->second.red_, it->second.green_, it->second.blue_);
        if (it->second.withNormal_ == true)
        {
          view.removeAllPointClouds();
          view.addPointCloudNormals<pcl::PointXYZI, pcl::Normal>(it->second.point_cloud_, it->second.normals_, 1, 1,
                                                                 it->first, 0);
        }
        else if (it->second.new_ == true)
        {
          it->second.new_ = false;
          view.addPointCloud(it->second.point_cloud_, sources_cloud_color, it->first);
          view.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, it->first);
        }
        else
        {
          view.updatePointCloud(it->second.point_cloud_, sources_cloud_color, it->first);
        }
      }
      releaseLock();
    }
    view.spinOnce(10);
    usleep(10000);
  }
}

}  // namespace localization
}  // namespace robosense
