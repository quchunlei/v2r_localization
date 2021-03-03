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

#include <ros/ros.h>
#include "map_server/map_server.h"
#include "map_server/celled_map.h"
#include "common/point_cloud_viewer.h"
#include "yaml/yaml_parser.h"
#include "common/prompt.h"
#include <ros/package.h>
#include "third_party/point_cloud.h"
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <chrono>

using namespace robosense::localization;

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                                                                pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  return (viewer);
}

// PointCloudViewer viewer("view");
int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_map_server");
  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");
  std::string parameter_file;
  std::string map_file;
  pri_nh.getParam("parameter_file", parameter_file);
  pri_nh.getParam("map_file", map_file);

  std::unique_ptr<YamlParser> ptr_param_;
  YAML::Node parameters_;
  try
  {
    std::string default_cfg_file = ros::package::getPath("rs_localization") + "/cfg/parameters.yaml";
    // INFO << "Parsing config file" << REND;
    YamlParser catenator("robosense@XiLiSZ");
    parameters_ = catenator.YamlParserMerge(default_cfg_file, parameter_file);
    // INFO << "Finish parsing config file" << REND;
  }
  catch (YAML::ParserException& e)
  {
    ERROR << " Can't Read Param file" << RESET << END;
    exit(-1);
  }

  parameters_["MapServer"]["map_file"] = map_file;

  MapServer map(parameters_["MapServer"]);
  DP local_map = map.getLocalMap(Eigen::Vector3f(0, 0, 0));
  sensor_msgs::PointCloud2 pc_ros =
      PointMatcher_ros::pointMatcherCloudToRosMsg<float>(local_map, "/rs_odom", ros::Time::now());

  DEBUG << local_map.descriptorLabels << REND;

  for (int i = 0; i < pc_ros.fields.size(); i++)
  {
    sensor_msgs::PointField pf = pc_ros.fields[i];
    WARNING << pf.name << " : " << pf.offset << " : " << (int)pf.datatype << " : " << pf.count << REND;
  }

  pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_normal_pcl(new pcl::PointCloud<pcl::PointXYZINormal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_pcl(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr pc_normal(new pcl::PointCloud<pcl::Normal>);
  pcl::moveFromROSMsg(pc_ros, *pc_normal_pcl);
  // pcl::moveFromROSMsg(pc_ros, *pc_pcl);

  for (int i = 0; i < pc_normal_pcl->points.size(); i++)
  {
    pcl::PointXYZ temp_point;
    temp_point.x = pc_normal_pcl->points[i].x;
    temp_point.y = pc_normal_pcl->points[i].y;
    temp_point.z = pc_normal_pcl->points[i].z;
    pc_pcl->points.push_back(std::move(temp_point));

    pcl::Normal temp_normal;
    temp_normal.normal_x = pc_normal_pcl->points[i].normal_x;
    temp_normal.normal_y = pc_normal_pcl->points[i].normal_y;
    temp_normal.normal_z = pc_normal_pcl->points[i].normal_z;
    pc_normal->points.push_back(std::move(temp_normal));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Generating example point clouds.\n\n";
  // We're going to make an ellipse extruded along the z-axis. The colour for
  // the XYZRGB cloud will gradually go from red to green to blue.
  uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
      basic_point.y = sinf(pcl::deg2rad(angle));
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);

      pcl::PointXYZRGB point;
      point.x = basic_point.x;
      point.y = basic_point.y;
      point.z = basic_point.z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      point.rgb = *reinterpret_cast<float*>(&rgb);
      point_cloud_ptr->points.push_back(point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
  point_cloud_ptr->height = 1;

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(point_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.05);
  ne.compute(*cloud_normals1);

  // ---------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.1-----
  // ---------------------------------------------------------------
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.1);
  ne.compute(*cloud_normals2);
  // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // ne.setInputCloud (pc_pcl);
  // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  // ne.setSearchMethod (tree);
  // pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
  // ne.setRadiusSearch (0.05);
  // ne.compute (*cloud_normals1);

  auto view_normal = local_map.getDescriptorViewByName("normals");
  for (int i(0); i < local_map.features.cols(); i++)
  {
    WARNING << (local_map.features.block<3, 1>(0, i)).transpose() << " | " << view_normal.col(i).transpose() << REND;
    DEBUG << pc_normal_pcl->points[i] << REND;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(pc_pcl, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(pc_pcl, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(pc_pcl, pc_normal, 1, 5, "normals");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D-viewer"));
  // viewer->setBackgroundColor(0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZ>(pc_pcl, "sample cloud");
  // viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (pc_pcl, cloud_normals1, 10, 0.05, "normals");

  // // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  // viewer->addCoordinateSystem(1.0);
  // viewer->initCameraParameters();
  // std::cout << "number of points in local map " << local_map.getNbPoints() << std::endl;
  // viewer.setPointCloud(local_map, Eigen::Vector3i(250, 0, 0), "local_map");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  while (ros::ok())
    sleep(1);
  return 0;
}
