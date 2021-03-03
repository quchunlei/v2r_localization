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

#ifndef RS_LOCALIZATION_RVIZDISPLAY_H
#define RS_LOCALIZATION_RVIZDISPLAY_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include "pointmatcher/point_cloud.h"
#include "sensor/gps_msg_process.h"

#include "common/prompt.h"
#include "yaml/yaml.h"

namespace robosense
{
namespace localization
{
typedef PointMatcher<float> PM;
typedef PM::DataPoints DP;

class RvizDisplay
{
public:
  RvizDisplay() = delete;
  RvizDisplay(const YAML::Node& param);
  bool loadParam(const YAML::Node& param);
  void setGpsOrigin(const Eigen::Vector3d& origin)
  {
    gps_process_->setGpsOrigin(origin);
    is_gps_origin_set_ = true;
  }
  const std::string name() const
  {
    return "RvizDisplay";
  }
  bool is_gps_origin_set(void)
  {
    return is_gps_origin_set_;
  }

  template<typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args&&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
public:
  void displayIDs(const ros::Publisher& markerID_pub, pcl::PointCloud<pcl::PointXYZ>::ConstPtr centers,
                  const std::string& frame_id);
  void showCylinder(const ros::Publisher& cylinder_pub, pcl::PointCloud<pcl::PointXYZI>::ConstPtr centers,
                    const ros::Time& msg_stamp, const std::string& frame_id);
  void showCylinder(const ros::Publisher& cylinder_pub, const std::vector<Eigen::Vector3f>& poles_points,
                    const ros::Time& msg_stamp, const std::string& frame_id);
  void displayFeatureMap(const ros::Publisher& featureMap_pub, pcl::PointCloud<pcl::PointXYZ>::ConstPtr centers,
                         const std::string& frame_id);
  void publishPoints(const ros::Publisher& points_pub, pcl::PointCloud<pcl::PointXYZI>::ConstPtr points_cloud,
                     ros::Time bag_time, const std::string& frame_id);
  void publishOdomPoints(const ros::Publisher& points_pub, pcl::PointCloud<pcl::PointXYZI>::ConstPtr points_cloud,
                         ros::Time bag_time, const std::string& frame_id);
  void publishMap(const ros::Publisher& map_pub, const sensor_msgs::PointCloud2& map);
  void publishMap(const ros::Publisher& map_pub, const DP& map, const std::string frame_id, const ros::Time stamp);

  void publishGPSOrigin(const ros::Publisher& pub, const Eigen::Vector3f& lon_lat_alt, const ros::Time& msg_time,
                        const std::string& frame_id);
  void publishGPSPath(const ros::Publisher& pub, const nav_msgs::Path gnss_path);

  void publishGPSLonLatAlt(const ros::Publisher& pub, const double lon, const double lat, const double alt,
                           const std::string frame_id, const ros::Time stamp);

private:
  bool is_gps_origin_set_;
  int num_gps_path_reserve_;
  float cylinder_radius_;
  float cylinder_length_;
  float cylinder_position_z_;
  float map_cylinder_color_r_;
  float map_cylinder_color_g_;
  float map_cylinder_color_b_;
  float instant_cylinder_color_r_;
  float instant_cylinder_color_g_;
  float instant_cylinder_color_b_;
  std::unique_ptr<GpsMsgProcess> gps_process_;
  nav_msgs::Path gps_path_;
};

}  // namespace localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_RVIZDISPLAY_H
