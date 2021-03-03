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

#include "visualization/rviz_display.h"

namespace robosense
{
namespace localization
{
RvizDisplay::RvizDisplay(const YAML::Node& param)
{
  is_gps_origin_set_ = false;
  gps_process_ = make_unique<GpsMsgProcess>();
  loadParam(param);
}

bool RvizDisplay::loadParam(const YAML::Node& param)
{
  common::yamlRead(param, "num_gps_path_reserve", num_gps_path_reserve_, 10000);
  common::yamlRead(param, "cylinder_radius", cylinder_radius_, 0.5f);
  common::yamlRead(param, "map_cylinder_color_r", map_cylinder_color_r_, 1.0f);
  common::yamlRead(param, "map_cylinder_color_g", map_cylinder_color_g_, 0.0f);
  common::yamlRead(param, "map_cylinder_color_b", map_cylinder_color_b_, 1.0f);
  common::yamlRead(param, "instant_cylinder_color_r", instant_cylinder_color_r_, 0.0f);
  common::yamlRead(param, "instant_cylinder_color_g", instant_cylinder_color_g_, 0.0f);
  common::yamlRead(param, "instant_cylinder_color_b", instant_cylinder_color_b_, 0.0f);
  common::yamlRead(param, "cylinder_position_z", cylinder_position_z_, 1.0f);
  common::yamlRead(param, "cylinder_length", cylinder_length_, 0.0f);

  return true;
}

void RvizDisplay::displayIDs(const ros::Publisher& markerID_pub, pcl::PointCloud<pcl::PointXYZ>::ConstPtr centers,
                             const std::string& frame_id)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(centers->size());
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "basic_shapes";

  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 2;
  marker.scale.y = 2;
  marker.scale.z = 2;

  marker.color.r = 1.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration(0);

  for (int i = 0; i < centers->size(); ++i)
  {
    marker.id = i;

    marker.pose.position.x = (*centers)[i].x;
    marker.pose.position.y = (*centers)[i].y;
    marker.pose.position.z = (*centers)[i].z + 1.0;
    // marker.text = boost::to_string(i);
    markers.markers[i] = marker;
  }

  markerID_pub.publish(markers);
}

void RvizDisplay::showCylinder(const ros::Publisher& cylinder_pub, const std::vector<Eigen::Vector3f>& poles_points,
                               const ros::Time& msg_stamp, const std::string& frame_id)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(poles_points.size());
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = msg_stamp;
  marker.ns = "basic_shapes";

  marker.type = visualization_msgs::Marker::CYLINDER;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = cylinder_radius_;
  marker.scale.y = cylinder_radius_;
  marker.scale.z = cylinder_length_;

  marker.color.r = instant_cylinder_color_r_;
  marker.color.g = instant_cylinder_color_g_;
  marker.color.b = instant_cylinder_color_b_;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(0);

  for (int i = 0; i < poles_points.size(); ++i)
  {
    marker.id = i;
    marker.pose.position.x = poles_points[i].x();
    marker.pose.position.y = poles_points[i].y();
    marker.pose.position.z = 0;
    markers.markers[i] = marker;
  }

  cylinder_pub.publish(markers);
}

void RvizDisplay::showCylinder(const ros::Publisher& cylinder_pub, pcl::PointCloud<pcl::PointXYZI>::ConstPtr centers,
                               const ros::Time& msg_stamp, const std::string& frame_id)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(centers->size());
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = msg_stamp;
  marker.ns = "basic_shapes";

  marker.type = visualization_msgs::Marker::CYLINDER;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = cylinder_radius_;
  marker.scale.y = cylinder_radius_;
  marker.scale.z = cylinder_length_;

  marker.color.r = instant_cylinder_color_r_;
  marker.color.g = instant_cylinder_color_g_;
  marker.color.b = instant_cylinder_color_b_;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(0);

  for (int i = 0; i < centers->size(); ++i)
  {
    marker.id = i;
    marker.pose.position.x = (*centers)[i].x;
    marker.pose.position.y = (*centers)[i].y;
    marker.pose.position.z = 0;
    markers.markers[i] = marker;
  }

  cylinder_pub.publish(markers);
}

void RvizDisplay::displayFeatureMap(const ros::Publisher& featureMap_pub,
                                    pcl::PointCloud<pcl::PointXYZ>::ConstPtr centers, const std::string& frame_id)
{
  visualization_msgs::MarkerArray markers;
  markers.markers.resize(centers->size());

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.ns = "basic_shapes";

  marker.type = visualization_msgs::Marker::CYLINDER;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = cylinder_radius_;
  marker.scale.y = cylinder_radius_;
  marker.scale.z = cylinder_length_;

  marker.color.r = map_cylinder_color_r_;
  marker.color.g = map_cylinder_color_g_;
  marker.color.b = map_cylinder_color_b_;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(0);

  for (int i = 0; i < centers->size(); ++i)
  {
    marker.id = i;
    marker.pose.position.x = (*centers)[i].x;
    marker.pose.position.y = (*centers)[i].y;
    marker.pose.position.z = (*centers)[i].z;
    markers.markers[i] = marker;
  }
  featureMap_pub.publish(markers);
}

void RvizDisplay::publishPoints(const ros::Publisher& points_pub,
                                pcl::PointCloud<pcl::PointXYZI>::ConstPtr points_cloud, ros::Time bag_time,
                                const std::string& frame_id)
{
  sensor_msgs::PointCloud2 output_poles;
  pcl::toROSMsg(*points_cloud, output_poles);
  output_poles.header.frame_id = frame_id;
  output_poles.header.stamp = bag_time;
  points_pub.publish(output_poles);
}

void RvizDisplay::publishOdomPoints(const ros::Publisher& points_pub,
                                    pcl::PointCloud<pcl::PointXYZI>::ConstPtr points_cloud, ros::Time bag_time,
                                    const std::string& frame_id)
{
  sensor_msgs::PointCloud2 output_poles;
  pcl::toROSMsg(*points_cloud, output_poles);
  output_poles.header.frame_id = frame_id;
  output_poles.header.stamp = bag_time;
  points_pub.publish(output_poles);
}

void RvizDisplay::publishMap(const ros::Publisher& map_pub, const sensor_msgs::PointCloud2& map)
{
  map_pub.publish(map);
}

void RvizDisplay::publishMap(const ros::Publisher& map_pub, const DP& map, const std::string frame_id,
                             const ros::Time stamp)
{
  sensor_msgs::PointCloud2 background_map = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(map, frame_id, stamp);
  map_pub.publish(background_map);
}

void RvizDisplay::publishGPSOrigin(const ros::Publisher& pub, const Eigen::Vector3f& lon_lat_alt,
                                   const ros::Time& msg_time, const std::string& frame_id)
{
  sensor_msgs::NavSatFix gps_origin_msg;
  gps_origin_msg.header.stamp = msg_time;
  gps_origin_msg.header.frame_id = frame_id;
  gps_origin_msg.longitude = lon_lat_alt[0];
  gps_origin_msg.latitude = lon_lat_alt[1];
  gps_origin_msg.altitude = lon_lat_alt[2];
  pub.publish(gps_origin_msg);
}

void RvizDisplay::publishGPSPath(const ros::Publisher& pub, const nav_msgs::Path gnss_path)
{
  pub.publish(gnss_path);
}

void RvizDisplay::publishGPSLonLatAlt(const ros::Publisher& pub, const double lon, const double lat, const double alt,
                                      const std::string frame_id, const ros::Time stamp)
{
  if (!is_gps_origin_set_)
    return;

  // Publish gnss message
  Eigen::Vector3d position;
  gps_process_->gps2xyz(lon, lat, alt, position);

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = position[0];
  pose.pose.position.y = position[1];
  pose.pose.position.z = position[2];

  gps_path_.poses.push_back(pose);
  gps_path_.header.frame_id = frame_id;
  if (gps_path_.poses.size() > num_gps_path_reserve_)
  {
    gps_path_.poses.erase(gps_path_.poses.begin(),
                          gps_path_.poses.begin() + (gps_path_.poses.size() - num_gps_path_reserve_));
  }
  pub.publish(gps_path_);
}

}  // namespace localization
}  // namespace robosense
