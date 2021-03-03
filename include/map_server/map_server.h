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

#ifndef RS_LOCALIZATION_MAPSERVER_H
#define RS_LOCALIZATION_MAPSERVER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <yaml-cpp/yaml.h>

#include "map_server/celled_map.h"
#include "common/common.h"
#include "common/prompt.h"
#include "common/point_cloud_viewer.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace robosense
{
namespace localization
{
class MapServer
{
public:
  MapServer();
  MapServer(const YAML::Node& params);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool loadMap(const YAML::Node& param);
  bool loadParams(const YAML::Node& param);
  bool getGnssOrigin(Eigen::Vector3d& gps_origin);
  bool getHeight(const float& x, const float& y, const float& z, float& car_height);
  const DP& getLocalMap(const Eigen::Vector3f& pose);
  const std::vector<Eigen::Vector2f>& getFeatureMap()
  {
    return poles_points_;
  };

  const std::string name() const
  {
    return "MapServer";
  }

  const std::vector<Eigen::Vector2f>& getICPPoints() const
  {
    return icp_points_;
  };
  const std::vector<Eigen::Vector2f>& getPolesPoints() const
  {
    return poles_points_;
  };
  const std::vector<KeyFramePtr>& getKeyFrames() const
  {
    return vec_key_frames_;
  };
  const DP& getBackgroundMap() const
  {
    return background_;
  };

  int getStatus()
  {
    return error_code_;
  }
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr getPolesMap() const
  {
    return poles_cloud_;
  };
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr getOdomMap() const
  {
    return mapping_odom_cloud_;
  };
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr getICPPointsMap() const
  {
    return icp_points_cloud_;
  };

private:
  bool init();
  void LoadDPFilter();
  bool loadPCDMap(const std::string& input_file, DP& map_out);
  bool isFilterExist(const YAML::Node& filter_chain, const std::string filter_name);
  void removeInvalidNormal(DP& point_cloud);
  void applyGlobalMapFilter(DP& point_cloud);
  void applyBackgroundMapFilter(DP& background);
  void configDefaultPointFilter(PM::DataPointsFilters& filter);

  int saveToRsmap(const std::string& rsmap_file, DP& map, const std::string& version, const std::string& point_type,
                  const int height);  // this member function is for debugging

  CelledMap global_point_cloud_map_;
  DP local_point_cloud_map_;
  Eigen::Vector3f last_pose_;

  bool is_first_time_;
  bool is_gps_msg_;
  bool is_odom_map_msg_;
  bool is_map_with_normal_;

  std::string version_;
  int error_code_;        // 0: normal, -1: warning during loading map; -2: error during loading map
  float disp_def_level_;  // filter level of background map for display;
  float lidar_height_;

  DP background_;
  Eigen::Vector3d gps_origin_;
  std::vector<Eigen::Vector2f> poles_points_;
  std::vector<Eigen::Vector2f> icp_points_;
  std::vector<KeyFramePtr> vec_key_frames_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr poles_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapping_odom_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr icp_points_cloud_;

  // Param:
  YAML::Node params_;
  bool debug_;
  float height_search_radius_;
  float map_cell_length_;
  float local_map_radius_;
  float t_dis_update_localmap_;

  PM::DataPointsFilters local_map_filter_;
  PM::DataPointsFilters local_map_octree_filter_;
  PM::DataPointsFilters map_normal_filter_;
};

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_MAPSERVER_H */