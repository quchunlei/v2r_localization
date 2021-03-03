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

#ifndef RS_LOCALIZATION_POLESLOCALIZATION_H
#define RS_LOCALIZATION_POLESLOCALIZATION_H

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "feature_localization/poles_localization_hm.h"
#include "common/save_to_csv.h"
#include "common/prompt.h"
#include "yaml/yaml.h"

namespace robosense
{
namespace localization
{
class PolesLocalization
{
private:
  struct PolesLocalizationOptions
  {
    PolesLocalizationOptions()
    {
      t_least_num_of_poles = 2;
      t_radius_kdsearch = 40;  // unit:m
      show_message = false;
    }
    int t_least_num_of_poles;
    int t_radius_kdsearch;
    bool show_message;
  };
  PolesLocalizationOptions options_;

public:
  explicit PolesLocalization(const std::string file_path);
  explicit PolesLocalization(const YAML::Node& param);

  bool loadConfigrationFile(const YAML::Node& param);

  bool locate(const std::vector<Eigen::Vector3f>& poles_points, const Eigen::Vector3f& initial_pose,
              Eigen::Vector3f& measure_pose);
  void setMap(const std::vector<Eigen::Vector2f>& poles_map);
  const std::vector<Eigen::Vector3f>& getLocalMapPoles() const
  {
    return loc_map_;
  };

  std::string name() const
  {
    return "FeatureLocalization";
  };

private:
  const PolesLocalizationOptions& getOptions() const
  {
    return options_;
  }
  void kdTreeRadiusSearch(const Eigen::Vector3f& initial_pose, float search_radius,
                          std::vector<Eigen::Vector3f>& resultant_points);

private:
  bool is_set_map_;
  std::vector<Eigen::Vector3f> loc_map_;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_save_;
  std::unique_ptr<PolesLocalizationHM> poles_localization_hm_;
};

}  // namespace localization
}  // namespace robosense
#endif  // RS_LOCALIZATION_POLESLOCALIZATION_H
