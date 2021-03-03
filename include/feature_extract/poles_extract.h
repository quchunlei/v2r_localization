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

#ifndef PROJECTS_POLES_EXTRACT_H
#define PROJECTS_POLES_EXTRACT_H

#include <pcl/io/pcd_io.h>
#include <pcl/console/time.h>
#include <pcl/common/transforms.h>  //allows us to use pcl::transformPointCloud function
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>  //随机样本一致性算法 方法类型
#include <pcl/segmentation/sac_segmentation.h>  //随机样本一致性算法 分割方法
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "feature_extract/cluster.h"
#include "feature_extract/ground_estimator.h"
#include "common/compute.h"
#include "common/prompt.h"
#include "common/time.h"
#include "yaml/yaml.h"

namespace robosense
{
namespace localization
{
using namespace pcl;
using namespace std;

#define PointType PointXYZI
#define PointCloudType PointCloud<PointType>
#define PointCloudTypePtr PointCloud<PointType>::Ptr
#define PointCloudTypeConstPtr PointCloud<PointType>::ConstPtr

class PolesExtract
{
private:
  struct ExtractOptions
  {
    ExtractOptions()
    {
      t_of_ground_seg = 0.4;  // m
      num_of_iteration_seg = 40;
      t_angle_of_z_axis = 15.0;  // deg
      t_radius_of_tree = 0.4;    // m
      t_scale_of_tree_point_number = 0.8;
      t_height_of_tree_z = 1.0;  // height threshold of a tree
      t_bottom_of_tree = 0;      // if object is tree, it's min z should small than the value
      t_max_z = 1.0f;            // the max z to filter point cloud
      t_point_num_of_tree = 10;
      start_to_ground = 0.3f;
      show_message = false;
      show_pointcloud = false;

      x_min = 0;
      x_max = 60;
      y_min = -30;
      y_max = 30;
      grid_size = 0.1875;  //=60/320
      scan_range = 1;
      cluster_z = 1.5f;
      enable_growth = false;
      growth_z = 3.0f;
      growth_step = 0.25f;
    };
    float t_of_ground_seg;
    int num_of_iteration_seg;
    float t_angle_of_z_axis;
    float t_radius_of_tree;
    float t_scale_of_tree_point_number;
    float t_height_of_tree_z;
    int t_point_num_of_tree;
    float start_to_ground;
    float t_bottom_of_tree;
    float t_max_z;
    bool show_message;
    bool show_pointcloud;
    // for cluster
    float x_max;
    float x_min;
    float y_max;
    float y_min;
    float grid_size;
    int scan_range;
    float cluster_z;
    bool enable_growth;
    float growth_z;
    float growth_step;
  };
  ExtractOptions options_;

private:
  const ExtractOptions& getOptions() const
  {
    return options_;
  };

public:
  explicit PolesExtract();
  explicit PolesExtract(const YAML::Node& config_file);

public:
  bool loadConfigrationFile(const YAML::Node& param);
  //提出的杆子中心坐标位于雷达自身坐标系下
  bool extract(PointCloudTypeConstPtr input_cloud, PointCloudTypePtr poles_position_cloud);
  bool extract(PointCloudTypeConstPtr input_cloud, PointCloudTypePtr poles_position_cloud, const Eigen::Vector3f& pose,
               const float& dis_lidar_to_ground);

  std::string name() const
  {
    return "FeatureExtract";
  }

private:
  boost::shared_ptr<GroundEstimator> ground_estimator_obj_;
  boost::shared_ptr<Cluster> cluster_obj_;
  bool removeGround(PointCloudTypeConstPtr input, PointCloudTypePtr output, float t_of_ground_seg,
                    float t_angle_of_z_axis, bool show_message, const string& name,
                    // output
                    Eigen::Vector3f& angle, float& distance);
  void filter(PointCloudTypeConstPtr input, PointCloudTypePtr output, float min, float max);
  void extract_op(PointCloudTypeConstPtr input_cloud, PointCloudTypePtr poles_position_cloud);
  PointType getCentriod(PointCloudTypePtr point_cloud);
  void check(vector<PointCloudTypePtr>& cloud_ptr_list, PointCloudTypePtr poles_position_cloud);
  bool isTree(PointCloudTypePtr point_cloud, PointType centriod);
};

inline static void Vector4f2PointXYZI(const Eigen::Vector4f& v, PointXYZI& p)
{
  p.x = v[0];
  p.y = v[1];
  p.z = v[2];
}

}  // namespace localization
}  // namespace robosense
#endif  // PROJECTS_POLES_EXTRACT_H
