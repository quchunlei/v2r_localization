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

#ifndef RS_LOCALIZATION_ICPSAMPLER_H
#define RS_LOCALIZATION_ICPSAMPLER_H

#include <iostream>
#include <cmath>
#include <list>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include "map_server/map_server.h"
#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include "icp_localization/icp_common.h"

namespace robosense
{
namespace localization
{
struct ICPSampler
{
  ICPSampler() = delete;
  explicit ICPSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param = YAML::Node());
  virtual ~ICPSampler() = default;

  virtual bool run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples) = 0;

protected:
  std::shared_ptr<PM::Transformation> rigid_trans_;
  std::shared_ptr<MapServer> map_server_;

  bool translatePointCloud(const Eigen::Vector3f& translate, DP& point_cloud);
  bool transformPointCloud(const Eigen::Matrix4f& transform, DP& point_cloud);
  bool getMapAndTf(const Eigen::Matrix4f& init_pose, DP& map, Eigen::Matrix4f& map_tf, Eigen::Matrix4f& scan_tf);

  YAML::Node yaml_param_;
  std::string name_;
  bool for_init_;
  bool transform_map_;
  const std::string& name(void)
  {
    return name_;
  }
};

struct ICPNullSampler : public ICPSampler
{
  ICPNullSampler() = delete;
  explicit ICPNullSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param = YAML::Node());

  virtual bool run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples);

protected:
};

/******************************** Sample point with rotation at the init point ********************/
struct ICPRotateSampler : public ICPSampler
{
  ICPRotateSampler() = delete;
  explicit ICPRotateSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param = YAML::Node());

  virtual bool run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples);

protected:
  bool sampleRotate(DP& map, DP& scan, Eigen::Matrix4f& map_tf, Eigen::Matrix4f& scan_tf, ICPRotatedSamples& sample);
  int num_of_rotate_;
  float rotate_delta_;
};

/******************************** Sample point with rotation at the init point ********************/
struct ICPTransRotateSampler : public ICPRotateSampler
{
  ICPTransRotateSampler() = delete;
  explicit ICPTransRotateSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param = YAML::Node());

  virtual bool run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples);

protected:
  int num_of_trans_;
  float trans_delta_;
};

/******************************** The sampled points is bounded within a bound_dist of key_points ********************/
// struct ICPBoundedTranRotateSampler : public ICPSampler
// {
//   struct Options
//   {
//     Options()
//     {
//       name = "ICPBoundedTranRotateSampler";
//       num_of_rotate = 16;
//       num_of_trans = 1;
//       trans_dist = 10.0f;
//       bound_dist = 10.0f;
//     }
//     std::string name;
//     int num_of_rotate;
//     int num_of_trans;
//     float trans_dist;
//     float bound_dist;
//   };

//   ICPBoundedTranRotateSampler(Options option = Options(), std::list<Eigen::list3f> key_points);

//   bool sample(const DP& cloud_in, const Eigen::Matrix4f& trans_in,std::list<DP>& cloud_out,
//   std::list<Eigen::Matrix4f>& trans_out);

// private:
//   std::unique_ptr<PM::Transformation> pure_trans_;
//   // TODO: replace with BDTree
//   int num_of_rotate_;
//   int num_of_trans_;
//   float trans_dist_;
//   std::list<Eigen::list3f> key_points_;
//   float bound_dist_;
// };

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_ICPSAMPLER_H */