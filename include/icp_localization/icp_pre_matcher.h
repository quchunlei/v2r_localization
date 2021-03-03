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

#ifndef RS_LOCALIZATION_ICPPREMATCHER_H
#define RS_LOCALIZATION_ICPPREMATCHER_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/random_sample.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/exception/to_string.hpp>

#include "common/common.h"
#include "common/prompt.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"
#include "icp_localization/icp_common.h"

namespace robosense
{
namespace localization
{
struct ICPPreMatcher
{
  explicit ICPPreMatcher(const YAML::Node& param = YAML::Node());
  virtual ~ICPPreMatcher() = default;

  virtual bool run(ICPSamples& samples) = 0;

protected:
  std::string name_;
  bool for_init_;
  YAML::Node yaml_param_;

  const std::string& name(void)
  {
    return name_;
  }
};

struct ICPNullPreMatcher : public ICPPreMatcher
{
  explicit ICPNullPreMatcher(const YAML::Node& param = YAML::Node());

  bool run(ICPSamples& samples);

protected:
};

// struct ICPMatchedPointsPreMatcher : public ICPPreMatcher
// {
//   struct Options
//   {
//     Options()
//     {
//       name = "ICPMatchedPointsPreMatcher";
//       min_matched_percent = 0.9f;
//       remain_percent = 0.3f;
//     }
//     std::string name;
//     float min_matched_percent;
//     float remain_percent;
//   };

//   ICPMatchedPointsPreMatcher(Options option = Options());

//   bool match(DP& map, std::vector<DP>& cloud, std::vector<Eigen::Matrix4f>& trans);

// private:
//   boost::shared_ptr<PM::Matcher> matcher_;
//   boost::shared_ptr<PM::OutlierFilters> outlier_filter_;
//   boost::shared_ptr<PM::DataPointsFilters> data_filter_;
// };

// struct ICPDownSampleScorePreMatcher : public ICPPreMatcher
// {
//   struct Options
//   {
//     Options()
//     {
//       name = "ICPMatchedPointsPreMatcher";
//       min_matched_percent = 0.9f;
//       remain_percent = 0.3f;
//     }
//     std::string name;
//     float min_matched_percent;
//     float remain_percent;
//   };

//   ICPDownSampleScorePreMatcher(Options option = Options());

//   bool match(DP& map, std::vector<DP>& cloud, std::vector<Eigen::Matrix4f>& trans);

// private:
//   boost::shared_ptr<PM::Matcher> matcher_;
//   boost::shared_ptr<PM::OutlierFilters> outlier_filter_;
//   boost::shared_ptr<PM::DataPointsFilters> data_filter_;
// };

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_ICPPREMATCHER_H */