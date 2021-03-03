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

#ifndef RS_LOCALIZATION_EVALUATOR_H
#define RS_LOCALIZATION_EVALUATOR_H

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
struct ICPEvaluator
{
  explicit ICPEvaluator(const YAML::Node& param);
  virtual ~ICPEvaluator() = default;

  virtual bool run(ICPSamples& samples, Eigen::Matrix4f& final_tf, Eigen::Matrix<float, 6, 6>& final_cov,
                   float& final_score) = 0;

protected:
  std::string name_;
  bool for_init_;
  YAML::Node yaml_param_;

  const std::string& name(void)
  {
    return name_;
  }
};

struct ICPNullEvaluator : public ICPEvaluator
{
  explicit ICPNullEvaluator(const YAML::Node& param);

  bool run(ICPSamples& samples, Eigen::Matrix4f& final_tf, Eigen::Matrix<float, 6, 6>& final_cov, float& final_score);
};

struct ICPMeanDistEvaluator : public ICPEvaluator
{
  explicit ICPMeanDistEvaluator(const YAML::Node& param);

  bool run(ICPSamples& samples, Eigen::Matrix4f& final_tf, Eigen::Matrix<float, 6, 6>& final_cov, float& final_score);

private:
  bool config(void);
  bool configMatcher(void);
  PM::ICPSequence scoring_icp_;
};

}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_EVALUATOR_H */