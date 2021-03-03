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

#include "icp_localization/icp_localization.h"
#include "common/time.h"

namespace robosense
{
namespace localization
{
// PointCloudViewer viewer("view");
ICPLocalization::ICPLocalization(const std::shared_ptr<MapServer> map_obj, const YAML::Node& param) : yaml_param_(param)
{
  loadParams();
  std::string filter_type;
  // ICP Point Filter
  {
    YAML::Node local_param;
    local_param = yaml_param_["point_filter"];
    common::yamlRead(local_param, "name", filter_type, "");
    local_param["init"] = for_init_;
    filter_ = std::unique_ptr<ICPPointFilter>(new ICPCustomPointFilter(local_param));
    // if (filter_type == "ICPCustomPointFilter")
    //   filter_ = std::unique_ptr<ICPPointFilter>(new ICPCustomPointFilter(local_param));
    // else if (for_init_)
    //   filter_ = std::unique_ptr<ICPPointFilter>(new ICPCustomPointFilter(local_param));
    // else
    //   filter_ = std::unique_ptr<ICPPointFilter>(new ICPCustomPointFilter(local_param));
  }

  // ICP Sampler
  {
    YAML::Node local_param;

    local_param = yaml_param_["sampler"];
    common::yamlRead(local_param, "name", filter_type, "");

    local_param["init"] = for_init_;
    if (filter_type == "RegistraNullSampler")
      sampler_ = std::unique_ptr<ICPSampler>(new ICPNullSampler(map_obj, local_param));
    else if (filter_type == "RegistraRotateSampler")
      sampler_ = std::unique_ptr<ICPSampler>(new ICPRotateSampler(map_obj, local_param));
    else if (filter_type == "RegistraTransRotateSampler")
      sampler_ = std::unique_ptr<ICPSampler>(new ICPTransRotateSampler(map_obj, local_param));
    else if (for_init_)
      sampler_ = std::unique_ptr<ICPSampler>(new ICPTransRotateSampler(map_obj, local_param));
    else
      sampler_ = std::unique_ptr<ICPSampler>(new ICPNullSampler(map_obj, local_param));
  }

  // ICP pre matcher
  {
    YAML::Node local_param;
    local_param = yaml_param_["pre_matcher"];
    common::yamlRead(local_param, "name", filter_type, "");

    local_param["init"] = for_init_;

    if (filter_type == "NullPreMatcher")
      pre_matcher_ = std::unique_ptr<ICPPreMatcher>(new ICPNullPreMatcher(local_param));
    else if (for_init_)
      pre_matcher_ = std::unique_ptr<ICPPreMatcher>(new ICPNullPreMatcher(local_param));
    else
      pre_matcher_ = std::unique_ptr<ICPPreMatcher>(new ICPNullPreMatcher(local_param));
  }

  // ICP matcher
  {
    YAML::Node local_param;
    local_param = yaml_param_["matcher"];
    common::yamlRead(local_param, "name", filter_type, "");
    local_param["init"] = for_init_;
    if (filter_type == "RegularMatcher")
      matcher_ = std::unique_ptr<ICPMatcher>(new PMICPMatcher(local_param));
    else if (filter_type == "InitMatcher")
      matcher_ = std::unique_ptr<ICPMatcher>(new PMICPMatcher(local_param));
    else
      matcher_ = std::unique_ptr<ICPMatcher>(new PMICPMatcher(local_param));

    // ICP evaluator
    local_param = yaml_param_["evaluator"];
    common::yamlRead(local_param, "name", filter_type, "");
    local_param["init"] = for_init_;
    if (filter_type == "MeanDistEvaluator")
      evaluator_ = std::unique_ptr<ICPEvaluator>(new ICPMeanDistEvaluator(local_param));
    else if (filter_type == "NullEvaluator")
      evaluator_ = std::unique_ptr<ICPEvaluator>(new ICPNullEvaluator(local_param));
    else if (for_init_)
      evaluator_ = std::unique_ptr<ICPEvaluator>(new ICPMeanDistEvaluator(local_param));
    else
      evaluator_ = std::unique_ptr<ICPEvaluator>(new ICPNullEvaluator(local_param));
  }
}

bool ICPLocalization::loadParams(void)
{
  if (!common::yamlRead(yaml_param_, "init", for_init_))
  {
    ERROR << name() << " init not set" << RESET << END;
    exit(-1);
  }

  return true;
}

bool ICPLocalization::locate(DP scan, const Eigen::Matrix4f& init_pose, Eigen::Matrix4f& output_pose)
{
  Eigen::Matrix<float, 6, 6> cov;
  return locate(std::move(scan), init_pose, output_pose, cov);
}

bool ICPLocalization::locate(DP scan, const Eigen::Matrix4f& init_pose, Eigen::Matrix4f& output_pose,
                             Eigen::Matrix<float, 6, 6>& output_cov)
{
  if (scan.getNbPoints() == 0)
  {
    ERROR << name() << ": The input cloud is empty" << RESET << END;
    return false;
  }

  // consume time
  Timer t;
  t.start();

  Eigen::Matrix4f init_tf = init_pose;
  Eigen::Matrix4f final_tf;
  Eigen::Matrix<float, 6, 6> final_cov;

  ICPSamples samples;
  bool success = filter_->run(scan);
  // DP scan_disp = scan;

  if (success)
    success = sampler_->run(scan, init_tf, samples);
  else
    return false;

  if (success)
    success = pre_matcher_->run(samples);
  else
  {
    WARNING << "Sampler is empty, pose estimate may be out of map." << REND;
    return false;
  }

  if (success)
    success = matcher_->run(samples);  // icp result stores in samples.rotated_samples[i].get<1>
  else
    return false;

  float final_score;
  if (success)
    evaluator_->run(samples, final_tf, final_cov, final_score);
  else
    return false;

  // t.stop();
  // INFO << "Localization time consumtion: " << RESET << t.get() * 1000 << " ms"
  //      << "\r" << std::flush;

  // std::cout << std::endl;
  // std::cout << init_pose << std::endl;
  // std::cout << final_tf << std::endl;
  // std::cout << output_cov << std::endl;
  // INFO << "-------------------------------" << REND;

  // auto* transform = PM::get().TransformationRegistrar.create("RigidTransformation");
  // DP init_scan = transform->compute(scan_disp, init_tf);
  // viewer.setPointCloud(init_scan, Eigen::Vector3i(250, 0, 0), "init_scan");
  // DP final_scan = transform->compute(scan_disp, final_tf);
  // DP map = samples.rotated_samples_.front().map_;
  // viewer.setPointCloud(final_scan, Eigen::Vector3i(0, 0, 250), "scan");
  // viewer.setPointCloud(map, Eigen::Vector3i(0, 50, 0), "map");

  output_pose = std::move(final_tf);
  output_cov = std::move(final_cov);
  return true;
}

}  // namespace localization
}  // namespace robosense
