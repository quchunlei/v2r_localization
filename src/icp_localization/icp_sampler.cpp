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

#include "icp_localization/icp_sampler.h"
#include <chrono>

// TODO : Can calc height from map for each sample
namespace robosense
{
namespace localization
{
/*************************************** ICPSampler ******************************************/
ICPSampler::ICPSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param)
  : yaml_param_(param), rigid_trans_(PM::get().TransformationRegistrar.create("RigidTransformation"))
{
  map_server_ = map_server;

  common::yamlRead(param, "name", name_, "ICPSampler");
  common::yamlRead(param, "transform_map", transform_map_, false);

  if (!common::yamlRead(param, "init", for_init_))
  {
    ERROR << name() << " init not set" << RESET << END;
    exit(-1);
  }
}

bool ICPSampler::getMapAndTf(const Eigen::Matrix4f& init_pose, DP& map, Eigen::Matrix4f& map_tf,
                             Eigen::Matrix4f& scan_tf)
{
  Eigen::Vector3f init_translation;
  init_translation.x() = init_pose(0, 3);
  init_translation.y() = init_pose(1, 3);
  init_translation.z() = init_pose(2, 3);

  if (transform_map_)
  {
    map = rigid_trans_->compute(map_server_->getLocalMap(init_translation), init_pose.inverse());
    scan_tf = Eigen::Matrix4f::Identity();
    map_tf = init_pose;
  }
  else
  {
    map = map_server_->getLocalMap(init_translation);
    scan_tf = init_pose;
    map_tf = Eigen::Matrix4f::Identity();
  }
  if (map.getNbPoints() > 0)
    return true;
  else
    return false;
}
/*************************************** ICPNullSampler ******************************************/
ICPNullSampler::ICPNullSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param)
  : ICPSampler(map_server, param)
{
}

bool ICPNullSampler::run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples)
{
  DP map;
  Eigen::Matrix4f scan_tf;
  Eigen::Matrix4f map_tf;
  if (!getMapAndTf(init_pose, map, map_tf, scan_tf))
    return false;
  ICPRotatedSamples rotated_sample;
  rotated_sample.map_ = std::move(map);
  rotated_sample.scan_ = std::move(scan);
  rotated_sample.samples_.emplace_back(99999.0f, std::move(scan_tf), std::move(Eigen::Matrix<float, 6, 6>::Identity()));
  rotated_sample.map_tf_ = std::move(map_tf);
  samples.rotated_samples_.emplace_back(std::move(rotated_sample));
  return true;
}
/*************************************** ICPTransRotateSampler ******************************************/

ICPRotateSampler::ICPRotateSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param)
  : ICPSampler(map_server, param)
{
  int num_of_rotate;
  common::yamlRead(param, "num_of_rotate", num_of_rotate, 24);
  if (num_of_rotate < 0)
  {
    ERROR << "[>> " << name() << "] num_of_rotate less than 0" << RESET << END;
    exit(0);
  }
  else
  {
    num_of_rotate_ = num_of_rotate;
  }

  float rotate_delta;
  common::yamlRead(param, "rotate_delta", rotate_delta, 0);
  if (rotate_delta < 0)
  {
    ERROR << "[>> " << name() << "] rotate_delta less than 0" << RESET << END;
    exit(0);
  }
  else
  {
    rotate_delta_ = rotate_delta;
  }
}

bool ICPRotateSampler::sampleRotate(DP& map, DP& scan, Eigen::Matrix4f& map_tf, Eigen::Matrix4f& scan_tf,
                                    ICPRotatedSamples& sample)
{
  float angle_delta;
  std::vector<float> rotates;
  if (rotate_delta_ == 0)
  {
    angle_delta = 2 * M_PI / (num_of_rotate_ + 1);
    for (int i = 0; i < num_of_rotate_; i++)
    {
      rotates.emplace_back(angle_delta * i);
    }
    if (rotates.empty())  // when num_of_rotate and rotate_delta are both set to 0 accidentally.
    {
      rotates.emplace_back(0);
    }
  }
  else
  {
    angle_delta = rotate_delta_;
    for (int i = -num_of_rotate_ / 2; i <= num_of_rotate_ / 2; i++)
    {
      rotates.emplace_back(angle_delta * i);
    }
  }
  sample.map_ = std::move(map);
  sample.scan_ = std::move(scan);

  for (auto& rotate : rotates)
  {
    Eigen::Matrix4f local_tf = Eigen::Matrix4f::Identity();
    local_tf.block<3, 3>(0, 0) = Eigen::AngleAxisf(rotate, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    // local_tf = map_tf * scan_tf * local_tf;
    local_tf = scan_tf * local_tf;

    sample.samples_.emplace_back(99999.0f, std::move(local_tf), Eigen::Matrix<float, 6, 6>::Identity());
  }
  sample.map_tf_ = std::move(map_tf);
  return true;
}

bool ICPRotateSampler::run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples)
{
  DP map;
  Eigen::Matrix4f scan_tf;
  Eigen::Matrix4f map_tf;
  if (getMapAndTf(init_pose, map, map_tf, scan_tf))
    return false;

  ICPRotatedSamples sample;
  DP scan_cpy = scan;
  sampleRotate(map, scan_cpy, map_tf, scan_tf, sample);
  samples.rotated_samples_.emplace_back(std::move(sample));

  return true;
}

/*************************************** ICPTransRotateSampler ******************************************/

ICPTransRotateSampler::ICPTransRotateSampler(const std::shared_ptr<MapServer> map_server, const YAML::Node& param)
  : ICPRotateSampler(map_server, param)
{
  int num_of_trans;
  common::yamlRead(param, "num_of_trans", num_of_trans, 7);
  if (num_of_trans < 0)
  {
    ERROR << "[>> " << name() << "] num_of_trans less than 0" << RESET << END;
    exit(0);
  }
  else
  {
    num_of_trans_ = num_of_trans;
  }

  float trans_delta;
  common::yamlRead(param, "trans_delta", trans_delta, 4.0f);
  if (trans_delta < 0)
  {
    ERROR << "[>> " << name() << "] trans_delta less than 0" << RESET << END;
    exit(0);
  }
  else
  {
    trans_delta_ = trans_delta;
  }
}

bool ICPTransRotateSampler::run(DP& scan, const Eigen::Matrix4f& init_pose, ICPSamples& samples)
{
  for (int i = -num_of_trans_; i <= num_of_trans_; i++)  // Travel X
  {
    for (int j = -num_of_trans_; j <= num_of_trans_; j++)  // Travel Y
    {
      auto pose = init_pose;
      float x_offset = i * trans_delta_;
      float y_offset = j * trans_delta_;
      pose(0, 3) += x_offset;
      pose(1, 3) += y_offset;

      DP map;
      Eigen::Matrix4f scan_tf;
      Eigen::Matrix4f map_tf;
      if (!getMapAndTf(pose, map, map_tf, scan_tf))
        continue;

      ICPRotatedSamples rotated_sample;
      DP scan_cpy = scan;
      sampleRotate(map, scan_cpy, map_tf, scan_tf, rotated_sample);
      samples.rotated_samples_.emplace_back(std::move(rotated_sample));
    }
  }

  if (samples.rotated_samples_.size() == 0)
    return false;
  else
    return true;
}

}  // namespace localization
}  // namespace robosense