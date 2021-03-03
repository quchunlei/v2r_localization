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

#include "icp_localization/icp_evaluator.h"
#include <chrono>

namespace robosense
{
namespace localization
{
// PointCloudViewer viewer("view");
/************************************ ICPEvaluator ***********************************/
ICPEvaluator::ICPEvaluator(const YAML::Node& param) : yaml_param_(param)
{
  common::yamlRead(yaml_param_, "name", name_, "ICPPreMatcher");

  if (!common::yamlRead(yaml_param_, "init", for_init_))
  {
    ERROR << name() << " init not set" << RESET << END;
    exit(-1);
  }
}

/************************************ ICPNullEvaluator ***********************************/
ICPNullEvaluator::ICPNullEvaluator(const YAML::Node& param) : ICPEvaluator(param)
{
}

bool ICPNullEvaluator::run(ICPSamples& samples, Eigen::Matrix4f& final_tf, Eigen::Matrix<float, 6, 6>& final_cov,
                           float& final_score)
{
  if (samples.rotated_samples_.empty())
  {
    return false;
  }
  if (samples.rotated_samples_.front().samples_.empty())
  {
    return false;
  }
  final_tf = std::get<1>(samples.rotated_samples_.front().samples_.front());
  final_cov = std::get<2>(samples.rotated_samples_.front().samples_.front());
  final_score = 999999.0f;
}

/************************************ ICPMeanDistEvaluator ***********************************/
ICPMeanDistEvaluator::ICPMeanDistEvaluator(const YAML::Node& param) : ICPEvaluator(param)
{
  config();
}

bool ICPMeanDistEvaluator::config(void)
{
  // TODO: deal with exception when invalid YAML node provided
  // this may be caused by missing of parameter file, include error, etc.
  if (0)
  {
    configMatcher();
    return true;
  }

  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // config matcher
  YAML::Node matcher_node = yaml_param_["matcher"];
  name = matcher_node["name"].as<std::string>();
  for (YAML::iterator it_map = matcher_node.begin(); it_map != matcher_node.end(); ++it_map)
  {
    std::string param_name(it_map->first.as<std::string>());
    if (param_name == "name")
    {
      continue;
    }
    std::string value_name(it_map->second.as<std::string>());
    params[param_name] = value_name;
  }
  std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create(name, params);
  params.clear();
  scoring_icp_.matcher = matcher;

  // config outlier filters
  YAML::Node filters_node = yaml_param_["outlierFilters"];
  for (std::size_t i = 0; i < filters_node.size(); ++i)
  {
    YAML::Node iterm = filters_node[i];
    if (iterm.Type() == YAML::NodeType::Map)
    {
      name = iterm["name"].as<std::string>();
      for (YAML::iterator it = iterm.begin(); it != iterm.end(); ++it)
      {
        std::string param_name(it->first.as<std::string>());
        if (param_name == "name")
        {
          continue;
        }
        std::string value_name(it->second.as<std::string>());
        params[param_name] = value_name;
      }
      std::shared_ptr<PM::OutlierFilter> out_filters = PM::get().OutlierFilterRegistrar.create(name, params);
      params.clear();
      scoring_icp_.outlierFilters.push_back(out_filters);
    }
    else
    {
      WARNING << "OutlierFilter config failed." << REND;
    }
  }

  // config errorMinimizer
  YAML::Node error_node = yaml_param_["errorMinimizer"];
  name = error_node["name"].as<std::string>();
  for (YAML::iterator it_map = error_node.begin(); it_map != error_node.end(); ++it_map)
  {
    std::string param_name(it_map->first.as<std::string>());
    if (param_name == "name")
    {
      continue;
    }
    std::string value_name(it_map->second.as<std::string>());
    params[param_name] = value_name;
  }

  std::shared_ptr<PM::ErrorMinimizer> errorMin = PM::get().ErrorMinimizerRegistrar.create(name, params);
  params.clear();
  scoring_icp_.errorMinimizer = (errorMin);

  // config transformationCheckers

  YAML::Node checker_node = yaml_param_["transformationCheckers"];
  for (std::size_t i = 0; i < checker_node.size(); ++i)
  {
    YAML::Node iterm = checker_node[i];
    if (iterm.Type() == YAML::NodeType::Map)
    {
      name = iterm["name"].as<std::string>();
      for (YAML::iterator it = iterm.begin(); it != iterm.end(); ++it)
      {
        std::string param_name(it->first.as<std::string>());
        if (param_name == "name")
        {
          continue;
        }
        std::string value_name(it->second.as<std::string>());
        params[param_name] = value_name;
      }
      std::shared_ptr<PM::TransformationChecker> trans_checkers =
          PM::get().TransformationCheckerRegistrar.create(name, params);
      params.clear();
      scoring_icp_.transformationCheckers.push_back(trans_checkers);
    }
    else
    {
      WARNING << "TransformChecker config failed." << REND;
    }
  }

  // config inspector
  std::shared_ptr<PM::Inspector> null_inspect = PM::get().InspectorRegistrar.create("NullInspector");
  scoring_icp_.inspector = null_inspect;

  std::shared_ptr<PM::Transformation> rigid_trans = PM::get().TransformationRegistrar.create("RigidTransformation");
  scoring_icp_.transformations.push_back(rigid_trans);

  return true;
}

bool ICPMeanDistEvaluator::configMatcher(void)
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // Prepare matching function
  name = "KDTreeMatcher";
  params["knn"] = "1";
  params["maxDist"] = "100.0";
  params["epsilon"] = "0.1";
  std::shared_ptr<PM::Matcher> kdtree = PM::get().MatcherRegistrar.create(name, params);
  params.clear();

  // Prepare outlier filters
  name = "TrimmedDistOutlierFilter";
  params["ratio"] = "0.95";
  std::shared_ptr<PM::OutlierFilter> trim = PM::get().OutlierFilterRegistrar.create(name, params);
  params.clear();

  // name = "SurfaceNormalOutlierFilter";
  // params["maxAngle"] = "0.6";
  // std::shared_ptr<PM::OutlierFilter> trim_surface_normal = PM::get().OutlierFilterRegistrar.create(name, params);
  // params.clear();

  // Prepare error minimization
  name = "PointToPlaneWithCovErrorMinimizer";
  params["force2D"] = "1";
  std::shared_ptr<PM::ErrorMinimizer> point_to_plane = PM::get().ErrorMinimizerRegistrar.create(name, params);
  params.clear();

  // Prepare transformation checker filters
  name = "DifferentialTransformationChecker";
  params["minDiffRotErr"] = "0.001";
  params["minDiffTransErr"] = "0.01";
  params["smoothLength"] = "5";
  std::shared_ptr<PM::TransformationChecker> diff = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  name = "CounterTransformationChecker";
  params["maxIterationCount"] = "20";
  std::shared_ptr<PM::TransformationChecker> max_iter = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  // name = "BoundTransformationChecker";
  // params["maxRotationNorm"] = "0.4";
  // params["maxTranslationNorm"] = "10.0";
  // std::shared_ptr<PM::TransformationChecker> bound_trans = PM::get().TransformationCheckerRegistrar.create(name,
  // params);
  // params.clear();

  // Prepare inspector
  std::shared_ptr<PM::Inspector> null_inspect = PM::get().InspectorRegistrar.create("NullInspector");

  // Prepare transformation
  std::shared_ptr<PM::Transformation> rigid_trans = PM::get().TransformationRegistrar.create("RigidTransformation");

  scoring_icp_.matcher = kdtree;

  scoring_icp_.outlierFilters.push_back(trim);
  // scoring_icp_.outlierFilters.push_back(trim_surface_normal);

  scoring_icp_.errorMinimizer = point_to_plane;

  scoring_icp_.transformationCheckers.push_back(diff);
  scoring_icp_.transformationCheckers.push_back(max_iter);
  //   scoring_icp_.transformationCheckers.push_back(bound_trans);

  scoring_icp_.inspector = (null_inspect);
  scoring_icp_.transformations.push_back(rigid_trans);

  return true;
}

bool ICPMeanDistEvaluator::run(ICPSamples& samples, Eigen::Matrix4f& final_tf, Eigen::Matrix<float, 6, 6>& final_cov,
                               float& final_score)
{
  Eigen::Matrix4f* best_tf;
  Eigen::Matrix<float, 6, 6>* best_cov;
  float best_score = 999999.0f;
  bool success = false;
  int total_num = samples.rotated_samples_.size();
  int pos_num = 0;
  for (auto& pose_sample : samples.rotated_samples_)
  {
    const DP& map = pose_sample.map_;
    const DP& scan = pose_sample.scan_;

    scoring_icp_.matcher->init(map);

    if (for_init_)
    {
      INFO << "Initialization progress: " << RESET << int(pos_num * 20.0 / total_num) + 80 << "%" << '\r' << std::flush;
      ++pos_num;
    }
    for (auto& angle_sample : pose_sample.samples_)
    {
      float& score = std::get<0>(angle_sample);
      // score < 0 means icp fail
      if (score < 0)
        continue;

      Eigen::Matrix4f& local_tf = std::get<1>(angle_sample);
      Eigen::Matrix<float, 6, 6>& local_cov = std::get<2>(angle_sample);

      try
      {
        DP match_rotated_scan = scan;
        scoring_icp_.transformations.apply(match_rotated_scan, local_tf);
        const PM::Matches matches = scoring_icp_.matcher->findClosests(match_rotated_scan);
        const PM::OutlierWeights outlierWeights = scoring_icp_.outlierFilters.compute(match_rotated_scan, map, matches);
        const PM::ErrorMinimizer::ErrorElements matchedPoints(match_rotated_scan, map, outlierWeights, matches);
        const int dim = matchedPoints.reading.getEuclideanDim();
        const int nbMatchedPoints = matchedPoints.reading.getNbPoints();
        const PM::Matrix matchedRead = matchedPoints.reading.features.topRows(dim);
        const PM::Matrix matchedRef = matchedPoints.reference.features.topRows(dim);
        const PM::Matrix dist = (matchedRead - matchedRef).colwise().squaredNorm();
        score = dist.sum() / nbMatchedPoints;
      }
      catch (PointMatcher<float>::ConvergenceError& e)
      {
        continue;
      }
      success = true;

      // DP tmp_scan = scan;
      // auto* transform = PM::get().TransformationRegistrar.create("RigidTransformation");
      // tmp_scan = transform->compute(scan, local_tf);

      // viewer.setPointCloud(tmp_scan, Eigen::Vector3i(0, 0, 250), "scan");
      // viewer.setPointCloud(map, Eigen::Vector3i(0, 250, 0), "map");

      // std::cout << "final score = " << score << std::endl;
      // sleep(5);

      if (score < best_score)
      {
        best_score = score;
        best_tf = &local_tf;
        best_cov = &local_cov;
      }
    }
  }
  final_tf = *best_tf;
  final_cov = *best_cov;
  final_score = best_score;

  if (for_init_)
  {
    INFO << "Initialization progress: " << RESET << 100 << "%" << std::endl;
  }
  return success;
}

}  // namespace localization
}  // namespace robosense
