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

#include "icp_localization/icp_matcher.h"
#include <chrono>

namespace robosense
{
namespace localization
{
// PointCloudViewer viewer("view");
/************************************ ICPMatcher ***********************************/
ICPMatcher::ICPMatcher(const YAML::Node& param) : yaml_param_(param)
{
  common::yamlRead(param, "name", name_, "ICPMatcher");

  if (!common::yamlRead(param, "init", for_init_))
  {
    ERROR << name() << " init not set" << RESET << END;
    exit(-1);
  }
}

/************************************ PMICPMatcher ***********************************/
PMICPMatcher::PMICPMatcher(const YAML::Node& param) : ICPMatcher(param)
{
  config();
}

bool PMICPMatcher::config(void)
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // TODO: deal with exception when invalid YAML node provided
  // this may be caused by missing of parameter file, include error, etc.

  // TODO: include yaml file failed, use default parameters. The following way is incorrect.
  if (yaml_param_.Type() != YAML::NodeType::Map)
  {
    if (for_init_)
    {
      configMatcherInit();
    }
    else
    {
      configMatcher();
    }

    return true;
  }

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
    // WARNING << param_name << ": " << value_name << RESET << END;
    params[param_name] = value_name;
  }
  std::shared_ptr<PM::Matcher> matcher = PM::get().MatcherRegistrar.create(name, params);
  params.clear();
  icp_.matcher = (matcher);

  // config outlier filters
  YAML::Node filters_node = yaml_param_["outlierFilters"];
  for (std::size_t i = 0; i < filters_node.size(); i++)
  {
    YAML::Node iterm = filters_node[i];
    if (iterm.Type() == YAML::NodeType::Map)
    {
      name = iterm["name"].as<std::string>();
      // WARNING << name << REND;
      for (YAML::iterator it = iterm.begin(); it != iterm.end(); ++it)
      {
        std::string param_name(it->first.as<std::string>());
        if (param_name == "name")
        {
          continue;
        }
        std::string value_name(it->second.as<std::string>());
        // WARNING << param_name << ": " << value_name << RESET << END;
        params[param_name] = value_name;
      }
      std::shared_ptr<PM::OutlierFilter> out_filters = PM::get().OutlierFilterRegistrar.create(name, params);
      params.clear();
      icp_.outlierFilters.push_back(out_filters);
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
    // WARNING << param_name << ": " << value_name << RESET << END;
    params[param_name] = value_name;
  }

  std::shared_ptr<PM::ErrorMinimizer> errorMin = PM::get().ErrorMinimizerRegistrar.create(name, params);
  params.clear();
  icp_.errorMinimizer = (errorMin);

  // config transformationCheckers

  YAML::Node checker_node = yaml_param_["transformationCheckers"];
  for (std::size_t i = 0; i < checker_node.size(); i++)
  {
    YAML::Node iterm = checker_node[i];
    if (iterm.Type() == YAML::NodeType::Map)
    {
      name = iterm["name"].as<std::string>();
      // WARNING << name << REND;
      for (YAML::iterator it = iterm.begin(); it != iterm.end(); ++it)
      {
        std::string param_name(it->first.as<std::string>());
        if (param_name == "name")
        {
          continue;
        }
        std::string value_name(it->second.as<std::string>());
        // WARNING << param_name << ": " << value_name << RESET << END;
        params[param_name] = value_name;
      }
      std::shared_ptr<PM::TransformationChecker> trans_checkers =
          PM::get().TransformationCheckerRegistrar.create(name, params);
      params.clear();
      icp_.transformationCheckers.push_back(trans_checkers);
    }
    else
    {
      WARNING << "TransformChecker config failed." << REND;
    }
  }

  // config inspector
  std::shared_ptr<PM::Inspector> null_inspect = PM::get().InspectorRegistrar.create("NullInspector");
  icp_.inspector = (null_inspect);

  std::shared_ptr<PM::Transformation> rigid_trans = PM::get().TransformationRegistrar.create("RigidTransformation");
  icp_.transformations.push_back(rigid_trans);

  return true;
}

bool PMICPMatcher::configMatcher()
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // Prepare matching function
  name = "KDTreeMatcher";
  params["knn"] = "1";
  params["maxDist"] = "15.0";
  params["epsilon"] = "1";
  std::shared_ptr<PM::Matcher> kdtree = PM::get().MatcherRegistrar.create(name, params);
  params.clear();

  // Prepare outlier filters
  name = "VarTrimmedDistOutlierFilter";
  params["minRatio"] = "0.4";
  params["maxRatio"] = "0.9";
  params["lambda"] = "0.9";
  std::shared_ptr<PM::OutlierFilter> trim = PM::get().OutlierFilterRegistrar.create(name, params);
  params.clear();

  // name = "SurfaceNormalOutlierFilter";
  // params["maxAngle"] = "1.5";
  // std::shared_ptr<PM::OutlierFilter> trim_surface_normal = PM::get().OutlierFilterRegistrar.create(name, params);
  // params.clear();

  // Prepare error minimization
  name = "PointToPlaneWithCovErrorMinimizer";
  params["force2D"] = "1";
  params["sensorStdDev"] = "1.5";
  std::shared_ptr<PM::ErrorMinimizer> point_to_plane = PM::get().ErrorMinimizerRegistrar.create(name, params);
  params.clear();

  // Prepare transformation checker filters
  name = "DifferentialTransformationChecker";
  params["minDiffRotErr"] = "0.001";
  params["minDiffTransErr"] = "0.01";
  params["smoothLength"] = "2";
  std::shared_ptr<PM::TransformationChecker> diff = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  name = "CounterTransformationChecker";
  params["maxIterationCount"] = "8";
  std::shared_ptr<PM::TransformationChecker> max_iter = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  name = "BoundTransformationChecker";
  params["maxRotationNorm"] = "0.1";
  params["maxTranslationNorm"] = "3.0";
  std::shared_ptr<PM::TransformationChecker> bound_trans =
      PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  // Prepare inspector
  std::shared_ptr<PM::Inspector> null_inspect = PM::get().InspectorRegistrar.create("NullInspector");

  // Prepare transformation
  std::shared_ptr<PM::Transformation> rigid_trans = PM::get().TransformationRegistrar.create("RigidTransformation");

  icp_.matcher = (kdtree);

  icp_.outlierFilters.push_back(trim);
  // icp_.outlierFilters.push_back(trim_surface_normal);

  icp_.errorMinimizer = (point_to_plane);

  icp_.transformationCheckers.push_back(diff);
  icp_.transformationCheckers.push_back(max_iter);
  icp_.transformationCheckers.push_back(bound_trans);

  icp_.inspector = (null_inspect);
  icp_.transformations.push_back(rigid_trans);

  return true;
}

bool PMICPMatcher::configMatcherInit()
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // Prepare matching function
  name = "KDTreeMatcher";
  params["knn"] = "1";
  params["maxDist"] = "10.0";
  params["epsilon"] = "0.3";
  std::shared_ptr<PM::Matcher> kdtree = PM::get().MatcherRegistrar.create(name, params);
  params.clear();

  // Prepare outlier filters
  name = "VarTrimmedDistOutlierFilter";
  params["minRatio"] = "0.4";
  params["maxRatio"] = "0.99";
  params["lambda"] = "0.95";
  std::shared_ptr<PM::OutlierFilter> trim = PM::get().OutlierFilterRegistrar.create(name, params);
  params.clear();

  // name = "SurfaceNormalOutlierFilter";
  // params["maxAngle"] = "1.5";
  // std::shared_ptr<PM::OutlierFilter> trim_surface_normal = PM::get().OutlierFilterRegistrar.create(name, params);
  // params.clear();

  // Prepare error minimization
  name = "PointToPlaneWithCovErrorMinimizer";
  params["force2D"] = "1";
  params["sensorStdDev"] = "1.5";
  std::shared_ptr<PM::ErrorMinimizer> point_to_plane = PM::get().ErrorMinimizerRegistrar.create(name, params);
  params.clear();

  // Prepare transformation checker filters
  name = "DifferentialTransformationChecker";
  params["minDiffRotErr"] = "0.02";
  params["minDiffTransErr"] = "0.3";
  params["smoothLength"] = "2";
  std::shared_ptr<PM::TransformationChecker> diff = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  name = "CounterTransformationChecker";
  params["maxIterationCount"] = "10";
  std::shared_ptr<PM::TransformationChecker> max_iter = PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  name = "BoundTransformationChecker";
  params["maxRotationNorm"] = "0.3";
  params["maxTranslationNorm"] = "6.0";
  std::shared_ptr<PM::TransformationChecker> bound_trans =
      PM::get().TransformationCheckerRegistrar.create(name, params);
  params.clear();

  // Prepare inspector
  std::shared_ptr<PM::Inspector> null_inspect = PM::get().InspectorRegistrar.create("NullInspector");

  // Prepare transformation
  std::shared_ptr<PM::Transformation> rigid_trans = PM::get().TransformationRegistrar.create("RigidTransformation");

  icp_.matcher = (kdtree);

  icp_.outlierFilters.push_back(trim);
  // icp_.outlierFilters.push_back(trim_surface_normal);

  icp_.errorMinimizer = (point_to_plane);

  // icp_.transformationCheckers.push_back(diff);
  icp_.transformationCheckers.push_back(max_iter);
  icp_.transformationCheckers.push_back(bound_trans);

  icp_.inspector = (null_inspect);
  icp_.transformations.push_back(rigid_trans);

  return true;
}

bool PMICPMatcher::run(ICPSamples& samples)
{
  int pos_num = 0;
  bool success = false;
  int total_num = samples.rotated_samples_.size();

  for (auto& pose_sample : samples.rotated_samples_)
  {
    const DP& map = pose_sample.map_;
    const DP& scan = pose_sample.scan_;
    // TODO: Add error checking
    bool ret = icp_.setMap(map);
    int angle_num = 0;
    if (for_init_)
    {
      INFO << "Initialization progress: " << RESET << int(pos_num * 80.0 / total_num) << "%" << '\r' << std::flush;
      ++pos_num;
    }
    for (auto& angle_sample : pose_sample.samples_)
    {
      angle_num++;
      float& score = std::get<0>(angle_sample);
      Eigen::Matrix4f& local_tf = std::get<1>(angle_sample);
      Eigen::Matrix<float, 6, 6>& cov = std::get<2>(angle_sample);

      // std::cout << "hello scan size = " << scan.getNbPoints() << " map size = " << map.getNbPoints() << std::endl;
      // DP tmp_scan = scan;
      // auto* transform = PM::get().TransformationRegistrar.create("RigidTransformation");
      // tmp_scan = transform->compute(scan, local_tf);
      try
      {
        local_tf = std::move(icp_(scan, local_tf));
      }
      catch (PointMatcher<float>::ConvergenceError& e)
      {
        score = -1;
        // ERROR << e.what() << RESET << END;
        continue;
      }
      success = true;
      cov = std::move(icp_.errorMinimizer->getCovariance());
      // std::cout << std::endl << icp_.errorMinimizer->getCovariance() << std::endl;
      // std::cout << local_tf << std::endl;
      // DP icp_tmp_scan = scan;
      // icp_tmp_scan = transform->compute(scan, local_tf);
      // viewer.setPointCloud(tmp_scan, Eigen::Vector3i(0, 0, 250), "scan");
      // viewer.setPointCloud(icp_tmp_scan, Eigen::Vector3i(250, 0, 0), "icp_scan");
      // viewer.setPointCloud(map, Eigen::Vector3i(0, 100, 0), "map");

      // sleep(1);
    }
  }

  return success;
}

}  // namespace localization
}  // namespace robosense
