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

#include "icp_localization/icp_point_filter.h"
#include <chrono>

namespace robosense
{
namespace localization
{
/************************************ ICPPointFilter ***********************************/
ICPPointFilter::ICPPointFilter(const YAML::Node& param) : yaml_param_(param)
{
  common::yamlRead(param, "name", name_, "ICPPointFilter");

  if (!common::yamlRead(param, "init", for_init_))
  {
    ERROR << name() << " init not set" << RESET << END;
    exit(-1);
  }
}

/************************************ ICPCustomPointFilter ***********************************/
ICPCustomPointFilter::ICPCustomPointFilter(const YAML::Node& param) : ICPPointFilter(param)
{
  config();
}

bool ICPCustomPointFilter::run(DP& scan)
{
  // TODO: add try catch
  // std::cout << "num points before filter = " << scan.getNbPoints() << std::endl;
  filter_.apply(scan);
  // std::cout << "num points after filter = " << scan.getNbPoints() << std::endl;
  return true;
}

bool ICPCustomPointFilter::config(void)
{
  common::yamlRead(yaml_param_, "use_default", use_default_, true);

  // TODO: deal with exception when invalid YAML node provided
  // this may be caused by missing of parameter file, include error, etc.
  if (use_default_ || !yaml_param_["filter_chain"])
  {
    WARNING << "Use default PointFilter config" << REND;
    if (for_init_)
    {
      configFilterInit();
    }
    else
    {
      configFilter();
    }
    return true;
  }

  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  YAML::Node filter_chain = yaml_param_["filter_chain"];

  for (std::size_t i = 0; i < filter_chain.size(); i++)
  // for (YAML::iterator it = filter_chain.begin(); it != filter_chain.end(); it++)
  {
    YAML::Node iterm = filter_chain[i];
    // ERROR << iterm << REND;
    if (iterm.Type() == YAML::NodeType::Map)
    {
      for (YAML::iterator it = iterm.begin(); it != iterm.end(); ++it)
      {
        name = it->first.as<std::string>();
        YAML::Node map = it->second;
        for (YAML::iterator it_map = map.begin(); it_map != map.end(); ++it_map)
        {
          std::string param_name(it_map->first.as<std::string>());
          std::string value_name(it_map->second.as<std::string>());
          // WARNING << param_name << ": " << value_name << RESET << END;
          params[param_name] = value_name;
        }
      }

      auto filter = PM::get().DataPointsFilterRegistrar.create(name, params);
      filter_.push_back(filter);
    }
    else
    {
      // WARNING << "here2" << REND;
      name = filter_chain[i].as<std::string>();
      // WARNING << "Scalar: " << name << RESET << END;
      auto filter = PM::get().DataPointsFilterRegistrar.create(name);
      filter_.push_back(filter);
    }
    params.clear();
  }
}

bool ICPCustomPointFilter::configFilterInit(void)
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // reading filter
  name = "RemoveNaNDataPointsFilter";
  std::shared_ptr<PM::DataPointsFilter> removenan_filter = PM::get().DataPointsFilterRegistrar.create(name);

  name = "MaxPointCountDataPointsFilter";
  params["seed"] = "4819354";
  params["maxCount"] = "2000";
  std::shared_ptr<PM::DataPointsFilter> maxcount_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  name = "OctreeGridDataPointsFilter";
  params["buildParallel"] = "1";
  params["maxPointByNode"] = "1";
  params["maxSizeByNode"] = "3";
  params["samplingMethod"] = "3";
  std::shared_ptr<PM::DataPointsFilter> octree_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // name = "SurfaceNormalDataPointsFilter";
  // params["knn"] = "8";
  // params["maxDist"] = "4";
  // params["epsilon"] = "3";
  // params["keepNormals"] = "1";
  // params["keepDensities"] = "0";
  // params["keepEigenValues"] = "0";
  // params["keepEigenVectors"] = "0";
  // params["smoothNormals"] = "0";
  // std::shared_ptr<PM::DataPointsFilter> surfacenormal_filter = PM::get().DataPointsFilterRegistrar.create(name,
  // params);
  // params.clear();

  // name = "OctreeGridDataPointsFilter";
  // params["buildParallel"] = "1";
  // params["maxPointByNode"] = "1";
  // params["maxSizeByNode"] = "3";
  // params["samplingMethod"] = "3";
  // std::shared_ptr<PM::DataPointsFilter> octree_filter1 = PM::get().DataPointsFilterRegistrar.create(name, params);
  // params.clear();

  PM::DataPointsFilters filter;
  filter_.push_back(removenan_filter);
  // filter_.push_back(maxcount_filter);
  filter_.push_back(octree_filter);
  // filter_.push_back(surfacenormal_filter);
  // filter_.push_back(maxcount_filter);

  return true;
}

bool ICPCustomPointFilter::configFilter(void)
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // reading filter
  name = "RemoveNaNDataPointsFilter";
  std::shared_ptr<PM::DataPointsFilter> removenan_filter = PM::get().DataPointsFilterRegistrar.create(name);

  // name = "MaxPointCountDataPointsFilter";
  // params["seed"] = "4819354";
  // params["maxCount"] = "2000";
  // std::shared_ptr<PM::DataPointsFilter> maxcount_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  // params.clear();

  name = "OctreeGridDataPointsFilter";
  params["buildParallel"] = "1";
  params["maxPointByNode"] = "1";
  params["maxSizeByNode"] = "3";
  params["samplingMethod"] = "3";
  std::shared_ptr<PM::DataPointsFilter> octree_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  // name = "SurfaceNormalDataPointsFilter";
  // params["knn"] = "8";
  // params["maxDist"] = "4";
  // params["epsilon"] = "3";
  // params["keepNormals"] = "1";
  // params["keepDensities"] = "0";
  // params["keepEigenValues"] = "0";
  // params["keepEigenVectors"] = "0";
  // params["smoothNormals"] = "0";
  // std::shared_ptr<PM::DataPointsFilter> surfacenormal_filter = PM::get().DataPointsFilterRegistrar.create(name,
  // params);
  // params.clear();

  // name = "OctreeGridDataPointsFilter";
  // params["buildParallel"] = "1";
  // params["maxPointByNode"] = "1";
  // params["maxSizeByNode"] = "3";
  // params["samplingMethod"] = "3";
  // std::shared_ptr<PM::DataPointsFilter> octree_filter1 = PM::get().DataPointsFilterRegistrar.create(name, params);
  // params.clear();

  PM::DataPointsFilters filter;
  filter_.push_back(removenan_filter);
  // filter_.push_back(maxcount_filter);
  filter_.push_back(octree_filter);
  // filter_.push_back(surfacenormal_filter);
  // filter_.push_back(maxcount_filter);

  return true;
}

}  // namespace localization
}  // namespace robosense
