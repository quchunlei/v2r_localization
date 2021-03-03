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

#include "rs_localization/rs_localization.h"
#include "rs_localization/rs_localization_impl.h"

namespace robosense
{
namespace localization
{
RSLocalization::RSLocalization(const std::string& map_file, const std::string& default_param_file,
                               const std::string& param_file)
{
  pimpl_ = make_unique<RSLocalizationImpl>(map_file, default_param_file, param_file);
}

RSLocalization::~RSLocalization()
{
}

void RSLocalization::reset(const msf::MsfState& state)
{
  return pimpl_->reset(state);
}

void RSLocalization::addGnssData(const msf::GnssData& data)
{
  return pimpl_->addGnssData(data);
}

void RSLocalization::addInitialGuess(const Eigen::Matrix4d& data)
{
  return pimpl_->addInitialGuess(data);
}

void RSLocalization::addLidarData(const msf::LidarData& data)
{
  return pimpl_->addLidarData(data);
}

bool RSLocalization::start(void)
{
  return pimpl_->start();
}

bool RSLocalization::stop(void)
{
  return pimpl_->stop();
}

bool RSLocalization::started(void)
{
  return pimpl_->started();
}

bool RSLocalization::getEstimatedState(msf::MsfState& state)
{
  return pimpl_->getEstimatedState(state);
}

bool RSLocalization::getEstimatedState(msf::MsfState& state, const double& t)
{
  return pimpl_->getEstimatedState(state, t);
}

Eigen::Vector3d RSLocalization::getGpsOrigin(void)
{
  return pimpl_->getGpsOrigin();
}

DP RSLocalization::getMap(void)
{
  return pimpl_->getMap();
}

int RSLocalization::getStatus(void)
{
  return pimpl_->getStatus();
}

YAML::Node& RSLocalization::getParam()
{
  return pimpl_->getParam();
}

}  // namespace localization
}  // namespace robosense
