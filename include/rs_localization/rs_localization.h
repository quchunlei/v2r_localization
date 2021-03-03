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

#ifndef RS_LOCALIZATION_H
#define RS_LOCALIZATION_H

#include <string>
#include <vector>
#include <iostream>
#include <exception>
#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>
#include "msf/core/msf_state.h"
#include "msf/core/sensor_data.h"

namespace robosense
{
namespace localization
{
class RSLocalizationImpl;  // forward declaration
class RSLocalization
{
public:
  RSLocalization() = delete;
  RSLocalization(const std::string& map_file, const std::string& default_param_file, const std::string& param_file);
  ~RSLocalization();
  void reset(const msf::MsfState& state);
  void addGnssData(const msf::GnssData& data);
  void addLidarData(const msf::LidarData& data);
  void addInitialGuess(const Eigen::Matrix4d& data);
  bool start(void);
  bool stop(void);
  bool started(void);
  bool getEstimatedState(msf::MsfState& state);                   // Latest
  bool getEstimatedState(msf::MsfState& state, const double& t);  // State at time
  Eigen::Vector3d getGpsOrigin(void);
  DP getMap(void);
  int getStatus(void);
  YAML::Node& getParam(void);

  template<typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args&&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
private:
  std::unique_ptr<RSLocalizationImpl> pimpl_;
};
}  // namespace localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_H
