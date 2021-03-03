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
#ifndef RS_LOCALIZATION_SENSOR_FUSION_H
#define RS_LOCALIZATION_SENSOR_FUSION_H

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cmath>
#include <vector>
#include <math.h>
#include <Eigen/Core>

#include "common/common.h"
#include "common/prompt.h"
#include "yaml/yaml.h"
#include "pointmatcher/point_cloud.h"
#include "pointmatcher/transform.h"

#include "msf/observer/observer_base.h"
#include "msf/core/msf_state.h"
#include "msf/core/observation_queue.h"

namespace robosense
{
namespace localization
{
namespace msf
{
class SensorFusion
{
public:
  SensorFusion() = delete;
  SensorFusion(const YAML::Node& param);

  ~SensorFusion();

  bool start(void);
  bool stop(void);
  bool started(void);
  void config(const YAML::Node& param);

  void addObservation(const Observation& obs);

  bool reset(const MsfState& init_state);

  bool getEstimatedState(MsfState& state);                      // Latest
  bool getEstimatedState(MsfState& state, const double& time);  // State at time
  bool getLidarState(MsfState& state);                          // get latest state updated by lidar

private:
  using FusionStates = std::map<double, MsfState>;
  using Observations = std::map<double, Observation>;
  void fusionThread(void);
  MsfState getPredictionState(double t, Observation& obs);
  MsfState getCorrectionState(double t, Observation& obs);
  MsfState fastApproximateState(const MsfState& state, const double& t);

  // clang-format off
  std::map<std::string, ObservationQueue>           observation_queues_;
  std::string                                       prediction_observer_name_;
  std::set<std::string>                             correction_observer_names_;

  std::shared_ptr<std::condition_variable>          obs_available_;
  std::atomic<bool>                                 fusion_thread_active_;
  std::atomic<bool>                                 init_state_set_;
  std::thread                                       fusion_thread_;
  Observations                                      observation_list_;
  std::mutex                                        observations_mutex_;
  FusionStates                                      fusion_states_;
  std::mutex                                        fusion_states_mutex_;
  FusionStates                                      lidar_states_;
  std::mutex                                        lidar_states_mutex_;
  std::string                                       name_;
  // clang-format on

  const std::string& name(void)
  {
    return name_;
  }
};

}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_SENSOR_FUSION_H */
