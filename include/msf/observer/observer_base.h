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

#ifndef RS_LOCALIZATION_OBSERVER_BASE_H
#define RS_LOCALIZATION_OBSERVER_BASE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <Eigen/Core>

// #include "common/common.h"
// #include "common/prompt.h"
// #include "common/read_xml.h"
// #include "pointmatcher/point_cloud.h"
// #include "pointmatcher/transform.h"

#include "msf/core/sensor_fusion.h"
#include "msf/core/msf_state.h"
#include "msf/core/observation_queue.h"
#include "yaml/yaml.h"

namespace robosense
{
namespace localization
{
namespace msf
{
// TODO : Why should I do this
class SensorFusion;
class Observer
{
public:
  Observer() = delete;
  Observer(const std::string name, const std::shared_ptr<SensorFusion> sensor_fusion, const YAML::Node& param)
    : name_(name)
    , sensor_fusion_(sensor_fusion)
    , params_(param)
    , observer_thread_active_(false)
    , status_(Init)
    , prev_status_(Init)
  {
  }
  virtual ~Observer() = default;

  virtual void start(void) = 0;
  virtual void stop(void) = 0;
  virtual bool ready(void) = 0;
  virtual void printStatus(int prev_stat, int cur_stat, std::string data_name)
  {
    static int pre_sum{ prev_stat + cur_stat };
    static int cur_sum{ pre_sum };

    if (prev_stat == Init && cur_stat == SensorConnected || (pre_sum == 4 && cur_sum == 2))
    {
      INFO << DELETE_LINE << name() << ": " << data_name << " data Received for the first time." << REND;
    }
    else if ((prev_stat == SensorConnected || prev_stat == Init) && cur_stat == SensorDisconnected ||
             ((pre_sum == 2 || pre_sum == 4) && cur_sum == 0))
    {
      WARNING << DELETE_LINE << name() << ": No " << data_name << " data received for more than " << watchdog_thresh_
              << " secs!" << REND;
    }
    else if (prev_stat == SensorDisconnected && cur_stat == SensorConnected || (pre_sum == 0 && cur_sum == 2))
    {
      INFO << DELETE_LINE << name() << ": " << data_name << " data recovered." << REND;
    }

    pre_sum = cur_sum;
  }

  virtual void addGnssData(const GnssData& data)
  {
  }
  virtual void addImuData(const ImuData& data)
  {
  }
  virtual void addOdomData(const OdomData& data)
  {
  }
  virtual void addImuRtkData(const ImuRtkData& data)
  {
  }
  virtual void addLidarData(const LidarData& data)
  {
  }

  const std::string& name(void) const
  {
    return name_;
  }

protected:
  virtual void observerThread(void) = 0;
  virtual void config(const YAML::Node& param) = 0;
  enum ObserverStatus
  {
    SensorDisconnected = 0,
    SensorConnected,
    Init
  };
  std::atomic<int> status_;
  std::atomic<int> prev_status_;
  std::atomic<bool> observer_thread_active_;
  std::thread observer_thread_;
  const YAML::Node& params_;
  std::shared_ptr<SensorFusion> sensor_fusion_;
  std::string name_;
  size_t data_queue_max_size_;
  double watchdog_thresh_;
};

}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /*RS_LOCALIZATION_OBSERVER_BASE_H*/
