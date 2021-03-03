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

#ifndef RS_LOCALIZATION_OBSERVATION_QUEUE_H
#define RS_LOCALIZATION_OBSERVATION_QUEUE_H

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <Eigen/Core>

#include "common/common.h"
#include "common/prompt.h"
#include "msf/core/observation.h"
#include "msf/core/sensor_data.h"

namespace robosense
{
namespace localization
{
namespace msf
{
class ObservationQueue
{
public:
  ObservationQueue() = delete;
  ObservationQueue(const std::shared_ptr<std::condition_variable> obs_available, const std::string& name)
  {
    obs_available_ = obs_available;
    name_ = name;
    expected_period_ = 0.1;
    obs_queue_max_size_ = 200;
    prev_time_ = 0;
  }

  // Get the oldest observation, return false if empty
  bool getObservation(Observation& obs)
  {
    std::unique_lock<std::mutex> obs_lock(obs_queue_mutex_);
    if (!obs_queue_.empty())
    {
      obs = std::move(obs_queue_.front());
      obs_queue_.pop();
      return true;
    }
    else
    {
      return false;
    }
  }

  bool ready(void)
  {
    return true;
  }

  void addObservation(const Observation& obs)
  {
    double curr_time = obs.timestamp;
    prev_time_ = curr_time;
    double delta_time = curr_time - prev_time_;
    if (delta_time < 0)
    {
      // TODO: Handle error
      ERROR << name() << ": Observation went back to prev time! prev = " << prev_time_ << " curr = " << curr_time
            << RESET << END;
      return;
    }
    else if (delta_time > expected_period_ * 10)
    {
      // TODO: Handle error
      ERROR << name() << ": Observation lost more than 10 packets!" << RESET << END;
    }
    prev_time_ = curr_time;

    std::unique_lock<std::mutex> obs_lock(obs_queue_mutex_);
    obs_queue_.emplace(obs);

    if (obs_queue_.size() > obs_queue_max_size_)
    {
      WARNING << name() << " OBS queue overflow!" << RESET << END;
      // TODO : Handle this error
    }
    obs_available_->notify_one();
  }

  const std::string& name(void) const
  {
    return name_;
  }

private:
  // prediction process thread
  // Measurement here is actually "control" for Kalman filter
  std::queue<Observation> obs_queue_;
  size_t obs_queue_max_size_;
  std::mutex obs_queue_mutex_;
  std::shared_ptr<std::condition_variable> obs_available_;
  double oldest_time_;
  double prev_time_;

  double expected_period_;

  std::string name_;
  Eigen::Matrix4f transform_;  // TODO: Perform extrinsic correction in the future
};

}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /*RS_LOCALIZATION_OBSERVATION_QUEUE_H*/
