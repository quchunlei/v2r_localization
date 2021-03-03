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

#include "msf/core/sensor_fusion.h"
#include "msf/observer/imu_odom_observer.h"
#include "common/time.h"
#include "common/eigen_util.h"

namespace robosense
{
namespace localization
{
namespace msf
{
SensorFusion::SensorFusion(const YAML::Node& param) : obs_available_(new std::condition_variable)
{
  fusion_thread_active_ = false;
  config(param);
  start();
}

SensorFusion::~SensorFusion()
{
  stop();
}

bool SensorFusion::start(void)
{
  // TODO: check if can be enabled
  fusion_thread_active_ = true;
  const auto& func = [this] { fusionThread(); };
  fusion_thread_ = std::thread(func);
  return true;
}

bool SensorFusion::stop(void)
{
  if (fusion_thread_active_.load())
  {
    // TODO: disable other modules
    fusion_thread_active_ = false;
    obs_available_->notify_one();
    fusion_thread_.join();
  }
  return true;
}

bool SensorFusion::started(void)
{
  if (fusion_thread_active_.load())
    return true;
  else
    return false;
}

void SensorFusion::config(const YAML::Node& param)
{
  if (fusion_thread_active_.load())
  {
    ERROR << "Sensor Fusion already enabled when Initializing !" << RESET << END;
    exit(-1);
  }

  init_state_set_ = false;

  common::yamlRead(param, "name", name_, "SensorFusion");

  /***************** Init Observer *******************/
  // TODO: Add more Init option for Observer

  bool is_prediciton_defined = false;
  for (auto observer_param : param["Observers"])
  {
    std::string obs_type;
    if (!common::yamlRead(observer_param, "type", obs_type))
    {
      ERROR << name() << " Undefined observer type" << RESET << END;
      exit(-1);
    }

    std::string obs_name;
    if (!common::yamlRead(observer_param, "name", obs_name))
    {
      ERROR << name() << " Undefined observer name" << RESET << END;
      exit(-1);
    }

    if (obs_type == "Prediction")
    {
      if (is_prediciton_defined == false)
        is_prediciton_defined = true;
      else
      {
        ERROR << name() << " Cannot have more than one observers" << RESET << END;
        exit(-1);
      }
      prediction_observer_name_ = obs_name;
    }
    else if (obs_type == "Correction")
    {
      correction_observer_names_.emplace(obs_name);
    }
    else
    {
      ERROR << name() << " Wrong observer type : " << obs_type << RESET << END;
      exit(-1);
    }

    // clang-format off
      observation_queues_.emplace(std::piecewise_construct, 
                                  std::forward_as_tuple(obs_name),
                                  std::forward_as_tuple(obs_available_, obs_name));
    // clang-format on
  }
  /***************** Init Sensor Fusion **************/
  name_ = "SensorFusion";
}

void SensorFusion::addObservation(const Observation& obs)
{
  if (!fusion_thread_active_.load())
    return;

  auto observation_queue_itr = observation_queues_.find(obs.obs_source);
  if (observation_queue_itr != observation_queues_.end())
  {
    observation_queue_itr->second.addObservation(obs);
  }
}

bool SensorFusion::getLidarState(MsfState& state)
{
  // TODO: If not located return false else return true
  std::unique_lock<std::mutex> lock(lidar_states_mutex_);
  if (lidar_states_.empty())
  {
    lock.unlock();
    return false;
  }

  state = lidar_states_.rbegin()->second;
  return true;
}

bool SensorFusion::getEstimatedState(MsfState& state)
{
  // TODO: If not located return false else return true
  std::unique_lock<std::mutex> lock(fusion_states_mutex_);
  if (fusion_states_.empty())
  {
    lock.unlock();
    return false;
  }

  state = fusion_states_.rbegin()->second;
  return true;
}

bool SensorFusion::getEstimatedState(MsfState& new_state, const double& t)
{
  // TODO: If not located return false else return true
  std::unique_lock<std::mutex> lock(fusion_states_mutex_);
  if (fusion_states_.empty())
  {
    lock.unlock();
    ERROR << "getEstimatedState : fusion_states_ empty, this should never happen" << RESET << END;
    exit(-1);
  }

  auto state_itr = fusion_states_.upper_bound(t);
  if (state_itr == fusion_states_.begin())
  {
    WARNING << "getEstimatedState : There is no earlier state" << RESET << END;
    new_state = state_itr->second;
    return true;
  }

  double dt = t - state_itr->first;

  // If very close to the early state just use the early state
  if (dt < 0.001)
  {
    new_state = state_itr->second;
    new_state.timestamp = t;
  }
  else
  {
    --state_itr;
    new_state = fastApproximateState(state_itr->second, t);
  }

  return true;
}

MsfState SensorFusion::fastApproximateState(const MsfState& state, const double& t)
{
  // TODO: Change implementation according to sola's paper
  MsfState new_state = state;
  new_state.seq = state.seq + 1;
  new_state.timestamp = t;
  double dt = t - state.timestamp;

  // Acc measurement and w measurement are stored in each state object
  auto acc = state.state_vec.segment<3>(StateLinearAccIdx);
  auto w = state.state_vec.segment<3>(StateAngleVelIdx);

  // Assume covariance don't change
  // Only update
  Eigen::Matrix4f OmegaMean = getOmegaMat(new_state.state_vec.segment<3>(StateAngleVelIdx)) * 0.5f * dt;
  new_state.state_vec.segment<4>(StateAngleIdx) =
      (Eigen::Matrix4f::Identity() + OmegaMean) * state.state_vec.segment<4>(StateAngleIdx);
  new_state.state_vec.segment<4>(StateAngleIdx).normalize();

  Eigen::Vector4f new_att = new_state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Vector4f att = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Quaternionf new_quat(new_att[3], new_att[0], new_att[1], new_att[2]);
  Eigen::Quaternionf quat(att[3], att[0], att[1], att[2]);
  Eigen::Matrix3f rot_mat = (new_quat * quat.conjugate()).toRotationMatrix();

  new_state.state_vec.segment<3>(StateLinearVelIdx) = rot_mat * state.state_vec.segment<3>(StateLinearVelIdx);
  new_state.state_vec.segment<3>(StateOdomBiasIdx) = rot_mat * state.state_vec.segment<3>(StateOdomBiasIdx);

  new_state.state_vec.segment<3>(StatePositionIdx) =
      state.state_vec.segment<3>(StatePositionIdx) +
      (state.state_vec.segment<3>(StateLinearVelIdx) + new_state.state_vec.segment<3>(StateLinearVelIdx)) * 0.5f * dt;

  // std::cout << "\n\rprev state : \n\r" << state.state_vec << std::endl;
  // std::cout << "curr state : \n\r" << new_state.state_vec << std::endl;
  return std::move(new_state);
}

MsfState SensorFusion::getPredictionState(double t, Observation& obs)
{
  MsfState new_state;

  auto state_itr = fusion_states_.upper_bound(t);
  if (state_itr == fusion_states_.begin())
  {
    ERROR << name() << " fusion_states_ cannot find a earlier state, this should never happen" << RESET << END;
    exit(-1);
  }
  --state_itr;

  obs.apply(obs, state_itr->second, new_state);
  // std::cout << "pred state vec = \n\r" << new_state.state_vec << std::endl;
  new_state.is_prediction = true;
  return std::move(new_state);
}

MsfState SensorFusion::getCorrectionState(double t, Observation& obs)
{
  MsfState new_state;

  // TODO : add invalid state verification
  auto state_itr = fusion_states_.upper_bound(t);
  if (state_itr == fusion_states_.begin())
  {
    ERROR << name() << " fusion_states_ cannot find a earlier state, this should never happen" << RESET << END;
    exit(-1);
  }

  double dt = t - state_itr->first;

  // If very close to the early state just use the early state
  if (state_itr != fusion_states_.end() && dt < 0.001)
  {
    // std::cout << "here" << std::endl;
    state_itr--;
    new_state = state_itr->second;
    new_state.timestamp = t;
    // std::cout << " Correction and prev too close, just copying the old state \n\r";
  }
  else
  {
    --state_itr;
    new_state = fastApproximateState(state_itr->second, t);
  }

  MsfState state = new_state;
  obs.apply(obs, state, new_state);
  new_state.is_prediction = false;

  // std::cout << "correct state vec = \n\r" << new_state.state_vec << std::endl;

  return std::move(new_state);
}

bool SensorFusion::reset(const MsfState& init_state)
{
  fusion_states_.clear();
  observation_list_.clear();
  fusion_states_.emplace(init_state.timestamp, init_state);
  init_state_set_ = true;
  return true;
}

void SensorFusion::fusionThread(void)
{
  // TODO(zyc): Add time limit obs_available_->wait(lock) and report this error
  // TODO(zyc): See if we need to process prediction in higher priority

  while (fusion_thread_active_.load())
  {
    // std::cout << std::endl;
    // Read new observations into local queue
    double update_start_time = std::numeric_limits<double>::infinity();  // Include this time stamp
    double trim_stop_time = 0;                                           // Not include this time stamp
    {
      size_t observation_size = observation_list_.size();
      for (auto& obs_queue : observation_queues_)
      {
        Observation obs;
        while (obs_queue.second.getObservation(obs))
        {
          if ((obs.timestamp < fusion_states_.begin()->first))
          {
            WARNING << "fusionThread : The observation is eariler than the first state, it will be discarded" << RESET
                    << END;
            continue;
          }
          // If a observation is 1 secs before the latest we think it is too old
          if ((obs.timestamp < fusion_states_.rbegin()->first - 1.0))
          {
            WARNING << "fusionThread : [" << obs.obs_source << "] The observation delayed, it will be discarded"
                    << RESET << END;
            continue;
          }

          // Make sure if two measurement at same time, prediction comes first
          const double init_obs_time = obs.timestamp;
          const bool is_prediction = (obs.obs_source == prediction_observer_name_);

          while (observation_list_.find(obs.timestamp) != observation_list_.end())
          {
            if (is_prediction)
            {
              if (observation_list_[obs.timestamp].obs_source == prediction_observer_name_)
              {
                // TODO : Handle this error
                ERROR << name() << " There could not be two prediction having the same timestamp" << RESET << END;
                exit(-1);
              }
              else
              {
                Observation temp_obs = std::move(observation_list_[obs.timestamp]);
                observation_list_[obs.timestamp] = std::move(obs);
                obs = std::move(temp_obs);
                obs.timestamp += 0.000001;
              }
            }
            else
            {
              obs.timestamp += 0.000001;
            }
          }
          observation_list_[obs.timestamp] = std::move(obs);

          double oldest_change_time = (is_prediction) ? init_obs_time : obs.timestamp;
          update_start_time = (oldest_change_time < update_start_time) ? oldest_change_time : update_start_time;
        }
        // std::cout << std::fixed << "obs_queue: " << obs_queue.second.name() << std::endl;
      }

      // if observation size not changed then no new data -> wait
      if (observation_size == observation_list_.size())
      {
        // std::cout << "No observations found start waiting" << std::endl;
        // TODO: Temp workaround try to find better method
        std::unique_lock<std::mutex> lock(observations_mutex_);
        // All observation queue empty so wait
        obs_available_->wait(lock);
        // std::cout << "\n\rNew observation comming start processing" << std::endl;
        continue;
      }

      // keep all observation and state for 5 secs
      trim_stop_time = observation_list_.rbegin()->second.timestamp - 5.0;
    }

    // std::cout << "Processing observation queue size = " << observation_list_.size() << std::endl;
    // std::cout << "update_start_time = " << update_start_time << " trim_stop_time = " << trim_stop_time << std::endl;
    // std::cout << "Processing fusion_states_ before deleting queue size = " << fusion_states_.size() << std::endl;

    std::unique_lock<std::mutex> lock(fusion_states_mutex_);
    std::unique_lock<std::mutex> lock_lidar(lidar_states_mutex_);
    // Discard all states at or after update_start_time, as they're about to recalculate soon.
    {
      auto state_itr = fusion_states_.lower_bound(update_start_time);
      if (state_itr != fusion_states_.end())
      {
        // std::cout << "deleting all states at and after time = " << state_itr->first << std::endl;
        fusion_states_.erase(state_itr, fusion_states_.end());
      }
      else
      {
        // std::cout << "not deleting any states" << std::endl;
      }
    }
    // std::cout << "Processing fusion_states_ after deleting queue size = " << fusion_states_.size() << std::endl;

    // find the first observation for update
    Observations::iterator update_itr = observation_list_.find(update_start_time);
    if (update_itr == observation_list_.end())
    {
      // TODO : Handle this error
      ERROR << name() << " update_itr not found but it should be always found" << RESET << END;
      exit(-1);
    }

    // Perform updates using the observations in the list
    for (; update_itr != observation_list_.end(); ++update_itr)
    {
      MsfState new_state;

      // Perform extrinsic correction
      auto closest_state_itr = fusion_states_.upper_bound(update_itr->first);
      closest_state_itr--;
      Observation& obs = update_itr->second;

      // Perform prediction or correction
      if (obs.obs_source == prediction_observer_name_)
      {
        new_state = getPredictionState(obs.timestamp, obs);
        // std::cout << "Fusion sequence started by a prediction performed for time = " << obs.timestamp << std::endl;
      }
      else
      {
        new_state = getCorrectionState(obs.timestamp, obs);
        // std::cout << "Fusion sequence started by a correction performed for time = " << obs.timestamp << std::endl;
      }
      // std::cout << "new state vec = \n\r" << new_state.state_vec << std::endl;
      if (obs.obs_source == "LidarRegistraObserver")
      {
        lidar_states_[new_state.timestamp] = new_state;
      }

      auto t = new_state.timestamp;
      fusion_states_[new_state.timestamp] = std::move(new_state);

      // auto state_itr = fusion_states_.upper_bound(t);
      // // state_itr--;
      // if(state_itr == fusion_states_.begin())
      //   std::cout << "hit begin" << std::endl;
      // else if(state_itr == fusion_states_.end())
      //   std::cout << "hit end" << std::endl;
      // std::cout << "first: " << state_itr->first << std::endl;
      // std::cout << "second: " << state_itr->second.timestamp << std::endl;
    }

    // Remove data too old
    {  // trimFusionStates
      if (trim_stop_time > fusion_states_.begin()->first)
      {
        auto itr = fusion_states_.upper_bound(trim_stop_time);
        --itr;
        // std::cout << "Trim fusion states deleting states before time = " << itr->first;
        fusion_states_.erase(fusion_states_.begin(), itr);
        // std::cout << " Remaining size = " << fusion_states_.size() << std::endl;
      }
      else
      {
        // std::cout << "Trim fusion states not deleting any states" << std::endl;
      }

      if (!lidar_states_.empty() && trim_stop_time > lidar_states_.begin()->first)
      {
        auto itr = lidar_states_.upper_bound(trim_stop_time);
        --itr;
        // std::cout << "Trim fusion states deleting states before time = " << itr->first;
        lidar_states_.erase(lidar_states_.begin(), itr);
        // std::cout << " Remaining size = " << fusion_states_.size() << std::endl;
      }
    }

    lock.unlock();
    lock_lidar.unlock();
    {  // trimObservations
      if (trim_stop_time > observation_list_.begin()->first)
      {
        auto itr = observation_list_.upper_bound(trim_stop_time);
        --itr;
        // std::cout << "Trim Observations deleting states before time = " << itr->first;
        observation_list_.erase(observation_list_.begin(), itr);
        // std::cout << " Remaining size = " << observation_list_.size() << std::endl;
      }
      else
      {
        // std::cout << "Trim Observations not deleting any states" << std::endl;
      }
    }
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense
