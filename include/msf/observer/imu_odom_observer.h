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
#ifndef RS_LOCALIZATION_IMU_ODOM_OBSERVER_H
#define RS_LOCALIZATION_IMU_ODOM_OBSERVER_H

#include "msf/observer/observer_base.h"
#include "msf/core/sensor_data.h"
#include <chrono>

namespace robosense
{
namespace localization
{
namespace msf
{
// For now the observer is sync with Imudata.
// The odom velocity is linear interpolated to the timestamp of imudata
class ImuOdomObserver : public Observer
{
public:
  ImuOdomObserver() = delete;
  ImuOdomObserver(const std::string name, const YAML::Node& param, const std::shared_ptr<SensorFusion> sensor_fusion);
  ~ImuOdomObserver();
  void addImuData(const ImuData& imu);
  void addOdomData(const OdomData& odom);
  bool ready(void);
  void correctObs(const MsfState& state, Observation& obs);
  virtual void start(void);
  virtual void stop(void);

  bool predicStateVec(const Observation& obs, const MsfState& state, MsfState& new_state);
  bool predicStateCov(const Observation& obs, const MsfState& state, MsfState& new_state);

protected:
  std::atomic<int> imu_status_;
  std::atomic<int> prev_imu_status_;
  std::atomic<int> odom_status_;
  std::atomic<int> prev_odom_status_;

  // Observation getObservation(ImuData& Imu, OdomData& Odom);
  Observation getObservation(ImuData& imu_data);
  virtual void observerThread(void);
  void timerThread(void);
  bool isOutlier(const ImuData& imu);
  bool isOutlier(const OdomData& odom);

  virtual void config(const YAML::Node& param);
  void printStatus(int prev_stat, int cur_stat, std::string data_name);

  std::thread timer_thread_;
  std::queue<ImuData> imu_queue_;
  std::mutex imu_queue_mutex_;
  std::condition_variable imu_available_;
  std::atomic<bool> imu_ready_;

  double imu_watchdog_thresh_;   // unit: ms
  double odom_watchdog_thresh_;  // unit: ms
  std::atomic<long> prev_odom_time_;

  // Now only consider last two odom (others are discarded)
  // TODO: Improve method
  std::queue<OdomData> odom_queue_;
  std::mutex odom_queue_mutex_;
  std::atomic<bool> odom_ready_;

  Eigen::Matrix4f imu_transform_;
  float v_noise_;  // per meter
  float w_noise_;  // per second
  float imu_period_;
  float odom_period_;
  float odom_bias_;
};
}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_IMU_ODOM_OBSERVER_H */
