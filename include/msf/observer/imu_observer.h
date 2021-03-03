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
#ifndef RS_LOCALIZATION_IMU_OBSERVER_H
#define RS_LOCALIZATION_IMU_OBSERVER_H

#include "msf/observer/observer_base.h"
#include "common/eigen_util.h"
#include "common/logger.h"

namespace robosense
{
namespace localization
{
namespace msf
{
class ImuObserver : public Observer
{
public:
  ImuObserver() = delete;
  ImuObserver(const std::string name, const YAML::Node& param, const std::shared_ptr<SensorFusion> sensor_fusion);
  ~ImuObserver();
  void addImuData(const ImuData& imu);
  bool ready(void);
  virtual void start(void);
  virtual void stop(void);

  // inline const Eigen::Matrix<float, 12, 12>& getNoiseCov()
  // {
  //   return noise_cov_;
  // }

protected:
  virtual void observerThread(void);

  virtual void config(const YAML::Node& param);

  Observation getObservation(ImuData& imu);
  bool predictStateVec2D(const Observation& obs, const MsfState& state, MsfState& new_state);
  bool predictStateVec3D(const Observation& obs, const MsfState& state, MsfState& new_state);
  bool predictStateCov2D(const Observation& obs, const MsfState& state, MsfState& new_state);
  bool predictStateCov3D(const Observation& obs, const MsfState& state, MsfState& new_state);
  EsJacob getEsJacob(const MsfState& state, const Observation&);

  bool isOutlier(const ImuData& imu);
  Eigen::Vector3f transformAcc2D(void);

  std::function<bool(const Observation&, const MsfState&, MsfState&)> predictState;
  std::function<bool(const Observation&, const MsfState&, MsfState&)> predictCov;

  double prev_imu_time_;
  ImuData prev_imu_data_;

  std::queue<ImuData> imu_queue_;
  std::mutex imu_queue_mutex_;
  std::condition_variable imu_available_;
  std::atomic<bool> imu_ready_;

  Eigen::Matrix4f imu_transform_;

  // IMU sensor noise, determined from IMU datasheet or from experimental measurements.
  float sigma_acc2_;       // variance of acc: m^2 / s^4
  float sigma_w2_;         // variance of angular rate: rad^2 / s^2
  float sigma_acc_bias2_;  // variance of acc random walk: m^2/(s^5)
  float sigma_w_bias2_;    // variance of angular rate random walk: rad^2 / s^3

  Eigen::Matrix<float, ErrorStateSize, 12> Fi_;
  Eigen::Matrix<float, 6, 6> control_noise_;  // white gaussian noise of acc and gyro
  Eigen::Matrix<float, 6, 6> perturb_noise_;  // random walk of acc and gyro

  bool IS_3DOF_;
  Logger logger_;
};

}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_IMU_OBSERVER_H */