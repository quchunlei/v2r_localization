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
#ifndef RS_LOCALIZATION_ODOM_OBSERVER2_H
#define RS_LOCALIZATION_ODOM_OBSERVER2_H

#include "msf/observer/observer_base.h"
#include "common/eigen_util.h"

namespace robosense
{
namespace localization
{
namespace msf
{
class OdomObserver2 : public Observer
{
public:
  OdomObserver2() = delete;
  OdomObserver2(const std::string name, const YAML::Node& param, const std::shared_ptr<SensorFusion> sensor_fusion);
  ~OdomObserver2();
  void addOdomData(const OdomData& odom);
  bool ready(void);
  virtual void start(void);
  virtual void stop(void);
  void correctObs(const MsfState& state, Observation& obs);
  bool applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state);

protected:
  static const int MEAS_SIZE = 1;
  typedef Eigen::Matrix<float, MEAS_SIZE, MEAS_SIZE> MatR;
  typedef Eigen::Matrix<float, MEAS_SIZE, 1> Measurement;

  virtual void config(const YAML::Node& param);
  virtual void observerThread(void);

  Observation getObservation(const OdomData& Odom);
  bool isOutlier(const OdomData& odom);

  // Now only consider last two odom (others are discarded)
  // TODO: Improve method
  double prev_odom_time_;
  double odom_offset_;
  double vel_noise_;
  OdomData prev_odom_data_;

  bool use_global_vel_;
  std::queue<OdomData> odom_queue_;
  std::mutex odom_queue_mutex_;
  std::condition_variable odom_available_;
  std::atomic<bool> odom_ready_;

  Eigen::Matrix4f odom_transform_;
  // float odom_offset_;  // odom_x - baselink_x, if odom from front wheel then positive
};
}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_ODOM_OBSERVER2_H */
