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
#ifndef RS_LOCALIZATION_Gnss_OBSERVER_H
#define RS_LOCALIZATION_Gnss_OBSERVER_H

#include <sensor_msgs/NavSatFix.h>
#include "msf/observer/observer_base.h"
#include "sensor/gps_msg_process.h"
#include "map_server/map_server.h"

namespace robosense
{
namespace localization
{
namespace msf
{
class GnssObserver : public Observer
{
public:
  GnssObserver() = delete;
  GnssObserver(const std::string name, const YAML::Node& param, const std::shared_ptr<SensorFusion> sensor_fusion,
               const std::shared_ptr<GpsMsgProcess> gnss_process, const std::shared_ptr<MapServer>);
  ~GnssObserver();
  void addGnssData(const GnssData& data);
  void correctObs(const MsfState& state, Observation& obs);
  bool ready(void);
  virtual void start(void);
  virtual void stop(void);
  bool applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state);
  bool applyEkfCorrect2(const Observation& obs, const MsfState& state, MsfState& new_state);

protected:
  static const int MEAS_SIZE = 6;  // dx,dy,dz,dtheta(3)
  typedef Eigen::Matrix<float, MEAS_SIZE, MEAS_SIZE> MatR;
  typedef Eigen::Matrix<float, MEAS_SIZE, 1> Measurement;

  virtual void observerThread(void);
  virtual void config(const YAML::Node& param);
  Observation getObservation(const GnssData& data);
  bool isOutlier(const GnssData& data);

  double gnss_watchdog_thresh_;
  bool is_discard_unqualified_;
  float pose_noise_;
  float vel_noise_;
  float angle_noise_;
  std::queue<GnssData> data_queue_;
  std::mutex data_queue_mutex_;
  std::condition_variable data_available_;
  std::atomic<bool> obs_ready_;

  std::shared_ptr<GpsMsgProcess> gnss_process_;
  std::shared_ptr<MapServer> map_server_ptr_;

  Eigen::Matrix4f sensor_transform_;
};
}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /* RS_LOCALIZATION_IMU_OBSERVER_H */
