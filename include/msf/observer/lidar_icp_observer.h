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

#ifndef LIDAR_ICP_OBSERVER_H
#define LIDAR_ICP_OBSERVER_H

#include "msf/observer/observer_base.h"
#include "map_server/map_server.h"
#include "icp_localization/icp_localization.h"

namespace robosense
{
namespace localization
{
namespace msf
{
class LidarIcpObserver : public Observer
{
public:
  LidarIcpObserver() = delete;
  LidarIcpObserver(const std::string name, const YAML::Node& param, const std::shared_ptr<SensorFusion> sensor_fusion,
                   const std::shared_ptr<MapServer> map_server);
  ~LidarIcpObserver();
  void addLidarData(const LidarData& data);
  bool ready(void);
  virtual void start(void);
  virtual void stop(void);

  // void correctObs(const MsfState&, Observations& obs);
  bool applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state);

protected:
  virtual void observerThread(void);
  virtual void config(const YAML::Node& param);

  bool updateSubMap(const LidarData* data);
  bool getObservation(const LidarData* data, Observation& obs);
  bool isOutlier(const LidarData* const data) const;

  std::queue<LidarData> data_queue_;
  std::mutex data_queue_mutex_;
  std::condition_variable data_available_;
  std::atomic<bool> obs_ready_;

  Eigen::Matrix4f sensor_transform_;

  std::shared_ptr<MapServer> map_server_;
  std::unique_ptr<ICPLocalization> icp_localization_;

private:
  PM::ICP icp_;
  PM::DataPointsFilters filter_;
  std::shared_ptr<PM::Transformation> rigid_trans_;
  int num_scan_per_local_map_;
  std::unique_ptr<LidarData> local_map_;
  std::unique_ptr<LidarData> prev_local_map_;
  Eigen::Matrix4f local_map_pose_;

  double lidar_watchdog_thresh_;
  float heading_pos_noise_;
  float side_pos_noise_;
  float angle_noise_;
};

}  // namespace msf
}  // namespace localization
}  // namespace robosense

#endif /*LIDAR_ICP_OBSERVER_H*/