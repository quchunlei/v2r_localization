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

#ifndef RS_LOCALIZATION_IMPL_H
#define RS_LOCALIZATION_IMPL_H

#include <string>
#include <vector>
#include <iostream>
#include <exception>

#include "icp_localization/icp_localization.h"
#include "map_server/map_server.h"
#include "visualization/rviz_display.h"
#include "common/common.h"
#include "common/prompt.h"
#include "msf/core/sensor_fusion.h"
#include "msf/observer/imu_observer.h"
#include "msf/observer/imu_odom_observer.h"
#include "msf/observer/odom_observer.h"
#include "msf/observer/odom_observer2.h"
#include "msf/observer/gnss_observer.h"
#include "msf/observer/lidar_icp_observer.h"
#include "msf/observer/lidar_es_observer.h"
#include "sensor/gps_msg_process.h"
#include "yaml/yaml.h"
#include "yaml/yaml_parser.h"

namespace robosense
{
namespace localization
{
class RSLocalizationImpl
{
public:
  RSLocalizationImpl() = delete;
  RSLocalizationImpl(const std::string& map_file, const std::string& default_param_file, const std::string& param_file);
  ~RSLocalizationImpl();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void reset(const msf::MsfState& state);

  void addGnssData(const msf::GnssData& data);
  void addLidarData(const msf::LidarData& data);
  void addInitialGuess(const Eigen::Matrix4d& init_pose);
  bool start(void);
  bool stop(void);
  bool started(void);
  // get latest state updated by lidar
  bool getEstimatedState(msf::MsfState& state);                   // Latest
  bool getEstimatedState(msf::MsfState& state, const double& t);  // State at time

  Eigen::Vector3d getGpsOrigin(void)
  {
    return gnss_ori_;
  }
  DP getMap(void)
  {
    return map_server_->getBackgroundMap();
  }
  int getStatus(void)
  {
    return state_.load();
  }
  YAML::Node& getParam()
  {
    return param_;
  }

  template<typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args&&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }
private:
  enum State
  {
    Localization_State_Suspend = 0,
    Localization_State_Lost = 1,
    Localization_State_Wait_Sensor = 2,
    Localization_State_Init = 3,
    Localization_State_Fixed = 4
  };

  enum StatusSwitching
  {
    Finish_Loading_Map = 0,
    Start_Initialization,
    Finish_Initialization
  };

  int rtk_type_ = 0;
  // Modules inside RSLocalizationImpl
  std::map<std::string, std::unique_ptr<msf::Observer>> observers_;
  std::shared_ptr<MapServer> map_server_;
  std::shared_ptr<msf::SensorFusion> sensor_fusion_;
  std::shared_ptr<GpsMsgProcess> gps_process_;
  std::unique_ptr<ICPLocalization> icp_initialization_;

  /* Multi-thread related variables */
  std::thread main_thread_;
  bool is_init_printer_first_ = false;  // default member initializer (equals-initializer)

  std::atomic<int> state_{ Localization_State_Suspend };  // default member initializer (brace-initializer)
  std::atomic<bool> started_{ false };
  std::atomic<bool> init_lidar_received_{ false };
  std::atomic<bool> init_gnss_received_{ false };
  std::atomic<bool> init_imu_received_{ false };
  std::atomic<bool> init_odom_received_{ false };
  std::atomic<bool> init_guess_received_{ false };

  std::mutex init_data_mx_;
  std::mutex init_imu_mx_;
  std::condition_variable imu_receive_;
  std::condition_variable init_data_receive_;

  // variables for initialization
  Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Zero();
  msf::LidarData init_lidar_;
  double init_timestamp_ = 0.0f;
  int init_seq_ = 0;
  bool is_acc_with_G_ = true;
  Eigen::Vector3f init_lvel_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f init_acc_ = { 0, 0, 0 };
  Eigen::Vector3f init_avel_ = { 0, 0, 0 };
  Eigen::Vector3d gnss_ori_ = { 0, 0, 0 };

  YAML::Node param_;

  void mainThread(void);
  void config(void);

  inline void resetInitPrinter(void)
  {
    is_init_printer_first_ = true;
  }
  inline const std::string name()
  {
    return "RSLocalization";
  }

  void printInitDataStatus(void);
  void loadParams(YAML::Node& param);
  bool initPose(msf::MsfState& init_state);
  bool startObservers();
};
}  // namespace localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_IMPL_H
