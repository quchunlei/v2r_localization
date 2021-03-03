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

#include "rs_localization/rs_localization_impl.h"

namespace robosense
{
namespace localization
{
RSLocalizationImpl::RSLocalizationImpl(const std::string& map_file, const std::string& default_param_file,
                                       const std::string& param_file)
{
  try
  {
    // INFO << "Parsing config file" << REND;
    YamlParser catenator("robosense@XiLiSZ");
    param_ = catenator.YamlParserMerge(default_param_file, param_file);
    // INFO << "Finish parsing config file" << REND;
  }
  catch (YAML::ParserException& e)
  {
    ERROR << name() << " Can't Read Param file" << RESET << END;
    exit(-1);
  }

  /* use map configured in yaml file directly */

  param_["MapServer"]["map_file"] = map_file;  // when use as lib rewrite the parameter
  state_ = Localization_State_Suspend;
  started_ = true;
  const auto& func = [this] { mainThread(); };
  main_thread_ = std::thread(func);
}

RSLocalizationImpl::~RSLocalizationImpl()
{
  stop();
}

bool RSLocalizationImpl::start(void)
{
  started_ = true;
  return true;
}

bool RSLocalizationImpl::stop(void)
{
  started_ = false;
  init_data_receive_.notify_all();
  if (main_thread_.joinable())
    main_thread_.join();
  return true;
}

void RSLocalizationImpl::loadParams(YAML::Node& param)
{
  YAML::Node local_param = param["General"];
  rtk_type_ = 0;
  // common::yamlRead(local_param, "rtk_type", rtk_type_, 0);
}

void RSLocalizationImpl::config(void)
{
  /*Load parameter*/
  loadParams(param_);

  // auto t1 = ros::WallTime::now();
  /*Setup MapServer*/
  INFO << "Map loading..." << REND;
  map_server_ = std::make_shared<MapServer>(param_["MapServer"]);
  // INFO << "[>> " << name() << " ] ~~ The background map was loaded successfully!" << RESET << END;
  // auto t2 = ros::WallTime::now();
  // std::cout << "map loading total process time     : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;
  /* Setup GNSSMsgProcessor */
  gps_process_ = std::make_shared<GpsMsgProcess>();
  if (!map_server_->getGnssOrigin(gnss_ori_))
  {
    ERROR << "[>> " << name() << " ] gnss Origin not set in map file!" << RESET << END;
    exit(-1);
  }
  gps_process_->setGpsOrigin(gnss_ori_);

  /* Setup SensorFusion and running */
  sensor_fusion_ = std::make_shared<msf::SensorFusion>(param_["RSLocalization"]);

  // TODO: try to get rid of "init" param
  param_["LidarRegitraInit"]["init"] = true;
  param_["RSLocalization"]["init"] = false;

  /* Setup initializaer */
  icp_initialization_ = make_unique<ICPLocalization>(map_server_, param_["LidarRegitraInit"]);

  /*  Setup Observers and run */
  if (!param_["RSLocalization"]["Observers"])
  {
    ERROR << name() << "No Observer defined " << RESET << END;
    exit(-1);
  }

  for (auto observer_param : param_["RSLocalization"]["Observers"])
  {
    if (!observer_param["name"])
    {
      ERROR << name() << "Observers definition param has no name field " << RESET << END;
      exit(-1);
    }

    if (observer_param["name"].as<std::string>() == "LidarEsObserver")
    {
      YAML::Node local_param = observer_param;
      // TODO: delete the following if statement?
      if (param_["RSLocalization"]["init"])
      {
        // ERROR << "IcpLocalization node set to init, weird" << REND;
        local_param["registration_process"]["init"] = param_["RSLocalization"]["init"];
      }
      observers_.emplace("LidarEsObserver", std::unique_ptr<msf::Observer>(new msf::LidarEsObserver(
                                                "LidarEsObserver", observer_param, sensor_fusion_, map_server_)));
    }
    else
    {
      ERROR << name() << "Invalid Observer Name : " << observer_param["name"].as<std::string>() << RESET << END;
      exit(-1);
    }
  }
}

void RSLocalizationImpl::reset(const msf::MsfState& state)
{
  // TODO: Reset MSF to this initial state
  sensor_fusion_->stop();
  sensor_fusion_->start();
  sensor_fusion_->reset(state);
  // INFO << "\nMSF reseted\n " << REND;
}

void RSLocalizationImpl::addGnssData(const msf::GnssData& data)
{
  if (state_ == Localization_State_Wait_Sensor)
  {
    Eigen::Vector3d position;
    gps_process_->gps2xyz(data.longitude, data.latitude, data.altitude, position);
    init_pose_.block<3, 1>(0, 3) << position.x(), position.y(), position.z();
    Eigen::Quaternionf init_quat(data.angle.data());

    init_pose_.block<3, 3>(0, 0) = init_quat.toRotationMatrix();
    init_lvel_ = data.linear_vel;
    init_timestamp_ = data.timestamp;
    init_seq_ = data.seq;

    init_gnss_received_ = true;
    init_data_receive_.notify_one();
  }
  else if (state_ == Localization_State_Fixed)
  {
    for (auto& observer : observers_)
    {
      observer.second->addGnssData(data);
    }
  }
}

void RSLocalizationImpl::addLidarData(const msf::LidarData& data)
{
  if (state_ == Localization_State_Wait_Sensor)
  {
    init_lidar_ = data;
    init_lidar_received_ = true;
    init_data_receive_.notify_one();
  }
  else if (state_ == Localization_State_Fixed)
  {
    for (auto& observer : observers_)
    {
      observer.second->addLidarData(data);
    }
  }
}

void RSLocalizationImpl::addInitialGuess(const Eigen::Matrix4d& init_guess)
{
  init_pose_ = init_guess.cast<float>();
  init_lvel_ << 0, 0, 0;
  init_timestamp_ = 0;
  if (state_ == Localization_State_Wait_Sensor)
  {
    init_pose_ = init_guess.cast<float>();
    init_lvel_ << 0, 0, 0;
    init_timestamp_ = 0;
    init_guess_received_ = true;
    init_data_receive_.notify_one();
  }
  else if (state_ == Localization_State_Fixed)
  {
    /* TODO: receive manual-set init_pose again, need to reset msf here */

    state_ = Localization_State_Init;
  }
}

bool RSLocalizationImpl::started(void)
{
  if (started_.load())
    return true;
  else
    return false;
}

bool RSLocalizationImpl::getEstimatedState(msf::MsfState& state)
{
  return sensor_fusion_->getEstimatedState(state);
}

bool RSLocalizationImpl::getEstimatedState(msf::MsfState& state, const double& t)
{
  return sensor_fusion_->getEstimatedState(state, t);
}

bool RSLocalizationImpl::initPose(msf::MsfState& init_state)
{
  /* Perform lidar measurement and update init_pose_ */
  INFO << "Running Initialization" << REND;
  float lidar_height;
  if (!map_server_->getHeight(init_pose_(0, 3), init_pose_(1, 3), 0, lidar_height))
  {
    WARNING << name() << '\t' << "Can't get height of car, using default value" << RESET << END;
    lidar_height = 1.0f;
  }

  init_pose_(2, 3) = lidar_height;

  Eigen::Matrix4f measure_pose = Eigen::Matrix4f::Identity();
  if (icp_initialization_->locate(init_lidar_.scan, init_pose_, measure_pose))
  {
    init_state.seq = init_seq_;

    init_timestamp_ = ros::Time::now().toSec();
    init_state.timestamp = init_timestamp_;
    // Eigen::Quaternionf att(init_pose_.block<3, 3>(0, 0));
    Eigen::Quaternionf att(measure_pose.block<3, 3>(0, 0));
    msf::EkfStateVec& state_vec = init_state.state_vec;

    state_vec = msf::EkfStateVec::Zero();
    init_state.state_vec.segment<3>(msf::StatePositionIdx) = measure_pose.block<3, 1>(0, 3);
    init_state.state_vec.segment<4>(msf::StateAngleIdx) << att.coeffs();

    init_state.es_cov.block<3, 3>(0, 0) = 1e-3 * Eigen::Matrix3f::Identity();                // init cov of p
    init_state.es_cov.block<3, 3>(3, 3) = 10000 * Eigen::Matrix3f::Identity();               // init cov of v
    init_state.es_cov.block<3, 3>(6, 6) = Eigen::Vector3f(1e-10, 1e-10, 1e-6).asDiagonal();  // init cov of theta

    init_state.es_cov.block<3, 3>(9, 9) = 10000 * Eigen::Matrix3f::Identity();    // init cov of acc bias
    init_state.es_cov.block<3, 3>(12, 12) = 10000 * Eigen::Matrix3f::Identity();  // init cov of omega bias
    init_state.es_cov.block<3, 3>(15, 15) = 10000 * Eigen::Matrix3f::Identity();  // init cov of gravity
    INFO << "Finished" << REND;
    return true;
  }
  else
  {
    WARNING << "Initializtion Failed, retrying..." << REND;
    return false;
  }
}

bool RSLocalizationImpl::startObservers()
{
  try
  {
    for (auto const& observer : observers_)
    {
      observer.second->start();
    }
    return true;
  }
  catch (std::exception& e)
  {
    return false;
  }
}

void RSLocalizationImpl::mainThread(void)
{
  while (started_.load())
  {
    if (state_ == Localization_State_Suspend)
    {
      config();
      state_ = Localization_State_Lost;
      INFO << "\n\n[>> " << name() << " ]  Ready Go!!!" << RESET << END;
      INFO << "[>> " << name() << " ] .................... \n\n" << RESET << END;
    }
    else if (state_ == Localization_State_Lost)
    {
      init_pose_.setIdentity();
      init_lidar_received_ = false;
      init_gnss_received_ = false;
      init_imu_received_ = false;
      init_odom_received_ = false;
      printInitDataStatus();
      state_ = Localization_State_Wait_Sensor;
    }
    else if (state_ == Localization_State_Wait_Sensor)
    {
      std::unique_lock<std::mutex> lk(init_data_mx_);
      init_data_receive_.wait(lk);
      printInitDataStatus();  // stop at here
      if (init_lidar_received_ && (init_guess_received_ || init_gnss_received_))
      {
        state_ = Localization_State_Init;
        resetInitPrinter();
      }
    }
    else if (state_ == Localization_State_Init)
    {
      msf::MsfState init_state;
      if (initPose(init_state))
      {
        param_["RSLocalization"]["init"] = false;
        reset(init_state);
        state_ = Localization_State_Fixed;
        startObservers();  // use for localization after initialization
      }
      else
      {
        state_ = Localization_State_Lost;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // usleep(10000);
  }
}

void RSLocalizationImpl::printInitDataStatus(void)
{
  std::function<std::string(bool)> getStatus = [](bool a) { return a ? "Ready!" : "Waiting..."; };

  if (!is_init_printer_first_)
  {
    for (int i = 0; i < 5; i++)
    {
      std::cout << MOVE_UP(1);
      std::cout << DELETE_LINE;
    }
  }

  INFO << "------------------------------" << REND;
  INFO << " Init Sensor Data |  Status    " << REND;
  INFO << "------------------------------" << REND;
  INFO << "     RS-LiDAR     |  " << getStatus(init_lidar_received_.load()) << REND;
  INFO << "------------------------------" << REND;
  is_init_printer_first_ = false;
}

}  // namespace localization
}  // namespace robosense
