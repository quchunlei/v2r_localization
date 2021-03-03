/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Localization Group  inline void resetInitPrinter(void)
  {
    is_init_printer_first_ = true;
  }
 * Version: 0.2.0
 * Date: 2018.05
 *
 * DESCRIPTION
 *
 * Robosense localization ROS package.
 *
 */

#include "rs_localization/rs_localization_ros.h"
#include <geometry_msgs/Point.h>

namespace robosense
{
namespace localization
{
RSLocalizationRos::RSLocalizationRos(const std::string& param_file, const std::string& map_file,
                                     const std::shared_ptr<ros::NodeHandle> nh)
  : nh_(nh), status_(0), force_quit_(false)
{
  std::string default_cfg_file = ros::package::getPath("lidar_pose_localization") + "/cfg/parameters.yaml";
  std::string param_file1 = ros::package::getPath("lidar_pose_localization") + "/user_config/user_config.yaml";
  localization_ = make_unique<RSLocalization>(map_file, default_cfg_file, param_file1);

  param_ = localization_->getParam();
  loadParams();
  configPublications();
  configSubscriptions();

  config();
}

RSLocalizationRos::~RSLocalizationRos()
{
  if (pub_thread_.joinable())
  {
    pub_thread_.join();
  }
  if (main_thread_.joinable())
  {
    main_thread_.join();
  }
}

bool RSLocalizationRos::loadParams(void)
{
  YAML::Node param = param_["General"];
  frame_id_ = "rs_odom";
  child_frame_id_ = "base_link";
  common::yamlRead(param, "sub_init_pose_topic", sub_init_pose_topic_, "/initialpose");
  common::yamlRead(param, "sub_point_cloud_topic", sub_point_cloud_topic_, "/rslidar_points");
  common::yamlRead(param, "pub_rate", pub_rate_, 20);
  common::yamlRead(param, "pub_pose_topic", pub_pose_topic_, "/rs_pose");
  common::yamlRead(param, "pub_point_cloud_topic", pub_point_cloud_topic_, "/rslidar_points_global");
  common::yamlRead(param, "pub_background_map_topic", pub_background_map_topic_, "/rs_map");
  pub_gnss_origin_topic_ = "/rs_gnss_origin";

  /*TODO: the following variables are useless in this branch, remove them */
  thread_number_ = 4;
  num_of_path_reserved_ = 10;
  rtk_type_ = 0;
  imu_ang_vel_ratio_ = 1;
  is_imu_reverse_ = false;
  is_use_unit_kmh_ = false;
  return true;
}

bool RSLocalizationRos::config(void)
{
  // Config Pubs and Subs
  configPublications();
  configSubscriptions();

  // Config Rviz Display
  rviz_display_ = make_unique<RvizDisplay>(param_["RvizDisplay"]);
  // rviz_display_->setGpsOrigin(localization_->getGpsOrigin());

  // Publish first TF
  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = frame_id_;
  tf.child_frame_id = child_frame_id_;
  tf.transform.rotation = tf::createQuaternionMsgFromYaw(0);
  tf_odom_broadcaster_.sendTransform(tf);

  // publishStatus(Finish_Loading_Map);
  return true;
}

void RSLocalizationRos::configSubscriptions(void)
{
  // TODO: Now is only a test version
  init_pose_sub_ = nh_->subscribe(sub_init_pose_topic_, 10, &RSLocalizationRos::initPoseCallback, this);
  lidar_sub_ = nh_->subscribe(sub_point_cloud_topic_, 10, &RSLocalizationRos::lidarCallback, this);
  // gnss_sub_ = nh_->subscribe(sub_gnss_topic_, 10, &RSLocalizationRos::gnssCallback, this);
  if (rtk_type_ == 1)
  {
    rtk_kinetic_sub_ = nh_->subscribe(sub_rkt_kinetic_topic_, 10, &RSLocalizationRos::rtkKineticCallback, this);
  }
}

void RSLocalizationRos::configPublications(void)
{
  // TODO: Now is only a test version
  gps_origin_pub_ = nh_->advertise<geometry_msgs::Point>(pub_gnss_origin_topic_, 1, true);
  background_map_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(pub_background_map_topic_, 1, true);
  odom_pub_ = nh_->advertise<nav_msgs::Odometry>(pub_pose_topic_, 1, true);
  point_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(pub_point_cloud_topic_, 1, true);
}

void RSLocalizationRos::run(void)
{
  const auto& func1 = [this] { pubToRosThread(); };
  const auto& func2 = [this] { mainThread(); };
  pub_thread_ = std::thread(func1);
  main_thread_ = std::thread(func2);

  ros::MultiThreadedSpinner s(0);  //多线程set to 0), it will use a thread for each CPU core.
  s.spin();
}

void RSLocalizationRos::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& init_pose)
{
  // clang-format off
  Eigen::Matrix4d init_matrix = Eigen::Matrix4d::Identity();
  init_matrix.block<3,1>(0,3) = Eigen::Vector3d(init_pose.pose.pose.position.x,
                                                init_pose.pose.pose.position.y,
                                                init_pose.pose.pose.position.z);
  
  init_matrix.block<3,3>(0,0) = Eigen::Quaterniond(init_pose.pose.pose.orientation.w,
                                                  init_pose.pose.pose.orientation.x,
                                                  init_pose.pose.pose.orientation.y,
                                                  init_pose.pose.pose.orientation.z).toRotationMatrix();
  // clang-format on
  localization_->addInitialGuess(init_matrix);
}

void RSLocalizationRos::lidarCallback(const sensor_msgs::PointCloud2& points_msg)
{
  // double cur_time = points_msg.header.stamp.toSec();
  double cur_time = ros::Time::now().toSec();
  static double prev_time = cur_time;

  msf::LidarData data;
  data.timestamp = cur_time;
  data.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(points_msg, true);
  localization_->addLidarData(data);

  sensor_msgs::PointCloud2 tmp_points = std::move(points_msg);
  tmp_points.header.frame_id = child_frame_id_;
  point_cloud_pub_.publish(tmp_points);

  prev_time = cur_time;
}

void RSLocalizationRos::gnssCallback(const sensor_msgs::NavSatFix& gnss_msg)
{
  msf::GnssData data;
  nav_msgs::Odometry rtk_data;

  static double prev_time = gnss_msg.header.stamp.toSec();
  double current_time = gnss_msg.header.stamp.toSec();
  if (current_time - prev_time < 0.1)
    return;
  prev_time = current_time;
  if (rtk_type_ == 1)
  {
    bool found = false;
    while (!found)
    {
      // wait for odom data if the queue is empty, and force_quit_ is set to
      // true in mainThread during destruction
      std::unique_lock<std::mutex> lk(rtk_kinematic_data_mutex_);
      while (rtk_kinematic_data_queue_.empty() && !force_quit_)
      {
        rtk_kinematic_data_arrived_.wait(lk);
      }
      if (force_quit_)
        return;
      // rtk_kinematic_data_arrived_.wait(lk, [this] { std::cout
      // <<rtk_kinematic_data_queue_.size() <<std::endl;  return
      // !rtk_kinematic_data_queue_.empty(); });
      nav_msgs::Odometry firt_rtk = rtk_kinematic_data_queue_.front();
      nav_msgs::Odometry latest_rtk = rtk_kinematic_data_queue_.back();
      auto dt1 = gnss_msg.header.stamp.toSec() - firt_rtk.header.stamp.toSec();
      auto dt2 = gnss_msg.header.stamp.toSec() - latest_rtk.header.stamp.toSec();
      if (dt1 < -0.001)
      {
        return;  // gnss timestamp is earlier than earliest odom data, return to
                 // get a new gnss data
      }
      else if (dt2 > 0.001)
      {
        // gnss timestamp is newer than latest odom data, release mutex to
        // acquire newer odom;
        std::queue<nav_msgs::Odometry> empty;
        rtk_kinematic_data_queue_.swap(empty);
        continue;
      }
      else
      {
        // traverse to find the closest odom data
        while (!found && !rtk_kinematic_data_queue_.empty())
        {
          rtk_data = rtk_kinematic_data_queue_.front();
          rtk_kinematic_data_queue_.pop();
          if (fabs(gnss_msg.header.stamp.toSec() - rtk_data.header.stamp.toSec()) < 0.001)
          {
            found = true;
            break;
          }
        }
      }
    }
    tf::Quaternion tf_quat;
    Eigen::Quaterniond eigen_quat;
    rtk_quat_ = rtk_data.pose.pose.orientation;  // only for display
    rtk_pose_ =
        Eigen::Vector3f(rtk_data.pose.pose.position.x, rtk_data.pose.pose.position.y, rtk_data.pose.pose.position.z);

    tf::quaternionMsgToTF(rtk_quat_, tf_quat);
    tf::quaternionTFToEigen(tf_quat, eigen_quat);
    if (eigen_quat.w() > 0)
      data.angle = eigen_quat.coeffs().cast<float>();
    else
      data.angle = -eigen_quat.coeffs().cast<float>();

    data.pos << rtk_pose_;
    data.linear_vel =
        Eigen::Vector3f(rtk_data.twist.twist.linear.x, rtk_data.twist.twist.linear.y, rtk_data.twist.twist.linear.z);
  }

  data.seq = gnss_msg.header.seq;
  data.timestamp = gnss_msg.header.stamp.toSec();
  data.latitude = gnss_msg.latitude;
  data.longitude = gnss_msg.longitude;
  data.altitude = gnss_msg.altitude;
  // data.pos_cov_ = gnss_msg.position_covariance;
  data.gnss_status = static_cast<msf::GnssData::GnssStatus>(gnss_msg.status.status);
  data.gnss_service = static_cast<msf::GnssData::GnssService>(gnss_msg.status.service);
  localization_->addGnssData(data);
}

void RSLocalizationRos::rtkKineticCallback(const nav_msgs::Odometry& rtk_kinematic_msg)
{
  std::unique_lock<std::mutex> lk(rtk_kinematic_data_mutex_);
  rtk_kinematic_data_queue_.emplace(std::move(rtk_kinematic_msg));
  rtk_kinematic_data_arrived_.notify_all();
}

void RSLocalizationRos::publishState(const msf::MsfState& state)
{
  Eigen::Vector3f position = state.state_vec.block<3, 1>(msf::StatePositionIdx, 0);
  Eigen::Vector3f velocity = state.state_vec.block<3, 1>(msf::StateLinearVelIdx, 0);

  // map_server_->getHeight(position(0), position(1), 0, position(2));
  ros::Time stamp;
  stamp.fromSec(state.timestamp);
  // TODO: Publish State here

  geometry_msgs::Quaternion quat;
  quat.w = state.state_vec(msf::StateAngleIdx + 3);
  quat.x = state.state_vec(msf::StateAngleIdx);
  quat.y = state.state_vec(msf::StateAngleIdx + 1);
  quat.z = state.state_vec(msf::StateAngleIdx + 2);

  /********************* Pub TF **********************************/
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = frame_id_;
  tf.child_frame_id = child_frame_id_;
  tf.transform.translation.x = position[0];
  tf.transform.translation.y = position[1];
  tf.transform.translation.z = position[2];
  tf.transform.rotation = quat;
  tf_odom_broadcaster_.sendTransform(tf);

  /********************* Pub Odom **********************************/
  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.seq = state.seq;
  odom.header.frame_id = frame_id_;
  odom.pose.pose.position.x = position[0];
  odom.pose.pose.position.y = position[1];
  odom.pose.pose.position.z = position[2];
  odom.pose.pose.orientation = quat;
  odom.twist.twist.linear.x = velocity[0];
  odom.twist.twist.linear.y = velocity[1];
  odom.twist.twist.linear.z = velocity[2];

  odom.twist.twist.angular.z = state.state_vec[msf::StateAngleVelIdx + 2];
  odom_pub_.publish(odom);

  // INFO << "Estimate Vel: " << velocity.norm() * 3.6 << REND;
}

void RSLocalizationRos::publishStatus(const int stat_switching)
{
  std_msgs::Int32 status;
  status.data = stat_switching;
  // status_pub_.publish(status);
}

void RSLocalizationRos::pubToRosThread(void)
{
  const std::chrono::nanoseconds total_duration(static_cast<int>(1.0 / pub_rate_ * 1000000000.0));
  while (ros::ok())
  {
    status_ = localization_->getStatus();
    auto start_time = std::chrono::system_clock::now();
    if (status_ == Localization_State_Fixed)
    {
      msf::MsfState cur_state;
      if (localization_->getEstimatedState(cur_state))
      {
        static double prev_time = -1.0;
        if (prev_time != cur_state.timestamp)
          publishState(cur_state);
        prev_time = cur_state.timestamp;
      }
    }
    auto end_time = std::chrono::system_clock::now();
    std::chrono::nanoseconds exec_duration = end_time - start_time;
    if (total_duration > exec_duration)
    {
      std::this_thread::sleep_for(total_duration - exec_duration);
    }
  }
}

void RSLocalizationRos::mainThread(void)
{
  static bool map_published = false;
  while (ros::ok())
  {
    // int status = localization_->getStatus();
    publishStatus(status_);
    if (status_ != Localization_State_Suspend && !map_published)
    {
      /* get gps origin */
      auto gps_origin = localization_->getGpsOrigin();
      geometry_msgs::Point origin;
      origin.x = gps_origin(0); /* latitude */
      origin.y = gps_origin(1); /* longtitude */
      origin.z = gps_origin(2); /* altitude */
      gps_origin_pub_.publish(origin);
      // publish background map
      auto map = localization_->getMap();
      rviz_display_->setGpsOrigin(gps_origin);
      rviz_display_->publishMap(background_map_pub_, map, frame_id_, ros::Time::now());
      std::cout << "Publish background map: " << std::endl;
      map_published = true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // force_quit_ set to true to force exit the while loop around
  // condition_variable in gnssCallback
  // to ensure return of ros::spin() without delay;
  // But this solution is ugly. Installing a user-defined signal handler would
  // be better
  force_quit_ = true;
  rtk_kinematic_data_arrived_.notify_all();
}

}  // namespace localization
}  // namespace robosense
