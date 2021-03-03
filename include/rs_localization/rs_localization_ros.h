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

#ifndef RS_LOCALIZATION_ROS_H
#define RS_LOCALIZATION_ROS_H

#include <queue>
#include <thread>
#include <string>
#include <vector>
#include <iostream>
#include <exception>
#include <ros/package.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <mutex>

#include "rs_localization/rs_localization.h"
#include "visualization/rviz_display.h"

namespace robosense
{
namespace localization
{
class RSLocalizationRos
{
public:
  RSLocalizationRos() = delete;
  RSLocalizationRos(const std::string& param_file, const std::string& map_fle,
                    const std::shared_ptr<ros::NodeHandle> nh);
  ~RSLocalizationRos();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  void run(void);

  const std::string name() const
  {
    return "RSLocalizationRos";
  }

  template<typename T, typename... Args>
  std::unique_ptr<T> make_unique(Args&&... args)
  {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
  }

private:
  void mainThread(void);
  bool config(void);
  bool loadParams(void);
  void configPublications(void);
  void configSubscriptions(void);

  void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& init_pose);
  void lidarCallback(const sensor_msgs::PointCloud2& points_msg);
  void gnssCallback(const sensor_msgs::NavSatFix& gnss_msg);
  void rtkKineticCallback(const nav_msgs::Odometry& msg);

  void pubToRosThread(void);
  void publishState(const msf::MsfState& state);
  void publishDebug(void);
  void publishStatus(const int status);

private:
  enum State
  {
    Localization_State_Suspend = 0,
    Localization_State_Lost = 1,
    Localization_State_Wait_Sensor = 2,
    Localization_State_Init = 3,
    Localization_State_Fixed = 4
  };

  std::shared_ptr<ros::NodeHandle> nh_;
  std::thread pub_thread_;
  std::thread main_thread_;
  int rtk_type_;
  int status_;
  bool force_quit_;

  std::unique_ptr<RSLocalization> localization_;
  std::unique_ptr<RvizDisplay> rviz_display_;

  Eigen::Affine3f lidar_pose_mx_;

  /****************** Pubs Subs ******************************/
  float pub_rate_;
  nav_msgs::Path car_path_;
  ros::Publisher gps_origin_pub_;
  ros::Publisher odom_pub_;
  ros::Publisher point_cloud_pub_;
  ros::Publisher background_map_pub_;
  tf::TransformBroadcaster tf_odom_broadcaster_;
  tf::TransformBroadcaster tf_rtk_broadcaster_;
  geometry_msgs::Quaternion rtk_quat_;
  Eigen::Vector3f rtk_pose_;

  ros::Subscriber gnss_sub_;
  ros::Subscriber init_pose_sub_;
  ros::Subscriber lidar_sub_;
  ros::Subscriber rtk_kinetic_sub_;

  std::queue<nav_msgs::Odometry> rtk_kinematic_data_queue_;
  std::mutex rtk_kinematic_data_mutex_;
  std::condition_variable rtk_kinematic_data_arrived_;

  /****************** Params ******************************/
  YAML::Node param_;

  std::string sub_init_pose_topic_;
  std::string sub_point_cloud_topic_;
  std::string sub_gnss_topic_;
  std::string sub_rkt_kinetic_topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  std::string pub_gnss_origin_topic_;
  std::string pub_pose_topic_;
  std::string pub_pose_lidar_topic_;
  std::string pub_path_topic_;
  std::string pub_point_cloud_topic_;
  std::string pub_gnss_topic_;
  std::string pub_background_map_topic_;
  std::string pub_mapping_odom_topic_;
  std::string pub_icp_points_topic_;
  std::string pub_status_topic_;

  float lidar_height_;
  int thread_number_;
  float imu_ang_vel_ratio_;
  bool is_imu_reverse_;
  bool is_use_unit_kmh_;
  int num_of_path_reserved_;
};

}  // namespace localization
}  // namespace robosense

#endif  // RS_LOCALIZATION_H
