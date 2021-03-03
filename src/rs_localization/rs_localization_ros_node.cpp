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

#include "rs_localization/rs_localization_ros.h"
#include "common/prompt.h"
#include "common/version.h"
#include <iostream>
#include <ros/ros.h>

void head()
{
  std::cout << "\n";
  std::cerr << "\033[1;32m ~~ ================================================ ~~\033[0m" << std::endl;
  std::cerr << "\033[1;32m ~~ =====        Robosense Localization        ===== ~~\033[0m" << std::endl;
  std::cerr << "\033[1;32m ~~ ================================================ ~~\033[0m" << std::endl;
  std::cout << "\n";
  INFO << "ROS Version: " << ROS_VERSION_MAJOR << "." << ROS_VERSION_MINOR << "." << ROS_VERSION_PATCH << REND;
  INFO << "PCL Version: " << PCL_VERSION_PRETTY << REND;
  INFO << "Eigen Version: " << EIGEN_WORLD_VERSION << "." << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << REND;
  INFO << "RS_LOCALIZATION Version: " << RS_LOC_MAJOR_VERSION << "." << RS_LOC_MINOR_VERSION << "."
       << RS_LOC_PATCH_VERSION << REND;
  std::cout << "\n" << std::endl;
}

int main(int argc, char** argv)
{
  using namespace robosense::localization;
  head();
  ros::init(argc, argv, "rs_localization_ros");
  std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle);
  ros::NodeHandle pri_nh("~");
  std::string param_file;
  std::string map_file;
  pri_nh.getParam("param_file", param_file);
  pri_nh.getParam("map_file", map_file);
  RSLocalizationRos rs_localization_ros(param_file, map_file, nh);
  rs_localization_ros.run();

  return 0;
}
