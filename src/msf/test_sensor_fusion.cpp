#include <ros/ros.h>
#include "common/point_cloud_viewer.h"
#include "common/time.h"
#include "msf/core/sensor_fusion.h"
#include <cstdlib>
#include <iostream>
#include <ctime>

using namespace robosense;
using namespace localization;
using namespace msf;

volatile double sim_time = 0.0;
const uint64_t MSEC = 1000;
void sim_time_thread(void)
{
  std::cout << "Into sim_time_thread \n\r";
  ros::Rate r(1000);
  while (ros::ok())
  {
    sim_time += 0.001;
    r.sleep();
  }
}

// void sim_odom_thread(const std::shared_ptr<SensorFusion> msf)
// {
//   std::cout << "Into sim_odom_thread \n\r";
//   double stamp = 0;
//   uint64_t seq = 0;
//   int32_t period = 93 * MSEC;
//   while (ros::ok())
//   {
//     stamp = sim_time;
//     seq++;
//     msf::Header header;
//     header.seq_ = seq;
//     header.timestamp = stamp;

//     msf::OdomData odom_data;
//     odom_data.header_ = header;
//     odom_data.linear_vel_[0] = 0.1;

//     msf->addOdomData(odom_data);
//     int32_t period = 50 + std::rand() / ((RAND_MAX + 1u) / 101);
//     period *= MSEC;
//     usleep(period);
//   }
// }

// void sim_imu_thread(const std::shared_ptr<SensorFusion> msf)
// {
//   std::cout << "Into sim_imu_thread \n\r";
//   double stamp = 0;
//   uint64_t seq = 0;
//   while (ros::ok())
//   {
//     stamp = sim_time;
//     seq++;
//     msf::Header header;
//     header.seq_ = seq;
//     header.timestamp = stamp;

//     msf::ImuData imu_data;
//     imu_data.header_ = header;
//     imu_data.angular_vel_[2] = 0.1;

//     msf->addImuData(imu_data);
//     int32_t period = 1 + std::rand() / ((RAND_MAX + 1u) / 20);
//     period *= MSEC;
//     usleep(period);
//   }
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "test_icp_localization");
//   ros::NodeHandle nh;
//   ros::NodeHandle pri_nh("~");
//   std::srand(std::time(nullptr));
//   std::shared_ptr<SensorFusion> msf = std::shared_ptr<SensorFusion>(new SensorFusion);

//   std::thread time_thread(sim_time_thread);
//   std::thread odom_thread(sim_odom_thread, msf);
//   std::thread imu_thread(sim_imu_thread, msf);

//   while (ros::ok())
//   {
//     sleep(1);
//   }

//   return 0;
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_icp_localization");
  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");

  std::string parameter_file;
  std::string map_file;
  pri_nh.getParam("parameter_file", parameter_file);
  pri_nh.getParam("map_file", map_file);
  YAML::Node parameters = YAML::LoadFile(parameter_file);
  parameters_["map_server"]["map_file"] = map_file;

  std::shared_ptr<SensorFusion> msf =
      std::shared_ptr<SensorFusion>(new SensorFusion(parameters_["RSLocalizationNormalRun"]));
  std::thread time_thread(sim_time_thread);

  Observation odom;
  odom.obs_source = "OdomObserver";

  Observation imu;
  imu.obs_source = "ImuObserver";

  imu.timestamp = 0.01;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.019;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.029;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.039;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.039001;
  msf->addObservation(imu);
  sleep(1);

  odom.timestamp = 0.039;
  msf->addObservation(odom);
  sleep(1);

  imu.timestamp = 0.041;
  msf->addObservation(imu);
  sleep(1);

  odom.timestamp = 0.049;
  msf->addObservation(odom);
  sleep(1);

  imu.timestamp = 0.05;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.06;
  msf->addObservation(imu);
  sleep(1);

  odom.timestamp = 0.07;
  msf->addObservation(odom);
  sleep(1);

  imu.timestamp = 0.07;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.08;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.09;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.1;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.11;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.12;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.122;
  msf->addObservation(imu);
  sleep(1);

  odom.timestamp = 0.08;
  msf->addObservation(odom);
  sleep(1);

  odom.timestamp = 0.14;
  msf->addObservation(odom);
  sleep(1);

  imu.timestamp = 0.13;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.14;
  msf->addObservation(imu);
  sleep(1);

  imu.timestamp = 0.15;
  msf->addObservation(imu);
  sleep(1);

  while (ros::ok())
  {
    sleep(1);
  }

  return 0;
}
