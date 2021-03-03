#include "msf/observer/imu_observer.h"
#include "msf/observer/imu_rtk_observer.h"
#include "msf/observer/imu_odom_observer.h"
#include <ros/ros.h>
#include "common/point_cloud_viewer.h"
#include "common/time.h"

using namespace robosense::localization;

volatile double sim_time = 0;
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

void sim_odom_thread(std::shared_ptr<msf::ImuOdomObserver> imu_odom_obs)
{
  std::cout << "Into sim_odom_thread \n\r";
  double stamp = 0;
  uint64_t seq = 0;
  int32_t period = 93 * MSEC;
  while (ros::ok())
  {
    stamp = sim_time;
    seq++;

    msf::OdomData odom_data;
    odom_data.seq_ = seq;
    odom_data.timestamp = stamp;
    odom_data.linear_vel_[0] = 0.1;

    imu_odom_obs->addOdomData(odom_data);
    usleep(period);
  }
}

void sim_imu_thread(std::shared_ptr<msf::ImuOdomObserver> imu_odom_obs)
{
  std::cout << "Into sim_imu_thread \n\r";
  double stamp = 0;
  uint64_t seq = 0;
  int32_t period = 10 * MSEC;
  while (ros::ok())
  {
    stamp = sim_time;
    seq++;

    msf::ImuData imu_data;
    imu_data.seq_ = seq;
    imu_data.timestamp = stamp;
    imu_data.angular_vel_[2] = 0.1;

    imu_odom_obs->addImuData(imu_data);
    usleep(period);
  }
}
std::mutex mut;
void obs_thread(std::shared_ptr<msf::ImuOdomObserver> imu_odom_obs,
                std::shared_ptr<std::condition_variable> obs_available)
{
  while (ros::ok())
  {
    std::unique_lock<std::mutex> lock(mut);
    msf::Observation obs;
    if (!imu_odom_obs->getObservation(obs))
    {
      obs_available->wait(lock);
      imu_odom_obs->getOgetObservationbs(obs);
    }
    std::cout << "got one obs \n\r";
  }
}

std::condition_variable cond;
int data = 0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_icp_localization");
  ros::NodeHandle nh;
  ros::NodeHandle pri_nh("~");

  std::shared_ptr<std::condition_variable> obs_available =
      std::shared_ptr<std::condition_variable>(new std::condition_variable);

  std::shared_ptr<msf::ImuOdomObserver> imu_odom_obs =
      std::shared_ptr<msf::ImuOdomObserver>(new msf::ImuOdomObserver(obs_available));

  std::thread time_th(sim_time_thread);
  std::thread odom_th(sim_odom_thread, imu_odom_obs);
  std::thread imu_th(sim_imu_thread, imu_odom_obs);
  std::thread obs_th(obs_thread, imu_odom_obs, obs_available);

  while (ros::ok())
  {
    sleep(1);
  }

  return 0;
}
