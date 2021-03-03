#include "msf/observer/imu_odom_observer.h"
#include "common/eigen_util.h"

namespace robosense
{
namespace localization
{
namespace msf
{
ImuOdomObserver::ImuOdomObserver(const std::string name, const YAML::Node& param,
                                 const std::shared_ptr<SensorFusion> sensor_fusion)
  : Observer(name, sensor_fusion, param)

{
  config(params_);
  /*please explicitly start a observer in RSLocalization*/
  // start();
}

ImuOdomObserver::~ImuOdomObserver()
{
  stop();
}

void ImuOdomObserver::config(const YAML::Node& param)
{
  // TODO: load params from yaml file
  imu_transform_ = Eigen::Matrix4f::Identity();
  // set default value
  common::yamlRead(param, "linear_velocity_noise", v_noise_, 0.2);
  common::yamlRead(param, "angular_velocity_noise", w_noise_, 0.01);
  common::yamlRead(param, "imu_period", imu_period_, 1. / 25.);
  common::yamlRead(param, "odom_period", odom_period_, 1. / 10.);
  common::yamlRead(param, "odom_bias", odom_bias_, 1.0);

  common::yamlRead(param, "imu_failure_threshold", imu_watchdog_thresh_, 5.0);
  common::yamlRead(param, "odom_failure_threshold", odom_watchdog_thresh_, 3.0);

  imu_status_.store(Init);
  odom_status_.store(Init);
  prev_imu_status_.store(Init);
  prev_odom_status_.store(Init);
}

void ImuOdomObserver::start(void)
{
  // Config Observation
  observer_thread_active_ = true;
  const auto& func = [this] { observerThread(); };
  const auto& func_timer = [this] { timerThread(); };

  observer_thread_ = std::thread(func);
  timer_thread_ = std::thread(func_timer);

  // Config Imu Thread
  data_queue_max_size_ = 200;
  imu_ready_ = false;

  // config Odom Thread
  odom_ready_ = false;
}

void ImuOdomObserver::stop(void)
{
  if (observer_thread_active_.load())
  {
    observer_thread_active_ = false;
    imu_available_.notify_one();
    if (observer_thread_.joinable())
    {
      observer_thread_.join();
    }
    if (timer_thread_.joinable())
    {
      timer_thread_.join();
    }
  }

  imu_ready_ = false;
  odom_ready_ = false;
}

bool ImuOdomObserver::ready(void)
{
  if (imu_ready_.load() && odom_ready_.load())
    return true;
  else
    return false;
}

bool ImuOdomObserver::isOutlier(const ImuData& imu)
{
  // TODO : Implement this
  return false;
}

bool ImuOdomObserver::isOutlier(const OdomData& odom)
{
  // TODO : Implement this
  return false;
}

void ImuOdomObserver::addImuData(const ImuData& imu)
{
  // Only start after odom is ready
  imu_status_.store(SensorConnected);
  static int count = 0;
  if (odom_ready_.load())
  {
    double curr_imu_time = imu.timestamp;
    static double prev_imu_time = curr_imu_time;
    // std::cout << "prev_imu_time = " << prev_imu_time << "curr_imu_time = " << curr_imu_time << std::endl;

    double delta_time = curr_imu_time - prev_imu_time;

    if (delta_time < 0)
    {
      // TODO: Handle error
      ERROR << name() << " IMU data went back to prev time!" << RESET << END;
      return;
    }
    else if (isOutlier(imu))
    {
      return;
    }
    else if (delta_time > 10 * imu_period_)
    {
      // TODO: Handle error
      if (prev_imu_status_.load() == SensorConnected && imu_status_.load() == SensorConnected)
        ERROR << name() << " IMU data lost more than 10 packets!" << RESET << END;
    }
    std::unique_lock<std::mutex> lock(imu_queue_mutex_);
    imu_queue_.emplace(imu);
    imu_available_.notify_one();
    if (imu_queue_.size() > data_queue_max_size_)
    {
      WARNING << " IMU queue overflow!" << RESET << END;
      // TODO : Handle this error
    }
    prev_imu_time = curr_imu_time;
  }
}

void ImuOdomObserver::addOdomData(const OdomData& odom)
{
  double curr_odom_time = odom.timestamp;
  static double prev_odom_time = curr_odom_time;
  odom_status_.store(SensorConnected);
  using namespace std::chrono;
  auto now_ms = time_point_cast<milliseconds>(steady_clock::now());
  prev_odom_time_.store(now_ms.time_since_epoch().count());

  double delta_time = curr_odom_time - prev_odom_time;

  if (delta_time < 0)
  {
    // TODO: Handle error
    ERROR << name() << " ImuOdomObserver: Odom data went back to prev time!" << RESET << END;
    return;
  }
  else if (isOutlier(odom))
  {
    return;
  }
  else if (delta_time > 10 * odom_period_)  // TODO: Set odom frequency by param
  {
    // TODO: Handle error
    if (prev_odom_status_.load() == SensorConnected && odom_status_.load() == SensorConnected)
      ERROR << name() << " ImuOdomObserver: Odom data lost more than 10 packets, previous data discarded!" << RESET
            << END;
    odom_queue_.pop();  // discard previous Odom data as it's too old.
  }

  OdomData corrected_odom = odom;
  corrected_odom.linear_vel *= odom_bias_;  // For hailiang bus only

  std::unique_lock<std::mutex> lock(odom_queue_mutex_);
  odom_queue_.emplace(corrected_odom);

  if (odom_queue_.size() > 0)
    odom_ready_ = true;
  if (odom_queue_.size() > 2)
    odom_queue_.pop();

  prev_odom_time = curr_odom_time;
}

void ImuOdomObserver::correctObs(const MsfState& state, Observation& obs)
{
}

bool ImuOdomObserver::predicStateCov(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  const auto& new_pos = new_state.state_vec.segment<3>(StatePositionIdx);
  const auto& pos = state.state_vec.segment<3>(StatePositionIdx);

  double dt = new_state.timestamp - state.timestamp;
  float ds = (new_pos - pos).norm();
  float dx = new_pos.x() - pos.x();
  float dy = new_pos.y() - pos.y();

  // TODO : This is not correct for 3D
  EkfGMat G = EkfGMat::Identity();
  G(StatePositionIdx, StateAngleIdx + 2) = -dy;
  G(StatePositionIdx + 1, StateAngleIdx + 2) = dx;

  Eigen::Vector4f new_att = new_state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Vector4f att = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Quaternionf new_quat(new_att[3], new_att[0], new_att[1], new_att[2]);
  Eigen::Quaternionf quat(att[3], att[0], att[1], att[2]);

  G.block<3, 3>(StateOdomBiasIdx, StateOdomBiasIdx) = (new_quat * quat.conjugate()).toRotationMatrix();

  EkfRMat R = EkfGMat::Identity() * 1e-10;
  R.block<3, 3>(StatePositionIdx, StatePositionIdx) = Eigen::Matrix3f::Identity() * obs.linear_vel_cov(0, 0) * ds * ds;
  R.block<3, 3>(StateAngleIdx, StateAngleIdx) = Eigen::Matrix3f::Identity() * obs.angular_vel_cov(0, 0) * dt * dt;
  // TODO : Need a more accurate gps bias process noise
  R.block<3, 3>(StateGpsBiasIdx, StateGpsBiasIdx) = Eigen::Matrix3f::Identity() * 0.01 * dt * dt;
  // TODO : Need a more accurate odom bias process noise
  R.block<3, 3>(StateOdomBiasIdx, StateOdomBiasIdx) = Eigen::Matrix3f::Identity() * 0.001 * dt * dt;

  new_state.cov = G * state.cov * G.transpose() + R;
}

bool ImuOdomObserver::predicStateVec(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  new_state = state;

  new_state.seq = state.seq + 1;
  new_state.timestamp = obs.timestamp;
  // Update Sensor Meas
  new_state.state_vec.segment<3>(StateLinearAccIdx) = obs.linear_acc;
  new_state.state_vec.segment<3>(StateAngleVelIdx) = obs.angular_vel;

  double dt = new_state.timestamp - state.timestamp;

  // New Attitude
  const auto& avel = state.state_vec.segment<3>(StateAngleVelIdx);
  const auto& new_avel = new_state.state_vec.segment<3>(StateAngleVelIdx);
  Eigen::Matrix4f OmegaMean = getOmegaMat((avel + new_avel) * 0.5f);

  // std::cout << "\n\rdt : \n\r" << dt << std::endl;
  // std::cout << "OmegaMean : \n\r" << OmegaMean << std::endl;
  // std::cout << "att : \n\r" << att << std::endl;
  // std::cout << "new_att : \n\r" << new_att << std::endl;

  Eigen::Vector4f att = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Quaternionf quat(att[3], att[0], att[1], att[2]);
  Eigen::Vector4f new_att =
      (Eigen::Matrix4f::Identity() + OmegaMean * 0.5f * dt) * state.state_vec.segment<4>(StateAngleIdx);
  new_att.normalize();
  new_state.state_vec.segment<4>(StateAngleIdx) = new_att;
  Eigen::Quaternionf new_quat(new_att[3], new_att[0], new_att[1], new_att[2]);

  new_state.state_vec.segment<3>(StateOdomBiasIdx) =
      (new_quat * quat.conjugate()).toRotationMatrix() * state.state_vec.segment<3>(StateOdomBiasIdx);

  // New velocity
  new_state.state_vec.segment<3>(StateLinearVelIdx) =
      new_quat.toRotationMatrix() * obs.linear_vel - new_state.state_vec.segment<3>(StateOdomBiasIdx);

  // TODO: angular velocity should also be rotated

  // New Pose

  const auto& pos = state.state_vec.segment<3>(StatePositionIdx);
  const auto& lvel = state.state_vec.segment<3>(StateLinearVelIdx);
  const auto& new_lvel = new_state.state_vec.segment<3>(StateLinearVelIdx);
  new_state.state_vec.segment<3>(StatePositionIdx) = pos + (lvel + new_lvel) * 0.5f * dt;

  return true;
}

Observation ImuOdomObserver::getObservation(ImuData& imu_data)
{
  // static double prev_imu_time = imu_data.timestamp;
  // float imu_vel;
  double t1;
  double t2;
  double timu;
  float v1;
  float v2;
  float vx;
  // Add velocity to Observation
  // TODO: Use better method
  timu = imu_data.timestamp;
  {
    std::unique_lock<std::mutex> odom_lock(odom_queue_mutex_);
    t1 = odom_queue_.front().timestamp;
    t2 = odom_queue_.back().timestamp;
    v1 = odom_queue_.front().linear_vel(0);
    v2 = odom_queue_.back().linear_vel(0);
  }

  if (t2 == t1)
  {
    timu = imu_data.timestamp;
    vx = (v1 + v2) / 2;
  }
  else
  {
    timu = imu_data.timestamp;
    vx = (v2 - v1) / (float)(t2 - t1) * (float)(timu - t1) + v1;
  }

  Eigen::Vector4f angular_vel;
  angular_vel[0] = imu_data.angular_vel[0];
  angular_vel[1] = imu_data.angular_vel[1];
  angular_vel[2] = imu_data.angular_vel[2];
  angular_vel[3] = 0;
  angular_vel = imu_transform_ * angular_vel;

  Eigen::Vector4f linear_acc;
  linear_acc[0] = imu_data.linear_acc[0];
  linear_acc[1] = imu_data.linear_acc[1];
  linear_acc[2] = imu_data.linear_acc[2];
  linear_acc[3] = 0;
  linear_acc = imu_transform_ * linear_acc;

  // Create Observation Object and Push to queue
  Observation obs;
  obs.seq = imu_data.seq;
  obs.timestamp = imu_data.timestamp;
  obs.obs_source = name();
  obs.linear_vel.x() = vx;
  obs.angular_vel = angular_vel.block<3, 1>(0, 0);
  obs.linear_acc = linear_acc.block<3, 1>(0, 0);

  // Set noise
  obs.linear_vel_cov = Eigen::Matrix3f::Identity() * v_noise_ * v_noise_;
  obs.angular_vel_cov.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * w_noise_ * w_noise_;
  obs.angular_vel_cov(3, 3) = 1e-10;

  // TODO(zyc) : this is not a clean way to use &obs
  obs.apply = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
    bool ret = false;
    ret = predicStateVec(obs, state, new_state);
    ret = predicStateCov(obs, state, new_state);
    new_state.is_prediction = true;
    return true;
  };
  return obs;
}

void ImuOdomObserver::printStatus(int prev_stat, int cur_stat, std::string data_name)
{
  static int imu_pre_sum{ prev_stat + cur_stat };
  static int imu_cur_sum{ imu_pre_sum };
  static int odom_pre_sum{ imu_pre_sum };
  static int odom_cur_sum{ imu_pre_sum };
  int cur_sum;
  int pre_sum;

  cur_sum = prev_stat + cur_stat;
  double watchdog_thresh;
  if (data_name == "IMU")
  {
    // WARNING << prev_stat << " " << cur_stat << " | "<< pre_sum << " " << cur_sum<< REND;
    watchdog_thresh = imu_watchdog_thresh_;
    imu_cur_sum = prev_stat + cur_stat;
    cur_sum = imu_cur_sum;
    pre_sum = imu_pre_sum;
    imu_pre_sum = imu_cur_sum;
  }
  else if (data_name == "Odom")
  {
    watchdog_thresh = odom_watchdog_thresh_;
    odom_cur_sum = prev_stat + cur_stat;
    cur_sum = odom_cur_sum;
    pre_sum = odom_pre_sum;
    odom_pre_sum = odom_cur_sum;
  }
  else
    watchdog_thresh = -1;

  if (prev_stat == Init && cur_stat == SensorConnected || (pre_sum == 4 && cur_sum == 2))
  {
    INFO << DELETE_LINE << name() << ": " << data_name << " data Received for the first time." << REND;
  }
  else if ((prev_stat == SensorConnected || prev_stat == Init) && cur_stat == SensorDisconnected ||
           ((pre_sum == 2 || pre_sum == 4) && cur_sum == 0))
  {
    if (watchdog_thresh > 0)
      WARNING << DELETE_LINE << name() << ": No " << data_name << " data received for more than " << watchdog_thresh
              << " secs!" << REND;
    else
      WARNING << DELETE_LINE << name() << ": No " << data_name << " data received for awhile! " << REND;
  }
  else if ((prev_stat == SensorDisconnected && cur_stat == SensorConnected) || (pre_sum == 0 && cur_sum == 2))
  {
    INFO << DELETE_LINE << name() << ": " << data_name << " data recovered." << REND;
  }
}

void ImuOdomObserver::timerThread(void)
{
  while (observer_thread_active_.load())
  {
    printStatus(prev_imu_status_.load(), imu_status_.load(), "IMU");
    printStatus(prev_odom_status_.load(), odom_status_.load(), "Odom");

    prev_imu_status_.store(imu_status_.load());
    prev_odom_status_.store(odom_status_.load());

    if (odom_ready_.load())
    {
      using namespace std::chrono;
      auto current_time = time_point_cast<milliseconds>(steady_clock::now());
      using MillisecTimePoint = decltype(current_time);

      MillisecTimePoint prev_odom_timepoint(milliseconds(prev_odom_time_.load()));
      duration<double> duration_since_last_odom_received = current_time - prev_odom_timepoint;

      // WARNING << duration_since_last_odom_received.count() << REND;
      if (duration_since_last_odom_received.count() > odom_watchdog_thresh_)
      {
        odom_status_.store(SensorDisconnected);
        imu_ready_ = false;
        continue;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void ImuOdomObserver::observerThread(void)
{
  while (observer_thread_active_.load())
  {
    ImuData imu_data;

    // Retrive IMU Observation
    int waiting_num;
    {
      std::unique_lock<std::mutex> imu_lock(imu_queue_mutex_);
      if (imu_queue_.size() == 0)
      {
        // std::chrono::duration<double> sec(imu_watchdog_thresh_);
        std::chrono::duration<double> wait_duration(imu_watchdog_thresh_);
        auto cv_status = imu_available_.wait_for(imu_lock, wait_duration);
        if (cv_status == std::cv_status::timeout)
        {
          imu_status_.store(SensorDisconnected);
        }
        continue;
      }

      imu_data = imu_queue_.front();
      imu_queue_.pop();
      waiting_num = imu_queue_.size();
    }
    if (waiting_num > 10)
    {
      WARNING << name() << ":More than 10 IMU data waiting!" << RESET << END;
    }

    // std::cout << "vimu_x = " << vimu_x << "vimu_y = " << vimu_y << std::endl;
    // std::cout << "angular_vel[0] = " << angular_vel[0] << "angular_vel[1] = " << angular_vel[1] << " angular_vel[2] =
    // " << angular_vel[2] << std::endl << std::endl;
    // TODO: Outlier rejection
    Observation obs = getObservation(imu_data);
    sensor_fusion_->addObservation(obs);
    imu_ready_ = true;
  }
  WARNING << name() << ": exited" << RESET << END;
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense
