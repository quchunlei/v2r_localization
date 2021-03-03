#include "msf/observer/lidar_es_observer.h"
#include "common/eigen_util.h"

namespace robosense
{
namespace localization
{
namespace msf
{
LidarEsObserver::LidarEsObserver(const std::string name, const YAML::Node& param,
                                 const std::shared_ptr<SensorFusion> sensor_fusion,
                                 const std::shared_ptr<MapServer> map_server)
  : Observer(name, sensor_fusion, param), map_server_(map_server)

{
  config(params_);
  /*please explicitly start a observer in RSLocalization*/
  // start();
}

LidarEsObserver::~LidarEsObserver()
{
  stop();
}
template<typename T, typename... Args>
std::unique_ptr<T> LidarEsObserver::make_unique(Args&&... args)
{
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

void LidarEsObserver::config(const YAML::Node& param)
{
  // TODO: load params from yaml file
  if (!param["registration_process"])
  {
    ERROR << name() << " Registration process not defined in param file" << RESET << END;
    exit(-1);
  }

  common::yamlRead(param, "only_correct_xy_yaw", CORRECT_XY_YAW_ONLY_, false);
  common::yamlRead(param, "heading_position_noise", heading_pos_noise_, 0.2);
  common::yamlRead(param, "side_position_noise", side_pos_noise_, 0.05);
  common::yamlRead(param, "angle_noise", angle_noise_, 0.01);
  common::yamlRead(param, "lidare_failure_threshold", watchdog_thresh_, 5.0);

  icp_localization_ = make_unique<ICPLocalization>(map_server_, param["registration_process"]);
  num_scan_per_local_map_ = 2;
}

void LidarEsObserver::start(void)
{
  if (observer_thread_active_)
    return;
  observer_thread_active_ = true;
  data_queue_max_size_ = 1;
  const auto& func = [this] { observerThread(); };
  observer_thread_ = std::thread(func);

  obs_ready_ = false;  // set to false initially
}

void LidarEsObserver::stop(void)
{
  if (observer_thread_active_.load())
  {
    observer_thread_active_ = false;
    data_available_.notify_one();
    if (observer_thread_.joinable())
      observer_thread_.join();
  }
  obs_ready_ = false;
}

bool LidarEsObserver::ready(void)
{
  return obs_ready_.load();
}

bool LidarEsObserver::isOutlier(const LidarData* const data) const
{
  return false;
}

void LidarEsObserver::addLidarData(const LidarData& data)
{
  double curr_data_time = data.timestamp;
  static double prev_data_time = curr_data_time;
  double delta_time = curr_data_time - prev_data_time;
  status_.store(SensorConnected);
  // std::cout << "curr_data time: " << std::setprecision(20) << curr_data_time << std::endl;
  // std::cout << "prev_data time: " << std::setprecision(20) << prev_data_time << std::endl;
  if (delta_time < 0)
  {
    // TODO: handle error
    ERROR << name() << " ICP data went back to prev time!" << RESET << END;
    // std::cout << "delta_time: " << delta_time << std::endl;
    return;
  }
  else if (isOutlier(&data))
  {
    return;
  }
  else if (delta_time > 1.0)
  {
    // TODO: Handle error
    if (prev_status_.load() == SensorConnected && status_.load() == SensorConnected)
      ERROR << name() << " ICP data lost more than 10 packets!" << RESET << END;
  }

  std::unique_lock<std::mutex> lock(data_queue_mutex_);
  data_queue_.emplace(data);
  data_available_.notify_one();
  if (data_queue_.size() > data_queue_max_size_)
  {
    // WARNING << " ICP queue overflow!" << RESET << END;
    data_queue_.pop();
    // TODO: handle this error
  }

  prev_data_time = curr_data_time;

  // std::cout << "addLidarData \n\r";
  return;
}

bool LidarEsObserver::applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  if (obs.timestamp != state.timestamp)
  {
    ERROR << "[correction]: Observation's timestamp is not aligned with that of input state!!!" << RESET << END;
    exit(-1);
  }

  new_state = state;  // this will copy those states that do not need update

  static double prev_time = obs.timestamp;
  double dt = (obs.timestamp == prev_time) ? 0.1 : obs.timestamp - prev_time;
  prev_time = obs.timestamp;

  // get nominal states
  auto p = state.state_vec.segment<3>(StatePositionIdx);
  auto v = state.state_vec.segment<3>(StateLinearVelIdx);
  auto q = state.state_vec.segment<4>(StateAngleIdx);
  auto a_b = state.state_vec.segment<3>(StateAccBiasIdx);
  auto w_b = state.state_vec.segment<3>(StateGyroBiasIdx);
  auto g = state.state_vec.segment<3>(StateGravityIdx);

  /*
  For convenience, assume we have a imaginary sensor which directly measures the error delta_p and delta_theta.
  Then the measurement Jacobian H(6X18) would simply consists of zeros and identiy matrixes.
  */
  Eigen::Matrix<float, MEAS_SIZE, ErrorStateSize> H = Eigen::Matrix<float, MEAS_SIZE, ErrorStateSize>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3f::Identity();
  auto PHt = state.es_cov * H.transpose();  // (18X6)

  MatR R = MatR::Identity();

  R.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * heading_pos_noise_ * heading_pos_noise_;
  R.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * angle_noise_ * angle_noise_;

  auto S = H * PHt + R;        // 6X6
  auto K = PHt * S.inverse();  // 18X6

  // WARNING << "K (a_b, w_b, g)\n" << K.block<9, 10>(9, 0) << REND;

  EsCov IKH = (EsCov::Identity() - K * H);
  new_state.es_cov =
      IKH * state.es_cov * IKH.transpose() + K * R * K.transpose();  // symmetric and positive Joseph form

  Measurement delta_z;
  Eigen::Quaternionf q_meas = Eigen::Quaternionf(obs.angle.data()).normalized();
  Eigen::Quaternionf q_nominal = Eigen::Quaternionf(q.data()).normalized();
  delta_z.segment<3>(0) = obs.pos - p;  // delta_p
  auto delta_q = q_nominal.conjugate() * q_meas;

  delta_z.segment<3>(3) = (q_nominal.conjugate() * q_meas).vec() * 2 / delta_q.w();
  if (delta_q.w() < 0)
  {
    delta_z.segment<3>(3) = delta_z.segment<3>(3) * -1;
  }

  // TODO: It works even when delta_z(3) and delta_z(4) are not set to 0 here

  EsVec errors = K * delta_z;

  // Injection of the observed error into the nominal state
  // Update position
  new_state.state_vec.segment<3>(StatePositionIdx) =
      new_state.state_vec.segment<3>(StatePositionIdx) + errors.segment<3>(EsPositionIdx);

  // Update vel
  new_state.state_vec.segment<3>(StateLinearVelIdx) =
      new_state.state_vec.segment<3>(StateLinearVelIdx) + errors.segment<3>(EsLinearVelIdx);

  // Update quaternion
  auto q_error = getQuatFromOmega(errors.segment<3>(EsAngleIdx));
  auto new_q = q_nominal * q_error;
  new_state.state_vec.segment<4>(StateAngleIdx) = quatCoeffsFix(new_q);

  // Update acc_bias and acc measurement
  auto new_acc_bias = new_state.state_vec.segment<3>(StateAccBiasIdx) + errors.segment<3>(EsAccBiasIdx);
  new_state.state_vec.segment<3>(StateAccBiasIdx) = new_acc_bias;

  // Update gyro_bias and angular vel measurement
  auto new_gyro_bias = new_state.state_vec.segment<3>(StateGyroBiasIdx) + errors.segment<3>(EsGyroBiasIdx);
  new_state.state_vec.segment<3>(StateGyroBiasIdx) = new_gyro_bias;

  // Update gravity
  new_state.state_vec.segment<3>(StateGravityIdx) =
      new_state.state_vec.segment<3>(StateGravityIdx) + errors.segment<3>(EsGravityIdx);

  // EsEKF reset
  // We don't restore the error state, which implies that error state is reset to zero
  EsCov G = EsCov::Identity();
  G.block<3, 3>(6, 6) =
      Eigen::Matrix3f::Identity() - getInvSkewMat(0.5 * errors.segment<3>(EsAngleIdx));  // angle of error_state(3x1)
  new_state.es_cov = G * new_state.es_cov * G.transpose();

  new_state.is_prediction = false;

  return true;
}

bool LidarEsObserver::getObservation(const LidarData* data, Observation& obs)
{
  Eigen::Matrix4f measure_pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix<float, 6, 6> measure_cov = Eigen::Matrix<float, 6, 6>::Zero();
  Eigen::Matrix4f init_pose = Eigen::Matrix4f::Identity();
  MsfState init_state;

  if (!sensor_fusion_->getEstimatedState(init_state, data->timestamp))
  {
    WARNING << " Failed to get estimation" << RESET << END;
    return false;
  }

  Eigen::Quaternionf quat(init_state.state_vec.segment<4>(StateAngleIdx).data());
  quat.normalize();
  init_pose.block<3, 3>(0, 0) = quat.toRotationMatrix();
  init_pose.block<3, 1>(0, 3) = init_state.state_vec.block<3, 1>(StatePositionIdx, 0);

  // TODO(zyc): Find a way donot need mapping odom for height
  // TODO(zyc): Verify work for multi level road

  // TODO : 3D height is not accurate
  // init_pose(2, 3) = 2.0;
  // map_server_->getHeight(init_pose(0, 3), init_pose(1, 3), 0, init_pose(2, 3));

  if (!icp_localization_->locate(std::move(data->scan), init_pose, measure_pose, measure_cov))
    return false;

  obs.seq = data->seq;
  obs.timestamp = data->timestamp;
  obs.obs_source = name();
  //*********get position observation***************************
  obs.pos = measure_pose.block<3, 1>(0, 3);
  obs.pos_cov = measure_cov.block<3, 3>(0, 0);

  // TODO : 3D height is not accurate
  // obs.pos(2) = 2.0;
  // map_server_->getHeight(obs.pos(0), obs.pos(1), 0, obs.pos(2));

  //*********get quaternion observation*************************
  Eigen::Quaternionf q_meas(measure_pose.block<3, 3>(0, 0));
  if (CORRECT_XY_YAW_ONLY_)
  {
    // roll and pitch directory from state
    Eigen::Vector3f ypr = eulerAnglesZYX(std::move(quat));
    // yaw from rtk measurement
    Eigen::Vector3f ypr_m = eulerAnglesZYX(std::move(q_meas));
    // combine roll pitch from state and yaw from measurement
    Eigen::Quaternionf combined_q(Eigen::AngleAxisf(ypr_m(0), Eigen::Vector3f::UnitZ()) *
                                  Eigen::AngleAxisf(ypr(1), Eigen::Vector3f::UnitY()) *
                                  Eigen::AngleAxisf(ypr(2), Eigen::Vector3f::UnitX()));
    obs.angle = quatCoeffsFix(combined_q);
    obs.angle_cov.block<3, 3>(0, 0) = measure_cov.block<3, 3>(3, 3);
    obs.angle_cov(4, 4) = 1e-10;
  }
  else
  {
    obs.angle = quatCoeffsFix(std::move(q_meas));
    obs.angle_cov.block<3, 3>(0, 0) = measure_cov.block<3, 3>(3, 3);
    obs.angle_cov(4, 4) = 1e-10;
  }

  //*********get velocity observation**************************
  {
    static Observation last_obs = obs;
    if (last_obs.timestamp == obs.timestamp)
      obs.linear_vel = Eigen::Vector3f::Zero();
    else
    {
      auto dt = obs.timestamp - last_obs.timestamp;
      obs.linear_vel = ((obs.pos - last_obs.pos) * dt).cast<float>();
    }
    last_obs = obs;
  }

  obs.apply = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
    return applyEkfCorrect(obs, state, new_state);
  };

  return true;
}

void LidarEsObserver::observerThread(void)
{
  while (observer_thread_active_.load())
  {
    printStatus(prev_status_.load(), status_.load(), "LiDAR");
    prev_status_.store(status_.load());
    LidarData data;
    // Retrive Gnss Observation
    int waiting_num;
    {
      std::unique_lock<std::mutex> lock(data_queue_mutex_);
      std::chrono::duration<double> wait_duration(watchdog_thresh_);
      if (data_queue_.size() == 0)
      {
        auto cv_status = data_available_.wait_for(lock, wait_duration);
        if (cv_status == std::cv_status::timeout)
        {
          status_.store(SensorDisconnected);
        }
        continue;
      }
      data = std::move(data_queue_.front());
      data_queue_.pop();
      waiting_num = data_queue_.size();
    }
    if (waiting_num > 2)
    {
      WARNING << name() << " : more than " << waiting_num << " Lidar data waiting! " << RESET << END;
    }

    // if (!updateSubMap(data))
    //   continue;
    // Create Observation Object and Push to queue
    Observation obs;
    // auto start = std::chrono::system_clock::now();
    if (!getObservation(&data, obs))
      continue;
    // auto end = std::chrono::system_clock::now();
    // std::chrono::duration<double> elapsed_seconds = end - start;
    // std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    // std::cout << "finished icp at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";
    sensor_fusion_->addObservation(obs);
    obs_ready_ = true;
  }
  WARNING << name() << ": exited" << RESET << END;
}

bool LidarEsObserver::updateSubMap(const LidarData* scan)
{
  auto start = std::chrono::system_clock::now();
  // TODO : Odometry could fail some times need to find why
  // TODO : Add icp for scan matching
  static int scan_count = 0;
  bool submap_full = false;
  scan_count++;
  // LidarData local_scan = iscan;
  // filter_.apply(scan.scan_);
  MsfState state;
  sensor_fusion_->getEstimatedState(state, scan->timestamp);

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

  Eigen::Matrix3f rot(Eigen::AngleAxisf(state.angle[2], Eigen::Vector3f::UnitZ()));
  pose.block<3, 3>(0, 0) = rot;
  pose.block<3, 1>(0, 3) = state.pos;

  if (prev_local_map_ == NULL)
  {
    local_map_ = make_unique<LidarData>(*scan);
    prev_local_map_ = make_unique<LidarData>(*scan);
    local_map_pose_ = pose;
    rigid_trans_ = PM::get().TransformationRegistrar.create("RigidTransformation");
    return false;
  }

  Eigen::Matrix4f tf = pose.inverse() * local_map_pose_;

  // viewer.setPointCloud(prev_local_map_->scan_, Eigen::Vector3i(0, 0, 250), "scan");
  // viewer.setPointCloud(scan.scan_, Eigen::Vector3i(0, 250, 0), "scan");

  if (scan_count > num_scan_per_local_map_)
  {
    prev_local_map_.swap(local_map_);
    local_map_.reset(new LidarData(*scan));
    scan_count = 1;
  }
  else
  {
    local_map_->scan = rigid_trans_->compute(local_map_->scan, tf);
    local_map_->scan.concatenate(scan->scan);
    local_map_->seq = scan->seq;
    local_map_->timestamp = scan->timestamp;
  }
  if (scan_count == num_scan_per_local_map_)
    submap_full = true;

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  std::cout << "finished submap at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";

  prev_local_map_->scan = rigid_trans_->compute(prev_local_map_->scan, tf);
  prev_local_map_->scan.concatenate(scan->scan);
  prev_local_map_->seq = scan->seq;
  prev_local_map_->timestamp = scan->timestamp;

  local_map_pose_ = pose;

  return submap_full;
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense
