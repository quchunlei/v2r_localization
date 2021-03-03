#include "msf/observer/gnss_observer.h"
#include "common/eigen_util.h"
#include "common/time.h"

namespace robosense
{
namespace localization
{
namespace msf
{
GnssObserver::GnssObserver(const std::string name, const YAML::Node& param,
                           const std::shared_ptr<SensorFusion> sensor_fusion,
                           const std::shared_ptr<GpsMsgProcess> gnss_process,
                           const std::shared_ptr<MapServer> map_server)
  : Observer(name, sensor_fusion, param), gnss_process_(gnss_process), map_server_ptr_(map_server)
{
  config(params_);
  /*please explicitly start a observer in RSLocalization*/
  // start();
}

GnssObserver::~GnssObserver()
{
  stop();
}

void GnssObserver::config(const YAML::Node& param)
{
  // TODO : load params from yaml file
  // TODO : Test offset correct
  common::yamlRead(param, "discard_unqualified_data", is_discard_unqualified_, false);
  common::yamlRead(param, "gnss_angle_noise", angle_noise_, 0.01);
  common::yamlRead(param, "gnss_vel_noise", vel_noise_, 0.01);
  common::yamlRead(param, "gnss_pose_noise", pose_noise_, 0.01);
  common::yamlRead(param, "gnss_failure_threshold", watchdog_thresh_, 10.0);
  sensor_transform_ = Eigen::Matrix4f::Identity();
}

bool GnssObserver::isOutlier(const GnssData& data)
{
  if (data.status <= GnssData::STATUS_FIX && is_discard_unqualified_)
    return true;
  else
    return false;
}

void GnssObserver::start(void)
{
  // Config Observation
  observer_thread_active_ = true;
  data_queue_max_size_ = 20;
  const auto& func = [this] { observerThread(); };
  observer_thread_ = std::thread(func);

  // Config Data Thread
  data_queue_max_size_ = 20;
  obs_ready_ = false;
}

void GnssObserver::stop(void)
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

bool GnssObserver::ready(void)
{
  if (obs_ready_.load())
    return true;
  else
    return false;
}

void GnssObserver::addGnssData(const GnssData& data)
{
  double curr_data_time = data.timestamp;
  static double prev_data_time = curr_data_time;

  double delta_time = curr_data_time - prev_data_time;
  if (delta_time < 0)
  {
    // TODO: Handle error
    ERROR << name() << " : data went back to prev time!" << RESET << END;
    return;
  }
  else if (isOutlier(data))
  {
    WARNING << "GNSS data outlier discarded." << REND;
    return;
  }
  else if (delta_time > 10)
  {
    // TODO: Handle error
    if (prev_status_.load() == SensorConnected && status_.load() == SensorConnected)
      ERROR << name() << " : data lost more than 10 packets!" << RESET << END;
  }

  std::unique_lock<std::mutex> lock(data_queue_mutex_);
  data_queue_.emplace(data);
  data_available_.notify_one();
  status_.store(SensorConnected);
  if (data_queue_.size() > data_queue_max_size_)
  {
    WARNING << name() << " : data queue overflow!" << RESET << END;
    // TODO : Handle this error
  }

  prev_data_time = curr_data_time;
}

void GnssObserver::correctObs(const MsfState& state, Observation& obs)
{
}

Observation GnssObserver::getObservation(const GnssData& data)
{
  static GnssData last_data = data;
  static float last_height = 1.8;  // set to 1.8 by default before successfully get height from map server
  Observation obs;
  obs.seq = data.seq;
  obs.timestamp = data.timestamp;
  obs.obs_source = name();

  //*********************Get Position***********************************
  Eigen::Vector3d xyz;
  float lidar_height;
  gnss_process_->gps2xyz(data.longitude, data.latitude, 0, xyz);
  if (!map_server_ptr_->getHeight(xyz.x(), xyz.y(), 0, lidar_height))
  {
    WARNING << name() << '\t' << "Can't get height of car, using default value" << RESET << END;
    lidar_height = 0.0f;
  }

  obs.pos << xyz.x(), xyz.y(), lidar_height;
  obs.pos_cov = data.pos_cov;

  //*********************Get Quaternion***********************************
  // retrieve a interpolated state in sensor_fuson to get roll and pitch
  MsfState state;
  if (!sensor_fusion_->getEstimatedState(state, obs.timestamp))
  {
    WARNING << " Failed to get estimation" << RESET << END;
  }

  // roll and pitch directory from state
  Eigen::Quaternionf q_nominal(state.state_vec.segment<4>(StateAngleIdx).data());
  Eigen::Vector3f ypr = eulerAnglesZYX(std::move(q_nominal));
  // yaw from rtk measurement
  Eigen::Vector3f ypr_m = eulerAnglesZYX(Eigen::Quaternionf(data.angle.data()));
  // combine roll pitch from state and yaw from measurement
  Eigen::Quaternionf combined_q(Eigen::AngleAxisf(ypr_m(0), Eigen::Vector3f::UnitZ()) *
                                Eigen::AngleAxisf(ypr(1), Eigen::Vector3f::UnitY()) *
                                Eigen::AngleAxisf(ypr(2), Eigen::Vector3f::UnitX()));
  obs.angle = quatCoeffsFix(combined_q);

  //*********************Get Velocity***********************************
  obs.linear_vel = data.linear_vel;

  // TODO : use data from sensor
  if (last_data.timestamp == data.timestamp)
  {
    obs.linear_vel = Eigen::Vector3f::Zero();
  }

  //*********************Define a lamda expression***********************************
  obs.apply = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
    return applyEkfCorrect2(obs, state, new_state);
  };

  last_data = data;

  // timer.print();
  return obs;
}

bool GnssObserver::applyEkfCorrect2(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  // WARNING << "------Correction------ " << REND;  // << obs.seq << ": " << std::fixed << obs.timestamp << REND;
  if (obs.timestamp != state.timestamp)
  {
    ERROR << "[correction]: Observation's timestamp is not aligned with that of input state!!!" << RESET << END;
    exit(-1);
  }

  new_state = state;  // this will copy those states that do not need update

  static double prev_time = obs.timestamp;
  double dt = (obs.timestamp == prev_time) ? 0.1 : obs.timestamp - prev_time;  // dt in unused
  prev_time = obs.timestamp;

  // get nominal states
  auto p = state.state_vec.segment<3>(StatePositionIdx);
  auto v = state.state_vec.segment<3>(StateLinearVelIdx);
  auto q = state.state_vec.segment<4>(StateAngleIdx);
  auto a_b = state.state_vec.segment<3>(StateAccBiasIdx);
  auto w_b = state.state_vec.segment<3>(StateGyroBiasIdx);
  auto g = state.state_vec.segment<3>(StateGravityIdx);

  /*
  For convenience, assume we have a imaginary sensor which directly measure the error delta_p and delta_theta.
  Then the measurement Jacobian H(6X18) would simply consists of zeros and identiy matrixes.
  */
  Eigen::Matrix<float, MEAS_SIZE, ErrorStateSize> H = Eigen::Matrix<float, MEAS_SIZE, ErrorStateSize>::Zero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  H.block<3, 3>(3, 6) = Eigen::Matrix3f::Identity();

  auto PHt = state.es_cov * H.transpose();  // (18X6)

  MatR R = MatR::Identity();
  R.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * pose_noise_ * pose_noise_;
  R.block<3, 3>(3, 3) = Eigen::Matrix3f::Identity() * angle_noise_ * angle_noise_;

  if (1)
  {
    // R(2,2) = 0;
    R(3, 3) = 0;
    R(4, 4) = 0;  // 1e-9
  }

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
  delta_z.segment<3>(3) = (q_nominal.conjugate() * q_meas).vec() * 2;  /// delta_q.w();
  if (delta_q.w() < 0)
  {
    ERROR << "It happened!!!" << REND;
    DEBUG << q_nominal.coeffs().transpose() << REND;
    DEBUG << q_meas.coeffs().transpose() << REND;
    delta_z.segment<3>(3) = delta_z.segment<3>(3) * -1;
  }

  if (1)
  {
    // delta_z(2) = 0;
    delta_z(3) = 0;
    delta_z(4) = 0;
  }

  EsVec errors = K * delta_z;
  WARNING << "delta_z: " << delta_z.transpose() << REND;
  WARNING << "error  : " << errors.transpose() << REND;
  // Injection of the observed error into the nominal state
  // position
  new_state.state_vec.segment<3>(StatePositionIdx) =
      new_state.state_vec.segment<3>(StatePositionIdx) + errors.segment<3>(EsPositionIdx);

  // vel
  new_state.state_vec.segment<3>(StateLinearVelIdx) =
      new_state.state_vec.segment<3>(StateLinearVelIdx) + errors.segment<3>(EsLinearVelIdx);

  // quaternion
  auto q_error = getQuatFromOmega(errors.segment<3>(EsAngleIdx));
  auto new_q = q_nominal * q_error;
  new_state.state_vec.segment<4>(StateAngleIdx) = quatCoeffsFix(new_q);

  // update acc_bias and acc measurement
  auto new_acc_bias = new_state.state_vec.segment<3>(StateAccBiasIdx) + errors.segment<3>(EsAccBiasIdx);
  new_state.state_vec.segment<3>(StateAccBiasIdx) = new_acc_bias;

  // update gyro_bias and angular vel measurement
  auto new_gyro_bias = new_state.state_vec.segment<3>(StateGyroBiasIdx) + errors.segment<3>(EsGyroBiasIdx);
  new_state.state_vec.segment<3>(StateGyroBiasIdx) = new_gyro_bias;

  // gravity
  new_state.state_vec.segment<3>(StateGravityIdx) =
      new_state.state_vec.segment<3>(StateGravityIdx) + errors.segment<3>(EsGravityIdx);

  // EsEKF reset
  // we don't restore the error state, which implies that error state is reset to zero
  EsCov G = EsCov::Identity();
  G.block<3, 3>(6, 6) =
      Eigen::Matrix3f::Identity() - getInvSkewMat(0.5 * errors.segment<3>(EsAngleIdx));  // angle of error_state(3x1)
  new_state.es_cov = G * new_state.es_cov * G.transpose();

  return true;
}

bool GnssObserver::applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  // WARNING << "------Correction------: " << obs.seq << ": " << std::fixed << obs.timestamp << REND;

  // if (obs.timestamp != state.timestamp)
  // {
  //   ERROR << "[correction]: Observation's timestamp is not aligned with that of input state!!!" << RESET << END;
  //   exit(-1);
  // }

  // static double prev_time = obs.timestamp;
  // double dt = (obs.timestamp == prev_time) ? 0.1 : obs.timestamp - prev_time; // dt in unused
  // prev_time = obs.timestamp;

  // new_state = state; // cpoy states that don't update by correction
  // new_state.timestamp = obs.timestamp;
  // new_state.seq = obs.seq;

  // EsHMat H = EsHMat::Zero();
  // H.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
  // H.block<3, 3>(3, 6) = Eigen::Matrix3f::Identity();

  // auto PHt = state.es_cov*H.transpose();
  // EsQMat Q = EsQMat::Identity();
  // Q.block<3,3>(0,0) = Eigen::Matrix3f::Identity() * pose_noise_ * pose_noise_ ;
  // Q.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * angle_noise_ * angle_noise_ ;

  // // ERROR << "P:\n " << state.es_cov << REND;
  // // ERROR << "Q:\n" << Q << REND;
  // // ERROR << "H:\n " << H << REND;
  // auto S = H*PHt+Q;
  // auto K = PHt*S.inverse();
  // // ERROR << "S:\n" << S << REND;
  // // WARNING <<"Kalman\n"<< K << REND;

  // new_state.es_cov = (Eigen::Matrix<float,18,18>::Identity()-K*H)*state.es_cov;
  // new_state.es_cov = 0.5*(new_state.es_cov+new_state.es_cov.transpose());

  // EkfMeasVec y;

  // y.segment<3>(0) = obs.pos - state.state_vec.segment<3>(StatePositionIdx);
  // auto att = state.state_vec.segment<4>(StateAngleIdx);

  // Eigen::Quaternionf q(att[3], att[0], att[1], att[2]);
  // Eigen::Quaternionf meas_q(obs.angle(3), obs.angle(0), obs.angle(1), obs.angle(2));
  // auto dq = q.conjugate()*meas_q;

  // // DEBUG << "nominal q: " << q.w() << " " << q.x() << " "<<q.y() << " "<<q.z() << REND;
  // // DEBUG << "measure q: " << meas_q.w() << " " << meas_q.x() << " "<<meas_q.y() << " "<<meas_q.z() << REND;
  // // DEBUG << "delta   q: " << dq.w() << " " << dq.x() << " "<<dq.y() << " "<<dq.z() << REND;
  // // DEBUG << "nominal w_bias: " << state.state_vec.segment<3>(StateGyroBiasIdx).transpose() << REND;
  // // ERROR << "obs pose: "<<obs.pos.transpose() << REND;
  // // ERROR << "nominal pose: " << state.state_vec.segment<3>(StatePositionIdx) << REND;

  // auto dq_real = Eigen::Vector3f(dq.x(), dq.y(), dq.z());
  // if(dq.w() < 0)
  // {
  //   dq_real = dq_real*-1;
  // }

  // double d_angle = asinf(dq_real.norm())*2;
  // Eigen::Vector3f axis;
  // if(d_angle < 1e-8)
  // {
  //   axis = Eigen::Vector3f::Zero();
  // }
  // else
  // {
  //   axis = dq_real / dq_real.norm();
  // }

  // y.segment<3>(3) = d_angle*axis;

  // auto error_state = K * y;

  // // INFO << y.transpose() << REND;/
  // // Injection of the observed error into the nominal state
  // // position
  // new_state.state_vec.segment<3>(StatePositionIdx) =
  //     new_state.state_vec.segment<3>(StatePositionIdx) + error_state.segment<3>(EsPositionIdx);

  // // vel
  // new_state.state_vec.segment<3>(StateLinearVelIdx) =
  //     new_state.state_vec.segment<3>(StateLinearVelIdx) + error_state.segment<3>(EsLinearVelIdx);

  // // quaternion
  // // Eigen::Quaternionf quat(q(3), q(0), q(1), q(2);
  // auto q_error = getQuatFromOmega(error_state.segment<3>(EsAngleIdx));
  // auto new_q = q * q_error;
  // new_state.state_vec.segment<4>(StateAngleIdx) =
  //     Eigen::Vector4f(new_q.x(), new_q.y(), new_q.z(), new_q.w());

  // // acc_bias
  // new_state.state_vec.segment<3>(StateAccBiasIdx) =
  //     new_state.state_vec.segment<3>(StateAccBiasIdx) + error_state.segment<3>(EsAccBiasIdx);

  // // gyro_bias
  // new_state.state_vec.segment<3>(StateGyroBiasIdx) =
  //     new_state.state_vec.segment<3>(StateGyroBiasIdx) + error_state.segment<3>(EsGyroBiasIdx);

  // // gravity
  // new_state.state_vec.segment<3>(StateGravityIdx) =
  //     new_state.state_vec.segment<3>(StateGravityIdx) + error_state.segment<3>(EsGravityIdx);
  // // EsEKF reset
  // // we don't restore the error state, which implies that error state is reset to zero
  // EsCov G = EsCov::Identity();
  // G.block<3, 3>(6, 6) = Eigen::Matrix3f::Identity() -
  //                       getInvSkewMat(0.5 * error_state.segment<3>(EsAngleIdx));  // angle of error_state(3x1)
  // new_state.es_cov = G * new_state.es_cov * G.transpose();

  // WARNING << "obs pose: " << obs.pos.transpose() << REND;
  // WARNING << "obs quat: " << obs.angle.transpose() << REND;
  // WARNING << "obs error: " << y.transpose() << REND;
  // WARNING << "ES pose: " << error_state.segment<3>(EsPositionIdx).transpose() << REND;
  // WARNING << "ES Vel: " << error_state.segment<3>(EsLinearVelIdx).transpose() << REND;
  // WARNING << "ES quat: " << error_state.segment<3>(EsAngleIdx).transpose() << REND;
  // WARNING << "ES a_bias: " << error_state.segment<3>(EsAccBiasIdx).transpose() << REND;
  // WARNING << "ES g_bias: " << error_state.segment<3>(EsGyroBiasIdx).transpose() << REND;
  // WARNING << "ES gravity: " << error_state.segment<3>(EsGravityIdx).transpose() << REND;
  // WARNING << "-----" << REND;

  // new_state.is_prediction = false;
  return true;
}

void GnssObserver::observerThread(void)
{
  while (observer_thread_active_.load())
  {
    printStatus(prev_status_.load(), status_.load(), "GNSS");
    prev_status_.store(status_.load());
    GnssData data;
    // Retrive Gnss Observation
    int waiting_num;
    {
      std::unique_lock<std::mutex> lock(data_queue_mutex_);
      if (data_queue_.size() == 0)
      {
        std::chrono::duration<double> wait_duration(watchdog_thresh_);
        auto cv_status = data_available_.wait_for(lock, wait_duration);
        if (cv_status == std::cv_status::timeout)
        {
          status_.store(SensorDisconnected);
        }
        continue;
      }
      data = data_queue_.front();
      data_queue_.pop();
      waiting_num = data_queue_.size();
    }
    if (waiting_num > 10)
    {
      WARNING << name() << " : more than 10 data waiting!" << RESET << END;
    }

    // Create Observation Object and Push to queue
    Observation obs = getObservation(data);
    sensor_fusion_->addObservation(obs);
    obs_ready_ = true;
  }
  WARNING << name() << ": exited" << RESET << END;
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense