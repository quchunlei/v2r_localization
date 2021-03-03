#include "msf/observer/odom_observer.h"

namespace robosense
{
namespace localization
{
namespace msf
{
OdomObserver::OdomObserver(const std::string name, const YAML::Node& param,
                           const std::shared_ptr<SensorFusion> sensor_fusion)
  : Observer(name, sensor_fusion, param), prev_odom_time_(0)

{
  config(params_);
  /*please explicitly start a observer in RSLocalization*/
  // start();
}

OdomObserver::~OdomObserver()
{
  stop();
}

void OdomObserver::config(const YAML::Node& param)
{
  // TODO load params from yaml file
  // TODO : Test offset correct
  common::yamlRead(param, "vel_noise", vel_noise_);
  common::yamlRead(param, "use_global_vel", use_global_vel_);
  common::yamlRead(param, "odom_failure_threshold", watchdog_thresh_, 10.0);
  odom_offset_ = 0;
}

void OdomObserver::start(void)
{
  // Config Observation
  observer_thread_active_ = true;
  data_queue_max_size_ = 200;
  const auto& func = [this] { observerThread(); };
  observer_thread_ = std::thread(func);

  // config Odom Thread
  data_queue_max_size_ = 200;
  odom_ready_ = false;
}

void OdomObserver::stop(void)
{
  if (observer_thread_active_.load())
  {
    observer_thread_active_ = false;
    odom_available_.notify_one();
    observer_thread_.join();
    WARNING << " OdomObserver existed" << RESET << END;
  }

  odom_ready_ = false;
}

bool OdomObserver::ready(void)
{
  if (odom_ready_.load())
    return true;
  else
    return false;
}

void OdomObserver::addOdomData(const OdomData& odom)
{
  double curr_odom_time = odom.timestamp;

  double delta_time = curr_odom_time - prev_odom_time_;

  if (delta_time < 0)
  {
    // TODO: Handle error
    ERROR << name() << "Odom data went back to prev time!" << RESET << END;
    return;
  }
  else if (delta_time > 1.0)
  {
    // TODO: Handle error
    ERROR << name() << "Odom data lost more than 10 packets!" << RESET << END;
  }

  std::unique_lock<std::mutex> lock(odom_queue_mutex_);
  odom_queue_.emplace(odom);
  odom_available_.notify_one();
  if (odom_queue_.size() > data_queue_max_size_)
  {
    WARNING << name() << " Odom queue overflow!" << RESET << END;
    // TODO : Handle this error
  }
  prev_odom_time_ = curr_odom_time;
}

void OdomObserver::correctObs(const MsfState& state, Observation& obs)
{
}

bool OdomObserver::applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  WARNING << "------Odom Correction------ " << REND;  // << obs.seq << ": " << std::fixed << obs.timestamp << REND;
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
  Eigen::Quaternionf q_nominal = Eigen::Quaternionf(q.data()).normalized();

  /* measurement [v]
  Measurement jacobian is derived in doc/Odom_Sensor_Measurement_Model.md
   */
  Eigen::Matrix<float, MEAS_SIZE, 19> Hx = Eigen::Matrix<float, MEAS_SIZE, 19>::Zero();  // 3X19
  Hx.block<3, 3>(0, 3) = q_nominal.toRotationMatrix().transpose();
  Hx.block<3, 3>(0, 6) = 2 * q_nominal.w() * getInvSkewMat(v);
  Hx.block<3, 3>(0, 6) +=
      2 * (q_nominal.vec().dot(v) * Eigen::Matrix<float, 3, 3>::Identity() + q_nominal.vec() * v.transpose());
  Hx.block<3, 3>(0, 6) += -2 * (v * q_nominal.vec().transpose());
  Hx.block<3, 1>(0, 9) = 2 * q_nominal.w() * (v + getInvSkewMat(v) * q_nominal.vec());

  WARNING << "Hx:\n" << Hx << REND;

  /*Hdx(19x18) is slightly different from the one in Sola's paper as we put the real part of quaternion at the end*/
  Eigen::Matrix<float, 19, 18> Hdx = Eigen::Matrix<float, 19, 18>::Zero();
  Hdx.block<6, 6>(0, 0) = Eigen::Matrix<float, 6, 6>::Identity();
  Hdx.block<9, 9>(10, 9) = Eigen::Matrix<float, 9, 9>::Identity();

  // clang-format off
  Hdx.block<4, 3>(6, 6) = 0.5 * (Eigen::MatrixXf(4, 3) << q(3), -q(2),  q(1), 
                                                          q(2),  q(3), -q(0), 
                                                         -q(1),  q(0),  q(3), 
                                                         -q(0), -q(1), -q(2)).finished();
  // clang-format on

  // Finally get the measurement Jacobian H(3X18)
  auto H = Hx * Hdx;
  auto PHt = state.es_cov * H.transpose();  // (18X3)

  // MatR R = MatR::Identity();
  // R.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity() * vel_noise_ * vel_noise_;

  MatR R = MatR::Zero();
  R(0, 0) = vel_noise_ * vel_noise_;
  R(1, 1) = vel_noise_ * vel_noise_;
  R(2, 2) = vel_noise_ * vel_noise_;
  auto S = H * PHt + R;        // 3X3
  auto K = PHt * S.inverse();  // 18X3

  WARNING << "H:\n" << H << REND;
  WARNING << "PHt:\n" << PHt << REND;
  WARNING << "K:\n" << K << REND;

  // WARNING << "K (a_b, w_b, g)\n" << K.block<9, 10>(9, 0) << REND;

  EsCov IKH = (EsCov::Identity() - K * H);
  new_state.es_cov =
      IKH * state.es_cov * IKH.transpose() + K * R * K.transpose();  // symmetric and positive Joseph form

  Measurement z, z_nominal, delta_z;
  z_nominal << q_nominal.toRotationMatrix().transpose() * v;
  z << obs.linear_vel;
  delta_z = z - z_nominal;

  // auto delta_q_vec = delta_q.vec() / delta_q.w();
  // delta_z.segment<4>(6) << 0, delta_q_vec(0), delta_q_vec(1), delta_q_vec(2);
  // WARNING << "q_mea: " << Eigen::Vector4f(q_meas.w(), q_meas.x(), q_meas.y(), q_meas.z()).transpose() << REND;
  // WARNING << "q_nom: " << Eigen::Vector4f(q_nominal.w(), q_nominal.x(), q_nominal.y(), q_nominal.z()).transpose()
  // << REND;
  //
  // WARNING << "delta_q: " << Eigen::Vector4f(delta_q.w(), delta_q.x(), delta_q.y(), delta_q.z()).transpose() << REND;
  // WARNING << "delta_q_vec: " << delta_q_vec.transpose() << REND;
  // WARNING << "delta_q_vec2: " << delta_q_vec.transpose()*2 << REND;
  // WARNING << "delta_z quat: " << delta_z.segment<4>(6).transpose() << REND;
  EsVec errors = K * delta_z;

  // Injection of the observed error into the nominal state
  // position
  // new_state.state_vec.segment<3>(StatePositionIdx) =
  //     new_state.state_vec.segment<3>(StatePositionIdx) + errors.segment<3>(EsPositionIdx);

  // vel
  new_state.state_vec.segment<3>(StateLinearVelIdx) =
      new_state.state_vec.segment<3>(StateLinearVelIdx) + errors.segment<3>(EsLinearVelIdx);

  // quaternion
  // Eigen::Quaternionf quat(q(0), q(1), q(1), q(3));
  auto q_error = getQuatFromOmega(errors.segment<3>(EsAngleIdx));
  ERROR << "q_error: " << q_error.coeffs().transpose() << REND;
  auto new_q = q_nominal * q_error;
  new_state.state_vec.segment<4>(StateAngleIdx) = Eigen::Vector4f(new_q.coeffs());

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

  new_state.is_prediction = false;

  WARNING << "obs vel: " << obs.linear_vel.transpose() << REND;
  WARNING << "nomial vel:" << z_nominal.transpose() << "  (" << z_nominal.norm() << ")" << REND;
  WARNING << "obs error: " << delta_z.transpose() << REND;
  WARNING << "ES pose: " << errors.segment<3>(EsPositionIdx).transpose() << REND;
  WARNING << "ES vel: " << errors.segment<3>(EsLinearVelIdx).transpose() << REND;
  WARNING << "ES quat: " << errors.segment<3>(EsAngleIdx).transpose() << REND;
  WARNING << "ES a_bias: " << errors.segment<3>(EsAccBiasIdx).transpose() << REND;
  WARNING << "ES g_bias: " << errors.segment<3>(EsGyroBiasIdx).transpose() << REND;
  WARNING << "ES gravity: " << errors.segment<3>(EsGravityIdx).transpose() << REND;
  printMsfState(new_state);
  return true;
  return true;
}

Observation OdomObserver::getObservation(const OdomData& odom_data)
{
  Observation obs;
  obs.seq = odom_data.seq;
  obs.timestamp = odom_data.timestamp;
  obs.obs_source = name();
  obs.linear_vel = odom_data.linear_vel;
  obs.apply = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
    return applyEkfCorrect(obs, state, new_state);
  };

  return obs;
}

void OdomObserver::observerThread(void)
{
  // TODO: Extrinsic is not consider now
  while (observer_thread_active_.load())
  {
    OdomData odom_data;
    // Retrive odom Observation
    int waiting_num;
    {
      std::unique_lock<std::mutex> odom_lock(odom_queue_mutex_);

      if (odom_queue_.size() == 0)
      {
        odom_available_.wait(odom_lock);
        continue;
      }

      odom_data = odom_queue_.front();
      odom_queue_.pop();
      waiting_num = odom_queue_.size();
    }
    if (waiting_num > 10)
    {
      WARNING << name() << "OdomObserver: more than 10 ODOM data waiting!" << RESET << END;
    }

    // Create Observation Object and Push to queue
    Observation obs = getObservation(odom_data);
    sensor_fusion_->addObservation(obs);
    odom_ready_ = true;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense