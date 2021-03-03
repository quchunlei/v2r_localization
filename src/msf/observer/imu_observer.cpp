#include "msf/observer/imu_observer.h"
#include "common/eigen_util.h"

namespace robosense
{
namespace localization
{
namespace msf
{
ImuObserver::ImuObserver(const std::string name, const YAML::Node& param,
                         const std::shared_ptr<SensorFusion> sensor_fusion)
  : Observer(name, sensor_fusion, param), prev_imu_time_(0)
{
  config(params_);
  /*please explicitly start a observer in RSLocalization*/
  // start();
}

ImuObserver::~ImuObserver()
{
  stop();
}

void ImuObserver::config(const YAML::Node& param)
{
  imu_transform_ = Eigen::Matrix4f::Identity();
  common::yamlRead(param, "acc_noise_sigma", sigma_acc2_, 0);
  common::yamlRead(param, "gyro_noise_sigma", sigma_w2_, 0);
  common::yamlRead(param, "acc_bias_sigma", sigma_acc_bias2_, 0);
  common::yamlRead(param, "gyro_bias_sigma", sigma_w_bias2_, 0);
  common::yamlRead(param, "only_predict_xyz_yaw", IS_3DOF_, false);

  if (IS_3DOF_)
  {
    predictState = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
      return this->predictStateVec2D(obs, state, new_state);
    };

    predictCov = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
      return this->predictStateCov2D(obs, state, new_state);
    };
  }
  else
  {
    predictState = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
      return this->predictStateVec3D(obs, state, new_state);
    };

    predictCov = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
      return this->predictStateCov3D(obs, state, new_state);
    };
  }

  sigma_acc2_ = sigma_acc2_ * sigma_acc2_;                 // variance of acc: m^2 / s^3
  sigma_w2_ = sigma_w2_ * sigma_w2_;                       // variance of angular rate: rad^2 / s^2
  sigma_acc_bias2_ = sigma_acc_bias2_ * sigma_acc_bias2_;  // variance of acc random walk: m^2/(s^5)
  sigma_w_bias2_ = sigma_w_bias2_ * sigma_w_bias2_;        // variance of angular rate random walk: rad^2 / (s^3)

  control_noise_ = Eigen::Matrix<float, 6, 6>::Identity();
  perturb_noise_ = Eigen::Matrix<float, 6, 6>::Identity();

  control_noise_.block<3, 3>(0, 0) = sigma_acc2_ * control_noise_.block<3, 3>(0, 0);
  control_noise_.block<3, 3>(3, 3) = sigma_w2_ * control_noise_.block<3, 3>(3, 3);

  perturb_noise_.block<3, 3>(0, 0) = sigma_acc_bias2_ * perturb_noise_.block<3, 3>(0, 0);
  perturb_noise_.block<3, 3>(3, 3) = sigma_w_bias2_ * perturb_noise_.block<3, 3>(3, 3);
}

void ImuObserver::start(void)
{
  if (observer_thread_active_)
    return;
  // Config Observation
  observer_thread_active_ = true;
  data_queue_max_size_ = 200;
  const auto& func = [this] { observerThread(); };
  observer_thread_ = std::thread(func);

  // Config Imu Thread
  imu_ready_ = false;
}

void ImuObserver::stop(void)
{
  if (observer_thread_active_.load())
  {
    observer_thread_active_ = false;
    imu_available_.notify_one();
    observer_thread_.join();
  }

  imu_ready_ = false;
}

bool ImuObserver::ready(void)
{
  if (imu_ready_.load())
    return true;
  else
    return false;
}

bool ImuObserver::isOutlier(const ImuData& imu)
{
  if (fabs(imu.angular_vel[2] - prev_imu_data_.angular_vel[2]) > 0.5f)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void ImuObserver::addImuData(const ImuData& imu)
{
  double curr_imu_time = imu.timestamp;

  if (!imu_ready_)
  {
    prev_imu_time_ = curr_imu_time;
    prev_imu_data_ = imu;
  }

  double delta_time = curr_imu_time - prev_imu_time_;
  if (delta_time < 0)
  {
    // TODO: Handle error
    ERROR << name() << " IMU data went back to prev time!" << RESET << END;
    return;
  }
  else if (delta_time > 0.1)
  {
    // TODO: Handle error
    ERROR << name() << " IMU data lost more than 10 packets!" << RESET << END;
  }

  // Remove Outlier
  if (isOutlier(imu))
  {
    return;
  }

  std::unique_lock<std::mutex> lock(imu_queue_mutex_);
  imu_queue_.emplace(imu);
  imu_available_.notify_one();
  if (imu_queue_.size() > data_queue_max_size_)
  {
    WARNING << " IMU queue overflow!" << RESET << END;
    // TODO : Handle this error
  }

  prev_imu_data_ = imu;
  prev_imu_time_ = curr_imu_time;
}

Eigen::Vector3f ImuObserver::transformAcc2D(void)
{
  // TODO transform linear acc from imu frame the baselink frame
  // Transforming acc to baselink requires angular acc according to rigid body dynamics, which seems infeasible.
}

EsJacob ImuObserver::getEsJacob(const MsfState& state, const Observation& obs)
{
  auto att = state.state_vec.segment<4>(StateAngleIdx);
  auto a_b = state.state_vec.segment<3>(StateAccBiasIdx);
  auto w_b = state.state_vec.segment<3>(StateGyroBiasIdx);
  Eigen::Quaternionf q(att.data());
  auto R = q.toRotationMatrix();

  EsJacob Fx = EsJacob::Zero();
  Fx.block<3, 3>(0, 3) = Eigen::Matrix3f::Identity();
  Fx.block<3, 3>(3, 6) = -R * getInvSkewMat(obs.linear_acc - a_b);
  Fx.block<3, 3>(3, 9) = -R;
  Fx.block<3, 3>(3, 15) = Eigen::Matrix3f::Identity();
  Fx.block<3, 3>(6, 6) = -getInvSkewMat(obs.angular_vel - w_b);
  Fx.block<3, 3>(6, 12) = -Eigen::Matrix3f::Identity();

  return Fx;
}

bool ImuObserver::predictStateCov3D(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  // States in new_state has been predicted. Now predict covariances
  double dt = new_state.timestamp - state.timestamp;

  EsJacob Fx = getEsJacob(state, obs);  // Jacob of continuous system
  // Use 3rd-orger truncated integration to compute transition matrix of dicrete system
  auto Fdt = Fx * dt;
  auto Fdt2 = Fdt * Fdt;
  auto Fdt3 = Fdt2 * Fdt;
  auto phi = EsJacob::Identity() + Fdt + 0.5 * Fdt2 + (1.0 / 6.0) * Fdt3;

  // Use trapezoidal integration to integration noise covariance
  Eigen::Matrix<float, ErrorStateSize, 12> Fq = Eigen::Matrix<float, ErrorStateSize, 12>::Zero();

  // Calculation of Qd refers to Sola's work(2017, Eq. 448-452)
  auto q_vec = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Matrix3f R = Eigen::Quaternionf(q_vec.data()).normalized().toRotationMatrix();

  // Linearized control matrix B (It's indeed linear in IMU error state situation)
  Eigen::Matrix<float, ErrorStateSize, 6> B = Eigen::Matrix<float, ErrorStateSize, 6>::Zero();
  B.block<3, 3>(3, 0) = -R;
  B.block<3, 3>(6, 3) = -Eigen::Matrix3f::Identity();

  // Perturbation matrix C (related to random walk noises of acc and gyro)

  Eigen::Matrix<float, ErrorStateSize, 6> C = Eigen::Matrix<float, ErrorStateSize, 6>::Zero();
  C.block<6, 6>(9, 0) = Eigen::Matrix<float, 6, 6>::Identity();

  auto Qd = dt * dt * B * control_noise_ * B.transpose() + dt * C * perturb_noise_ * C.transpose();

  new_state.es_cov = phi * (state.es_cov) * phi.transpose() + Qd;
  // //DEBUG << "Fx:\n" << Fx << REND;
  // //DEBUG << "Fq:\n" << Fq << REND;
  // //DEBUG << "B:\n" << B << REND;
  // //DEBUG << "C:\n" << C << REND;
  // //DEBUG << "Control noise:\n" << control_noise_ << REND;
  // //DEBUG << "Perturb noise:\n" << perturb_noise_ << REND;
  // //DEBUG << "dt: " << dt << REND;
  // //DEBUG << std::fixed <<"Qd:\n" << Qd << REND;

  // auto Qd = dt * phi * Fq * noise_cov_ * Fq.transpose() * phi.transpose();  // use zero order integration
  // auto Qd = dt * Fq * noise_cov_ * Fq.transpose() ;  // use zero order integration
  // logger_.log("Preditct --:" ,obs.timestamp, ":\n", new_state.es_cov);
}

bool ImuObserver::predictStateVec3D(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  new_state = state;
  new_state.seq = state.seq + 1;
  new_state.timestamp = obs.timestamp;

  auto& acc_bias = state.state_vec.segment<3>(StateAccBiasIdx);
  auto& gyro_bias = state.state_vec.segment<3>(StateGyroBiasIdx);
  // Update sensor meas
  new_state.state_vec.segment<3>(StateLinearAccIdx) = obs.linear_acc;
  new_state.state_vec.segment<3>(StateAngleVelIdx) = obs.angular_vel;

  auto mid_acc = 0.5 * (obs.linear_acc + state.state_vec.segment<3>(StateLinearAccIdx));
  auto mid_omega = 0.5 * (obs.angular_vel + state.state_vec.segment<3>(StateAngleVelIdx));

  // //DEBUG << "former_meas_acc: " << state.state_vec.segment<3>(StateLinearAccIdx).transpose() << REND;
  // //DEBUG << "former_meas_w  : " << state.state_vec.segment<3>(StateAngleVelIdx).transpose() << REND;
  // //DEBUG << "new_meas_acc: " << new_state.state_vec.segment<3>(StateLinearAccIdx).transpose() << REND;
  // //DEBUG << "new_meas_w: " << new_state.state_vec.segment<3>(StateAngleVelIdx).transpose() << REND;
  // //DEBUG << "ave_am: " << linear_acc.transpose() << REND;
  // //DEBUG << "ave_wm: " << angular_vel.transpose() << REND;

  double dt = new_state.timestamp - state.timestamp;

  // quaternion integration (zero-order)
  Eigen::Vector4f att = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Quaternionf q(att.data());
  Eigen::Vector3f theta = (mid_omega - gyro_bias) * dt;
  Eigen::Quaternionf dq1 = getQuatFromOmega(theta);
  Eigen::Quaternionf new_q = q * dq1.normalized();
  new_state.state_vec.segment<4>(StateAngleIdx) = new_q.coeffs();

  // velocity integration (RK4 method)
  Eigen::Vector3f theta_half = theta * 0.5;  // this will be used in RK4 integration of vel and position
  Eigen::Quaternionf dq_half = getQuatFromOmega(theta_half);
  Eigen::Quaternionf new_q_half = (q * dq_half).normalized();
  auto R = q.toRotationMatrix();  // ger R from q_t or q_t+1 ?
  auto R_half = new_q_half.toRotationMatrix();
  auto R_next = new_q.toRotationMatrix();

  Eigen::Vector3f vel = state.state_vec.segment<3>(StateLinearVelIdx);
  Eigen::Vector3f g = state.state_vec.segment<3>(StateGravityIdx);

  auto v_k1 = R * (obs.linear_acc - acc_bias) + g;
  auto v_k2 = R_half * (obs.linear_acc - acc_bias) + g;
  auto v_k3 = v_k2;  // this is correct, don't worry;
  auto v_k4 = R_next * (obs.linear_acc - acc_bias) + g;
  auto v_next = vel + dt * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4) / 6;
  new_state.state_vec.segment<3>(StateLinearVelIdx) = v_next;

  // DEBUG << "R(acc-a_b) + g = " << v_k1.transpose() << REND;
  // //DEBUG << "a_m - a_b: " << (linear_acc-acc_bias).transpose() << REND;
  // //DEBUG << "w_m - w_b: " << (angular_vel - gyro_bias).transpose() << REND;

  // position integration
  auto p_k1 = vel;
  auto p_k2 = vel + 0.5 * dt * v_k1;
  auto p_k3 = vel + 0.5 * dt * v_k2;
  auto p_k4 = vel + dt * v_k3;
  Eigen::Vector3f position = state.state_vec.segment<3>(StatePositionIdx);
  auto p_next = position + dt * (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4) / 6;
  new_state.state_vec.segment<3>(StatePositionIdx) = p_next;

  // //DEBUG << "meas_w: " << obs.angular_vel.transpose() << REND;
  // //DEBUG << "theta: " << theta.transpose() << REND;
  // //DEBUG << "quat: " << q.coeffs().transpose() << REND;
  // //DEBUG << "d_quat: " << dq.coeffs().transpose() << REND;
  // //DEBUG << "new_quat: " << new_q.coeffs().transpose() << REND;
  // //DEBUG << "----------------------" << REND;
  // //DEBUG << "vel" << new_state.state_vec.segment<3>(StateLinearVelIdx).norm()*3.6 << REND;
  // //DEBUG << "quat: " << q.x() << " " << q.y() << " " << q.z() << " "<< q.w() << REND;
  // //DEBUG << "dq: " << dq.x() << " " << dq.y() << " " << dq.z() << " "<< dq.w() << REND;
  // //DEBUG << "new_q: " << new_state.state_vec.segment<4>(StateAngleIdx).transpose() << REND;
  // printMsfState(new_state);
  return true;
}

bool ImuObserver::predictStateVec2D(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  new_state = state;
  new_state.seq = state.seq + 1;
  new_state.timestamp = obs.timestamp;

  auto& acc_bias = state.state_vec.segment<3>(StateAccBiasIdx);
  auto& gyro_bias = state.state_vec.segment<3>(StateGyroBiasIdx);
  // Update sensor meas
  new_state.state_vec.segment<3>(StateLinearAccIdx) = obs.linear_acc;
  new_state.state_vec.segment<3>(StateAngleVelIdx) = obs.angular_vel;

  auto mid_acc = 0.5 * (obs.linear_acc + state.state_vec.segment<3>(StateLinearAccIdx));
  auto mid_omega = 0.5 * (obs.angular_vel + state.state_vec.segment<3>(StateAngleVelIdx));

  double dt = new_state.timestamp - state.timestamp;

  // quaternion integration (zero-order)
  Eigen::Vector4f att = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Quaternionf q(att.data());
  Eigen::Vector3f theta = (mid_omega - gyro_bias) * dt;
  Eigen::Quaternionf dq1 = getQuatFromOmega(theta);
  Eigen::Quaternionf new_q = q * dq1.normalized();

  // velocity integration (RK4 method)
  Eigen::Vector3f theta_half = theta * 0.5;  // this will be used in RK4 integration of vel and position
  Eigen::Quaternionf dq_half = getQuatFromOmega(theta_half);
  Eigen::Quaternionf new_q_half = (q * dq_half).normalized();
  auto R = q.toRotationMatrix();  // ger R from q_t or q_t+1 ?
  auto R_half = new_q_half.toRotationMatrix();
  auto R_next = new_q.toRotationMatrix();

  Eigen::Vector3f vel = state.state_vec.segment<3>(StateLinearVelIdx);
  Eigen::Vector3f g = state.state_vec.segment<3>(StateGravityIdx);

  auto v_k1 = R * (obs.linear_acc - acc_bias) + g;
  auto v_k2 = R_half * (obs.linear_acc - acc_bias) + g;
  auto v_k3 = v_k2;  // this is correct, don't worry;
  auto v_k4 = R_next * (obs.linear_acc - acc_bias) + g;
  auto v_next = vel + dt * (v_k1 + 2 * v_k2 + 2 * v_k3 + v_k4) / 6;
  new_state.state_vec.segment<3>(StateLinearVelIdx) = v_next;

  // position integration
  auto p_k1 = vel;
  auto p_k2 = vel + 0.5 * dt * v_k1;
  auto p_k3 = vel + 0.5 * dt * v_k2;
  auto p_k4 = vel + dt * v_k3;
  Eigen::Vector3f position = state.state_vec.segment<3>(StatePositionIdx);
  auto p_next = position + dt * (p_k1 + 2 * p_k2 + 2 * p_k3 + p_k4) / 6;
  new_state.state_vec.segment<3>(StatePositionIdx) = p_next;

  // The following is the difference from predictiStateVec3D
  // Extract roll and pitch angle from imu measurement to form a quaternion together with yaw from above process
  Eigen::Vector3f ypr_m = eulerAnglesZYX(Eigen::Quaternionf(obs.angle));  // Don't use Eigen::Matrix::eulerAngles()
  Eigen::Vector3f ypr = eulerAnglesZYX(new_q);

  // clang-format off
  new_q = Eigen::AngleAxisf(ypr(0), Eigen::Vector3f::UnitZ()) * 
          Eigen::AngleAxisf(ypr_m(1), Eigen::Vector3f::UnitY()) *
          Eigen::AngleAxisf(ypr_m(2), Eigen::Vector3f::UnitX());
  // clang-format on
  new_state.state_vec.segment<4>(StateAngleIdx) = quatCoeffsFix(new_q);

  return true;
}

bool ImuObserver::predictStateCov2D(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  // States in new_state has been predicted. Now predict covariances
  double dt = new_state.timestamp - state.timestamp;

  EsJacob Fx = getEsJacob(state, obs);  // Jacob of continuous system
  // Use 3rd-orger truncated integration to compute transition matrix of dicrete system
  auto Fdt = Fx * dt;
  auto Fdt2 = Fdt * Fdt;
  auto Fdt3 = Fdt2 * Fdt;
  auto phi = EsJacob::Identity() + Fdt + 0.5 * Fdt2 + (1.0 / 6.0) * Fdt3;

  // Use trapezoidal integration to integration noise covariance
  Eigen::Matrix<float, ErrorStateSize, 12> Fq = Eigen::Matrix<float, ErrorStateSize, 12>::Zero();

  // Calculation of Qd refers to Sola's work(2017, Eq. 448-452)
  auto q_vec = state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Matrix3f R = Eigen::Quaternionf(q_vec.data()).normalized().toRotationMatrix();

  // Linearized control matrix B (It's indeed linear in IMU error state situation)
  Eigen::Matrix<float, ErrorStateSize, 6> B = Eigen::Matrix<float, ErrorStateSize, 6>::Zero();
  B.block<3, 3>(3, 0) = -R;
  B.block<3, 3>(6, 3) = -Eigen::Matrix3f::Identity();

  // Perturbation matrix C (related to random walk noises of acc and gyro)

  Eigen::Matrix<float, ErrorStateSize, 6> C = Eigen::Matrix<float, ErrorStateSize, 6>::Zero();
  C.block<6, 6>(9, 0) = Eigen::Matrix<float, 6, 6>::Identity();

  auto Qd = dt * dt * B * control_noise_ * B.transpose() + dt * C * perturb_noise_ * C.transpose();
  new_state.es_cov = phi * (state.es_cov) * phi.transpose() + Qd;
}

Observation ImuObserver::getObservation(ImuData& imu_data)
{
  // Transform Imu to base_link

  Eigen::Vector3f linear_acc;

  // linear_acc = transformAcc2D();
  // Do Correction
  // angular_vel = imu_transform_ * angular_vel;
  // Create Observation Object and Push to queue
  Observation obs;
  obs.seq = imu_data.seq;
  obs.timestamp = imu_data.timestamp;
  obs.obs_source = name();

  obs.angle = imu_data.angle;
  obs.linear_acc = imu_data.linear_acc;
  obs.angular_vel = imu_data.angular_vel;

  // IMU noise and perturbation are set as ImuObserver private member
  // But in ImuOdomObserver these are set in this function, which is better?
  obs.apply = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
    bool ret = false;
    ret = predictState(obs, state, new_state);
    ret = predictCov(obs, state, new_state);
    new_state.is_prediction = true;
    return ret;
  };
  return obs;
}

void ImuObserver::observerThread(void)
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
        imu_available_.wait(imu_lock);
        continue;
      }

      imu_data = imu_queue_.front();
      imu_queue_.pop();
      waiting_num = imu_queue_.size();
    }
    if (waiting_num > 10)
    {
      // TODO: Handle this warning
      WARNING << name() << ": more than 10 IMU data waiting!" << RESET << END;
    }

    Observation obs = getObservation(imu_data);
    sensor_fusion_->addObservation(obs);
    imu_ready_ = true;
  }
}

}  // namespace msf
}  // namespace localization
}  // namespace robosense