#include "msf/observer/lidar_icp_observer.h"
#include "common/eigen_util.h"
namespace robosense
{
namespace localization
{
namespace msf
{
// PointCloudViewer viewer("view");
LidarIcpObserver::LidarIcpObserver(const std::string name, const YAML::Node& param,
                                   const std::shared_ptr<SensorFusion> sensor_fusion,
                                   const std::shared_ptr<MapServer> map_server)
  : Observer(name, sensor_fusion, param), map_server_(map_server)

{
  config(params_);
  /*please explicitly start a observer in RSLocalization*/
  // start();
}

LidarIcpObserver::~LidarIcpObserver()
{
  stop();
}

void LidarIcpObserver::config(const YAML::Node& param)
{
  // TODO: load params from yaml file
  if (!param["registration_process"])
  {
    ERROR << name() << " Registration process not defined in param file" << RESET << END;
    exit(-1);
  }

  common::yamlRead(param, "heading_position_noise", heading_pos_noise_, 0.2);
  common::yamlRead(param, "side_position_noise", side_pos_noise_, 0.05);
  common::yamlRead(param, "angle_noise", angle_noise_, 0.01);
  common::yamlRead(param, "lidare_failure_threshold", watchdog_thresh_, 5.0);

  icp_localization_ = std::make_unique<ICPLocalization>(map_server_, param["registration_process"]);
  num_scan_per_local_map_ = 2;
  /*
    PointMatcherSupport::Parametrizable::Parameters params;
    std::string name;

    // Prepare matching function
    name = "KDTreeMatcher";
    params["knn"] = "1";
    params["maxDist"] = "2";
    params["epsilon"] = "1";
    std::shared_ptr<PM::Matcher> kdtree = PM::get().MatcherRegistrar.create(name, params);
    params.clear();

    // Prepare outlier filters
    // name = "TrimmedDistOutlierFilter";
    // params["ratio"] = "0.85";
    // std::shared_ptr<PM::OutlierFilter> trim = PM::get().OutlierFilterRegistrar.create(name, params);
    // params.clear();

    // Prepare error minimization
    name = "PointToPointErrorMinimizer";
   std::shared_ptr< PM::ErrorMinimizer> point_to_point = PM::get().ErrorMinimizerRegistrar.create(name);
    params.clear();

    name = "CounterTransformationChecker";
    params["maxIterationCount"] = "2";
    std::shared_ptr<PM::TransformationChecker> max_iter = PM::get().TransformationCheckerRegistrar.create(name, params);
    params.clear();

    // Prepare inspector
    std::shared_ptr<PM::Inspector> null_inspect = PM::get().InspectorRegistrar.create("NullInspector");

    // Prepare transformation
    std::shared_ptr<PM::Transformation> rigid_trans = PM::get().TransformationRegistrar.create("RigidTransformation");

    icp_.matcher.reset(kdtree);
    // icp_.outlierFilters.push_back(trim);
    icp_.errorMinimizer.reset(point_to_point);
    icp_.transformationCheckers.push_back(max_iter);
    icp_.inspector.reset(null_inspect);
    icp_.transformations.push_back(rigid_trans);

    name = "RemoveNaNDataPointsFilter";
    std::shared_ptr<PM::DataPointsFilter> removenan_filter = PM::get().DataPointsFilterRegistrar.create(name);

    name = "OctreeGridDataPointsFilter";
    params["buildParallel"] = "1";
    params["maxPointByNode"] = "1";
    params["maxSizeByNode"] = "0.5";
    params["samplingMethod"] = "3";
    std::shared_ptr<PM::DataPointsFilter> octree_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    name = "MaxPointCountDataPointsFilter";
    params["seed"] = "414";
    params["maxCount"] = "30000";
    std::shared_ptr<PM::DataPointsFilter> maxcount_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
    params.clear();

    filter_.push_back(removenan_filter);
    filter_.push_back(maxcount_filter);
    filter_.push_back(octree_filter);
   */
}

void LidarIcpObserver::start(void)
{
  observer_thread_active_ = true;
  data_queue_max_size_ = 1;
  const auto& func = [this] { observerThread(); };
  observer_thread_ = std::thread(func);

  obs_ready_ = false;  // set to false initially
}

void LidarIcpObserver::stop(void)
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

bool LidarIcpObserver::ready(void)
{
  return obs_ready_.load();
}

bool LidarIcpObserver::isOutlier(const LidarData* const data) const
{
  return false;
}

void LidarIcpObserver::addLidarData(const LidarData& data)
{
  double curr_data_time = data.timestamp;
  static double prev_data_time = curr_data_time;
  double delta_time = curr_data_time - prev_data_time;
  status_.store(SensorConnected);

  if (delta_time < 0)
  {
    // TODO: handle error
    ERROR << name() << " ICP data went back to prev time!" << RESET << END;
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
    WARNING << " ICP queue overflow!" << RESET << END;
    data_queue_.pop();
    // TODO: handle this error
  }

  prev_data_time = curr_data_time;

  // std::cout << "addLidarData \n\r";
  return;
}

bool LidarIcpObserver::applyEkfCorrect(const Observation& obs, const MsfState& state, MsfState& new_state)
{
  if (obs.timestamp != state.timestamp)
  {
    ERROR << "[correction]: Observation's timestamp is not aligned with that of input state!!!" << RESET << END;
    exit(-1);
  }

  static double prev_time = obs.timestamp;
  double dt = (obs.timestamp == prev_time) ? 0.1 : obs.timestamp - prev_time;
  prev_time = obs.timestamp;

  // Eigen::Vector3f noise(heading_pos_noise_, side_pos_noise_, 0);
  // std::cout << "noise vec = \n\r" << noise << std::endl;
  // Eigen::Vector3f heading_dir = Eigen::AngleAxisf(obs.angle(2), Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(1, 0, 0);
  // Eigen::Vector3f side_dir = Eigen::AngleAxisf(obs.angle(2), Eigen::Vector3f::UnitZ()) * Eigen::Vector3f(0, 1, 0);

  Eigen::Quaternionf heading_dir_quat(obs.angle(3), obs.angle(0), obs.angle(1), obs.angle(2));

  Eigen::Vector3f heading_vec = heading_dir_quat.toRotationMatrix() * Eigen::Vector3f::UnitX();
  Eigen::Vector3f side_vec = heading_dir_quat.toRotationMatrix() * Eigen::Vector3f::UnitY();
  Eigen::Vector3f top_vec = heading_dir_quat.toRotationMatrix() * Eigen::Vector3f::UnitZ();

  Eigen::Vector3f noise = Eigen::Vector3f::Zero();

  float hx2 = heading_vec.x() * heading_vec.x();
  float sx2 = side_vec.x() * side_vec.x();
  float tx2 = top_vec.x() * top_vec.x();

  noise.x() = 1 / (hx2 / heading_pos_noise_ / heading_pos_noise_ + (1 - hx2) / side_pos_noise_ / side_pos_noise_);
  noise.y() = 1 / (sx2 / heading_pos_noise_ / heading_pos_noise_ + (1 - sx2) / side_pos_noise_ / side_pos_noise_);
  noise.z() = 1 / (tx2 / heading_pos_noise_ / heading_pos_noise_ + (1 - tx2) / side_pos_noise_ / side_pos_noise_);

  // std::cout << "heading_pos_noise_ " << heading_pos_noise_ << std::endl;
  // std::cout << "side_pos_noise_ " << side_pos_noise_ << std::endl;
  // std::cout << "hx2 =  " << hx2 << " sx2 = " << sx2 << " tx2 = " << tx2 << std::endl;
  // std::cout << "orientation " << obs.angle << std::endl;
  // std::cout << "noise vec after rotate " << noise << std::endl;

  EkfHMat H = EkfHMat::Zero();
  H.block<3, 3>(ObservationPositionIdx, StatePositionIdx) = Eigen::Matrix3f::Identity();
  H.block<4, 4>(ObservationAngleIdx, StateAngleIdx) = Eigen::Matrix4f::Identity();

  // TODO :: Pitch and row from icp is very inaccurate now
  EkfQMat Q = EkfQMat::Identity();
  Q.block<3, 3>(ObservationPositionIdx, ObservationPositionIdx) = (noise * dt * dt).asDiagonal();
  Q(ObservationAngleIdx, ObservationAngleIdx) = angle_noise_ * angle_noise_ * dt * dt * 25;
  Q(ObservationAngleIdx + 1, ObservationAngleIdx + 1) = angle_noise_ * angle_noise_ * dt * dt * 25;
  Q(ObservationAngleIdx + 2, ObservationAngleIdx + 2) = angle_noise_ * angle_noise_ * dt * dt;
  Q(ObservationAngleIdx + 3, ObservationAngleIdx + 3) = 1e-10 * 1e-10 * dt * dt;

  new_state = state;
  EkfKMat K = state.cov * H.transpose() * (H * state.cov * H.transpose() + Q).inverse();

  // TODO : The equation in papar is very strange try to figure out why
  new_state.cov = (EkfCovMat::Identity() - K * H) * state.cov;

  EkfMeasErrVec r = EkfMeasErrVec::Zero();
  r.segment<3>(ObservationPositionIdx) = obs.pos - state.state_vec.segment<3>(StatePositionIdx);

  // Eigen::Quaternionf obs_q(Eigen::AngleAxisf(obs.angle(2), Eigen::Vector3f::UnitZ()));
  Eigen::Quaternionf obs_q(obs.angle(3), obs.angle(0), obs.angle(1), obs.angle(2));
  obs_q.normalize();
  Eigen::Quaternionf pre_q(state.state_vec(StateAngleIdx + 3), state.state_vec(StateAngleIdx),
                           state.state_vec(StateAngleIdx + 1), state.state_vec(StateAngleIdx + 2));
  pre_q.normalize();

  // TODO: This is different from ETH_MSF but same as paper
  Eigen::Quaternionf diff_q = (obs_q * pre_q.conjugate()).normalized();

  // std::cout << "\n\robs yaw = \n\r" << obs.angle(2) << std::endl;
  // std::cout << "obs_q = \n\r" << obs_q.coeffs() << std::endl;

  // std::cout << "pre_q = \n\r" << pre_q.coeffs() << std::endl;
  // std::cout << "err_q = \n\r" << err_q.coeffs() << std::endl;
  r.segment<3>(ObservationAngleIdx) = diff_q.vec() / diff_q.w() * 2.0;

  EkfStateErrVec correction = K * r;

  // std::cout << "correction = \n\r" << correction << std::endl;
  // std::cout << "K = \n\r" << K << std::endl;
  // std::cout << "r = \n\r" << r << std::endl;

  new_state.state_vec.segment<3>(StatePositionIdx) += correction.segment<3>(StatePositionIdx);
  // new_state.state_vec.block<3, 1>(StatePositionIdx, 0) = obs.pos;

  Eigen::Vector4f quat_vec = new_state.state_vec.segment<4>(StateAngleIdx);
  Eigen::Quaterniond quat(quat_vec(3), quat_vec(0), quat_vec(1), quat_vec(2));
  Eigen::Quaterniond correct_quat = smallAngleQuat(correction.segment<3>(StateAngleIdx));
  quat = correct_quat * quat;  // TODO: Use pre_q is easier to understand
  quat.normalize();

  // new_state.state_vec.segment<4>(StateAngleIdx) << quat.x(), quat.y(), quat.z(), quat.w();
  new_state.state_vec.block<4, 1>(StateAngleIdx, 0) << obs_q.x(), obs_q.y(), obs_q.z(), obs_q.w();

  new_state.is_prediction = false;
  return true;
}

bool LidarIcpObserver::getObservation(const LidarData* data, Observation& obs)
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

  Eigen::Quaternionf quat(init_state.state_vec(StateAngleIdx + 3), init_state.state_vec(StateAngleIdx),
                          init_state.state_vec(StateAngleIdx + 1), init_state.state_vec(StateAngleIdx + 2));
  quat.normalize();

  Eigen::Matrix3f rot(quat);
  init_pose.block<3, 3>(0, 0) = rot;
  init_pose.block<3, 1>(0, 3) = init_state.state_vec.block<3, 1>(StatePositionIdx, 0);

  // TODO(zyc): Find a way donot need mapping odom for height
  // TODO(zyc): Verify work for multi level road

  // TODO : 3D height is not accurate
  // init_pose(2, 3) = 2.0;
  // map_server_->getHeight(init_pose(0, 3), init_pose(1, 3), 0, init_pose(2, 3));

  if (!icp_localization_->locate(std::move(data->scan), init_pose, measure_pose, measure_cov))
    return false;

  std::cout << "\n\rinit_state is prediction = " << init_state.is_prediction;
  std::cout << "\n\rinit_state = \n\r" << init_state.state_vec << std::endl;
  std::cout << "\n\rinit_pose = \n\r" << init_pose << "\n\rmeasure_pose = \n\r" << measure_pose << std::endl;
  obs.seq = data->seq;
  obs.timestamp = data->timestamp;
  obs.obs_source = name();

  obs.pos = measure_pose.block<3, 1>(0, 3);

  // TODO : 3D height is not accurate
  // obs.pos(2) = 2.0;
  // map_server_->getHeight(obs.pos(0), obs.pos(1), 0, obs.pos(2));

  obs.pos_cov = measure_cov.block<3, 3>(0, 0);

  Eigen::Quaternionf obs_quat(measure_pose.block<3, 3>(0, 0));
  obs.angle << obs_quat.x(), obs_quat.y(), obs_quat.z(), obs_quat.w();
  obs.angle_cov.block<3, 3>(0, 0) = measure_cov.block<3, 3>(3, 3);
  obs.angle_cov(4, 4) = 1e-10;

  {
    static Observation last_obs = obs;
    if (last_obs.timestamp == obs.timestamp)
      obs.linear_vel = Eigen::Vector3f::Zero();
    else
      obs.linear_vel = ((obs.pos - last_obs.pos) * 10.0).cast<float>();
    last_obs = obs;
  }

  obs.apply = [this](const Observation& obs, const MsfState& state, MsfState& new_state) {
    return applyEkfCorrect(obs, state, new_state);
  };

  return true;
}

void LidarIcpObserver::observerThread(void)
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
}  // namespace msf

bool LidarIcpObserver::updateSubMap(const LidarData* scan)
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
    local_map_ = std::make_unique<LidarData>(*scan);
    prev_local_map_ = std::make_unique<LidarData>(*scan);
    local_map_pose_ = pose;
    rigid_trans_ = PM::get().TransformationRegistrar.create("RigidTransformation");
    return false;
  }

  Eigen::Matrix4f tf = pose.inverse() * local_map_pose_;

  // viewer.setPointCloud(prev_local_map_->scan_, Eigen::Vector3i(0, 0, 250), "scan");
  // viewer.setPointCloud(scan.scan_, Eigen::Vector3i(0, 250, 0), "scan");

  // tf = icp_(scan.scan_, prev_local_map_->scan_, tf.inverse());
  // tf = tf.inverse();

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
