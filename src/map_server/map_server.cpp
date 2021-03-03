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

#include "map_server/map_server.h"
#include "yaml/yaml.h"
namespace robosense
{
namespace localization
{
struct vtkPoint
{
  int kind1;
  int kind2;
  double pose_x;
  double pose_y;
  double pose_z;
};

// TODO: now it only supports one descriptor normals.
// use template to support arbitrary number of descriptors with different dimentions.
struct vtkPointNormal : vtkPoint
{
  float normal_x;
  float normal_y;
  float normal_z;
};

struct rsPoint
{
  int kind1;
  int kind2;
  float x;
  float y;
  float z;
};

struct rsPointNormal : rsPoint
{
  float normal_x;
  float normal_y;
  float normal_z;
};

struct rsPointXYZINormal : rsPointNormal
{
  float intensity;
};

struct mapHeader
{
  char version[10];
  char height[10];
  char nb_points[100];
  char point_type[100];
};

MapServer::MapServer()
{
  init();
}

MapServer::MapServer(const YAML::Node& param)
{
  params_ = param;
  init();
  loadParams(param);
  loadMap(param);
}

bool MapServer::init(void)
{
  error_code_ = 0;
  is_gps_msg_ = false;
  is_odom_map_msg_ = false;
  is_map_with_normal_ = false;
  gps_origin_ = Eigen::Vector3d::Zero();
  icp_points_.clear();
  poles_points_.clear();
  vec_key_frames_.clear();
  kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  mapping_odom_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  icp_points_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  poles_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  last_pose_ << 0, 0, 0;
  is_first_time_ = true;
  return true;
}

bool MapServer::loadParams(const YAML::Node& param)
{
  common::yamlRead(param, "debug", debug_, false);

  common::yamlRead(param, "height_search_radius", height_search_radius_, 30.0f);

  common::yamlRead(param, "map_cell_length", map_cell_length_, 10.0f);
  common::yamlRead(param, "local_map_radius", local_map_radius_, 100.0f);
  common::yamlRead(param, "t_dis_update_localmap", t_dis_update_localmap_, 10.0f);
  common::yamlRead(param, "display_definition", disp_def_level_, 10.0f);
  common::yamlRead(param, "lidar_height_from_ground", lidar_height_, 2.0f);

  global_point_cloud_map_.loadConfigration(map_cell_length_, local_map_radius_);
  return true;
}

bool MapServer::loadMap(const YAML::Node& param)
{
  // TODO : Make sure thread safe, especially when reloading a map
  std::string map_file;
  bool is_map_with_normal = false;
  if (!common::yamlRead(param, "map_file", map_file))
  {
    ERROR << "[>> " << name() << " ] Map path not set!" << RESET << END;
    exit(-1);
  }
  // auto t1 = ros::WallTime::now();
  DP all_map;  // The map with all features
  if (!loadPCDMap(map_file, all_map))
  {
    ERROR << "[>> " << name() << " ] Load map failed!" << RESET << END;
    exit(-1);
  }
  // auto t2 = ros::WallTime::now();
  // std::cout << "process time     : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  // auto t3 = ros::WallTime::now();
  DP global_point_cloud = all_map.createSimilarEmpty();
  DP map_for_display = all_map.createSimilarEmpty();
  background_ = all_map.createSimilarEmpty();
  int global_point_cloud_cnt = 0;
  int new_points = 0;
  int count_map_for_display = 0;

  View view(all_map.getDescriptorViewByName("kind"));
  int vec_key_frames_max_idx = 0;
  for (int i = 0; i < all_map.features.cols(); ++i)
  {
    auto const& feature = all_map.features.col(i);
    if (view(0, i) == 0)  // background
    {
      global_point_cloud.setColFrom(global_point_cloud_cnt++, all_map, i);
      background_.setColFrom(new_points++, all_map, i);
    }
    else if (view(0, i) == 8)
    {
      map_for_display.setColFrom(count_map_for_display++, all_map, i);
    }
    else if (view(0, i) == 1)  // poles
    {
      poles_points_.emplace_back(feature.x(), feature.y());
      poles_cloud_->push_back(pcl::PointXYZ(feature.x(), feature.y(), 0));
    }
    else if (view(0, i) == 2)  // mapping odom
    {
      pcl::PointXYZI tmp_point;
      tmp_point.x = feature.x();
      tmp_point.y = feature.y();
      tmp_point.z = feature.z();
      tmp_point.intensity = 0;
      mapping_odom_cloud_->push_back(tmp_point);
    }
    else if (view(0, i) == 3)  // ICP points
    {
      icp_points_.emplace_back(feature.x(), feature.y());

      pcl::PointXYZI tmp_icp_point;
      tmp_icp_point.x = feature.x();
      tmp_icp_point.y = feature.y();
      tmp_icp_point.z = -1.05;
      tmp_icp_point.intensity = 0;
      icp_points_cloud_->push_back(tmp_icp_point);
    }
    else if (view(0, i) == 4)  // Map origin in GPS Coordinates
    {
      // lat, lon, alt
      gps_origin_ << Eigen::Vector3d(feature.x(), feature.y(), 1.0f);
      // INFO << "GPS Origin: " << gps_origin_.transpose() << REND;
      if (fabs(gps_origin_[0]) < 0.1)
      {
        gps_origin_[0] = 0.0;
        gps_origin_[1] = 0.0;
        gps_origin_[2] = 0.0;
      }
      is_gps_msg_ = true;
    }
    else if (view(0, i) == 6)  // ICP key frames
    {
      vec_key_frames_max_idx = std::max<int>(view(1, i), vec_key_frames_max_idx);
    }
  }

  vec_key_frames_.resize(vec_key_frames_max_idx + 1);
  for (size_t j = 0; j < vec_key_frames_.size(); ++j)
  {
    vec_key_frames_[j] = KeyFramePtr(new KeyFrame);
  }

  for (int i = 0; i < all_map.features.cols(); ++i)
  {
    auto const& feature = all_map.features.col(i);
    if (view(0, i) == 5)  // keyframe
    {
      pcl::PointXYZI tmp_point;
      tmp_point.x = feature.x();
      tmp_point.y = feature.y();
      tmp_point.z = feature.z();
      tmp_point.intensity = 0;
      vec_key_frames_[view(1, i)]->cloud_points->push_back(tmp_point);
      if (!is_map_with_normal_)
        global_point_cloud.setColFrom(global_point_cloud_cnt++, all_map, i);
      // background_.setColFrom(new_points++, all_map, i);
    }
    else if (view(0, i) == 6)
    {
      vec_key_frames_[view(1, i)]->trans_x = feature.x();
      vec_key_frames_[view(1, i)]->trans_y = feature.y();
      vec_key_frames_[view(1, i)]->trans_z = feature.z();
    }
    else if (view(0, i) == 7)
    {
      vec_key_frames_[view(1, i)]->roll = feature.x();
      vec_key_frames_[view(1, i)]->pitch = feature.y();
      vec_key_frames_[view(1, i)]->yaw = feature.z();
    }
  }

  // auto t4 = ros::WallTime::now();
  // std::cout << "11 process time     : " << (t4 - t3).toSec() * 1000 << "[msec]" << std::endl;

  // auto t5 = ros::WallTime::now();
  /************** Process global_point_cloud_map_  *************************/
  global_point_cloud.conservativeResize(global_point_cloud_cnt);

  // viewer.setPointCloud(global_point_cloud_no_discriptor,Eigen::Vector3i(250,0,0),"before");

  if (debug_)
    INFO << name() << ": Number of points before filter " << global_point_cloud.getNbPoints() << RESET << END;

  removeInvalidNormal(global_point_cloud);
  applyGlobalMapFilter(global_point_cloud);

  if (debug_)
    INFO << name() << ": Number of points after filter " << global_point_cloud.getNbPoints() << RESET << END;

  global_point_cloud.removeDescriptor("kind");
  if (global_point_cloud.descriptorLabels.size() == 0)
  {
    ERROR << "No descriptors in map, please check your point filter configuaration.\n" << REND;
    exit(0);
  }
  // viewer.setPointCloud(global_point_cloud_no_discriptor,Eigen::Vector3i(0,250,0),"after");

  global_point_cloud_map_.setGlobalMap(global_point_cloud);

  /************** Process key frames **************************************/
  for (auto& item : vec_key_frames_)
  {
    item->trans_pose =
        pcl::getTransformation(item->trans_x, item->trans_y, item->trans_z, item->roll, item->pitch, item->yaw)
            .matrix();
  }

  /****************** Process background **********************************/

  if (count_map_for_display == 0)
  {
    background_.conservativeResize(new_points);
  }
  else
  {
    background_ = map_for_display;
    background_.conservativeResize(count_map_for_display);
  }

  applyBackgroundMapFilter(background_);

  /****************** Process mapping odom and kdtree *********************/
  if (!mapping_odom_cloud_->empty())
  {
    kdtree_->setInputCloud(mapping_odom_cloud_);
    is_odom_map_msg_ = true;
  }

  /******************* Add tree height ************************************/
  float tree_height = 0;
  for (int i = 0; i < poles_cloud_->size(); ++i)
  {
    if (getHeight(poles_cloud_->points[i].x, poles_cloud_->points[i].y, tree_height, tree_height))
    {
      poles_cloud_->points[i].z = tree_height;
    }
    else
    {
      WARNING << "[>> " << name() << "] The height is zero!" << RESET << END;
    }
  }

  // auto t6 = ros::WallTime::now();
  // std::cout << "22 process time     : " << (t6 - t5).toSec() * 1000 << "[msec]" << std::endl;
  return true;
}

bool MapServer::loadPCDMap(const std::string& input_file, DP& map_out)
{
  std::ifstream ifile(input_file, std::ios::in | std::ios::binary);
  if (!ifile)
  {
    ERROR << "[>> " << name() << " ] pcd file does not exist!" << RESET << END;
    exit(-1);
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI>(input_file, *cloud);
  // conversation
  sensor_msgs::PointCloud2 background_map_ros;
  pcl::toROSMsg(*cloud, background_map_ros);
  // std::cout << "load pcd file: " << input_file << std::endl;
  DP raw_background_map = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(background_map_ros, true);
  // std::cout << "to point matcher: " << std::endl;
  LoadDPFilter();
  // std::cout << "DP filter: " << std::endl;
  // auto t1 = ros::WallTime::now();

  local_map_filter_.apply(raw_background_map);
  DP background_map = raw_background_map;

  ////////////////////////////////////////////////////////////////////////
  local_map_filter_.apply(background_map);
  map_normal_filter_.apply(background_map);
  local_map_octree_filter_.apply(background_map);

  // auto t2 = ros::WallTime::now();
  // std::cout << "loadPCDMap filter process time     : " << (t2 - t1).toSec() * 1000 << "[msec]" << std::endl;

  auto t3 = ros::WallTime::now();

  int points_count = 0;
  points_count += background_map.getNbPoints();
  points_count++;

  Eigen::Matrix4Xf vec_points(4, points_count);

  Eigen::Matrix2Xf vec_kinds(2, points_count);

  Eigen::Matrix3Xf vec_normals(3, points_count);

  int points_idx = 0;

  View normal_view = background_map.getDescriptorViewByName("normals");

  for (size_t i = 0; i < background_map.features.cols(); ++i, points_idx++)
  {
    vec_points(0, points_idx) = background_map.features(0, i);
    vec_points(1, points_idx) = background_map.features(1, i);
    vec_points(2, points_idx) = background_map.features(2, i);
    vec_points(3, points_idx) = 1.0;

    vec_kinds(0, points_idx) = 0;
    vec_kinds(1, points_idx) = 0;

    vec_normals(0, points_idx) = normal_view(0, i);
    vec_normals(1, points_idx) = normal_view(1, i);
    vec_normals(2, points_idx) = normal_view(2, i);
  }

  vec_points(0, points_idx) = 0;
  vec_points(1, points_idx) = 0;
  vec_points(2, points_idx) = 0;
  vec_points(3, points_idx) = 1.0;

  vec_normals(0, points_idx) = 0;
  vec_normals(1, points_idx) = 0;
  vec_normals(2, points_idx) = 0;

  vec_kinds(0, points_idx) = 4;
  vec_kinds(1, points_idx++) = 0;

  // map_out.addFeature("points", vec_points);
  map_out.addFeature("x", vec_points.row(0));
  map_out.addFeature("y", vec_points.row(1));
  map_out.addFeature("z", vec_points.row(2));
  map_out.addFeature("pad", vec_points.row(3));
  map_out.addDescriptor("kind", vec_kinds);
  map_out.addDescriptor("normals", vec_normals);

  // auto t4 = ros::WallTime::now();
  // std::cout << "loadPCDMap filter process time     : " << (t4 - t3).toSec() * 1000 << "[msec]" << std::endl;
  return true;
}

void MapServer::LoadDPFilter()
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // reading filter
  name = "RemoveNaNDataPointsFilter";
  std::shared_ptr<PM::DataPointsFilter> removenan_filter = PM::get().DataPointsFilterRegistrar.create(name);

  // name = "RandomSamplingDataPointsFilter";
  // params["prob"] = "0.9";
  // PM::DataPointsFilter* randomsample_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  // params.clear();

  name = "OctreeGridDataPointsFilter";
  params["buildParallel"] = "1";
  params["maxPointByNode"] = "1";
  params["maxSizeByNode"] = "0.4";
  params["samplingMethod"] = "3";
  std::shared_ptr<PM::DataPointsFilter> octree_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  local_map_filter_.push_back(removenan_filter);
  local_map_filter_.push_back(octree_filter);

  name = "SurfaceNormalDataPointsFilter";
  params["knn"] = "8";
  params["maxDist"] = "2";
  params["epsilon"] = "0";
  params["keepNormals"] = "1";
  params["keepDensities"] = "0";
  params["keepEigenValues"] = "0";
  params["keepEigenVectors"] = "0";
  params["smoothNormals"] = "0";
  std::shared_ptr<PM::DataPointsFilter> surfacenormal_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  map_normal_filter_.push_back(surfacenormal_filter);

  name = "OctreeGridDataPointsFilter";
  params["buildParallel"] = "1";
  params["maxPointByNode"] = "1";
  params["maxSizeByNode"] = "0.8";
  params["samplingMethod"] = "3";
  std::shared_ptr<PM::DataPointsFilter> final_octree_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  local_map_octree_filter_.push_back(final_octree_filter);
}

const DP& MapServer::getLocalMap(const Eigen::Vector3f& pose)
{
  float update_th = t_dis_update_localmap_;

  if (is_first_time_)
  {
    global_point_cloud_map_.getLocalMap(pose, local_point_cloud_map_);
    last_pose_ = pose;
    is_first_time_ = false;
  }
  else if (pow((last_pose_[0] - pose[0]), 2) + pow((last_pose_[1] - pose[1]), 2) > pow(update_th, 2))
  {
    global_point_cloud_map_.getLocalMap(pose, local_point_cloud_map_);
    last_pose_ = pose;
  }

  return local_point_cloud_map_;
}

void MapServer::configDefaultPointFilter(PM::DataPointsFilters& filter)
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;

  // reading filter
  name = "RemoveNaNDataPointsFilter";
  std::shared_ptr<PM::DataPointsFilter> removenan_filter = PM::get().DataPointsFilterRegistrar.create(name);

  name = "OctreeGridDataPointsFilter";
  params["buildParallel"] = "1";
  params["maxPointByNode"] = "1";
  params["maxSizeByNode"] = "0.2";
  params["samplingMethod"] = "3";
  std::shared_ptr<PM::DataPointsFilter> octree_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  name = "SurfaceNormalDataPointsFilter";
  params["knn"] = "8";
  params["maxDist"] = "2";
  params["epsilon"] = "0";
  params["keepNormals"] = "1";
  params["keepDensities"] = "0";
  params["keepEigenValues"] = "0";
  params["keepEigenVectors"] = "0";
  params["smoothNormals"] = "1";
  std::shared_ptr<PM::DataPointsFilter> surfacenormal_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  name = "OctreeGridDataPointsFilter";
  params["buildParallel"] = "1";
  params["maxPointByNode"] = "1";
  params["maxSizeByNode"] = "3";
  params["samplingMethod"] = "3";
  std::shared_ptr<PM::DataPointsFilter> octree_filter1 = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  filter.push_back(removenan_filter);
  filter.push_back(octree_filter);
  filter.push_back(surfacenormal_filter);
  filter.push_back(octree_filter1);
}

void MapServer::removeInvalidNormal(DP& point_cloud)
{
  if (!point_cloud.descriptorExists("normals"))
  {
    is_map_with_normal_ = false;
    return;
  }

  View normal_view = point_cloud.getDescriptorViewByName("normals");
  if (normal_view.row(2).norm() / normal_view.cols() < 1e-6)
  {
    error_code_ = -1;
    point_cloud.removeDescriptor("normals");
    is_map_with_normal_ = false;

    WARNING << "\nInvalid normal descriptors are detected in the current map. " << REND;
    WARNING << "This will slow down the map loading process." << REND;
    WARNING << "It's recommanded to use a map with valid normal descriptors\n" << REND;
  }
}

bool MapServer::isFilterExist(const YAML::Node& filter_chain, const std::string)
{
  if (!filter_chain)
    return false;

  for (std::size_t i = 0; i < filter_chain.size(); i++)
  {
    YAML::Node iterm = filter_chain[i];
    std::string name;
    // ERROR << iterm << REND;
    if (iterm.Type() == YAML::NodeType::Map)
    {
      name = iterm.begin()->first.as<std::string>();
      std::size_t found = name.find("SurfaceNormal");
      if (found != std::string::npos)
      {
        return true;
      }
    }
    else
    {
      name = filter_chain[i].as<std::string>();
      std::size_t found = name.find("SurfaceNormal");
      if (found != std::string::npos)
      {
        return true;
      }
    }
  }

  return false;
}

void MapServer::applyGlobalMapFilter(DP& point_cloud)
{
  if (point_cloud.getNbPoints() <= 0)
  {
    ERROR << "[>> " << name() << " ] global map empty!" << RESET << END;
    exit(-1);
  }
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;
  PM::DataPointsFilters filter;

  // TODO: deal with exception when invalid YAML node provided
  // this may be caused by missing of parameter file, include error, etc.
  YAML::Node filter_chain = params_["filter_chain"];
  auto if_normal_filter_exist = isFilterExist(filter_chain, "SurfaceNormal");

  if (!filter_chain)
  {
    WARNING << "\nNo filter_chain specified in the configuration file" << REND;
    WARNING << "Using default map filtering method." << REND;
    WARNING << "You may want to modify your configuration file and relaunch.\n" << REND;
    configDefaultPointFilter(filter);
  }
  else if (!is_map_with_normal_ && !if_normal_filter_exist)
  {
    // TODO add normal in pcd map
    // WARNING << "\nNo normals in the map and no SurfaceNormal filter configured. " << REND;
    // WARNING << "Using default map filtering method." << REND;
    // WARNING << "You may want to modify your configuration file and relaunch.\n" << REND;
    configDefaultPointFilter(filter);
  }
  else
  {
    // WARNING << filter_chain << REND;
    if (is_map_with_normal_ && if_normal_filter_exist)
    {
      WARNING << "\nDoing surface normal filtering on a map with surface normals. " << REND;
      WARNING << "Ignore this warning if this is what you want. Otherwise, change your map_server configuration.\n"
              << REND;
    }
    for (std::size_t i = 0; i < filter_chain.size(); i++)
    {
      YAML::Node iterm = filter_chain[i];
      if (iterm.Type() == YAML::NodeType::Map)
      {
        for (YAML::iterator it = iterm.begin(); it != iterm.end(); ++it)
        {
          name = it->first.as<std::string>();
          YAML::Node map = it->second;
          for (YAML::iterator it_map = map.begin(); it_map != map.end(); ++it_map)
          {
            std::string param_name(it_map->first.as<std::string>());
            std::string value_name(it_map->second.as<std::string>());
            // WARNING << param_name << ": " << value_name << RESET << END;
            params[param_name] = value_name;
          }
        }
        auto single_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
        filter.push_back(single_filter);
      }
      else
      {
        name = filter_chain[i].as<std::string>();
        params.clear();
        auto single_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
        filter.push_back(single_filter);
      }
      params.clear();
    }
  }
  filter.apply(point_cloud);
  if (debug_)
  {
    INFO << this->name() << ": features " << point_cloud.featureLabels.size() << "descriptor "
         << point_cloud.descriptorLabels.size() << RESET << END;
  }
}

void MapServer::applyBackgroundMapFilter(DP& background)
{
  PointMatcherSupport::Parametrizable::Parameters params;
  std::string name;
  auto nb_points = background.getNbPoints();

  name = "MaxPointCountDataPointsFilter";
  auto max_count = static_cast<unsigned int>(nb_points * 0.1 * disp_def_level_);
  params["maxCount"] = std::to_string(max_count);
  std::shared_ptr<PM::DataPointsFilter> max_filter = PM::get().DataPointsFilterRegistrar.create(name, params);
  params.clear();

  PM::DataPointsFilters filter;
  filter.push_back(max_filter);
  filter.apply(background);
}

bool MapServer::getGnssOrigin(Eigen::Vector3d& gps_origin)
{
  if (is_gps_msg_)
  {
    gps_origin = gps_origin_;
    return true;
  }
  else
  {
    ERROR << "[>> " << name() << " ] Can't get gps origin!" << RESET << END;
    return false;
  }
}

bool MapServer::getHeight(const float& x, const float& y, const float& z, float& car_height)
{
  if (is_odom_map_msg_)
  {
    float car_z = 0;
    // int z_numbers = 0;
    pcl::PointXYZI searchPoint;
    searchPoint.x = x;
    searchPoint.y = y;
    searchPoint.z = z;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> a;
    if (kdtree_->radiusSearch(searchPoint, height_search_radius_, pointIdxRadiusSearch, a) > 0)
    {
      for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        car_z += mapping_odom_cloud_->points[pointIdxRadiusSearch[i]].z;
    }

    if (pointIdxRadiusSearch.size() != 0)
    {
      car_height = car_z / pointIdxRadiusSearch.size();
      if (version_ == "0.0.1" || version_ == "0.0.2" ||
          version_ == "0.0.3")  // TODO: A version comparision function is needed here
      {
        // INFO << "Using map version lower than 0.1.0, lidar height has been compensated. " << REND;
        car_height -= lidar_height_;
      }
    }
    else
    {
      return false;
    }
    return true;
  }
  else
  {
    return false;
  }
}

int MapServer::saveToRsmap(const std::string& rsmap_file, DP& map, const std::string& version = "0.0.1",
                           const std::string& point_type = "vtkPoint", const int height = 32)
{
  std::ofstream output(rsmap_file, std::ios::out | std::ios::trunc | std::ios::binary);
  int NbPoints = map.features.cols();

  if (version == "0.0.2")
  {
    int negetive_NbPoints = -1 * NbPoints;  // set to negetive to represent map with descriptors.
    output.write(reinterpret_cast<char*>(&negetive_NbPoints), sizeof(int));

    mapHeader header;
    const char* cstr_version = version.c_str();
    const char* cstr_point_type = point_type.c_str();
    const char* cstr_height = std::to_string(height).c_str();
    const char* cstr_nb_points = std::to_string(NbPoints).c_str();

    strncpy(header.version, cstr_version, sizeof(header.version));
    strncpy(header.point_type, cstr_point_type, sizeof(header.point_type));
    strncpy(header.height, cstr_height, sizeof(header.height));
    strncpy(header.nb_points, cstr_nb_points, sizeof(header.nb_points));

    output.write(reinterpret_cast<char*>(&header), sizeof(mapHeader));

    rsPointNormal point_normal;
    View normal_view = map.getDescriptorViewByName("normals");
    View kind_view = map.getDescriptorViewByName("kind");
    for (int i = 0; i < NbPoints; i++)
    {
      point_normal.kind1 = kind_view(0, i);
      point_normal.kind2 = kind_view(1, i);
      point_normal.x = map.features(0, i);
      point_normal.y = map.features(1, i);
      point_normal.z = map.features(2, i);
      point_normal.normal_x = normal_view(0, i);
      point_normal.normal_y = normal_view(1, i);
      point_normal.normal_z = normal_view(2, i);

      output.write(reinterpret_cast<char*>(&point_normal), sizeof(rsPointNormal));
    }
  }

  else if (version == "0.0.3")
  {
    int negetive_NbPoints = -1 * NbPoints;  // set to negetive to represent map with descriptors.
    output.write(reinterpret_cast<char*>(&negetive_NbPoints), sizeof(int));

    mapHeader header;
    const char* cstr_version = version.c_str();
    const char* cstr_point_type = point_type.c_str();
    const char* cstr_height = std::to_string(height).c_str();
    const char* cstr_nb_points = std::to_string(NbPoints).c_str();

    strncpy(header.version, cstr_version, sizeof(header.version));
    strncpy(header.point_type, cstr_point_type, sizeof(header.point_type));
    strncpy(header.height, cstr_height, sizeof(header.height));
    strncpy(header.nb_points, cstr_nb_points, sizeof(header.nb_points));

    output.write(reinterpret_cast<char*>(&header), sizeof(mapHeader));

    rsPointXYZINormal point;  // new in version 0.0.3
    View normal_view = map.getDescriptorViewByName("normals");
    View kind_view = map.getDescriptorViewByName("kind");
    View intensity_view = map.getDescriptorViewByName("intensity");
    for (int i = 0; i < NbPoints; i++)
    {
      point.kind1 = kind_view(0, i);
      point.kind2 = kind_view(1, i);
      point.x = map.features(0, i);
      point.y = map.features(1, i);
      point.z = map.features(2, i);
      point.normal_x = normal_view(0, i);
      point.normal_y = normal_view(1, i);
      point.normal_z = normal_view(2, i);
      point.intensity = intensity_view(0, i);

      output.write(reinterpret_cast<char*>(&point), sizeof(rsPointXYZINormal));
    }
  }

  else if (version == "0.0.1")
  {
    output.write(reinterpret_cast<char*>(&NbPoints), sizeof(int));
    vtkPoint point;
    for (int i = 0; i < NbPoints; i++)
    {
      point.kind1 = map.descriptors(0, i);
      point.kind2 = map.descriptors(1, i);
      point.pose_x = map.features(0, i);
      point.pose_y = map.features(1, i);
      point.pose_z = map.features(2, i);
      output.write(reinterpret_cast<char*>(&point), sizeof(vtkPoint));
    }
  }

  output.close();
  return NbPoints;
}

}  // namespace localization
}  // namespace robosense
