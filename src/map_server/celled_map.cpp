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

#include "map_server/celled_map.h"

namespace robosense
{
namespace localization
{
CelledMap::CelledMap()
{
  cell_count_[0] = 0;
  cell_count_[1] = 0;
  cell_min_[0] = 0.0f;
  cell_min_[1] = 0.0f;
  cell_max_[0] = 0.0f;
  cell_max_[1] = 0.0f;

  cell_length_ = 10.0f;
  local_map_radius_ = 60.0f;
}

void CelledMap::loadConfigration(const int cell_length, const int local_map_radius)
{
  cell_length_ = cell_length;
  local_map_radius_ = local_map_radius;
}

void CelledMap::calcMapSize(const DP& point_cloud)
{
  Eigen::Vector4f point = point_cloud.features.col(0);

  float x_min, x_max, y_min, y_max;

  x_max = x_min = point.x();
  y_max = y_min = point.y();

  for (int i = 1; i < point_cloud.getNbPoints(); i++)
  {
    point = point_cloud.features.col(i);

    x_max = (x_max < point.x()) ? point.x() : x_max;
    x_min = (x_min > point.x()) ? point.x() : x_min;
    y_max = (y_max < point.y()) ? point.y() : y_max;
    y_min = (y_min > point.y()) ? point.y() : y_min;
  }

  cell_min_[0] = x_min;
  cell_max_[0] = x_max;
  cell_count_[0] = std::ceil((x_max - x_min) / cell_length_);
  cell_min_[1] = y_min;
  cell_max_[1] = y_max;
  cell_count_[1] = std::ceil((y_max - y_min) / cell_length_);
}

/***********  MUST call after resize **********************/
void CelledMap::setGlobalMap(const DP& point_cloud)
{
  /*****************Resize Map*********************/
  if (local_map_radius_ <= 0 || cell_length_ <= 0)
  {
    ERROR << "[>> " << name() << "] Wrong Initialization" << RESET << END;
    exit(-1);
  }
  if (point_cloud.getNbPoints() <= 0)
  {
    ERROR << "[>> " << name() << "] The map is empty" << RESET << END;
    exit(-1);
  }

  calcMapSize(point_cloud);
  // DEBUG << "cell_min: "<< cell_min_.transpose() << REND;
  // DEBUG << "cell_max: "<< cell_max_.transpose() << REND;
  // DEBUG << "cell_count: " <<cell_count_.transpose() << REND;

  /***************Copy Point Cloud******************/
  // Calculate point cloud size for each cell
  Eigen::MatrixXi nb_points_each_cell = Eigen::MatrixXi::Zero(cell_count_[0], cell_count_[1]);
  Eigen::MatrixXi cell_idx_of_points = Eigen::MatrixXi::Zero(2, point_cloud.getNbPoints());

  for (int i = 0; i < point_cloud.getNbPoints(); i++)
  {
    Eigen::Vector4f point = point_cloud.features.col(i);
    // x_cell and y_cell is not checked if out of range
    int cell_x = (point(0) - cell_min_[0]) / cell_length_;
    int cell_y = (point(1) - cell_min_[1]) / cell_length_;
    cell_idx_of_points(0, i) = cell_x;
    cell_idx_of_points(1, i) = cell_y;
    nb_points_each_cell(cell_x, cell_y)++;
  }

  // Resize each cell and map
  // Something to verift 1. if I first copy points to a local variable say "map" then copy map to map_ then the speed is
  // slow, use std::move can solve the problem
  Eigen::Matrix<DP, Eigen::Dynamic, Eigen::Dynamic> map;
  map.conservativeResize(cell_count_[0], cell_count_[1]);

  for (int i = 0; i < cell_count_[0]; i++)
  {
    for (int j = 0; j < cell_count_[1]; j++)
    {
      map(i, j) = point_cloud.createSimilarEmpty(nb_points_each_cell(i, j));
    }
  }

  // Copy points into cells
  Eigen::MatrixXf curr_point_idx_cell = Eigen::MatrixXf::Zero(cell_count_[0], cell_count_[1]);
  for (int i = 0; i < point_cloud.getNbPoints(); i++)
  {
    int cell_x = cell_idx_of_points(0, i);
    int cell_y = cell_idx_of_points(1, i);

    map(cell_x, cell_y).setColFrom(curr_point_idx_cell(cell_x, cell_y), point_cloud, i);
    curr_point_idx_cell(cell_x, cell_y)++;
  }

  // Assign to class object
  map_ = std::move(map);
}

float CelledMap::distance2Cell(const Eigen::Vector3f pose, const Eigen::Vector3i cell)
{
  std::vector<Eigen::Vector3f> corner_poses(4);

  // Not the best solution, only the four corners are checked
  corner_poses[0][0] = cell_min_[0] + cell[0] * cell_length_;
  corner_poses[0][1] = cell_min_[1] + cell[1] * cell_length_;
  corner_poses[1][0] = cell_min_[0] + cell[0] * cell_length_;
  corner_poses[1][1] = cell_min_[1] + (cell[1] + 1) * cell_length_;
  corner_poses[2][0] = cell_min_[0] + (cell[0] + 1) * cell_length_;
  corner_poses[2][1] = cell_min_[1] + cell[1] * cell_length_;
  corner_poses[3][0] = cell_min_[0] + (cell[0] + 1) * cell_length_;
  corner_poses[3][1] = cell_min_[1] + (cell[1] + 1) * cell_length_;

  float shortest_dist = 10000000.0f;
  for (auto& corner_pose : corner_poses)
  {
    float dist = std::sqrt(std::pow((pose[0] - corner_pose[0]), 2) + std::pow((pose[1] - corner_pose[1]), 2));
    if (shortest_dist > dist)
    {
      shortest_dist = dist;
    }
  }

  return shortest_dist;
}

bool CelledMap::isIdxValid(const int& i, const int& j)
{
  return (i < cell_count_[0]) && (j < cell_count_[1]) && (i >= 0) && (j >= 0);
}

void CelledMap::getLocalMap(const Eigen::Vector3f pose, DP& local_map)
{
  if ((pose[0] != pose[0]) || (pose[1] != pose[1]))
  {
    ERROR << "[>> " << name() << "] pose is NAN" << RESET << END;
    exit(-1);
  }
  if (getSize() <= 0)
  {
    ERROR << "[>> " << name() << "] Global map not loaded" << RESET << END;
    exit(-1);
  }

  std::vector<Eigen::Vector3i> cell_list;
  int center_x, center_y;  // cell_id

  center_x = (pose[0] - cell_min_[0]) / cell_length_;
  center_y = (pose[1] - cell_min_[1]) / cell_length_;

  // DEBUG << "GETTING LOCAL MAP: " << REND;
  // DEBUG << "Center: " << center_x << "  " << center_y << REND;

  // Search cell in four sectors
  for (int i = center_x; i < cell_count_[0]; i++)
  {
    int list_size = cell_list.size();
    // WARNING << "loop1" << REND;
    for (int j = center_y; j < cell_count_[1]; j++)
    {
      float dist = distance2Cell(pose, Eigen::Vector3i(i, j, 0));
      // DEBUG<< "i,j,dist= " <<i <<", "<<j<<", "<<dist << REND;
      if (dist <= local_map_radius_)
      {
        if (!isIdxValid(i, j))
          continue;
        cell_list.emplace_back(i, j, 0);
        // DEBUG<< "push_back cell_list: " << Eigen::Vector2i(i,j).transpose() << REND;
      }
      else
        break;
    }
    // WARNING << "loop2" << REND;

    for (int j = center_y - 1; j >= 0; j--)
    {
      float dist = distance2Cell(pose, Eigen::Vector3i(i, j, 0));
      // DEBUG<< "i,j,dist= " <<i <<", "<<j<<", "<<dist << REND;
      if (dist <= local_map_radius_)
      {
        // WARNING << "here" << REND;
        if (!isIdxValid(i, j))
        {
          // WARNING << "here2:continue" << REND;
          continue;
        }
        cell_list.emplace_back(i, j, 0);
        // DEBUG<< "push_back cell_list: " << Eigen::Vector2i(i,j).transpose() << REND;
      }
      else
        break;
    }
    if (list_size == cell_list.size())
    {
      break;
    }
  }

  for (int i = center_x - 1; i >= 0; i--)
  {
    int list_size = cell_list.size();
    for (int j = center_y; j < cell_count_[1]; j++)
    {
      float dist = distance2Cell(pose, Eigen::Vector3i(i, j, 0));
      if (dist <= local_map_radius_)
      {
        if (!isIdxValid(i, j))
          continue;
        cell_list.emplace_back(i, j, 0);
        // DEBUG<< "push_back cell_list: " << Eigen::Vector2i(i,j).transpose() << REND;
      }
      else
        break;
    }
    for (int j = center_y - 1; j >= 0; j--)
    {
      float dist = distance2Cell(pose, Eigen::Vector3i(i, j, 0));
      if (dist <= local_map_radius_)
      {
        if (!isIdxValid(i, j))
          continue;
        cell_list.emplace_back(i, j, 0);
        // DEBUG<< "push_back cell_list: " << Eigen::Vector2i(i,j).transpose() << REND;
      }
      else
        break;
    }
    if (list_size == cell_list.size())
    {
      break;
    }
  }

  // Copy points
  bool first_time = true;
  int row_limit = map_.rows();
  int col_limit = map_.cols();
  // WARNING << "cell list size: "<<cell_list.size() << REND;
  for (Eigen::Vector3i& idx : cell_list)
  {
    // WARNING << idx.transpose() << REND;
    if (idx[0] > row_limit)
    {
      ERROR << "Celled map index row overflow: max = " << row_limit << " requested = " << idx[0] << REND;
      exit(-1);
    }
    else if (idx[1] > col_limit)
    {
      ERROR << "Celled map index col overflow: max = " << col_limit << " requested = " << idx[1] << REND;
      exit(-1);
    }

    // WARNING << "here3" << REND;
    // WARNING << "map_ size: " << map_.rows() << ", " << map_.cols()<< REND;
    int curr_cell_size = map_(idx[0], idx[1]).getNbPoints();
    // WARNING << "here4" << REND;

    if (curr_cell_size > 0)
    {
      if (first_time == true)
      {
        local_map = map_(idx[0], idx[1]);
        first_time = false;
      }
      else
      {
        local_map.concatenate(map_(idx[0], idx[1]));
      }
    }
  }

  // DEBUG << "Finish getting local map: " << local_map.getNbPoints() << REND;
  return;
}

}  // namespace localization
}  // namespace robosense
