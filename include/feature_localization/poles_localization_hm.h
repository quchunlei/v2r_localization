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

#ifndef PROJECTS_POLES_LOCALIZATION_HM_H
#define PROJECTS_POLES_LOCALIZATION_HM_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <vector>

#include "yaml/yaml.h"
#include "third_party/hungarian_bigraph_matcher.h"
#include "common/save_to_csv.h"
#include "common/compute.h"
#include "common/display.h"
#include "common/prompt.h"

namespace robosense
{
namespace localization
{
using namespace Eigen;
using namespace std;

//使用匈牙利匹配实现杆子定位
class PolesLocalizationHM
{
private:
  struct Options
  {
    explicit Options()
    {
      t_assign_distance_max = 1.0;
      t_least_point_num = 3;
      scale_of_filter_cur = 0.2;
      t_dis_of_merge_tree = 1.5;
      num_of_theta_stored = 5;
      t_theta_std_ratio = 3.0;
      t_theta_change_max = 4.0;  // deg
      t_theta_change_min = 1.0;  // deg
      t_update_theta_pre_count = 2;
      show_message = false;
    }
    float t_assign_distance_max;  //配对的树之间距离的最大值
    int t_least_point_num;        //当前帧中点的最小数量
    float
        scale_of_filter_cur;    //对当前帧中的点进行过滤时，将与地图中最近的点的距离过大的当前点剔除，此值乘在标准方差上,
    float t_dis_of_merge_tree;  //两棵树的距离少于此值时，会将其合并成一颗树
    int num_of_theta_stored;    //保存的历史theta值的数量
    float t_theta_std_ratio;  //判断当前的theta是否可接受时，使用的历史的均值与标准方差进行判断
    float t_theta_change_max;      // theta变化量的最大阈值
    float t_theta_change_min;      // 认为theta变化量小于此值，都是可接受的
    int t_update_theta_pre_count;  // 更新theta_pre的计数器，此值就尽可能小
    bool show_message;             // show output message
  };
  vector<float> thetas_;
  float theta_pre_;
  Options options_;
  int update_theta_pre_count_;

public:
  PolesLocalizationHM();
  PolesLocalizationHM(const YAML::Node& param);
  bool loadConfigrationFile(const YAML::Node& param);
  bool match(const vector<Vector3f>& poles_cur, const vector<Vector3f>& poles_map, const Vector3f& initial_pose,
             Vector3f& result_pose);

  std::string name() const
  {
    return "FeatureLocalizationHM";
  }

private:
  const Options& getOptions() const
  {
    return options_;
  };

private:
  void transformCurToMap(const vector<Vector3f>& in, const Vector3f& pose, vector<Vector3f>& out);
  size_t mergePolesInCur(vector<Vector3f>& cur);
  int filterPolesInCur(vector<Vector3f>& cur, const vector<Vector3f>& map);
  void genCostMatrix(const vector<Vector3f>& cur, const vector<Vector3f>& map, vector<vector<double> >& cost);
  void genTreePairs(const vector<vector<double> >& cost, const vector<Vector3f>& cur_trees,
                    const vector<int>& cur_index, const vector<Vector3f>& map_trees, const vector<int>& map_index,
                    vector<pair<Vector3f, Vector3f> >& tree_pairs);
  Vector3f getTransformation(const vector<pair<Vector3f, Vector3f> >& tree_pairs, size_t num);
  Vector3f resolve(const vector<pair<Vector3f, Vector3f> >& tree_pairs, size_t num);
  Vector3f transformCurToGlobal(const Vector3f& cur, const Vector3f& ref);
  bool check(const Vector3f& pose);
};

inline static bool treePairDis(const pair<Vector3f, Vector3f>& a, const pair<Vector3f, Vector3f>& b)
{
  float dis_a = distanceTwoPoint(a.first.x(), a.first.y(), a.second.x(), a.second.y());
  float dis_b = distanceTwoPoint(b.first.x(), b.first.y(), b.second.x(), b.second.y());
  return dis_a < dis_b;
}

}  // namespace localization
}  // namespace robosense

#endif  // PROJECTS_POLES_LOCALIZATION_HM_H
