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

#include "common/save_to_csv.h"

namespace robosense
{
namespace localization
{
void saveToCSV(const string& path, const Vector3f& data)
{
  std::ofstream out;
  out.open(path.c_str());
  if (out.is_open())
  {
    for (int i = 0; i < data.size(); ++i)
    {
      out << data[i];
      if (i != data.size() - 1)
        out << ",";
    }
  }
  out << std::endl;
  out.close();
  std::cout << "save in " << path << std::endl;
}

void saveToCSV(const string& path, const Vector3f& data, bool is_append)
{
  std::ofstream out;
  if (is_append)
    out.open(path.c_str(), ios::app);
  else
    out.open(path.c_str());
  if (out.is_open())
  {
    for (int i = 0; i < data.size(); ++i)
    {
      out << data[i];
      if (i != data.size() - 1)
        out << ",";
    }
  }
  out << std::endl;
  out.close();
  std::cout << "save in " << path << std::endl;
}

template <typename T>
void saveToCSV(const string& path, const vector<T>& data)
{
  std::ofstream out;
  out.open(path.c_str());
  if (out.is_open())
  {
    for (int i = 0; i < data.size(); ++i)
    {
      out << data[i];
      if (i != data.size() - 1)
        out << ",";
    }
  }
  out << std::endl;
  out.close();
  std::cout << "save: " << data.size() << " in " << path << std::endl;
}

Vector3f readPose(const string& path)
{
  std::ifstream in;
  std::string line_str;
  Vector3f pose;
  in.open(path.c_str());
  while (in.good())
  {
    getline(in, line_str, '\n');
    // std::cout << line_str << std::endl;
    vector<string> line_after_split;
    if (line_str != "\n" && !line_str.empty())
    {
      boost::split(line_after_split, line_str, boost::is_any_of(","));
      pose[0] = boost::lexical_cast<float>(line_after_split[0]);
      pose[1] = boost::lexical_cast<float>(line_after_split[1]);
      pose[2] = boost::lexical_cast<float>(line_after_split[2]);
      // cout << pose.transpose() << endl;
    }
  }
  in.close();
  cout << "read: " << pose.size() << " from " << path << endl;
  return pose;
}

template <typename T>
int readTheta(const string& path, vector<T>& data)
{
  std::ifstream in;
  std::string line_str;
  in.open(path.c_str());
  while (in.good())
  {
    getline(in, line_str, '\n');
    // std::cout << line_str << std::endl;
    vector<string> line_after_split;
    if (line_str != "\n" && !line_str.empty())
    {
      boost::split(line_after_split, line_str, boost::is_any_of(","));
      for (int i = 0; i < line_after_split.size(); ++i)
      {
        data.push_back(boost::lexical_cast<T>(line_after_split[i]));
      }
    }
  }
  in.close();
  cout << "read: " << data.size() << " from " << path << endl;
  return data.size();
}

}  // namespace localization
}  // namespace robosense
