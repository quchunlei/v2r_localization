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

#ifndef PROJECTS_SAVE_TO_CSV_H
#define PROJECTS_SAVE_TO_CSV_H

#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/lexical_cast.hpp>

namespace robosense
{
namespace localization
{
using namespace std;
using namespace Eigen;

void saveToCSV(const string& path, const Vector3f& data);

void saveToCSV(const string& path, const Vector3f& data, bool is_append);

template <typename T>
void saveToCSV(const string& path, const vector<T>& data);

Vector3f readPose(const string& path);

template <typename T>
int readTheta(const string& path, vector<T>& data);
}  // namespace localization
}  // namespace robosense

#endif  // PROJECTS_SAVE_TO_CSV_H