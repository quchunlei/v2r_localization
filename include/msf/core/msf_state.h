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
#ifndef RS_LOCALIZATION_MSF_COMMON_H
#define RS_LOCALIZATION_MSF_COMMON_H

#include <iostream>
#include <cmath>
#include <math.h>
#include <Eigen/Core>

#include "common/common.h"
#include "common/prompt.h"

namespace robosense
{
namespace localization
{
namespace msf
{
// state vector in this filter
// [p, v, q, acc_bias, gyro_bias, grabity]
enum EkfStateIdx
{
  // -------State------------//
  StatePositionIdx = 0,
  StateLinearVelIdx = 3,
  StateAngleIdx = 6,  // quternion: [w,x,y,z], using Hamliton convention
  StateAccBiasIdx = 10,
  StateGyroBiasIdx = 13,
  StateGravityIdx = 16,
  // ------SensorMeas---------//
  StateLinearAccIdx = 19,
  StateAngleVelIdx = 22,
  StateGpsBiasIdx = 25,
  StateOdomBiasIdx = 28,
  StateSize = 31,
  SystemStateSize = 19,
  ErrorStateSize = 18
  // error state size is SystemStateSize-1 because angle error is represented by d_theta(3x1), not quaternion
};

enum EsStateIdx
{
  EsPositionIdx = StatePositionIdx,
  EsLinearVelIdx = StateLinearVelIdx,
  EsAngleIdx = StateAngleIdx,
  EsAccBiasIdx = StateAccBiasIdx - 1,
  EsGyroBiasIdx = StateGyroBiasIdx - 1,
  EsGravityIdx = StateGravityIdx - 1
};

enum EkfObservationIdx
{
  ObservationPositionIdx = 0,
  ObservationVelIdx = 3,
  ObservationAngleIdx = 6,
  ObservationSize = 10
};

// for imuObserver and lidarEsObservser
using EsVec = Eigen::Matrix<float, ErrorStateSize, 1>;
using EsCov = Eigen::Matrix<float, ErrorStateSize, ErrorStateSize>;
using EsJacob = Eigen::Matrix<float, ErrorStateSize, ErrorStateSize>;
using EsPertbJacob = Eigen::Matrix<float, 12, 12>;

// for imuOdomObserver and lidarRegistraObserver
using EkfStateVec = Eigen::Matrix<float, StateSize, 1>;
using EkfStateErrVec = Eigen::Matrix<float, SystemStateSize, 1>;
using EkfMeasVec = Eigen::Matrix<float, ObservationSize, 1>;
using EkfMeasErrVec = Eigen::Matrix<float, ObservationSize, 1>;
using EkfCovMat = Eigen::Matrix<float, SystemStateSize, SystemStateSize>;
using EkfGMat = Eigen::Matrix<float, SystemStateSize, SystemStateSize>;
using EkfRMat = Eigen::Matrix<float, SystemStateSize, SystemStateSize>;
using EkfHMat = Eigen::Matrix<float, ObservationSize, SystemStateSize>;
using EkfQMat = Eigen::Matrix<float, ObservationSize, ObservationSize>;
using EkfKMat = Eigen::Matrix<float, SystemStateSize, ObservationSize>;

struct MsfState
{
  MsfState()
  {
    state_vec = EkfStateVec::Zero();
    // state_vec.segment<3>(StateGravityIdx) = Eigen::Vector3f(0, 0, -9.79);
    // satte_vec.segment<3>(Statet)
  }

  uint64_t seq = 0;
  double timestamp = 0.0;
  bool is_prediction = true;

  EkfStateVec state_vec;
  EsCov es_cov = EsCov::Identity();

  EkfCovMat cov = EkfCovMat::Identity();

  Eigen::Vector3f getStateVar(const EkfStateIdx& idx) const
  {
    return state_vec.block<3, 1>(idx, 0);
  }

  Eigen::Matrix3f getCov(const EkfStateIdx& idx) const
  {
    return cov.block<3, 3>(idx, idx);
  }

  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Matrix3f pos_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f linear_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_vel_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f linear_acc = Eigen::Vector3f::Zero();
  Eigen::Matrix3f linear_acc_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f angle = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angle_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f angular_vel = Eigen::Vector3f::Zero();
  Eigen::Matrix3f angular_vel_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f gps_bias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f gps_bias_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f acc_bias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f acc_bias_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f gyro_bias_cov = Eigen::Matrix3f::Identity();

  Eigen::Vector3f gravity = Eigen::Vector3f::Zero();
  Eigen::Matrix3f gravity_cov = Eigen::Matrix3f::Identity();
};

/* Utilities*/
inline void printMsfState(MsfState& state)
{
  std::cout << "State -- " << state.seq << " -- " << state.timestamp << std::endl;
  std::cout << "pos | " << state.state_vec.segment<3>(StatePositionIdx).transpose() << std::endl;
  std::cout << "vel | " << state.state_vec.segment<3>(StateLinearVelIdx).transpose() << std::endl;
  std::cout << "q   | " << state.state_vec.segment<4>(StateAngleIdx).transpose() << std::endl;
  std::cout << "a_b | " << state.state_vec.segment<3>(StateAccBiasIdx).transpose() << std::endl;
  std::cout << "w_b | " << state.state_vec.segment<3>(StateGyroBiasIdx).transpose() << std::endl;
  std::cout << "g   | " << state.state_vec.segment<3>(StateGravityIdx).transpose() << std::endl;
  std::cout << "acc | " << state.state_vec.segment<3>(StateLinearAccIdx).transpose() << std::endl;
  std::cout << "omg | " << state.state_vec.segment<3>(StateAngleVelIdx).transpose() << std::endl;
}

}  // namespace msf

}  // namespace localization
}  // namespace robosense

#endif /*RS_LOCALIZATION_MSF_STATE_H*/
