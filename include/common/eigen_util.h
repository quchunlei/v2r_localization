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

#ifndef RS_LOCALIZATION_EIGEN_UTIL_H
#define RS_LOCALIZATION_EIGEN_UTIL_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <type_traits>
#include <iostream>
#include <functional>

inline Eigen::Matrix4f getOmegaMat(const Eigen::Vector3f& vec)
{
  Eigen::Matrix4f result=Eigen::Matrix4f::Identity();
  result << 0, vec[2], -vec[1], vec[0], -vec[2], 0, vec[0], vec[1], vec[1], -vec[0], 0, vec[2],
          -vec[0], -vec[1], -vec[2], 0;
  return result;
}

inline Eigen::Quaterniond smallAngleQuat(const Eigen::Vector3f& theta)
{
  Eigen::Quaterniond ret(1, theta[0] * 0.5, theta[1] * 0.5, theta[2] * 0.5);
  ret.normalize();
  return std::move(ret);
}

inline Eigen::Matrix3f getInvSkewMat(const Eigen::Vector3f& vec)
{
  return (Eigen::Matrix3f() << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0).finished();
}

inline Eigen::Quaternionf getQuatFromOmega(const Eigen::Vector3f& vec)
{
  if (vec.norm() < 1e-8)
    return Eigen::Quaternionf(1, 0, 0, 0);

  float w_norm = vec.norm();
  float w_norm_half = w_norm * 0.5;
  Eigen::Vector3f q_xyz = sinf(w_norm_half) * vec / w_norm;
  Eigen::Quaternionf q(cosf(w_norm_half), q_xyz[0], q_xyz[1], q_xyz[2]);
  q.normalize();
  return q;
}

// Equations are derived using Matlab
inline Eigen::Matrix<float, 4, 3> getOmega2QuatJacobian(const Eigen::Vector3f& vec)
{
  float vec_norm = vec.norm();
  float sqr_norm = vec.squaredNorm();  // vec_norm^2
  float half_norm = 0.5 * vec_norm;
  Eigen::Matrix<float, 4, 3> J = Eigen::Matrix<float, 4, 3>::Zero();

  auto s_half_norm = sinf(half_norm);
  auto c_half_norm = cosf(half_norm);
  auto norm_inv = 1 / vec_norm;
  auto sqr_norm_inv = 1 / sqr_norm;
  auto norm_3_inv = norm_inv * sqr_norm_inv;
  auto a = s_half_norm * norm_inv;
  auto b = 0.5 * c_half_norm * sqr_norm_inv;
  auto c = s_half_norm * norm_3_inv;

  J(0, 0) = -0.5 * vec(0) * a;
  J(0, 1) = -0.5 * vec(1) * a;
  J(0, 2) = -0.5 * vec(2) * a;

  J(1, 0) = a + vec(0) * vec(0) * b - vec(0) * vec(0) * c;
  J(1, 1) = vec(0) * vec(1) * b - vec(0) * vec(1) * c;
  J(1, 2) = J(1, 1) / vec(1) * vec(2);

  J(2, 0) = J(1, 1);
  J(2, 1) = a + vec(1) * vec(1) * b - vec(1) * vec(1) * c;
  J(2, 2) = J(1, 2) / vec(0) * vec(1);

  J(3, 0) = J(1, 2);
  J(3, 1) = J(2, 2);
  J(3, 2) = a + vec(2) * vec(2) * b - vec(2) * vec(2) * c;
}

// This function returns ZYX euler angles with pitch angle in range of [-pi/2, pi/2]
// Don't use the Eigen::Matrix::eulerAngles(2,1,0) function
// since it returns a pitch angle in range of [-pi,pi], which is not suitable in our case.
template <typename T>
Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 3, 1> eulerAnglesZYX(T&& q)
{
  typedef typename std::remove_reference<T>::type::Scalar Scalar;

  q.normalize();
  Scalar s = -2 * (q.x() * q.z() - q.w() * q.y());
  if (s > 1)
    s = 1;
   
  Eigen::Matrix<Scalar, 3, 1> result;
  result << atan2f(2 * (q.x() * q.y() + q.w() * q.z()),
            q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z()),
          asin(s),
          atan2(2 * (q.y() * q.z() + q.w() * q.x()), q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z());
  return result;
}

// Returns coeffiencies of a quaternion with a positve w guarenteed
template <typename T>
Eigen::Matrix<typename std::remove_reference<T>::type::Scalar, 4, 1> quatCoeffsFix(T&& q)
{
  if (q.w() > 0)
    return q.coeffs();
  else
    return -q.coeffs();
}

#endif /*RS_LOCALLIZATION_EIGEN_UTIL_H*/
