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

#include "sensor/gps_msg_process.h"

namespace robosense
{
namespace localization
{
GpsMsgProcess::GpsMsgProcess()
{
  is_set_gps_origin_ = false;
  gps_origin_ << 0, 0, 0;
  origin_ECEF_ << 0, 0, 0;
  // std::cout << "hello!!!\n\r";
}

void GpsMsgProcess::setGpsOrigin(const Eigen::Vector3d& gps_origin_lat_lon_alt)
{
  is_set_gps_origin_ = true;
  // latitude, longitude, altitude
  gps_origin_ = gps_origin_lat_lon_alt;  // 39.9680502, 116.3045885,

  Eigen::Vector3d origin_WGS84(gps_origin_[1], gps_origin_[0], gps_origin_[2]);
  origin_ECEF_ = WGS84toECEF(origin_WGS84);
}

bool GpsMsgProcess::gps2xyz(const Eigen::Vector3d& lon_lat_alt, Eigen::Vector3d& xyz)
{
  gps2xyz(lon_lat_alt[0], lon_lat_alt[1], lon_lat_alt[2], xyz);
}

bool GpsMsgProcess::gps2xyz(const double& longitude, const double& latitude, const double& altitude,
                            Eigen::Vector3d& xyz)
{
  if (!is_set_gps_origin_)
  {
    ERROR << "[>> " << name() << "] Don't set gps origin!" << RESET << END;
    return false;
  }

  if (std::isnan(longitude) || std::isnan(latitude) || std::isnan(altitude))
  {
    ERROR << "[>> " << name() << "] GPS value is NAN!" << RESET << END;
    return false;
  }

  if (fabs(longitude) < 0.01 || fabs(latitude) < 0.01)
  {
    ERROR << "[>> " << name() << "] GPS value is smaller than 0.01!" << RESET << END;
    return false;
  }
  // gps << gps_msg.longitude, gps_msg.latitude, gps_msg.altitude;
  Eigen::Vector3d gps(longitude, latitude, altitude);
  Eigen::Vector3d gps_ECEF = WGS84toECEF(gps);

  //××××××××处理GPS数据
  double rad_lon = gps_origin_[1] / 180 * M_PI;
  double rad_lat = gps_origin_[0] / 180 * M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
  // clang-format off
  rot << -sin_lon, cos_lon, 0,
        -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
        cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  // clang-format on

  Eigen::Vector3d diff_ECEF = gps_ECEF - origin_ECEF_;
  Eigen::Vector3d xyz_ECEF = rot * diff_ECEF;
  xyz << xyz_ECEF[0], xyz_ECEF[1], xyz_ECEF[2];
  return true;
}

bool GpsMsgProcess::xyz2gps(const Eigen::Vector3d& xyz, Eigen::Vector3d& lon_lat_alt)
{
  xyz2gps(xyz, lon_lat_alt[0], lon_lat_alt[1], lon_lat_alt[2]);
}

bool GpsMsgProcess::xyz2gps(const Eigen::Vector3d& xyz, double& longitude, double& latitude, double& altitude)
{
  if (!is_set_gps_origin_)
  {
    ERROR << "[>> " << name() << "] Don't set gps origin!" << RESET << END;
    return false;
  }

  if (std::isnan(xyz.x()) || std::isnan(xyz.x()) || std::isnan(xyz.x()))
  {
    ERROR << "[>> " << name() << "] XYZ value is NAN!" << RESET << END;
    return false;
  }

  if (fabs(xyz.x()) < 0.01 || fabs(xyz.y()) < 0.01)
  {
    ERROR << "[>> " << name() << "] XYZ value is smaller than 0.01!" << RESET << END;
    return false;
  }

  double rad_lon = gps_origin_[1] / 180 * M_PI;
  double rad_lat = gps_origin_[0] / 180 * M_PI;
  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();

  // clang-format off
    rot << -sin_lon, cos_lon, 0,
           -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
            cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
  // clang-format on

  Eigen::Vector3d diff_ECEF = rot.inverse() * xyz;
  Eigen::Vector3d gps_ECEF = diff_ECEF + origin_ECEF_;
  Eigen::Vector3d gps_WGS84 = ECEFtoWGS84(gps_ECEF);
  // std::cout << gps_WGS84.transpose() << std::endl;
  longitude = gps_WGS84[0];
  latitude = gps_WGS84[1];
  altitude = gps_WGS84[2];
  return true;
}

Eigen::Vector3d GpsMsgProcess::WGS84toECEF(const Eigen::Vector3d& gps)

{
  double SEMI_MAJOR_AXIS = 6378137.0;
  //    double RECIPROCAL_OF_FLATTENING = 298.257223563;
  //    double SEMI_MINOR_AXIS = 6356752.3142;
  double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;
  //    double SECOND_ECCENTRICITY_SQUARED = 6.73949674228e-3;

  double lon = gps[0];
  double lat = gps[1];
  double ati = gps[2];

  if (!std::isnan(gps[2]))
    ati = gps[2];

  double rad_lon = lon / 180 * M_PI;
  double rad_lat = lat / 180 * M_PI;

  double sin_lon = sin(rad_lon);
  double cos_lon = cos(rad_lon);
  double sin_lat = sin(rad_lat);
  double cos_lat = cos(rad_lat);

  double chi = sqrt(1.0 - FIRST_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
  double N = SEMI_MAJOR_AXIS / chi + ati;

  // clang-format off
  Eigen::Vector3d ret = Eigen::Vector3d::Zero();
  ret << N * cos_lat * cos_lon,
        N * cos_lat * sin_lon,
        (SEMI_MAJOR_AXIS * (1.0 - FIRST_ECCENTRICITY_SQUARED) / chi + ati) * sin_lat;
  // clang-format on
  return ret;
}

// refer from 'https://github.com/sameeptandon/sail-car-log/blob/master/process/WGS84toENU.py'
Eigen::Vector3d GpsMsgProcess::ECEFtoWGS84(const Eigen::Vector3d& xyz)
{
  double X = xyz[0];
  double Y = xyz[1];
  double Z = xyz[2];

  double a = 6378137.0;          // earth semimajor axis in meters
  double f = 1 / 298.257223563;  // reciprocal flattening
  double b = a * (1.0 - f);      // semi-minor axis

  double e2 = 2.0 * f - pow(f, 2.0);               // first eccentricity squared
  double ep2 = f * (2.0 - f) / pow((1.0 - f), 2);  // second eccentricity squared

  double r2 = pow(X, 2) + pow(Y, 2);
  double r = sqrt(r2);
  double E2 = pow(a, 2) - pow(b, 2);
  double F = 54 * pow(b, 2) * pow(Z, 2);
  double G = r2 + (1.0 - e2) * pow(Z, 2) - e2 * E2;
  double c = (e2 * e2 * F * r2) / (G * G * G);
  double s = pow((1.0 + c + sqrt(c * c + 2 * c)), 1.0 / 3.0);
  double P = F / (3.0 * pow((s + 1.0 / s + 1), 2) * G * G);
  double Q = sqrt(1 + 2 * e2 * e2 * P);
  double ro = -(e2 * P * r) / (1.0 + Q) +
              sqrt((a * a / 2.0) * (1 + 1.0 / Q) - ((1.0 - e2) * P * pow(Z, 2)) / (Q * (1.0 + Q)) - P * r2 / 2.0);
  double tmp = pow((r - e2 * ro), 2);
  double U = sqrt(tmp + pow(Z, 2));
  double V = sqrt(tmp + (1.0 - e2) * pow(Z, 2));
  double zo = (pow(b, 2) * Z) / (a * V);

  double h = U * (1.0 - pow(b, 2) / (a * V));
  double phi = atan((Z + ep2 * zo) / r);
  double lam = atan2(Y, X);
  // longitude, latitude, altitude
  return Eigen::Vector3d(180 / M_PI * lam, 180 / M_PI * phi, h);
}

}  // localization
}  // robosense
