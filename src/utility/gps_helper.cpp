// Copyright 2024 Jacob Hartzer
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "gps_helper.hpp"

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <cmath>


static constexpr double g_wgs84_a {+6.37813700000000000000e+0006};  // WGS84 a - Semi-Major Axis
static constexpr double g_wgs84_f {+3.35281066474748071998e-0003};  // WGS84 f - Flattening
static constexpr double g_wgs84_e {+8.18191908426214947083e-0002};  // WGS84 e - Eccentricity
static constexpr double g_deg_to_rad = M_PI / 180.0;

Eigen::Vector3d lla_to_ecef(const Eigen::Vector3d & lla)
{
  Eigen::Vector3d out;

  if ((lla(0) < -90.0) || (lla(0) > +90.0) || (lla(1) < -180.0) || (lla(1) > +360.0)) {
    std::cout << "WGS lat or WGS lon out of range" << std::endl;
    return out;
  }

  double NAV_E2 = (2.0 - g_wgs84_f) * g_wgs84_f;  // also e^2

  double sin_lat = std::sin(lla(0) * g_deg_to_rad);
  double cos_lat = std::cos(lla(0) * g_deg_to_rad);
  double r_n = g_wgs84_a / sqrt(1.0 - NAV_E2 * sin_lat * sin_lat);

  out(0) = (r_n + lla(2)) * cos_lat * std::cos(lla(1) * g_deg_to_rad);
  out(1) = (r_n + lla(2)) * cos_lat * std::sin(lla(1) * g_deg_to_rad);
  out(2) = (r_n * (1.0 - NAV_E2) + lla(2)) * sin_lat;

  return out;
}

Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d & in_ecef, const Eigen::Vector3d & ref_lla)
{
  Eigen::Vector3d ref_ecef, diff_ecef;

  ref_ecef = lla_to_ecef(ref_lla);

  // ECEF difference from reference point
  diff_ecef(0) = in_ecef(0) - ref_ecef(0);
  diff_ecef(1) = in_ecef(1) - ref_ecef(1);
  diff_ecef(2) = in_ecef(2) - ref_ecef(2);

  Eigen::Matrix3d R;
  R(0, 0) = -std::sin(ref_lla(1) * g_deg_to_rad);
  R(0, 1) = std::cos(ref_lla(1) * g_deg_to_rad);
  R(0, 2) = 0.0;
  R(1, 0) = -std::cos(ref_lla(1) * g_deg_to_rad) * std::sin(ref_lla(0) * g_deg_to_rad);
  R(1, 1) = -std::sin(ref_lla(1) * g_deg_to_rad) * std::sin(ref_lla(0) * g_deg_to_rad);
  R(1, 2) = std::cos(ref_lla(0) * g_deg_to_rad);
  R(2, 0) = std::cos(ref_lla(1) * g_deg_to_rad) * std::cos(ref_lla(0) * g_deg_to_rad);
  R(2, 1) = std::sin(ref_lla(1) * g_deg_to_rad) * std::cos(ref_lla(0) * g_deg_to_rad);
  R(2, 2) = std::sin(ref_lla(0) * g_deg_to_rad);

  Eigen::Vector3d out_enu = R * diff_ecef;

  return out_enu;
}

Eigen::Vector3d enu_to_ecef(const Eigen::Vector3d & in_enu, const Eigen::Vector3d & ref_lla)
{
  Eigen::Vector3d ref_ecef = lla_to_ecef(ref_lla);
  Eigen::Matrix3d R(3, 3);
  R(0, 0) = -std::sin(ref_lla(1) * g_deg_to_rad);
  R(0, 1) = -std::cos(ref_lla(1) * g_deg_to_rad) * std::sin(ref_lla(0) * g_deg_to_rad);
  R(0, 2) = std::cos(ref_lla(1) * g_deg_to_rad) * std::cos(ref_lla(0) * g_deg_to_rad);
  R(1, 0) = std::cos(ref_lla(1) * g_deg_to_rad);
  R(1, 1) = -std::sin(ref_lla(1) * g_deg_to_rad) * std::sin(ref_lla(0) * g_deg_to_rad);
  R(1, 2) = std::sin(ref_lla(1) * g_deg_to_rad) * std::cos(ref_lla(0) * g_deg_to_rad);
  R(2, 0) = 0.0;
  R(2, 1) = std::cos(ref_lla(0) * g_deg_to_rad);
  R(2, 2) = std::sin(ref_lla(0) * g_deg_to_rad);

  Eigen::Vector3d out_ecef = ref_ecef + R * in_enu;
  return out_ecef;
}

Eigen::Vector3d lla_to_enu(const Eigen::Vector3d & point_lla, const Eigen::Vector3d & ref_lla)
{
  Eigen::Vector3d point_ecef = lla_to_ecef(point_lla);
  Eigen::Vector3d out = ecef_to_enu(point_ecef, ref_lla);
  return out;
}

Eigen::Vector3d ecef_to_lla(const Eigen::Vector3d & ecef)
{
  // The variables below correspond to symbols used in the paper
  // Karl Olsen. Accurate Conversion of Earth-Fixed Earth-Centered Coordinates to Geodetic
  // Coordinates. [Research Report] Norwegian University of Science and Technology. 2017.
  // hal-01704943v2
  double x = ecef[0];
  double y = ecef[1];
  double z = ecef[2];
  double ww = x * x + y * y;
  double m = ww / g_wgs84_a / g_wgs84_a;
  double n = z * z * ((1 - pow(g_wgs84_e, 2)) / pow(g_wgs84_a, 2));
  double mpn = m + n;
  double p = (mpn - pow(g_wgs84_e, 4)) / 6;
  double G = m * n * pow(g_wgs84_e, 4) / 4;
  double H = 2 * p * p * p + G;

  double C = pow(H + G + 2 * std::sqrt(H * G), 1.0 / 3.0) * (1 / pow(2.0, 1.0 / 3.0));
  double i = -(pow(g_wgs84_e, 4) / 4) - 0.5 * mpn;
  double P = p * p;
  double beta = i / 3.0 - C - P / C;
  double k = (pow(g_wgs84_e, 4) / 4) * ((pow(g_wgs84_e, 4) / 4) - mpn);

  // Compute left part of t
  double t1 = beta * beta - k;
  double t2 = std::sqrt(t1);
  double t3 = t2 - 0.5 * (beta + i);
  double t4 = std::sqrt(t3);
  double t5 = std::fabs(0.5 * (beta - i));
  double t6 = std::sqrt(t5);
  double t7 = (m < n) ? t6 : -t6;

  // Add left and right parts
  double t = t4 + t7;

  // Use Newton-Raphson's method to compute t correction
  double j = pow(g_wgs84_e, 2) / 2 * (m - n);
  double g = 2 * j;
  double tt = t * t;
  double ttt = tt * t;
  double tttt = tt * tt;
  double F = tttt + 2 * i * tt + g * t + k;
  double dFdt = 4 * ttt + 4 * i * t + g;
  double dt = -F / dFdt;

  // compute latitude (range -90..90)
  double u = t + dt + pow(g_wgs84_e, 2) / 2;
  double v = t + dt - pow(g_wgs84_e, 2) / 2;
  double w = std::sqrt(ww);
  double zu = z * u;
  double wv = w * v;
  double lat = std::atan2(zu, wv);

  // compute altitude
  double inv_uv = 1 / (u * v);
  double dw = w - wv * inv_uv;
  double dz = z - zu * (1.0 - pow(g_wgs84_e, 2)) * inv_uv;
  double da = std::sqrt(dw * dw + dz * dz);
  double alt = (u < 1) ? -da : da;

  // compute longitude (range -90..90)
  double lon = std::atan2(y, x);

  Eigen::Vector3d lla {lat / g_deg_to_rad, lon / g_deg_to_rad, alt};

  return lla;
}

Eigen::Vector3d enu_to_lla(const Eigen::Vector3d & enu_in, const Eigen::Vector3d & ref_lla)
{
  Eigen::Vector3d ecef = enu_to_ecef(enu_in, ref_lla);
  Eigen::Vector3d lla = ecef_to_lla(ecef);
  return lla;
}

Eigen::Vector3d local_to_enu(const Eigen::Vector3d & local_in, const double ang_l_to_g)
{
  Eigen::Vector3d enu;
  enu(0) = std::cos(ang_l_to_g) * local_in(0) + std::sin(ang_l_to_g) * local_in(1);
  enu(1) = -std::sin(ang_l_to_g) * local_in(0) + std::cos(ang_l_to_g) * local_in(1);
  enu(2) = local_in(2);

  return enu;
}

Eigen::Vector3d enu_to_local(const Eigen::Vector3d & enu_in, const double ang_l_to_g)
{
  Eigen::Vector3d local;
  local(0) = std::cos(ang_l_to_g) * enu_in(0) - std::sin(ang_l_to_g) * enu_in(1);
  local(1) = std::sin(ang_l_to_g) * enu_in(0) + std::cos(ang_l_to_g) * enu_in(1);
  local(2) = enu_in(2);

  return local;
}

double wgs84_m_to_deg(const double meters)
{
  return meters / (g_wgs84_a * g_deg_to_rad);
}
