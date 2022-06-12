// Copyright 2022 Jacob Hartzer
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

#ifndef EKF__SENSORS__CAMERA_HPP_
#define EKF__SENSORS__CAMERA_HPP_

#include <string>

#include "ekf/sensors/Sensor.hpp"

///
/// @class Camera Sensor Class
///
class Camera : public Sensor
{
public:
  typedef struct Params
  {
    std::string name;
    bool intrinsic{false};
    double rate{1.0};
    Eigen::Vector3d posOffset{0.0, 0.0, 0.0};
    Eigen::Quaterniond angOffset{1.0, 0.0, 0.0, 0.0};
  } Params;

  ///
  /// @class Sensor
  /// @brief
  ///
  explicit Camera(Camera::Params params);
  Eigen::VectorXd PredictMeasurement();
  Eigen::VectorXd GetMeasurementJacobian();

protected:
private:
};

#endif  // EKF__SENSORS__CAMERA_HPP_
