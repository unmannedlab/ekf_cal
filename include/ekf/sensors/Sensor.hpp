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

#ifndef EKF__SENSORS__SENSOR_HPP_
#define EKF__SENSORS__SENSOR_HPP_

#include <eigen3/Eigen/Eigen>

#include <string>
#include <vector>

///
/// @class Base sensor class
///
class Sensor
{
public:
  typedef struct Params
  {
    std::string name;
    double rate{1.0};
    Eigen::Vector3d posOffset{0.0, 0.0, 0.0};
    Eigen::Quaterniond angOffset{1.0, 0.0, 0.0, 0.0};
  } Params;

  explicit Sensor(std::string name);

  Eigen::VectorXd PredictMeasurement();
  Eigen::MatrixXd GetMeasurementJacobian();

  Eigen::Vector3d GetPosOffset();
  Eigen::Quaterniond GetAngOffset();

  void SetPosOffset(Eigen::Vector3d posOffset);
  void SetAngOffset(Eigen::Quaterniond angOffset);

  std::string GetName();
  unsigned int GetId();
  unsigned int GetStateStartIndex();
  unsigned int GetStateSize();
  void SetStateStartIndex(unsigned int stateStartIndex);
  void SetState(Eigen::VectorXd state);

protected:
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_angOffset{0.0, 0.0, 0.0, 0.0};

  unsigned int m_stateStartIndex{0};
  unsigned int m_stateSize{0};

  static Eigen::Vector3d m_bodyPos;
  static Eigen::Vector3d m_bodyVel;
  static Eigen::Vector3d m_bodyAcc;
  static Eigen::Quaterniond m_bodyAngPos;
  static Eigen::Vector3d m_bodyAngVel;
  static Eigen::Vector3d m_bodyAngAcc;

private:
  unsigned int m_id;
  std::string m_name;
  static unsigned int _idCount;
};

#endif  // EKF__SENSORS__SENSOR_HPP_
