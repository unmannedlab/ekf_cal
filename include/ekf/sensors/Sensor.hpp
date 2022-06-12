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
/// @class Sensor
/// @brief Base sensor class
///
class Sensor
{
public:
  ///
  /// @brief Sensor class constructor
  /// @param name Sensor name
  ///
  explicit Sensor(std::string name);

  ///
  /// @brief Predict measurement method
  /// @return Predicted measurement vector
  ///
  Eigen::VectorXd PredictMeasurement();

  ///
  /// @brief Measurement Jacobian method
  /// @return Measurement Jacobian matrix
  ///
  Eigen::MatrixXd GetMeasurementJacobian();

  ///
  /// @brief Sensor position offset getter method
  /// @return Position offset vector
  ///
  Eigen::Vector3d GetPosOffset();

  ///
  /// @brief Sensor angular offset getter method
  /// @return Angular offset quaternion
  ///
  Eigen::Quaterniond GetAngOffset();

  ///
  /// @brief Sensor position offset setter method
  /// @param posOffset Position offset vector
  ///
  void SetPosOffset(Eigen::Vector3d posOffset);

  ///
  /// @brief Sensor angular offset setter method
  /// @param angOffset Angular offset quaternion
  ///
  void SetAngOffset(Eigen::Quaterniond angOffset);

  ///
  /// @brief Sensor name getter
  /// @return Sensor name
  ///
  std::string GetName();

  ///
  /// @brief Sensor ID getter method
  /// @return Sensor ID
  ///
  unsigned int GetId();

  ///
  /// @brief Sensor state start index getter method
  /// @return Sensor state start index
  ///
  unsigned int GetStateStartIndex();

  ///
  /// @brief Sensor state size getter method
  /// @return Sensor state size
  ///
  unsigned int GetStateSize();

  ///
  /// @brief Sensor state start setter method
  /// @param stateStartIndex Sensor state start
  ///
  void SetStateStartIndex(unsigned int stateStartIndex);

  ///
  /// @brief Sensor state setter
  /// @param state Sensor state vector
  ///
  void SetState(Eigen::VectorXd state);

protected:
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};          ///< @brief Sensor position offset vector
  Eigen::Quaterniond m_angOffset{0.0, 0.0, 0.0, 0.0};  ///< @brief Sensor angular offset quaternion

  unsigned int m_stateStartIndex{0};  ///< @brief Sensor state start index
  unsigned int m_stateSize{0};        ///< @brief Sensor state size

  static Eigen::Vector3d m_bodyPos;         ///< @brief Body position vector
  static Eigen::Vector3d m_bodyVel;         ///< @brief Body velocity vector
  static Eigen::Vector3d m_bodyAcc;         ///< @brief Body acceleration vector
  static Eigen::Quaterniond m_bodyAngPos;   ///< @brief Body angular position quaternion
  static Eigen::Vector3d m_bodyAngVel;      ///< @brief Body angular velocity vector
  static Eigen::Vector3d m_bodyAngAcc;      ///< @brief Body angular acceleration vector

private:
  unsigned int m_id;
  std::string m_name;
  static unsigned int _idCount;
};

#endif  // EKF__SENSORS__SENSOR_HPP_
