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

#ifndef EKF__EKF_HPP_
#define EKF__EKF_HPP_

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "infrastructure/Logger.hpp"

///
/// @class EKF
/// @brief Calibration EKF class
/// @todo Implement check for correlation coefficients to be between +/- 1
/// @todo Add gravity initialization check
///
class EKF
{
private:
  static EKF * instancePointer;
  ///
  /// @brief EKF class constructor
  ///
  EKF() {}

public:
  ///
  /// @brief Delete copy constructor
  ///
  EKF(const EKF & obj) = delete;

  ///
  /// @brief Singleton reference getter
  /// @return Pointer to singleton instance
  ///
  static EKF * getInstance()
  {
    // If there is no instance of class
    // then we can create an instance.
    if (instancePointer == NULL) {
      // We can access private members
      // within the class.
      instancePointer = new EKF();

      // returning the instance pointer
      return instancePointer;
    } else {
      // if instancePointer != NULL that means
      // the class already have an instance.
      // So, we are returning that instance
      // and not creating new one.
      return instancePointer;
    }
  }

  ///
  /// @brief Getter for state vector reference
  /// @return State vector reference
  ///
  Eigen::VectorXd & getState();

  ///
  /// @brief Getter for the body state vector reference
  /// @return Body state vector reference
  ///
  Eigen::VectorXd getBodyState();

  ///
  /// @brief Getter method for state covariance matrix reference
  /// @return State covariance matrix reference
  ///
  Eigen::MatrixXd & getCov();

  ///
  /// @brief Getter method for state size
  /// @return State size
  ///
  unsigned int getStateSize();

  ///
  /// @brief Predict state to given time
  /// @param currentTime Time for prediction
  ///
  void predict(double currentTime);

  ///
  /// @brief State transition matrix getter method
  /// @param dT State transition time
  /// @return State transition matrix
  ///
  Eigen::MatrixXd getStateTransition(double dT);

  ///
  /// @brief Process input matrix getter method
  /// @return Process input matrix
  ///
  Eigen::MatrixXd getProcessInput();

  ///
  /// @brief EKF state initialization method
  /// @param timeInit Initial time
  /// @param bodyStateInit Initial state
  ///
  void initialize(double timeInit, Eigen::VectorXd bodyStateInit);

  ///
  /// @brief Get transforms between body and sensors
  /// @param baseIMUName Name of base IMU representing body
  /// @param sensorNames Vector of sensor names
  /// @param sensorPosOffsets Vector of sensor positional offsets
  /// @param sensorAngOffsets Vector of sensor rotational offsets
  ///
  void getTransforms(
    std::string & baseIMUName,
    std::vector<std::string> & sensorNames,
    std::vector<Eigen::Vector3d> & sensorPosOffsets,
    std::vector<Eigen::Quaterniond> & sensorAngOffsets
  );

  ///
  /// @brief Extend EKF state and covariance
  /// @param sensorStateSize Size of state extension
  /// @param sensorState State extension initialization
  /// @param sensorCov Covariance extension initialization
  ///
  void extendState(
    unsigned int sensorStateSize, Eigen::VectorXd sensorState,
    Eigen::MatrixXd sensorCov);

private:
  unsigned int m_stateSize{18U};
  Eigen::VectorXd m_state = Eigen::VectorXd::Zero(18U);
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(18U, 18U);
  double m_currentTime {0};
  bool m_timeInitialized {false};
  Logger * m_logger = Logger::getInstance();

  Eigen::MatrixXd m_processNoise = Eigen::MatrixXd::Identity(18U, 18U) * 1e-3;
  Eigen::MatrixXd m_processInput = Eigen::MatrixXd::Identity(18U, 18U) * 1e-3;
};

#endif  // EKF__EKF_HPP_
