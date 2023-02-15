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

#include "ekf/Types.hpp"
#include "infrastructure/Logger.hpp"

///
/// @class EKF
/// @brief Calibration EKF class
/// @todo Implement check for correlation coefficients to be between +/- 1
/// @todo Add gravity initialization/check
/// @todo Create generic function to update(r,H,R)
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
  /// @todo Remove singleton pattern
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
  State & getState();

  ///
  /// @brief Getter for the body state vector reference
  /// @return Body state vector reference
  ///
  BodyState getBodyState();

  ///
  /// @brief Getter method for state covariance matrix reference
  /// @return State covariance matrix reference
  ///
  Eigen::MatrixXd & getCov();

  ///
  /// @brief Predict state to given time
  /// @param currentTime Time for prediction
  ///
  void processModel(double currentTime);

  ///
  /// @brief State transition matrix getter method
  /// @param dT State transition time
  /// @return State transition matrix
  ///
  Eigen::MatrixXd getStateTransition(double dT);

  unsigned int getImuStateStartIndex(unsigned int m_id);

  unsigned int getCamStateStartIndex(unsigned int m_id);

  unsigned int getAugStateStartIndex(unsigned int camID, unsigned int frameID);

  ///
  /// @brief EKF state initialization method
  /// @param timeInit Initial time
  /// @param bodyStateInit Initial state
  ///
  void initialize(double timeInit, BodyState bodyStateInit);

  void registerIMU(unsigned int imuID, ImuState imuState, Eigen::MatrixXd covariance);

  void registerCamera(unsigned int camID, CamState camState, Eigen::MatrixXd covariance);

  void augmentState(unsigned int cameraID, unsigned int frameID);

  void update_msckf(unsigned int cameraID, FeatureTracks featureTracks);

  AugmentedState matchState(std::vector<AugmentedState> augmentedStates, unsigned int frameID);

private:
  unsigned int m_stateSize{18U};
  State m_state;
  Eigen::MatrixXd m_cov = Eigen::MatrixXd::Identity(18U, 18U);
  double m_currentTime {0};
  bool m_timeInitialized {false};
  Logger * m_logger = Logger::getInstance();

  Eigen::MatrixXd m_processNoise = Eigen::MatrixXd::Identity(18U, 18U) * 1e-3;
  Eigen::MatrixXd m_processInput = Eigen::MatrixXd::Identity(18U, 18U);
};

#endif  // EKF__EKF_HPP_
