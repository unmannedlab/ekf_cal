// Copyright 2023 Jacob Hartzer
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

#ifndef EKF__TYPES_HPP_
#define EKF__TYPES_HPP_

#include <eigen3/Eigen/Eigen>

#include <map>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

///
/// @brief BodyState structure
///
class BodyState
{
public:
  ///
  /// @brief EKF State constructor
  ///
  BodyState() {}

  ///
  /// @brief Get EKF state as a vector
  /// @return EKF state as a vector
  ///
  Eigen::VectorXd toVector();

  ///
  /// @brief Function to set state using vector
  /// @param state vector for setting body state
  ///
  void SetState(Eigen::VectorXd state);

  Eigen::Vector3d position{0.0, 0.0, 0.0};             ///< @brief Body state position
  Eigen::Vector3d velocity{0.0, 0.0, 0.0};             ///< @brief Body state velocity
  Eigen::Vector3d acceleration{0.0, 0.0, 0.0};         ///< @brief Body state acceleration
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  ///< @brief Body state orientation
  Eigen::Vector3d angularVelocity{0.0, 0.0, 0.0};      ///< @brief Body state angular rate
  Eigen::Vector3d angularAcceleration{0.0, 0.0, 0.0};  ///< @brief Body state angular acceleration
};

///
/// @brief ImuState structure
///
typedef struct ImuState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};             ///< @brief IMU state position
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  ///< @brief IMU state orientation
  Eigen::Vector3d accBias{0.0, 0.0, 0.0};              ///< @brief IMU state acceleration bias
  Eigen::Vector3d omgBias{0.0, 0.0, 0.0};              ///< @brief IMU state angular rate bias
} ImuState;

///
/// @brief AugmentedState structure
///
typedef struct AugmentedState
{
  unsigned int frameID;                                   ///< @brief Augmented frame ID
  Eigen::Vector3d imuPosition{0.0, 0.0, 0.0};             ///< @brief Augmented IMU position
  Eigen::Quaterniond imuOrientation{1.0, 0.0, 0.0, 0.0};  ///< @brief Augmented IMU orientation
  Eigen::Vector3d position{0.0, 0.0, 0.0};                ///< @brief Augmented position
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};     ///< @brief Augmented orientation
} AugmentedState;

///
/// @brief CamState structure
///
typedef struct CamState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};             ///< @brief Camera state position
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera state orientation
  std::vector<AugmentedState> augmentedStates;         ///< @brief Camera augmented states
} CamState;

///
/// @brief FeatureTrack structure
/// @todo change to FeaturePoint
///
typedef struct FeatureTrack
{
  unsigned int frameID;   ///< @brief Feature track frame ID
  cv::KeyPoint keyPoint;  ///< @brief Feature track keypoint
} FeatureTrack;

typedef std::vector<std::vector<FeatureTrack>> FeatureTracks;


///
/// @class State
/// @brief EKF State Class
///
class State
{
public:
  ///
  /// @brief EKF State constructor
  ///
  State() {}

  ///
  /// @brief Get EKF state as a vector
  /// @return EKF state as a vector
  ///
  Eigen::VectorXd toVector();

  ///
  /// @brief Get EKF state size
  /// @return EKF state size as an integer
  ///
  unsigned int getStateSize();

  BodyState bodyState {};                        ///< @brief Body state
  std::map<unsigned int, ImuState> imuStates{};  ///< @brief IMU states
  std::map<unsigned int, CamState> camStates{};  ///< @brief Camera States
};

BodyState & operator+=(BodyState & lBodyState, BodyState & rBodyState);
BodyState & operator+=(BodyState & lBodyState, Eigen::VectorXd & rVector);
std::map<unsigned int, ImuState> & operator+=(
  std::map<unsigned int, ImuState> & lImuState,
  Eigen::VectorXd & rVector);
State & operator+=(State & lState, State & rState);
State & operator+=(State & lState, Eigen::VectorXd & rVector);

#endif  // EKF__TYPES_HPP_
