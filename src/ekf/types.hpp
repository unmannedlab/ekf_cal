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
  Eigen::VectorXd ToVector();

  ///
  /// @brief Function to set state using vector
  /// @param state vector for setting body state
  ///
  void SetState(Eigen::VectorXd state);

  Eigen::Vector3d m_position{0.0, 0.0, 0.0};             ///< @brief Body position
  Eigen::Vector3d m_velocity{0.0, 0.0, 0.0};             ///< @brief Body velocity
  Eigen::Vector3d m_acceleration{0.0, 0.0, 0.0};         ///< @brief Body acceleration
  Eigen::Quaterniond m_orientation{1.0, 0.0, 0.0, 0.0};  ///< @brief Body orientation
  Eigen::Vector3d m_angular_velocity{0.0, 0.0, 0.0};      ///< @brief Body angular rate
  Eigen::Vector3d m_angular_acceleration{0.0, 0.0, 0.0};  ///< @brief Body angular acceleration
};

///
/// @brief ImuState structure
///
typedef struct ImuState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};             ///< @brief IMU state position
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};  ///< @brief IMU state orientation
  Eigen::Vector3d acc_bias{0.0, 0.0, 0.0};              ///< @brief IMU state acceleration bias
  Eigen::Vector3d omg_bias{0.0, 0.0, 0.0};              ///< @brief IMU state angular rate bias
} ImuState;

///
/// @brief AugmentedState structure
///
typedef struct AugmentedState
{
  unsigned int frame_id;                                   ///< @brief Augmented frame ID
  Eigen::Vector3d imu_position{0.0, 0.0, 0.0};             ///< @brief Augmented IMU position
  Eigen::Quaterniond imu_orientation{1.0, 0.0, 0.0, 0.0};  ///< @brief Augmented IMU orientation
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
  std::vector<AugmentedState> augmented_states;         ///< @brief Camera augmented states
} CamState;

///
/// @brief FeatureTrack structure
/// @todo change to FeaturePoint
///
typedef struct FeatureTrack
{
  unsigned int frame_id;   ///< @brief Feature track frame ID
  cv::KeyPoint key_point;  ///< @brief Feature track key point
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
  Eigen::VectorXd ToVector();

  ///
  /// @brief Get EKF state size
  /// @return EKF state size as an integer
  ///
  unsigned int GetStateSize();

  BodyState m_body_state {};                        ///< @brief Body state
  std::map<unsigned int, ImuState> m_imu_states{};  ///< @brief IMU states
  std::map<unsigned int, CamState> m_cam_states{};  ///< @brief Camera States
};

BodyState & operator+=(BodyState & l_body_state, BodyState & r_body_state);
BodyState & operator+=(BodyState & l_body_state, Eigen::VectorXd & r_vector);
std::map<unsigned int, ImuState> & operator+=(
  std::map<unsigned int, ImuState> & l_imu_state,
  Eigen::VectorXd & r_vector);
std::map<unsigned int, CamState> & operator+=(
  std::map<unsigned int, CamState> & l_cam_state,
  Eigen::VectorXd & r_vector);
std::vector<AugmentedState> & operator+=(
  std::vector<AugmentedState> & l_augState,
  Eigen::VectorXd & r_vector);

State & operator+=(State & l_state, State & rState);
State & operator+=(State & l_state, Eigen::VectorXd & r_vector);

#endif  // EKF__TYPES_HPP_
