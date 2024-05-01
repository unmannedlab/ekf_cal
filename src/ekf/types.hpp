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

#include <opencv2/opencv.hpp>

///
/// @brief BodyState structure
/// @todo Make member variable naming consistent
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

  Eigen::Vector3d m_position{0.0, 0.0, 0.0};              ///< @brief Body position
  Eigen::Vector3d m_velocity{0.0, 0.0, 0.0};              ///< @brief Body velocity
  Eigen::Vector3d m_acceleration{0.0, 0.0, 0.0};          ///< @brief Body acceleration
  Eigen::Quaterniond m_ang_b_to_g{1.0, 0.0, 0.0, 0.0};    ///< @brief Body orientation
  Eigen::Vector3d m_angular_velocity{0.0, 0.0, 0.0};      ///< @brief Body angular rate
  Eigen::Vector3d m_angular_acceleration{0.0, 0.0, 0.0};  ///< @brief Body angular acceleration
};

///
/// @brief ImuState structure
///
class ImuState
{
public:
  ImuState() {}

  ///
  /// @brief Get IMU state as a vector
  /// @return IMU state as a vector
  ///
  Eigen::VectorXd ToVector();

  bool is_extrinsic{false};                           ///< @brief Extrinsic calibration flag
  bool is_intrinsic{false};                           ///< @brief Intrinsic calibration flag
  double pos_stability {1e-9};                        ///< @brief Extrinsic position stability
  double ang_stability {1e-9};                        ///< @brief Extrinsic orientation stability
  double acc_bias_stability {1e-9};                   ///< @brief Accelerometer bias stability
  double omg_bias_stability {1e-9};                   ///< @brief Gyroscope bias stability
  Eigen::Vector3d pos_i_in_b{0.0, 0.0, 0.0};          ///< @brief Position
  Eigen::Quaterniond ang_i_to_b{1.0, 0.0, 0.0, 0.0};  ///< @brief Orientation
  Eigen::Vector3d acc_bias{0.0, 0.0, 0.0};            ///< @brief Acceleration bias
  Eigen::Vector3d omg_bias{0.0, 0.0, 0.0};            ///< @brief Angular rate bias
};

///
/// @brief GpsState structure
///
class GpsState
{
public:
  GpsState() {}

  ///
  /// @brief Get GPS state as a vector
  /// @return GPS state as a vector
  ///
  Eigen::VectorXd ToVector();

  Eigen::Vector3d pos_a_in_b{0.0, 0.0, 0.0};  ///< @brief Antenna position in body frame
  double pos_stability {1e-9};                ///< @brief Antenna position stability
};

///
/// @brief AugmentedState structure
///
typedef struct AugmentedState
{
  int frame_id {-1};                                  ///< @brief Augmented frame ID
  Eigen::Vector3d pos_b_in_g{0.0, 0.0, 0.0};          ///< @brief Augmented IMU position
  Eigen::Quaterniond ang_b_to_g{1.0, 0.0, 0.0, 0.0};  ///< @brief Augmented IMU orientation
  Eigen::Vector3d pos_c_in_b{0.0, 0.0, 0.0};          ///< @brief Augmented camera position
  Eigen::Quaterniond ang_c_to_b{1.0, 0.0, 0.0, 0.0};  ///< @brief Augmented camera orientation
} AugmentedState;

///
/// @brief CamState structure
///
class CamState
{
public:
  CamState() {}

  ///
  /// @brief Get camera state as a vector
  /// @return Camera state as a vector
  ///
  Eigen::VectorXd ToVector();

  double pos_stability {1e-9};                        ///< @brief Extrinsic position stability
  double ang_stability {1e-9};                        ///< @brief Extrinsic orientation stability
  Eigen::Vector3d pos_c_in_b{0.0, 0.0, 0.0};          ///< @brief Camera state position
  Eigen::Quaterniond ang_c_to_b{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera state orientation
  std::vector<AugmentedState> augmented_states;       ///< @brief Camera augmented states
};

///
/// @brief FeaturePoint structure
///
typedef struct FeaturePoint
{
  int frame_id;   ///< @brief Feature track frame ID
  cv::KeyPoint key_point;  ///< @brief Feature track key point
} FeaturePoint;

typedef std::vector<std::vector<FeaturePoint>> FeatureTracks;

///
/// @brief BoardDetection structure
///
typedef struct BoardDetection
{
  int frame_id;          ///< @brief Image frame ID
  cv::Vec3d t_vec_f_in_c;  ///< @brief Rotation vector of the board
  cv::Vec3d r_vec_f_to_c;  ///< @brief Translation vector of the board
} BoardDetection;

typedef std::vector<BoardDetection> BoardTrack;

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
  std::map<unsigned int, GpsState> m_gps_states{};  ///< @brief GPS states
  std::map<unsigned int, CamState> m_cam_states{};  ///< @brief Camera States
};

BodyState & operator+=(BodyState & l_body_state, BodyState & r_body_state);
BodyState & operator+=(BodyState & l_body_state, Eigen::VectorXd & r_vector);
std::map<unsigned int, ImuState> & operator+=(
  std::map<unsigned int, ImuState> & l_imu_state,
  Eigen::VectorXd & r_vector);
std::map<unsigned int, GpsState> & operator+=(
  std::map<unsigned int, GpsState> & l_gps_state,
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
