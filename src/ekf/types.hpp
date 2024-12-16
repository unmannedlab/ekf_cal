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

#include "ekf/constants.hpp"

#include <opencv2/opencv.hpp>

enum class SensorType
{
  IMU,
  Camera,
  Tracker,
  GPS
};

enum class AugmentationType
{
  ALL,      // All camera frames are augmented
  PRIMARY,  // Only the primary camera frames are augmented
  TIME,     // Time-based frame augmentation
  ERROR     // Linear Error-based frame augmentation
};

///
/// @brief Camera intrinsics data structure
///
class Intrinsics
{
public:
  Intrinsics() {}

  ///
  /// @brief Generate camera matrix from intrinsics
  /// @return Camera matrix
  ///
  cv::Mat ToCameraMatrix();

  ///
  /// @brief Generate distortion vector from intrinsics
  /// @return Distortion vector
  ///
  cv::Mat ToDistortionVector();

  /// @todo Develop better defaults
  double f_x {0.01};           ///< @brief X focal length [px]
  double f_y {0.01};           ///< @brief Y focal length [px]
  double k_1 {0.0};            ///< @brief Radial coefficient 1
  double k_2 {0.0};            ///< @brief Radial coefficient 2
  double p_1 {0.0};            ///< @brief Tangential coefficient 1
  double p_2 {0.0};            ///< @brief Tangential coefficient 1
  double width {1920};         ///< @brief Image width [px]
  double height {1080};        ///< @brief Image height [px]
  double pixel_size {5.0e-6};  ///< @brief Pixel size [mm]
};

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
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief Function to set state using vector
  /// @param state vector for setting body state
  ///
  void SetState(Eigen::VectorXd state);

  Eigen::Vector3d pos_b_in_l{0.0, 0.0, 0.0};          ///< @brief Body position
  Eigen::Vector3d vel_b_in_l{0.0, 0.0, 0.0};          ///< @brief Body velocity
  Eigen::Vector3d acc_b_in_l{0.0, 0.0, 0.0};          ///< @brief Body acceleration
  Eigen::Quaterniond ang_b_to_l{1.0, 0.0, 0.0, 0.0};  ///< @brief Body orientation
  Eigen::Vector3d ang_vel_b_in_l{0.0, 0.0, 0.0};      ///< @brief Body angular velocity
  Eigen::Vector3d ang_acc_b_in_l{0.0, 0.0, 0.0};      ///< @brief Body angular acceleration
  unsigned int size{g_body_state_size};               ///< @brief State size
  int index{-1};                                      ///< @brief State index
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
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief is_extrinsic getter function
  /// @return is_extrinsic
  ///
  bool GetIsExtrinsic() const;
  ///
  /// @brief is_intrinsic getter function
  /// @return is_intrinsic
  ///
  bool GetIsIntrinsic() const;
  ///
  /// @brief is_extrinsic setter function
  /// @param extrinsic value to use for setting
  ///
  void SetIsExtrinsic(bool extrinsic);
  ///
  /// @brief is_intrinsic setter function
  /// @param intrinsic value to use for setting
  ///
  void SetIsIntrinsic(bool intrinsic);

  double pos_stability {1e-9};                        ///< @brief Extrinsic position stability
  double ang_stability {1e-9};                        ///< @brief Extrinsic orientation stability
  double acc_bias_stability {1e-9};                   ///< @brief Accelerometer bias stability
  double omg_bias_stability {1e-9};                   ///< @brief Gyroscope bias stability
  Eigen::Vector3d pos_i_in_b{0.0, 0.0, 0.0};          ///< @brief Position
  Eigen::Quaterniond ang_i_to_b{1.0, 0.0, 0.0, 0.0};  ///< @brief Orientation
  Eigen::Vector3d acc_bias{0.0, 0.0, 0.0};            ///< @brief Acceleration bias
  Eigen::Vector3d omg_bias{0.0, 0.0, 0.0};            ///< @brief Angular rate bias
  unsigned int size{0};                               ///< @brief State size
  int index{-1};                                      ///< @brief State index
  int index_intrinsic{-1};                            ///< @brief Intrinsic state index
  int index_extrinsic{-1};                            ///< @brief Extrinsic state index

private:
  void refresh_size();
  bool is_extrinsic{false};                           ///< @brief Extrinsic calibration flag
  bool is_intrinsic{false};                           ///< @brief Intrinsic calibration flag
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
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief is_extrinsic getter function
  /// @return is_extrinsic
  ///
  bool GetIsExtrinsic() const;
  ///
  /// @brief is_extrinsic setter function
  /// @param extrinsic value to use for setting
  ///
  void SetIsExtrinsic(bool extrinsic);

  Eigen::Vector3d pos_a_in_b{0.0, 0.0, 0.0};  ///< @brief Antenna position in body frame
  double pos_stability {1e-9};                ///< @brief Antenna position stability
  unsigned int size{0};                       ///< @brief State size
  int index{-1};                              ///< @brief State index

private:
  void refresh_size();
  bool is_extrinsic{false};                   ///< @brief Extrinsic calibration flag
};

///
/// @brief AugState structure
///
class AugState
{
public:
  AugState() {}

  ///
  /// @brief Get augmented state as a vector
  /// @return Augmented state as a vector
  ///
  Eigen::VectorXd ToVector() const;

  int frame_id {-1};                                  ///< @brief Augmented frame ID
  double time {0.0};                                  ///< @brief Augmented frame ID
  Eigen::Vector3d pos_b_in_l{0.0, 0.0, 0.0};          ///< @brief Augmented IMU position
  Eigen::Quaterniond ang_b_to_l{1.0, 0.0, 0.0, 0.0};  ///< @brief Augmented IMU orientation
  unsigned int size{g_aug_state_size};                ///< @brief State size
  int index{-1};                                      ///< @brief State index
  double alpha {0.0};                                 ///< @brief Interpolation Factor
};

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
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief is_extrinsic getter function
  /// @return is_extrinsic
  ///
  bool GetIsExtrinsic() const;
  ///
  /// @brief is_extrinsic setter function
  /// @param extrinsic value to use for setting
  ///
  void SetIsExtrinsic(bool extrinsic);

  double pos_stability {1e-9};                        ///< @brief Extrinsic position stability
  double ang_stability {1e-9};                        ///< @brief Extrinsic orientation stability
  Eigen::Vector3d pos_c_in_b{0.0, 0.0, 0.0};          ///< @brief Camera state position
  Eigen::Quaterniond ang_c_to_b{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera state orientation
  Intrinsics intrinsics;                              ///< @brief Camera Intrinsics
  double rate{1.0};                                   ///< @brief Frame rate
  unsigned int size{0};                               ///< @brief State size
  int index{-1};                                      ///< @brief State index

private:
  void refresh_size();
  bool is_extrinsic{false};
};

///
/// @brief FeaturePoint structure
///
typedef struct FeaturePoint
{
  int frame_id;            ///< @brief Feature track frame ID
  double frame_time;       ///< @brief Feature frame time
  cv::KeyPoint key_point;  ///< @brief Feature track key point
} FeaturePoint;

///
/// @brief FeatureTrack structure
///
typedef struct FeatureTrack
{
  std::vector<FeaturePoint> track;        ///< @brief Vector of tracked feature keypoints
  /// @todo: Remove true feature position once MSCKF complete
  Eigen::Vector3d true_feature_position;  ///< @brief True feature position (sim only)
} FeatureTrack;

///
/// @brief FeatureTracks type
///
typedef std::vector<FeatureTrack> FeatureTracks;

///
/// @brief BoardDetection structure
///
typedef struct BoardDetection
{
  int frame_id{-1};         ///< @brief Image frame ID
  double frame_time{-1.0};  ///< @brief Feature frame time
  cv::Vec3d t_vec_f_in_c;   ///< @brief Rotation vector of the board
  cv::Vec3d r_vec_f_to_c;   ///< @brief Translation vector of the board
} BoardDetection;

typedef std::vector<BoardDetection> BoardTrack;

///
/// @brief Fiducial state structure
///
class FidState
{
public:
  FidState() {}

  ///
  /// @brief Get fiducial state as a vector
  /// @return Fiducial state as a vector
  ///
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief is_extrinsic getter function
  /// @return is_extrinsic
  ///
  bool GetIsExtrinsic() const;
  ///
  /// @brief is_extrinsic setter function
  /// @param extrinsic value to use for setting
  ///
  void SetIsExtrinsic(bool extrinsic);

  int frame_id{-1};               ///< @brief Fiducial board ID
  Eigen::Vector3d pos_f_in_l;     ///< @brief Fiducial position in the local frame
  Eigen::Quaterniond ang_f_to_l;  ///< @brief Fiducial position in the local frame
  double pos_stability {1e-9};    ///< @brief Fiducial position stability
  double ang_stability {1e-9};    ///< @brief Fiducial orientation stability
  unsigned int size{0};           ///< @brief State size
  int index{-1};                  ///< @brief State index
  unsigned int id{0};             ///< @brief Fiducial ID

private:
  void refresh_size();
  bool is_extrinsic{false};       ///< @brief Extrinsic calibration flag
};

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
  Eigen::VectorXd ToVector() const;

  ///
  /// @brief Get EKF state size
  /// @return EKF state size as an integer
  ///
  unsigned int GetStateSize() const;

  BodyState body_state {};                        ///< @brief Body state
  std::map<unsigned int, ImuState> imu_states{};  ///< @brief IMU states
  std::map<unsigned int, GpsState> gps_states{};  ///< @brief GPS states
  std::map<unsigned int, CamState> cam_states{};  ///< @brief Camera states
  std::map<unsigned int, FidState> fid_states{};  ///< @brief Fiducial states
  std::map<unsigned int, std::vector<AugState>> aug_states{};  ///< @brief Fiducial states
};

BodyState & operator+=(BodyState & l_body_state, BodyState & r_body_state);
BodyState & operator+=(BodyState & l_body_state, Eigen::VectorXd & r_vector);
ImuState & operator+=(ImuState & l_imu_state, Eigen::VectorXd & r_vector);
std::map<unsigned int, ImuState> & operator+=(
  std::map<unsigned int, ImuState> & l_imu_state, Eigen::VectorXd & r_vector);
std::map<unsigned int, GpsState> & operator+=(
  std::map<unsigned int, GpsState> & l_gps_state, Eigen::VectorXd & r_vector);
std::map<unsigned int, CamState> & operator+=(
  std::map<unsigned int, CamState> & l_cam_state, Eigen::VectorXd & r_vector);
std::map<unsigned int, FidState> & operator+=(
  std::map<unsigned int, FidState> & l_fid_state, Eigen::VectorXd & r_vector);
std::vector<AugState> & operator+=(
  std::vector<AugState> & l_augState, Eigen::VectorXd & r_vector);

State & operator+=(State & l_state, State & rState);
State & operator+=(State & l_state, Eigen::VectorXd & r_vector);

enum class GpsInitType
{
  CONSTANT,
  BASELINE_DIST,
  ERROR_THRESHOLD
};

#endif  // EKF__TYPES_HPP_
