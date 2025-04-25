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

#ifndef SENSORS__CAMERA_HPP_
#define SENSORS__CAMERA_HPP_

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"
#include "sensors/camera_message.hpp"
#include "sensors/sensor.hpp"
#include "trackers/feature_tracker.hpp"
#include "trackers/fiducial_tracker.hpp"


///
/// @class Camera
/// @brief Camera Sensor Class
///
class Camera : public Sensor
{
public:
  ///
  /// @brief Camera initialization parameters structure
  ///
  typedef struct Parameters : public Sensor::Parameters
  {
    bool is_extrinsic{false};                           ///< @brief Flag for extrinsic calibration
    Eigen::Vector3d pos_c_in_b{0.0, 0.0, 0.0};          ///< @brief Camera initial position offset
    Eigen::Quaterniond ang_c_to_b{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera initial angular offset
    double pos_stability {1e-9};                        ///< @brief Position stability
    double ang_stability {1e-9};                        ///< @brief Angular stability
    Eigen::VectorXd variance {{0, 0, 0, 0, 0, 0}};      ///< @brief Initial state variance
    std::string tracker;                                ///< @brief Tracker name
    std::string fiducial;                               ///< @brief Fiducial name
    Intrinsics intrinsics;                              ///< @brief Camera intrinsics
  } Parameters;

  ///
  /// @brief Camera sensor constructor
  /// @param cam_params Parameter struct for camera sensor
  ///
  explicit Camera(Camera::Parameters cam_params);

  ///
  /// @brief Method to add tracker object to camera sensor
  /// @param tracker Tracker pointer for camera to use during callbacks
  ///
  void AddTracker(std::shared_ptr<FeatureTracker> tracker);

  ///
  /// @brief Method to add fiducial object to camera sensor
  /// @param fiducial Fiducial pointer for camera to use during callbacks
  ///
  void AddFiducial(std::shared_ptr<FiducialTracker> fiducial);

  ///
  /// @brief Callback method for camera
  /// @param camera_message camera message
  ///
  void Callback(const CameraMessage & camera_message);

  cv::Mat m_out_img{0, 0, CV_8UC1};  ///< @brief Published output test image

protected:
  unsigned int GenerateFrameID();

  std::shared_ptr<EKF> m_ekf;  ///< @brief EKF to update

private:
  std::map<unsigned int, std::shared_ptr<FeatureTracker>> m_trackers;
  std::map<unsigned int, std::shared_ptr<FiducialTracker>> m_fiducials;

  std::vector<double> m_rad_distortion_k{0.0, 0.0, 0.0};
  std::vector<double> m_tan_distortion_d{0.0, 0.0};
};

#endif  // SENSORS__CAMERA_HPP_
