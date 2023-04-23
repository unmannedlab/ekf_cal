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

#include <memory>
#include <string>
#include <vector>

#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>

#include "ekf/Types.hpp"
#include "sensors/Sensor.hpp"
#include "trackers/FeatureTracker.hpp"

///
/// @class CameraMessage
/// @brief Camera message class
///
class CameraMessage : public SensorMessage
{
public:
  ///
  /// @brief Camera message constructor
  /// @param img_in Input image
  ///
  explicit CameraMessage(cv::Mat & img_in);

  /// @brief Message image
  cv::Mat & image;
};

///
/// @class Camera
/// @brief Camera Sensor Class
/// @todo Implement update methods
///
class Camera : public Sensor
{
public:
  ///
  /// @brief Camera initialization parameters structure
  ///
  typedef struct Parameters
  {
    std::string name;                                   ///< @brief Camera name
    std::string topic;                                  ///< @brief Camera topic name
    double rate{1.0};                                   ///< @brief Camera update rate
    Eigen::Vector3d pos_offset{0.0, 0.0, 0.0};          ///< @brief Camera initial position offset
    Eigen::Quaterniond ang_offset{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera initial angular offset
    Eigen::VectorXd variance {{0, 0, 0, 0, 0, 0}};      ///< @brief Initial state variance
    std::string tracker;                                ///< @brief Tracker name
    std::string output_directory {""};                  ///< @brief IMU data logging directory
    bool data_logging_on {false};                       ///< @brief IMU data logging flag
  } Parameters;

  ///
  /// @brief Camera sensor constructor
  /// @param cam_params Parameter struct for camera sensor
  ///
  explicit Camera(Camera::Parameters cam_params);

  void AddTracker(std::shared_ptr<FeatureTracker> tracker);

  ///
  /// @brief Callback method for camera
  /// @param camera_message camera message
  ///
  void Callback(std::shared_ptr<CameraMessage> camera_message);

protected:
  unsigned int GenerateFrameID();
  cv::Mat m_out_img;  ///< @brief Published output test image

private:
  std::vector<std::shared_ptr<FeatureTracker>> m_trackers;

  std::vector<double> m_rad_distortion_k{0.0, 0.0, 0.0};
  std::vector<double> m_tan_distortion_d{0.0, 0.0};
};

#endif  // SENSORS__CAMERA_HPP_
