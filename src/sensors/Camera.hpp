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
  /// @param imgIn Input image
  ///
  explicit CameraMessage(cv::Mat & imgIn);

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
    std::string name;                                  ///< @brief Camera name
    std::string topic;                                 ///< @brief Camera topic name
    double rate{1.0};                                  ///< @brief Camera update rate
    Eigen::Vector3d posOffset{0.0, 0.0, 0.0};          ///< @brief Camera initial position offset
    Eigen::Quaterniond angOffset{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera initial angular offset
    Eigen::VectorXd variance {{0, 0, 0, 0, 0, 0}};     ///< @brief Initial state variance
    std::string tracker;                               ///< @brief Tracker name
    std::string outputDirectory {""};                  ///< @brief IMU data logging directory
    bool dataLoggingOn {false};                        ///< @brief IMU data logging flag
  } Parameters;

  ///
  /// @brief Camera sensor constructor
  /// @param cParams Parameter struct for camera sensor
  ///
  explicit Camera(Camera::Parameters cParams);

  void addTracker(std::shared_ptr<FeatureTracker> tracker);

  ///
  /// @brief Callback method for camera
  /// @param cameraMessage camera message
  ///
  void callback(std::shared_ptr<CameraMessage> cameraMessage);

protected:
  unsigned int generateFrameID();
  cv::Mat m_outImg;  ///< @brief Published output test image

private:
  std::vector<std::shared_ptr<FeatureTracker>> m_trackers;

  std::vector<double> m_radDistortionK{0.0, 0.0, 0.0};
  std::vector<double> m_tanDistortionD{0.0, 0.0};
};

#endif  // SENSORS__CAMERA_HPP_
