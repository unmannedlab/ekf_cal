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

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "sensors/Sensor.hpp"
#include "sensors/Tracker.hpp"

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
  typedef struct Params
  {
    std::string name;                                  ///< @brief Camera name
    double rate{1.0};                                  ///< @brief Camera update rate
    Eigen::Vector3d posOffset{0.0, 0.0, 0.0};          ///< @brief Camera initial position offset
    Eigen::Quaterniond angOffset{1.0, 0.0, 0.0, 0.0};  ///< @brief Camera initial angular offset
    Eigen::VectorXd variance {{0, 0, 0, 0, 0, 0}};     ///< @brief Initial state variance
  } Params;

  ///
  /// @brief Camera sensor constructor
  /// @param cParams Parameter struct for camera sensor
  /// @param tParams Parameter struct for tracker
  ///
  Camera(Camera::Params cParams, Tracker::Params tParams);

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
  /// @brief Sensor state getter method
  /// @return Sensor state vector
  ///
  Eigen::VectorXd GetState();

  ///
  /// @brief Callback method for camera
  /// @param time Measurement time
  /// @param imgIn Image pointer
  ///
  void Callback(double time, cv::Mat & imgIn);

protected:
  cv::Mat m_outImg;

private:
  ///
  /// @brief Protected state setter method
  ///
  void SetState();


  std::vector<cv::KeyPoint> m_prevKeyPoints;
  std::vector<cv::KeyPoint> m_currKeyPoints;
  cv::Mat m_prevDescriptors;
  cv::Mat m_currDescriptors;

  Tracker m_tracker;
};

#endif  // SENSORS__CAMERA_HPP_
