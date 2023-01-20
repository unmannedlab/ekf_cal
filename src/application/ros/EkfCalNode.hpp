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

#ifndef APPLICATION__ROS__EKFCALNODE_HPP_
#define APPLICATION__ROS__EKFCALNODE_HPP_


#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "ekf/EKF.hpp"
#include "infrastructure/Logger.hpp"
#include "sensors/Camera.hpp"
#include "sensors/IMU.hpp"
#include "sensors/ros/RosCamera.hpp"
#include "sensors/ros/RosIMU.hpp"
#include "sensors/Tracker.hpp"

///
/// @class EkfCalNode
/// @brief A ROS2 node for EKF-based sensor calibration
/// @todo Bias Stability and Noise process inputs for IMUs
/// @todo Make flag for base sensor in IMU
/// @todo Camera Methods
/// @todo Software Paper
/// @todo Architecture Design
/// @todo TF2 Publishing Flag
/// @todo Debugging Info
/// @todo Warnings as errors
/// @todo Option to publish health metrics
/// @todo Option to publish visualization messages
///
class EkfCalNode : public rclcpp::Node
{
public:
  ///
  /// @brief Constructor for the Calibration EKF Node
  ///
  EkfCalNode();

  ///
  /// @brief Loading method for IMU sensors
  /// @param imuName Name of IMU to find and load from YAML
  ///
  void LoadIMU(std::string imuName);

  ///
  /// @brief Loading method for IMU sensors
  /// @param camName Name of IMU to find and load from YAML
  ///
  void LoadCamera(std::string camName);

  ///
  /// @brief Function for declaring and loading IMU parameters
  /// @param imuName Name of parameter structure
  /// @return imuParameters
  ///
  IMU::Params GetImuParameters(std::string imuName);

  ///
  /// @brief Function for declaring and loading camera parameters
  /// @param cameraName Name of parameter structure
  /// @return cameraParameters
  ///
  Camera::Params GetCameraParameters(std::string cameraName);

  ///
  /// @brief Function for declaring and loading tracker parameters
  /// @param trackerName Name of parameter structure
  /// @return trackerParameters
  ///
  Tracker::Params GetTrackerParameters(std::string trackerName);


  ///
  /// @brief Callback method for IMU sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id);
  ///
  /// @brief Callback method for Camera sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id);

private:
  /// @brief Vector of subscribers for IMU sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> m_IMUSubs;

  /// @brief Vector of subscribers for Camera sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_CameraSubs;

  bool m_baseIMUAssigned {false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imgPublisher;
  Logger * m_logger = Logger::getInstance();
  std::map<int, std::shared_ptr<RosIMU>> m_mapIMU{};
  std::map<int, std::shared_ptr<RosCamera>> m_mapCamera{};
};

#endif  // APPLICATION__ROS__EKFCALNODE_HPP_
