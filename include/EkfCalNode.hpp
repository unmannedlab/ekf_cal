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

#ifndef EKFCALNODE_HPP_
#define EKFCALNODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

#include <cstdio>
#include <string>
#include <vector>

#include "ekf/EKF.hpp"

///
/// @class EkfCalNode
/// @brief A ROS2 node for EKF-based sensor calibration
/// @todo Bias Stability and Noise process inputs for IMUs
/// @todo Make flag for base sensor in IMU
/// @todo Camera Functions
/// @todo LIDAR Functions
/// @todo Software Paper
/// @todo Architecture Design
/// @todo TF2 Publishing Flag
/// @todo Debugging Info
/// @todo Warnings as errors
///
class EkfCalNode : public rclcpp::Node
{
public:
  ///
  /// @brief Constructor for the Calibration EKF Node
  /// @todo  Literally everything
  ///
  EkfCalNode();

  ///
  /// @brief Loading function for IMU sensors
  /// @param imuName Name of IMU to find and load from YAML
  ///
  void LoadImu(std::string imuName);

  ///
  /// @brief Loading function for IMU sensors
  /// @param camName Name of IMU to find and load from YAML
  ///
  void LoadCamera(std::string camName);

  ///
  /// @brief Loading function for IMU sensors
  /// @param lidarName Name of LIDAR to find and load from YAML
  ///
  void LoadLidar(std::string lidarName);

  ///
  /// @brief Callback function for Imu sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id);
  ///
  /// @brief Callback function for Camera sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id);
  ///
  /// @brief Callback function for Lidar sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void LidarCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg, unsigned int id);

private:
  /// @brief Calibration EKF object
  EKF m_ekf;

  /// @brief Vector of subscribers for IMU sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> m_ImuSubs;

  /// @brief Vector of subscribers for Camera sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_CameraSubs;

  /// @brief Vector of subscribers for Lidar sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr> m_LidarSubs;

  bool m_baseImuAssigned {false};
};

#endif  // EKFCALNODE_HPP_
