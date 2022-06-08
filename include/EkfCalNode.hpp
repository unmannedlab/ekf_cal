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
/// @brief A ROS2 node for interfacing with the calibration EKF
/// @todo Testing
/// @todo Bias Stability and Noise process inputs for IMUs
/// @todo Combine Extrinsic/Intrinsic Sensors
/// @todo Make flags for Extrinsic/Intrinsic in base sensor
/// @todo Make flag for base sensor in IMU
/// @todo Camera Functions
/// @todo LIDAR Functions
/// @todo Software Paper
/// @todo Architecture Design
/// @todo TF2 Publishing Flag
/// @todo Debugging Info
/// @todo Warnings as errors
/// @todo Implement code style:
/// https://docs.ros.org/en/rolling/Contributing/Code-Style-Language-Versions.html#id1
/// @todo Implement static analysis:
/// https://docs.ros.org/en/rolling/Contributing/Quality-Guide.html#static-code-analysis-as-part-of-the-ament-package-build
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

  void ImuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    unsigned int id) const;
  void CameraCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    unsigned int id) const;
  void LidarCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg,
    unsigned int id) const;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subscription;

private:
  std::vector<std::string> m_imuList;
  std::vector<std::string> m_camList;
  std::vector<std::string> m_lidarList;

  EKF m_ekf;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> ImuSubs;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>
  CameraSubs;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr>
  LidarSubs;
};

#endif  // EKFCALNODE_HPP_
