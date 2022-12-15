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

#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ekf/EKF.hpp"
#include "infrastructure/Logger.hpp"
#include "sensors/ros/RosCamera.hpp"
#include "sensors/ros/RosIMU.hpp"
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

  ///
  /// @brief Publish EKF state information
  ///
  void PublishState();

  ///
  /// @brief Publish sensor transforms
  ///
  void PublishTransforms();

  ///
  /// @brief Get transforms between body and sensors
  /// @param baseIMUName Name of base IMU representing body
  /// @param sensorNames Vector of sensor names
  /// @param sensorPosOffsets Vector of sensor positional offsets
  /// @param sensorAngOffsets Vector of sensor rotational offsets
  ///
  void GetTransforms(
    std::string & baseIMUName,
    std::vector<std::string> & sensorNames, std::vector<Eigen::Vector3d> & sensorPosOffsets,
    std::vector<Eigen::Quaterniond> & sensorAngOffsets);

private:
  /// @brief Calibration EKF object
  EKF * m_ekf = EKF::getInstance();

  /// @brief Vector of subscribers for IMU sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> m_IMUSubs;

  /// @brief Vector of subscribers for Camera sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_CameraSubs;

  bool m_baseIMUAssigned {false};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_PosePub;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_TwistPub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_StatePub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfBroadcaster;
  rclcpp::TimerBase::SharedPtr m_tfTimer;
  Logger * m_Logger = Logger::getInstance();
  std::map<int, std::shared_ptr<RosIMU>> m_mapIMU{};
  std::map<int, std::shared_ptr<RosCamera>> m_mapCamera{};
};

#endif  // APPLICATION__ROS__EKFCALNODE_HPP_
