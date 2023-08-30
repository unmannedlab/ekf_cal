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

#ifndef APPLICATION__ROS__NODE__EKF_CAL_NODE_HPP_
#define APPLICATION__ROS__NODE__EKF_CAL_NODE_HPP_


#include <cstdio>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "ekf/ekf.hpp"
#include "infrastructure/debug_logger.hpp"
#include "sensors/camera.hpp"
#include "sensors/imu.hpp"
#include "sensors/ros/ros_camera.hpp"
#include "sensors/ros/ros_imu.hpp"
#include "trackers/feature_tracker.hpp"

///
/// @class EkfCalNode
/// @brief A ROS2 node for EKF-based sensor calibration
/// @todo Option to publish health metrics
/// @todo Option to publish visualization messages
/// @todo Create generic callback that can be used to store and sort measurements
/// @todo TF2 Publishing Flag
/// @todo Option to publish health metrics
/// @todo Option to publish visualization messages
/// @todo debug issue with future extrapolation in RVIZ
///
class EkfCalNode : public rclcpp::Node
{
public:
  ///
  /// @brief Constructor for the Calibration EKF Node
  ///
  EkfCalNode();

  ///
  /// @brief Initialize EKF calibration node
  ///
  void Initialize();

  ///
  /// @brief Load lists of sensors
  ///
  void LoadSensors();

  ///
  /// @brief Loading method for IMU sensors
  /// @param imu_name Name of IMU to find and load from YAML
  ///
  void LoadIMU(std::string imu_name);

  ///
  /// @brief Loading method for IMU sensors
  /// @param cam_name Name of IMU to find and load from YAML
  ///
  void LoadCamera(std::string cam_name);

  ///
  /// @brief Function for declaring and loading IMU parameters
  /// @param imu_name Name of parameter structure
  /// @return imuParameters
  ///
  IMU::Parameters GetImuParameters(std::string imu_name);

  ///
  /// @brief Function for declaring and loading camera parameters
  /// @param camera_name Name of parameter structure
  /// @return cameraParameters
  ///
  Camera::Parameters GetCameraParameters(std::string camera_name);

  ///
  /// @brief Declare parameters for all sensors
  ///
  void DeclareSensors();

  ///
  /// @brief Declare IMU parameters
  /// @param imu_name IMU parameter
  ///
  void DeclareImuParameters(std::string imu_name);

  ///
  /// @brief Declare camera parameters
  /// @param camera_name Camera parameter
  ///
  void DeclareCameraParameters(std::string camera_name);

  ///
  /// @brief Declare tracker PARAMETERS
  /// @param tracker_name Tracker parameter
  ///
  void DeclareTrackerParameters(std::string tracker_name);

  ///
  /// @brief Function for declaring and loading tracker parameters
  /// @param tracker_name Name of parameter structure
  /// @return trackerParameters
  ///
  FeatureTracker::Parameters GetTrackerParameters(std::string tracker_name);
  ///
  /// @brief Callback method for IMU sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id);
  ///
  /// @brief Callback method for Camera sensor messages
  /// @param msg Sensor message pointer
  /// @param id Sensor ID number
  ///
  void CameraCallback(const sensor_msgs::msg::Image::SharedPtr msg, unsigned int id);

  ///
  /// @brief Register IMU sensor
  /// @param imu_ptr IMU sensor shared pointer
  /// @param topic Topic to subscribe
  ///
  void RegisterImu(std::shared_ptr<RosIMU> imu_ptr, std::string topic);
  ///
  /// @brief Register camera sensor
  /// @param cam_ptr Camera sensor shared pointer
  /// @param topic Topic to subscribe
  ///
  void RegisterCamera(std::shared_ptr<RosCamera> cam_ptr, std::string topic);

  ///
  /// @brief State publisher callback
  ///
  void PublishState();

private:
  /// @brief Vector of subscribers for IMU sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> m_imu_subs;

  /// @brief Vector of subscribers for Camera sensor messages
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> m_camera_subs;

  std::vector<std::string> m_imu_list {};
  std::vector<std::string> m_camera_list {};
  std::vector<std::string> m_tracker_list {};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_img_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr m_state_pub;
  rclcpp::TimerBase::SharedPtr m_state_pub_timer;

  EKF * m_ekf = EKF::GetInstance();
  DebugLogger * m_logger = DebugLogger::GetInstance();
  DataLogger m_state_data_logger;


  std::map<int, std::shared_ptr<RosIMU>> m_map_imu{};
  std::map<int, std::shared_ptr<RosCamera>> m_map_camera{};
};

#endif  // APPLICATION__ROS__NODE__EKF_CAL_NODE_HPP_
