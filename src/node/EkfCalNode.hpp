//--------------------------------------------------------------------------------------------------------------------//
//                                                                                                                    //
//                                                      EKF-CAL                                                       //
//                                                                                                                    //
//                                       Kalman Filter-Based Sensor Calibration                                       //
//                                                                                                                    //
//                                          Copyright (C) 2021 Jacob Hartzer                                          //
//                                                                                                                    //
// This program is free software: you can redistribute it and/or modify it under the terms of the                     //
// GNU General Public License as published by the Free Software Foundation, either version 3 of the License,          //
// or (at your option) any later version.                                                                             //
//                                                                                                                    //
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;                          //
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                          //
// See the GNU General Public License for more details.                                                               //
//                                                                                                                    //
// You should have received a copy of the GNU General Public License along with this program.                         //
// If not, see <https://www.gnu.org/licenses/>.                                                                       //
//                                                                                                                    //
//--------------------------------------------------------------------------------------------------------------------//

#ifndef EKF_CAL_NODE_HPP
#define EKF_CAL_NODE_HPP

#include "ekf/EKF.hpp"

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

class EkfCalNode : public rclcpp::Node
{
  public:
    ///
    /// @class EkfCalNode
    /// @brief A ROS2 node for interfacing with the calibration EKF
    /// @todo  Literally everything
    ///
    EkfCalNode();

    ///
    /// @brief Loading function for Extrinsic IMU sensors
    /// @param imuName Name of IMU to find and load from YAML
    ///
    void LoadImuExt(std::string imuName);

    ///
    /// @brief Loading function for Intrinsic IMU sensors
    /// @param imuName Name of IMU to find and load from YAML
    ///
    void LoadImuInt(std::string imuName);

    ///
    /// @brief Loading function for Extrinsic IMU sensors
    /// @param camName Name of IMU to find and load from YAML
    ///
    void LoadCameraExt(std::string camName);

    ///
    /// @brief Loading function for Intrinsic IMU sensors
    /// @param camName Name of IMU to find and load from YAML
    ///
    void LoadCameraInt(std::string camName);

    ///
    /// @brief Loading function for Extrinsic IMU sensors
    /// @param lidarName Name of LIDAR to find and load from YAML
    ///
    void LoadLidarExt(std::string lidarName);

    void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id) const;
    void CameraCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id) const;
    void LidarCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id) const;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_subscription;

  private:
    std::vector<std::string> m_imuList;
    std::vector<std::string> m_camList;
    std::vector<std::string> m_lidarList;

    EKF m_ekf;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr> ImuSubs;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> CameraSubs;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr> LidarSubs;
};

#endif