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

#include "EkfCalNode.hpp"

#include "TypeHelper.hpp"
#include "ekf/EKF.hpp"
#include "ekf/sensors/Camera.hpp"
#include "ekf/sensors/Imu.hpp"
#include "ekf/sensors/Lidar.hpp"

#include <cstdio>
#include <eigen3/Eigen/Eigen>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <string>

using std::placeholders::_1;

EkfCalNode::EkfCalNode() : Node("EkfCalNode")
{
    this->declare_parameter("IMU_list");
    this->declare_parameter("Camera_list");
    this->declare_parameter("LIDAR_list");
    m_imuList   = this->get_parameter("IMU_list").as_string_array();
    m_camList   = this->get_parameter("Camera_list").as_string_array();
    m_lidarList = this->get_parameter("LIDAR_list").as_string_array();

    for (std::string &imuName : m_imuList)
    {
        LoadImu(imuName);
    }

    for (std::string &camName : m_camList)
    {
        LoadCamera(camName);
    }

    for (std::string &lidarName : m_lidarList)
    {
        LoadLidar(lidarName);
    }
}

void EkfCalNode::LoadImu(std::string imuName)
{
    this->declare_parameter("IMUs." + imuName + ".Intrinsic");
    this->declare_parameter("IMUs." + imuName + ".Rate");
    this->declare_parameter("IMUs." + imuName + ".Topic");
    this->declare_parameter("IMUs." + imuName + ".PosOffInit");
    this->declare_parameter("IMUs." + imuName + ".QuatOffInit");
    this->declare_parameter("IMUs." + imuName + ".AccBiasInit");
    this->declare_parameter("IMUs." + imuName + ".OmgBiasInit");

    bool intrinsic              = this->get_parameter("IMUs." + imuName + ".Intrinsic").as_bool();
    double rate                 = this->get_parameter("IMUs." + imuName + ".Rate").as_double();
    std::string topic           = this->get_parameter("IMUs." + imuName + ".Topic").as_string();
    std::vector<double> posOff  = this->get_parameter("IMUs." + imuName + ".PosOffInit").as_double_array();
    std::vector<double> quatOff = this->get_parameter("IMUs." + imuName + ".QuatOffInit").as_double_array();
    std::vector<double> accBias = this->get_parameter("IMUs." + imuName + ".AccBiasInit").as_double_array();
    std::vector<double> omgBias = this->get_parameter("IMUs." + imuName + ".OmgBiasInit").as_double_array();

    Imu::Params imuParams;
    imuParams.name       = imuName;
    imuParams.intrinsic  = intrinsic;
    imuParams.rate       = rate;
    imuParams.posOffset  = TypeHelper::StdToEigVec(posOff);
    imuParams.quatOffset = TypeHelper::StdToEigQuat(quatOff);
    imuParams.accBias    = TypeHelper::StdToEigVec(accBias);
    imuParams.omgBias    = TypeHelper::StdToEigVec(omgBias);

    unsigned int id = m_ekf.RegisterSensor<Imu>(imuParams);
    std::function<void(std::shared_ptr<sensor_msgs::msg::Imu>)> function;
    function = std::bind(&EkfCalNode::ImuCallback, this, _1, id);
    ImuSubs.push_back(this->create_subscription<sensor_msgs::msg::Imu>(topic, 10, function));

    RCLCPP_INFO(get_logger(), "Loaded Intrinsic IMU: '%s'", imuName.c_str());
}

void EkfCalNode::LoadCamera(std::string camName)
{
    RCLCPP_INFO(get_logger(), "Camera not Loaded: '%s'", camName.c_str());
}

void EkfCalNode::LoadLidar(std::string lidarName)
{
    RCLCPP_INFO(get_logger(), "LIDAR not Loaded: '%s'", lidarName.c_str());
}

void EkfCalNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id) const
{
}

void EkfCalNode::CameraCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id) const
{
}

void EkfCalNode::LidarCallback(const sensor_msgs::msg::Imu::SharedPtr msg, unsigned int id) const
{
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfCalNode>());
    rclcpp::shutdown();

    return 0;
}