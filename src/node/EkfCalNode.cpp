//--------------------------------------------------------------------------------------------------------------------//
//                                                                                                                    //
//                                                      EKF-Cal                                                       //
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

#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <string>

EkfCalNode::EkfCalNode() : Node("EkfCalNode")
{
    this->declare_parameter("IMU_list");
    this->declare_parameter("Cam_list");
    this->declare_parameter("LIDAR_list");
    m_imuList = this->get_parameter("IMU_list").as_string_array();

    // int id {0};
    for (std::string &imuName : m_imuList)
    {
        this->declare_parameter("IMUs." + imuName + ".Base");
        this->declare_parameter("IMUs." + imuName + ".Rate");
        this->declare_parameter("IMUs." + imuName + ".Topic");
        this->declare_parameter("IMUs." + imuName + ".PosInit");
        this->declare_parameter("IMUs." + imuName + ".QuatInit");
        this->declare_parameter("IMUs." + imuName + ".BiasInit");

        bool base                    = get_parameter("IMUs." + imuName + ".Base").as_bool();
        double rate                  = get_parameter("IMUs." + imuName + ".Rate").as_double();
        std::string topic            = get_parameter("IMUs." + imuName + ".Topic").as_string();
        std::vector<double> posInit  = get_parameter("IMUs." + imuName + ".PosInit").as_double_array();
        std::vector<double> biasInit = get_parameter("IMUs." + imuName + ".BiasInit").as_double_array();
        std::vector<double> quatInit = get_parameter("IMUs." + imuName + ".QuatInit").as_double_array();

        RCLCPP_INFO(get_logger(), "%s.base:     '%i'", imuName.c_str(), base);
        RCLCPP_INFO(get_logger(), "%s.rate:     '%f'", imuName.c_str(), rate);
        RCLCPP_INFO(get_logger(), "%s.topic:    '%s'", imuName.c_str(), topic.c_str());
        RCLCPP_INFO(get_logger(), "%s.posInit:  '[%f,%f,%f]'", imuName.c_str(), posInit[0], posInit[1], posInit[2]);
        RCLCPP_INFO(get_logger(), "%s.biasInit: '[%f,%f,%f]'", imuName.c_str(), biasInit[0], biasInit[1], biasInit[2]);
        RCLCPP_INFO(get_logger(), "%s.quatInit: '[%f,%f,%f,%f]'", imuName.c_str(), quatInit[0], quatInit[1],
                    quatInit[2], quatInit[3]);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfCalNode>());
    rclcpp::shutdown();

    return 0;
}