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

class EkfCalNode : public rclcpp::Node
{
  public:
    ///
    /// @class EkfCalNode
    /// @brief A ROS2 node for interfacing with the calibration EKF
    /// @todo  Literally everything
    ///
    EkfCalNode();

    void LoadIntIMU(std::string imuName);
    void LoadExtIMU(std::string imuName);
    void LoadIntCam(std::string camName);
    void LoadExtCam(std::string camName);
    void LoadLIDAR(std::string lidarName);

  private:
    std::vector<std::string> m_imuList;
    std::vector<std::string> m_camList;
    std::vector<std::string> m_lidarList;

    EKF m_ekf;
};

#endif