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

#ifndef IMU_EXT_HPP
#define IMU_EXT_HPP

#include "Sensor.hpp"

///
/// @class IMU Sensor Class
///
class Imu : public Sensor
{
  public:
    typedef struct Params
    {
        std::string name;
        bool baseSensor {false};
        bool intrinsic {false};
        double rate {1.0};
        Eigen::Vector3d posOffset {0.0, 0.0, 0.0};
        Eigen::Quaterniond quatOffset {1.0, 0.0, 0.0, 0.0};
        Eigen::Vector3d accBias {0.0, 0.0, 0.0};
        Eigen::Vector3d omgBias {0.0, 0.0, 0.0};
        Eigen::Matrix3d obsCovR {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    } Params;

    ///
    /// @brief
    ///
    Imu(Imu::Params params);

  protected:
  private:
};

#endif