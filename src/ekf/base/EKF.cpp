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

#include "ekf/base/EKF.hpp"

#include "ekf/update/Camera.hpp"
#include "ekf/update/IMU.hpp"
#include "ekf/update/LIDAR.hpp"

EKF::EKF()
{
}

void EKF::RegisterSensor(Sensor::Params params)
{
}

// void EKF::RegisterIMU(IMU::Params params)
// {
//     IMU imu = IMU(params);
//     m_imuList.push_back(imu);
// }

// void EKF::RegisterCamera(Camera::Params params)
// {
//     Camera cam = Camera(params);
//     m_camList.push_back(cam);
// }

// void EKF::RegisterLIDAR(LIDAR::Params params)
// {
//     LIDAR lidar = LIDAR(params);
//     m_lidarList.push_back(lidar);
// }
