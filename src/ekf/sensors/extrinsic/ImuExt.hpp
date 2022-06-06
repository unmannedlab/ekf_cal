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

#include "../Sensor.hpp"

///
/// @class IMU Extrinsic Sensor
///
class ImuExt : public Sensor
{
  public:
    typedef struct Params
    {
        double rate {1.0};
        Eigen::Vector3d posOffset {0.0, 0.0, 0.0};
        Eigen::Quaterniond quatOffset {1.0, 0.0, 0.0, 0.0};
        Eigen::Matrix3d obsCovR {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}};
    } Params;

    ///
    /// @brief
    ///
    ImuExt(ImuExt::Params params);

    const unsigned int STATE_SIZE {12U};

  protected:
  private:
};

#endif