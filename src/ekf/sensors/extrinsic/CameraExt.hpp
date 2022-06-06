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

#ifndef CAMERA_EXT_HPP
#define CAMERA_EXT_HPP

#include "../Sensor.hpp"

///
/// @class Camera Extrinsic Sensor
///
class CameraExt : public Sensor
{
  public:
    typedef struct Params
    {
        double rate {1.0};
        Eigen::Vector3d posOffset {0.0, 0.0, 0.0};
        Eigen::Quaterniond quatOffset {1.0, 0.0, 0.0, 0.0};
    } Params;

    ///
    /// @class Sensor
    /// @brief
    ///
    CameraExt(CameraExt::Params params);

    const unsigned int STATE_SIZE {6U};

  protected:
  private:
};

#endif