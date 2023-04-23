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


#ifndef SENSORS__SIM__SIM_IMU_MESSAGE_HPP_
#define SENSORS__SIM__SIM_IMU_MESSAGE_HPP_

#include "sensors/imu_message.hpp"

class SimImuMessage : public ImuMessage
{
public:
  ///
  /// @brief Define SimImuMessage constructor with ImuMessage's
  ///
  using ImuMessage::ImuMessage;
};


#endif  // SENSORS__SIM__SIM_IMU_MESSAGE_HPP_
