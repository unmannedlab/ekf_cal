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

#ifndef SENSORS__CAMERA_MESSAGE_HPP_
#define SENSORS__CAMERA_MESSAGE_HPP_

#include <opencv2/opencv.hpp>

#include "sensors/sensor_message.hpp"

///
/// @class CameraMessage
/// @brief Camera message class
///
class CameraMessage : public SensorMessage
{
public:
  ///
  /// @brief Camera message constructor
  /// @param img_in Input image
  ///
  explicit CameraMessage(cv::Mat img_in);

  /// @brief Message image
  cv::Mat image;
};

#endif  // SENSORS__CAMERA_MESSAGE_HPP_
