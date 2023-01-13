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

#include <gtest/gtest.h>
#include <iostream>

#include "sensors/Camera.hpp"
#include "sensors/ros/RosCamera.hpp"
#include "sensors/Tracker.hpp"

///
/// @todo Write this test
///
TEST(test_RosCamera, constructor) {
  Camera::Params cParams;
  Tracker::Params tParams;
  cParams.name = "test_Camera";

  /// @todo: Re-enable this test once ROS node is not inside camera
  // RosCamera RosCamera(cParams, tParams);
  EXPECT_TRUE(true);
}
