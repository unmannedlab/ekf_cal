// Copyright 2024 Jacob Hartzer
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

#include "sensors/gps.hpp"


TEST(test_gps, Callback) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(logger, 0.0, false, "");

  GPS::Parameters params;
  params.logger = logger;
  params.ekf = ekf;

  GPS gps(params);

  auto gps_message = std::make_shared<GpsMessage>();
  gps_message->sensor_id = 1;
  gps_message->sensor_type = SensorType::GPS;
  gps_message->time = 0.0;
  gps_message->gps_lla = Eigen::Vector3d{0.0, 0.0, 0.0};

  gps.Callback(gps_message);
}
