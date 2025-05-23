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
  EKF::Parameters ekf_params;
  ekf_params.gps_init_type = GpsInitType::ERROR_THRESHOLD;
  ekf_params.gps_init_pos_thresh = 10.0;
  ekf_params.gps_init_ang_thresh = 10.0;
  ekf_params.debug_logger = std::make_shared<DebugLogger>(LogLevel::INFO, "");
  auto ekf = std::make_shared<EKF>(ekf_params);
  ekf->m_state.body_state.vel_b_in_l.x() = 1;

  GPS::Parameters params;
  params.logger = ekf_params.debug_logger;
  params.ekf = ekf;

  GPS gps(params);

  GpsMessage gps_message;
  gps_message.sensor_id = 1;
  gps_message.sensor_type = SensorType::GPS;
  gps_message.gps_lla = Eigen::Vector3d{0.0, 0.0, 0.0};

  for (unsigned int i = 0; i < 10; ++i) {
    gps_message.time = static_cast<double>(i);
    gps.Callback(gps_message);
  }
}
