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

#include <eigen3/Eigen/Eigen>
#include <gtest/gtest.h>

#include "ekf/update/fiducial_updater.hpp"

TEST(test_imu_updater, constructor) {
  int cam_id {1U};
  Eigen::Vector3d fiducial_pos{0.0, 0.0, 0.0};
  Eigen::Quaterniond fiducial_ang{1.0, 0.0, 0.0, 0.0};
  std::string log_file_directory("");
  bool data_logging_on{true};
  double data_log_rate{1.0};
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");

  FiducialUpdater fiducial_updater(
    cam_id,
    fiducial_pos,
    fiducial_ang,
    log_file_directory,
    data_logging_on,
    data_log_rate,
    logger);
}
