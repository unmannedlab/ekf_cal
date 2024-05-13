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

#include <memory>

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>

#include "trackers/fiducial_tracker.hpp"
#include "sensors/imu.hpp"
#include "sensors/camera.hpp"

TEST(test_fiducial_tracker, track) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::DEBUG, "");
  auto ekf = std::make_shared<EKF>(logger, 10.0, false, "");

  IMU::Parameters imu_params;
  imu_params.ekf = ekf;
  imu_params.logger = logger;
  IMU imu(imu_params);

  Camera::Parameters cam_params;
  cam_params.ekf = ekf;
  cam_params.logger = logger;
  Camera cam(cam_params);

  FiducialTracker::Parameters fid_params;
  fid_params.detector_type = FiducialTypeEnum::CHARUCO_BOARD;
  fid_params.squares_x = 5;
  fid_params.squares_y = 7;
  fid_params.square_length = 0.04;
  fid_params.marker_length = 0.02;
  fid_params.initial_id = 0;
  fid_params.camera_id = cam.GetId();
  fid_params.ekf = ekf;
  fid_params.logger = logger;
  fid_params.max_track_length = 1;

  auto fid_tracker = std::make_shared<FiducialTracker>(fid_params);
  cam.AddFiducial(fid_tracker);

  CamState cam_state;
  Eigen::MatrixXd cam_covariance(6, 6);

  cv::Ptr<cv::aruco::Dictionary> dictionary =
    cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::CharucoBoard> board =
    cv::aruco::CharucoBoard::create(5, 7, 0.04f, 0.02f, dictionary);
  cv::Mat boardImage;
  board->draw(cv::Size(600, 500), boardImage, 10, 1);

  auto cam_msg = std::make_shared<CameraMessage>(boardImage);
  cam.Callback(cam_msg);
}
