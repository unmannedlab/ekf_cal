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

#include <memory>

#include "ekf/ekf.hpp"
#include "trackers/feature_tracker.hpp"
#include "sensors/imu.hpp"
#include "sensors/camera.hpp"

TEST(test_feature_tracker, initialization) {
  FeatureTracker::Parameters params;
  params.camera_id = 1;

  params.detector = FeatureDetectorEnum::BRISK;
  params.descriptor = DescriptorExtractorEnum::ORB;
  params.matcher = DescriptorMatcherEnum::BRUTE_FORCE;
  FeatureTracker feature_tracker_1 {params};

  params.detector = FeatureDetectorEnum::FAST;
  FeatureTracker feature_tracker_2 {params};

  params.detector = FeatureDetectorEnum::GFTT;
  FeatureTracker feature_tracker_3 {params};

  params.detector = FeatureDetectorEnum::MSER;
  FeatureTracker feature_tracker_4 {params};

  params.detector = FeatureDetectorEnum::ORB;
  FeatureTracker feature_tracker_5 {params};

  params.detector = FeatureDetectorEnum::SIFT;
  FeatureTracker feature_tracker_6 {params};

  params.descriptor = DescriptorExtractorEnum::SIFT;
  FeatureTracker feature_tracker_7 {params};

  params.matcher = DescriptorMatcherEnum::FLANN;
  FeatureTracker feature_tracker_8 {params};

  EXPECT_EQ(feature_tracker_1.GetID(), 1U);
}

TEST(test_feature_tracker, track) {
  auto logger = std::make_shared<DebugLogger>(LogLevel::INFO, "");
  auto ekf = std::make_shared<EKF>(logger, 10.0, false, "");
  BodyState body_state_init;
  body_state_init.m_velocity[1] = -0.1;
  ekf->Initialize(0.0, body_state_init);

  IMU::Parameters imu_params;
  imu_params.ekf = ekf;
  imu_params.logger = logger;
  IMU imu(imu_params);

  Camera::Parameters cam_params;
  cam_params.ang_c_to_b = Eigen::Quaterniond{0.5, -0.5, 0.5, -0.5};
  cam_params.intrinsics.height = 555;
  cam_params.intrinsics.width = 641;
  cam_params.intrinsics.c_x = 320.5;
  cam_params.intrinsics.c_y = 277.5;
  cam_params.intrinsics.f_x = 512.8;
  cam_params.intrinsics.f_y = 512.8;
  cam_params.ekf = ekf;
  cam_params.logger = logger;
  Camera cam(cam_params);

  FeatureTracker::Parameters tracker_params;
  tracker_params.px_error = 0.1;
  tracker_params.max_track_length = 2;
  tracker_params.camera_id = cam.GetId();
  tracker_params.ekf = ekf;
  tracker_params.min_feat_dist = 0.1;
  tracker_params.logger = logger;
  tracker_params.detector = FeatureDetectorEnum::FAST;

  auto feature_tracker = std::make_shared<FeatureTracker>(tracker_params);
  cam.AddTracker(feature_tracker);

  cv::Mat img_1 =
    cv::imread("../../src/ekf_cal/src/trackers/test/images/tsukuba_l.png", cv::IMREAD_GRAYSCALE);
  cv::Mat img_2 =
    cv::imread("../../src/ekf_cal/src/trackers/test/images/tsukuba_r.png", cv::IMREAD_GRAYSCALE);

  auto cam_msg_1 = std::make_shared<CameraMessage>(img_1);
  auto cam_msg_2 = std::make_shared<CameraMessage>(img_2);

  cam_msg_1->m_time = 0.0;
  cam_msg_2->m_time = 1.0;

  cam.Callback(cam_msg_1);
  cam.Callback(cam_msg_2);

  cv::imwrite("../../src/ekf_cal/src/trackers/test/images/track.png", cam.m_out_img);

  EXPECT_NEAR(ekf->m_state.m_body_state.m_position[0], 0.0, 1e-1);
  EXPECT_NEAR(ekf->m_state.m_body_state.m_position[1], -0.1, 1e-1);
  EXPECT_NEAR(ekf->m_state.m_body_state.m_position[2], 0.0, 1e-1);
}
