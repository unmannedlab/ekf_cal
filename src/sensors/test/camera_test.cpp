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

#include "trackers/feature_tracker.hpp"

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
