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

#ifndef SENSORS__SIM__SIMFeatureTRACKER_HPP_
#define SENSORS__SIM__SIMFeatureTRACKER_HPP_

#include <string>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "infrastructure/sim/TruthEngine.hpp"
#include "sensors/Camera.hpp"
#include "sensors/Sensor.hpp"
#include "trackers/FeatureTracker.hpp"
#include "utility/sim/SimRNG.hpp"

///
/// @class SimFeatureTrackerMessage
/// @brief Simulated Tracker Message class
///
class SimFeatureTrackerMessage : public SensorMessage
{
public:
  SimFeatureTrackerMessage() {}
  unsigned int trackerID {0};
  std::vector<std::vector<FeatureTrack>> featureTracks;
};

///
/// @class SimFeatureTracker
/// @brief Simulated Tracker class
///
class SimFeatureTracker : public FeatureTracker
{
public:
  ///
  /// @brief Sim IMU initialization parameters structure
  ///
  typedef struct Parameters
  {
    unsigned int featureCount{0};                       ///< @brief Number of features to generate
    double roomSize{10.0};                              ///< @brief Size of "Room" for features
    double pxError{1.0};                                ///< @brief Pixel Error
    FeatureTracker::Parameters trackerParams;           ///< @brief Tracker parameters
  } Parameters;

  ///
  /// @brief Simulation IMU constructor
  /// @param params Simulation IMU parameters
  /// @param truthEngine Truth engine
  ///
  SimFeatureTracker(Parameters params, std::shared_ptr<TruthEngine> truthEngine);

  ///
  /// @brief Generate simulated tracker messages
  /// @param maxTime Maximum time of generated messages
  ///
  std::vector<std::shared_ptr<SimFeatureTrackerMessage>> generateMessages(
    double maxTime,
    unsigned int sensorID);

  ///
  /// @brief Return currently visible keypoints
  /// @param time Current time
  ///
  std::vector<cv::KeyPoint> visibleKeypoints(double time);

  void callback(double time, unsigned int cameraID, std::shared_ptr<SimFeatureTrackerMessage> msg);

private:
  double m_tBias{0.0};
  double m_tSkew{0.0};
  double m_tError{1e-9};
  double m_pxError{1e-9};
  Eigen::Vector3d m_posOffset{0.0, 0.0, 0.0};
  Eigen::Quaterniond m_angOffset{1.0, 0.0, 0.0, 0.0};
  std::shared_ptr<TruthEngine> m_truth;
  double m_rate{1.0};
  SimRNG m_rng;
  unsigned int m_featureCount {0};
  std::vector<cv::Point3d> m_featurePoints;

  double m_focalLength {100};
  unsigned int m_imageHeight {480};
  unsigned int m_imageWidth {640};
  cv::Mat m_projMatrix;
};


#endif  // SENSORS__SIM__SIMFeatureTRACKER_HPP_
