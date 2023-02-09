// Copyright 2023 Jacob Hartzer
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

#ifndef EKF__TYPES_HPP_
#define EKF__TYPES_HPP_

#include <eigen3/Eigen/Eigen>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

typedef struct BodyState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Vector3d velocity{0.0, 0.0, 0.0};
  Eigen::Vector3d acceleration{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d angularVelocity{0.0, 0.0, 0.0};
  Eigen::Vector3d angularAcceleration{0.0, 0.0, 0.0};
} BodyState;

typedef struct ImuState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d accBias{0.0, 0.0, 0.0};
  Eigen::Vector3d omgBias{0.0, 0.0, 0.0};
} ImuState;

typedef struct AugmentedState
{
  unsigned int frameID;
  Eigen::Vector3d imuPosition{0.0, 0.0, 0.0};
  Eigen::Quaterniond imuOrientation{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
} AugmentedState;

typedef struct CamState
{
  Eigen::Vector3d position{0.0, 0.0, 0.0};
  Eigen::Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
  std::vector<AugmentedState> augmentedStates;
} CamState;

typedef struct FeatureTrack
{
  unsigned int frameID;
  cv::KeyPoint keyPoint;
} FeatureTrack;

typedef std::vector<std::vector<FeatureTrack>> FeatureTracks;


class State
{
public:
  State();
  Eigen::VectorXd toVector();
  unsigned int getStateSize();

  BodyState bodyState {};
  std::map<unsigned int, ImuState> imuStates{};
  std::map<unsigned int, CamState> camStates{};
};

State & operator+=(State & lState, State & rState);
State & operator+=(State & lState, Eigen::VectorXd & rVector);

#endif  // EKF__TYPES_HPP_
