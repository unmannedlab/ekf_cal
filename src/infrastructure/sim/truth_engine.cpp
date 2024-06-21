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

#include "infrastructure/sim/truth_engine.hpp"

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <string>
#include <vector>

#include "ekf/types.hpp"
#include "infrastructure/data_logger.hpp"
#include "infrastructure/debug_logger.hpp"
#include "utility/sim/sim_rng.hpp"
#include "utility/string_helper.hpp"

TruthEngine::TruthEngine(double max_time, std::shared_ptr<DebugLogger> logger)
: m_max_time(max_time), m_logger(logger) {}

Eigen::Vector3d TruthEngine::GetImuPosition(unsigned int sensor_id)
{
  return m_imu_pos[sensor_id];
}

Eigen::Quaterniond TruthEngine::GetImuAngularPosition(unsigned int sensor_id)
{
  return m_imu_ang_pos[sensor_id];
}

Eigen::Vector3d TruthEngine::GetImuAccelerometerBias(unsigned int sensor_id)
{
  return m_imu_acc_bias[sensor_id];
}

Eigen::Vector3d TruthEngine::GetImuGyroscopeBias(unsigned int sensor_id)
{
  return m_imu_gyro_bias[sensor_id];
}

Eigen::Vector3d TruthEngine::GetCameraPosition(unsigned int sensor_id)
{
  return m_cam_pos[sensor_id];
}

Eigen::Quaterniond TruthEngine::GetCameraAngularPosition(unsigned int sensor_id)
{
  return m_cam_ang_pos[sensor_id];
}

Eigen::Vector3d TruthEngine::GetGpsPosition(unsigned int sensor_id)
{
  return m_gps_pos[sensor_id];
}
Eigen::Vector3d TruthEngine::GetLocalPosition()
{
  return m_lla_reference;
}
double TruthEngine::GetLocalHeading()
{
  return m_heading;
}

void TruthEngine::SetImuPosition(unsigned int sensor_id, Eigen::Vector3d imu_pos)
{
  m_imu_pos[sensor_id] = imu_pos;
}

void TruthEngine::SetImuAngularPosition(unsigned int sensor_id, Eigen::Quaterniond imu_ang_pos)
{
  m_imu_ang_pos[sensor_id] = imu_ang_pos;
}

void TruthEngine::SetImuAccelerometerBias(unsigned int sensor_id, Eigen::Vector3d imu_acc_bias)
{
  m_imu_acc_bias[sensor_id] = imu_acc_bias;
}

void TruthEngine::SetImuGyroscopeBias(unsigned int sensor_id, Eigen::Vector3d imu_gyro_bias)
{
  m_imu_gyro_bias[sensor_id] = imu_gyro_bias;
}

void TruthEngine::SetCameraPosition(unsigned int sensor_id, Eigen::Vector3d cam_pos)
{
  m_cam_pos[sensor_id] = cam_pos;
}

void TruthEngine::SetCameraAngularPosition(unsigned int sensor_id, Eigen::Quaterniond cam_ang_pos)
{
  m_cam_ang_pos[sensor_id] = cam_ang_pos;
}

void TruthEngine::SetBoardPosition(unsigned int board_id, Eigen::Vector3d board_position)
{
  m_board_pos[board_id] = board_position;
}

void TruthEngine::SetGpsPosition(unsigned int sensor_id, Eigen::Vector3d gps_position)
{
  m_gps_pos[sensor_id] = gps_position;
}

void TruthEngine::SetLocalPosition(Eigen::Vector3d lla_reference)
{
  m_lla_reference = lla_reference;
}

void TruthEngine::SetLocalHeading(double heading)
{
  m_heading = heading;
}

Eigen::Vector3d TruthEngine::GetBoardPosition(unsigned int board_id)
{
  return m_board_pos[board_id];
}

void TruthEngine::SetBoardOrientation(unsigned int board_id, Eigen::Quaterniond board_orientation)
{
  m_board_ang[board_id] = board_orientation;
}

Eigen::Quaterniond TruthEngine::GetBoardOrientation(unsigned int board_id)
{
  return m_board_ang[board_id];
}

void TruthEngine::GenerateFeatures(unsigned int feature_count, double room_size, SimRNG rng)
{
  m_feature_points.push_back(cv::Point3d(room_size, 0, 0));
  m_feature_points.push_back(cv::Point3d(room_size, room_size / 10, 0));
  m_feature_points.push_back(cv::Point3d(room_size, 0, room_size / 10));
  m_feature_points.push_back(cv::Point3d(-room_size, 0, 0));
  m_feature_points.push_back(cv::Point3d(0, room_size, 0));
  m_feature_points.push_back(cv::Point3d(0, -room_size, 0));
  m_feature_points.push_back(cv::Point3d(0, 0, room_size));
  m_feature_points.push_back(cv::Point3d(room_size / 10, 0, room_size));
  m_feature_points.push_back(cv::Point3d(0, room_size / 10, room_size));
  m_feature_points.push_back(cv::Point3d(0, 0, -room_size));
  for (unsigned int i = 0; i < feature_count; ++i) {
    cv::Point3d vec;
    vec.x = rng.UniRand(-room_size, room_size);
    vec.y = rng.UniRand(-room_size, room_size);
    vec.z = rng.UniRand(-room_size / 10, room_size / 10);
    m_feature_points.push_back(vec);
  }
}

std::vector<cv::Point3d> TruthEngine::GetFeatures()
{
  return m_feature_points;
}

TruthEngine::~TruthEngine() {}

void TruthEngine::WriteTruthData(
  double body_data_rate,
  std::string output_directory)
{
  std::cout << output_directory << std::endl;
  DataLogger truth_logger(output_directory, "body_truth.csv");
  truth_logger.SetLogging(true);

  std::stringstream header;
  header << "time";
  header << EnumerateHeader("body_pos", 3);
  header << EnumerateHeader("body_vel", 3);
  header << EnumerateHeader("body_acc", 3);
  header << EnumerateHeader("body_ang_pos", 4);
  header << EnumerateHeader("body_ang_vel", 3);
  header << EnumerateHeader("body_ang_acc", 3);

  unsigned int sensor_count{0};
  for (unsigned int i = 0; i < m_imu_pos.size(); ++i) {
    ++sensor_count;
    header << EnumerateHeader(std::string("imu_pos_") + std::to_string(sensor_count), 3);
    header << EnumerateHeader(std::string("imu_ang_pos_") + std::to_string(sensor_count), 4);
    header << EnumerateHeader(std::string("imu_acc_bias_") + std::to_string(sensor_count), 3);
    header << EnumerateHeader(std::string("imu_gyr_bias_") + std::to_string(sensor_count), 3);
  }
  for (unsigned int i = 0; i < m_cam_pos.size(); ++i) {
    ++sensor_count;
    header << EnumerateHeader(std::string("cam_pos_") + std::to_string(sensor_count), 3);
    header << EnumerateHeader(std::string("cam_ang_pos_") + std::to_string(sensor_count), 4);
  }
  for (unsigned int i = 0; i < m_gps_pos.size(); ++i) {
    ++sensor_count;
    header << EnumerateHeader(std::string("gps_pos_") + std::to_string(sensor_count), 3);
  }
  if (m_gps_pos.size() >= 1) {
    header << ",ref_lat,ref_lon,ref_alt,ref_heading";
  }

  truth_logger.DefineHeader(header.str());

  unsigned int num_measurements = static_cast<int>(std::floor((m_max_time + 1.0) * body_data_rate));
  for (unsigned int i = 0; i < num_measurements; ++i) {
    sensor_count = 0;
    std::stringstream msg;
    double time = static_cast<double>(i) / body_data_rate;
    msg << time;
    msg << VectorToCommaString(GetBodyPosition(time));
    msg << VectorToCommaString(GetBodyVelocity(time));
    msg << VectorToCommaString(GetBodyAcceleration(time));
    msg << QuaternionToCommaString(GetBodyAngularPosition(time));
    msg << VectorToCommaString(GetBodyAngularRate(time));
    msg << VectorToCommaString(GetBodyAngularAcceleration(time));
    for (unsigned int i = 0; i < m_imu_pos.size(); ++i) {
      ++sensor_count;
      msg << VectorToCommaString(GetImuPosition(sensor_count));
      msg << QuaternionToCommaString(GetImuAngularPosition(sensor_count));
      msg << VectorToCommaString(GetImuAccelerometerBias(sensor_count));
      msg << VectorToCommaString(GetImuGyroscopeBias(sensor_count));
    }
    for (unsigned int i = 0; i < m_cam_pos.size(); ++i) {
      ++sensor_count;
      msg << VectorToCommaString(GetCameraPosition(sensor_count));
      msg << QuaternionToCommaString(GetCameraAngularPosition(sensor_count));
    }
    for (unsigned int i = 0; i < m_gps_pos.size(); ++i) {
      ++sensor_count;
      msg << VectorToCommaString(GetGpsPosition(sensor_count));
    }
    if (m_gps_pos.size() >= 1) {
      msg << VectorToCommaString(GetLocalPosition());
      msg << "," << GetLocalHeading();
    }

    truth_logger.Log(msg.str());
  }

  DataLogger board_logger(output_directory, "boards.csv");
  board_logger.SetLogging(true);
  board_logger.DefineHeader("board,pos_x,pos_y,pos_z,quat_w,quat_x,quat_y,quat_z");

  for (auto const & board : m_board_pos) {
    std::stringstream msg;
    msg << std::to_string(board.first);
    msg << VectorToCommaString(m_board_pos[board.first]);
    msg << QuaternionToCommaString(m_board_ang[board.first]);
    board_logger.Log(msg.str());
  }

  DataLogger feature_logger(output_directory, "feature_points.csv");
  feature_logger.SetLogging(true);
  feature_logger.DefineHeader("Feature,x,y,z");

  for (unsigned int i = 0; i < m_feature_points.size(); ++i) {
    std::stringstream msg;
    msg << std::to_string(i);
    msg << "," << m_feature_points[i].x;
    msg << "," << m_feature_points[i].y;
    msg << "," << m_feature_points[i].z;
    feature_logger.Log(msg.str());
  }
}

Intrinsics TruthEngine::GetCameraIntrinsics(unsigned int sensor_id)
{
  return m_cam_intrinsics[sensor_id];
}

void TruthEngine::SetCameraIntrinsics(unsigned int sensor_id, Intrinsics intrinsics)
{
  m_cam_intrinsics[sensor_id] = intrinsics;
}
