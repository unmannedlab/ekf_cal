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

#include "ekf/sensors/Imu.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ekf/sensors/Sensor.hpp"
#include "MathHelper.hpp"

Imu::Imu(Imu::Params params)
: Sensor(params.name)
{
  if (params.intrinsic) {
    if (params.baseSensor) {
      RCLCPP_WARN(rclcpp::get_logger("IMU"), "Base IMU should be extrinsic for filter stability");
    }
    m_stateSize = 12U;
  } else {
    m_stateSize = 6U;
  }

  m_baseSensor = params.baseSensor;
  m_intrinsic = params.intrinsic;
  m_rate = params.rate;
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_accBias = params.accBias;
  m_omgBias = params.omgBias;
  m_accBiasStability = params.accBiasStability;
  m_omgBiasStability = params.omgBiasStability;
}


double Imu::GetAccBiasStability()
{
  return m_accBiasStability;
}

double Imu::GetOmgBiasStability()
{
  return m_omgBiasStability;
}

bool Imu::IsBaseSensor()
{
  return m_baseSensor;
}
bool Imu::IsIntrinsic()
{
  return m_intrinsic;
}

Eigen::VectorXd Imu::PredictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(m_stateSize);

  if (m_baseSensor == true) {
    predictedMeasurement << m_bodyAcc, m_bodyAngVel;
  } else {
    // Transform acceleration to IMU location
    Eigen::Vector3d imuAcc = m_bodyAcc +
      m_bodyAngAcc.cross(m_posOffset) +
      m_bodyAngVel.cross((m_bodyAngVel.cross(m_posOffset)));

    // Rotate measurements in place
    Eigen::Vector3d imuAccRot = m_angOffset * imuAcc + m_accBias;
    Eigen::Vector3d imuOmgRot = m_angOffset * m_bodyAngVel + m_omgBias;

    predictedMeasurement << imuAccRot, imuOmgRot;
  }

  return predictedMeasurement;
}

Eigen::MatrixXd Imu::GetMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(6, m_stateSize);

  if (m_baseSensor == true) {
    // Base Acceleration
    measurementJacobian.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3);

    // Base Angular Velocity
    measurementJacobian.block<3, 3>(0, 12) = Eigen::MatrixXd::Identity(3, 3);
  } else {
    // Body Acceleration
    measurementJacobian.block<3, 3>(0, 6) = m_angOffset.toRotationMatrix();

    // Body Angular Velocity
    Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(3, 3);
    temp(0, 0) = m_posOffset(1) * m_bodyAngVel(1) + 1 * m_posOffset(2) * m_bodyAngVel(2);
    temp(0, 1) = m_posOffset(1) * m_bodyAngVel(0) - 2 * m_posOffset(0) * m_bodyAngVel(1);
    temp(0, 2) = m_posOffset(2) * m_bodyAngVel(0) - 2 * m_posOffset(0) * m_bodyAngVel(2);
    temp(1, 0) = m_posOffset(0) * m_bodyAngVel(1) - 2 * m_posOffset(1) * m_bodyAngVel(0);
    temp(1, 1) = m_posOffset(0) * m_bodyAngVel(0) + 1 * m_posOffset(2) * m_bodyAngVel(2);
    temp(1, 2) = m_posOffset(2) * m_bodyAngVel(1) - 2 * m_posOffset(1) * m_bodyAngVel(2);
    temp(2, 0) = m_posOffset(0) * m_bodyAngVel(2) - 2 * m_posOffset(2) * m_bodyAngVel(0);
    temp(2, 1) = m_posOffset(1) * m_bodyAngVel(2) - 2 * m_posOffset(2) * m_bodyAngVel(1);
    temp(2, 2) = m_posOffset(0) * m_bodyAngVel(0) + 1 * m_posOffset(1) * m_bodyAngVel(1);
    measurementJacobian.block<3, 3>(0, 12) = m_angOffset * temp;

    // Body Angular Acceleration
    measurementJacobian.block<3, 3>(0, 15) = m_angOffset * MathHelper::CrossProductMatrix(
      m_posOffset);

    // IMU Positional Offset
    temp = Eigen::MatrixXd::Zero(0, 3);
    temp(0, 0) = -(m_bodyAngVel(1) * m_bodyAngVel(1)) - (m_bodyAngVel(2) * m_bodyAngVel(2));
    temp(0, 1) = m_bodyAngVel(0) * m_bodyAngVel(1);
    temp(0, 2) = m_bodyAngVel(0) * m_bodyAngVel(2);
    temp(1, 0) = m_bodyAngVel(0) * m_bodyAngVel(1);
    temp(1, 1) = -(m_bodyAngVel(0) * m_bodyAngVel(0)) - (m_bodyAngVel(2) * m_bodyAngVel(2));
    temp(1, 2) = m_bodyAngVel(1) * m_bodyAngVel(2);
    temp(2, 0) = m_bodyAngVel(0) * m_bodyAngVel(2);
    temp(2, 1) = m_bodyAngVel(1) * m_bodyAngVel(2);
    temp(2, 2) = -(m_bodyAngVel(0) * m_bodyAngVel(0)) - (m_bodyAngVel(1) * m_bodyAngVel(1));
    measurementJacobian.block<3, 3>(0, m_stateStartIndex + 0) =
      m_angOffset * MathHelper::CrossProductMatrix(m_bodyAngAcc) + temp;

    // IMU Angular Offset
    Eigen::Vector3d imu_acc = m_bodyAcc +
      m_bodyAngAcc.cross(m_posOffset) +
      m_bodyAngVel.cross(m_bodyAngVel.cross(m_posOffset));

    measurementJacobian.block<3, 3>(0, m_stateStartIndex + 3) =
      -(m_angOffset * MathHelper::CrossProductMatrix(imu_acc));

    // IMU Accelerometer Bias
    measurementJacobian.block<3, 3>(0, m_stateStartIndex + 6) = Eigen::MatrixXd::Identity(3, 3);

    // IMU Body Angular Velocity
    measurementJacobian.block<3, 3>(3, m_stateStartIndex + 3) = m_angOffset.toRotationMatrix();

    // IMU Angular Offset
    measurementJacobian.block<3, 3>(3, m_stateStartIndex + 3) =
      -(m_angOffset * MathHelper::CrossProductMatrix(m_bodyAngVel));

    // IMU Gyroscope Bias
    measurementJacobian.block<3, 3>(3, m_stateStartIndex + 9) = Eigen::MatrixXd::Identity(3, 3);
  }
  return measurementJacobian;
}

void Imu::SetState(Eigen::VectorXd state)
{
  m_posOffset = state.segment(0, 3);
  Eigen::Vector3d rotVec = state.segment(3, 3);
  double angle = rotVec.norm();
  Eigen::Vector3d axis = rotVec / rotVec.norm();
  Eigen::AngleAxisd angAxis{angle, axis};
  m_angOffset = Eigen::Quaterniond(angAxis);

  if (m_intrinsic == true) {
    m_accBias = state.segment(6, 3);
    m_omgBias = state.segment(9, 3);
  }
}
