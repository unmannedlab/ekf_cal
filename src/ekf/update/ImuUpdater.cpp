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

#include "ekf/update/ImuUpdater.hpp"

#include <unistd.h>

#include <string>
#include <sstream>

#include "ekf/Types.hpp"
#include "infrastructure/DebugLogger.hpp"
#include "sensors/Sensor.hpp"
#include "utility/MathHelper.hpp"
#include "utility/TypeHelper.hpp"

const Eigen::Vector3d ImuUpdater::GRAVITY = Eigen::Vector3d(0, 0, 9.80665);

/// @todo Should combine IMU with file name for multiple IMU logs
ImuUpdater::ImuUpdater(
  unsigned int imuID, double accBiasStability, double omgBiasStability,
  std::string logFileDirectory, bool dataLoggingOn)
: Updater(imuID), m_accBiasStability(accBiasStability), m_omgBiasStability(omgBiasStability),
  m_dataLogger(logFileDirectory, "imu.csv")
{
  std::stringstream msg;
  msg << "time";
  for (unsigned int i = 0; i < 6; ++i) {
    msg << ",residual_" + std::to_string(i);
  }
  for (unsigned int i = 0; i < 12; ++i) {
    msg << ",update_" + std::to_string(i);
  }
  msg << "\n";

  m_dataLogger.defineHeader(msg.str());
  m_dataLogger.setLogging(dataLoggingOn);
}

Eigen::VectorXd ImuUpdater::predictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(6);
  // Transform acceleration to IMU location
  Eigen::Vector3d imuAcc = m_bodyAcc + GRAVITY +
    m_bodyAngAcc.cross(m_posOffset) +
    m_bodyAngVel.cross((m_bodyAngVel.cross(m_posOffset)));

  // Rotate measurements in place
  Eigen::Vector3d imuAccRot = m_angOffset * imuAcc;
  Eigen::Vector3d imuOmgRot = m_angOffset * m_bodyAngVel;

  // Add bias
  imuAccRot += m_accBias;
  imuOmgRot += m_omgBias;

  predictedMeasurement.segment<3>(0) = imuAccRot;
  predictedMeasurement.segment<3>(3) = imuOmgRot;

  return predictedMeasurement;
}

Eigen::MatrixXd ImuUpdater::getMeasurementJacobian(bool isBaseSensor, bool isIntrinsic)
{
  Eigen::MatrixXd measurementJacobian =
    Eigen::MatrixXd::Zero(6, EKF::BODY_STATE_SIZE + IMU_STATE_SIZE);

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
  measurementJacobian.block<3, 3>(0, 15) = m_angOffset * skewSymmetric(
    m_posOffset);

  // IMU Body Angular Velocity
  measurementJacobian.block<3, 3>(3, 12) = m_angOffset.toRotationMatrix();

  // IMU Positional Offset
  if (!isBaseSensor) {
    temp.setZero();
    temp(0, 0) = -(m_bodyAngVel(1) * m_bodyAngVel(1)) - (m_bodyAngVel(2) * m_bodyAngVel(2));
    temp(0, 1) = m_bodyAngVel(0) * m_bodyAngVel(1);
    temp(0, 2) = m_bodyAngVel(0) * m_bodyAngVel(2);
    temp(1, 0) = m_bodyAngVel(0) * m_bodyAngVel(1);
    temp(1, 1) = -(m_bodyAngVel(0) * m_bodyAngVel(0)) - (m_bodyAngVel(2) * m_bodyAngVel(2));
    temp(1, 2) = m_bodyAngVel(1) * m_bodyAngVel(2);
    temp(2, 0) = m_bodyAngVel(0) * m_bodyAngVel(2);
    temp(2, 1) = m_bodyAngVel(1) * m_bodyAngVel(2);
    temp(2, 2) = -(m_bodyAngVel(0) * m_bodyAngVel(0)) - (m_bodyAngVel(1) * m_bodyAngVel(1));
    measurementJacobian.block<3, 3>(0, IMU_STATE_SIZE) =
      m_angOffset * skewSymmetric(m_bodyAngAcc) + temp;

    // IMU Angular Offset
    Eigen::Vector3d imu_acc = m_bodyAcc +
      m_bodyAngAcc.cross(m_posOffset) +
      m_bodyAngVel.cross(m_bodyAngVel.cross(m_posOffset));

    measurementJacobian.block<3, 3>(0, EKF::BODY_STATE_SIZE + 3) =
      -(m_angOffset * skewSymmetric(imu_acc));

    // IMU Angular Offset
    measurementJacobian.block<3, 3>(3, EKF::BODY_STATE_SIZE + 3) =
      -(m_angOffset * skewSymmetric(m_bodyAngVel));
  }

  if (isIntrinsic) {
    // IMU Accelerometer Bias
    measurementJacobian.block<3, 3>(0, EKF::BODY_STATE_SIZE + 6) = Eigen::MatrixXd::Identity(3, 3);

    // IMU Gyroscope Bias
    measurementJacobian.block<3, 3>(3, EKF::BODY_STATE_SIZE + 9) = Eigen::MatrixXd::Identity(3, 3);
  }

  return measurementJacobian;
}

void ImuUpdater::RefreshStates()
{
  BodyState bodyState = m_ekf->getBodyState();
  m_bodyPos = bodyState.position;
  m_bodyVel = bodyState.velocity;
  m_bodyAcc = bodyState.acceleration;
  m_bodyAngPos = bodyState.orientation;
  m_bodyAngVel = bodyState.angularVelocity;
  m_bodyAngAcc = bodyState.angularAcceleration;

  ImuState imuState = m_ekf->getImuState(m_id);
  m_posOffset = imuState.position;
  m_angOffset = imuState.orientation;
  m_accBias = imuState.accBias;
  m_omgBias = imuState.omgBias;
}


void ImuUpdater::updateEKF(
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance, bool isBaseSensor, bool isIntrinsic)
{
  RefreshStates();
  m_ekf->processModel(time);

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angularRate;

  Eigen::VectorXd z_pred = predictMeasurement();
  Eigen::VectorXd resid = z - z_pred;
  std::stringstream msg0;
  msg0 << "IMU resid: " << resid.transpose() << "\n";
  m_logger->log(LogLevel::DEBUG, msg0.str());

  unsigned int stateStartIndex = m_ekf->getImuStateStartIndex(m_id);

  unsigned int updateSize = EKF::BODY_STATE_SIZE + IMU_STATE_SIZE * m_ekf->getImuCount();

  Eigen::MatrixXd subH = getMeasurementJacobian(isBaseSensor, isIntrinsic);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, updateSize);

  H.block<6, EKF::BODY_STATE_SIZE>(0, 0) = subH.block<6, EKF::BODY_STATE_SIZE>(0, 0);

  H.block(0, stateStartIndex, 6, IMU_STATE_SIZE) =
    subH.block<6, IMU_STATE_SIZE>(0, EKF::BODY_STATE_SIZE);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = minBoundDiagonal(accelerationCovariance, 1e-3);
  R.block<3, 3>(3, 3) = minBoundDiagonal(angularRateCovariance, 1e-2);

  Eigen::MatrixXd S = H * m_ekf->getCov().block(0, 0, updateSize, updateSize) * H.transpose() + R;
  Eigen::MatrixXd K =
    m_ekf->getCov().block(0, 0, updateSize, updateSize) * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * resid;
  Eigen::VectorXd bodyUpdate = update.segment<EKF::BODY_STATE_SIZE>(0);
  Eigen::VectorXd imuUpdate =
    update.segment(EKF::BODY_STATE_SIZE, updateSize - EKF::BODY_STATE_SIZE);

  m_ekf->getState().bodyState += bodyUpdate;
  m_ekf->getState().imuStates += imuUpdate;
  m_ekf->getCov().block(0, 0, updateSize, updateSize) =
    (Eigen::MatrixXd::Identity(updateSize, updateSize) - K * H) *
    m_ekf->getCov().block(0, 0, updateSize, updateSize);

  // Write outputs
  std::stringstream msg;
  Eigen::VectorXd imuSubUpdate = update.segment(stateStartIndex, IMU_STATE_SIZE);
  msg << time;
  for (unsigned int i = 0; i < resid.size(); ++i) {
    msg << "," << resid[i];
  }
  for (unsigned int i = 0; i < imuSubUpdate.size(); ++i) {
    msg << "," << imuSubUpdate[i];
  }
  msg << "\n";
  m_dataLogger.log(msg.str());
}
