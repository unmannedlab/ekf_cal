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

#include "sensors/IMU.hpp"

#include "ekf/Types.hpp"
#include "sensors/Sensor.hpp"
#include "utility/MathHelper.hpp"
#include "utility/TypeHelper.hpp"


const Eigen::Vector3d IMU::GRAVITY = Eigen::Vector3d(0, 0, -9.80665);

/// @todo Consider moving EKF update equations into EKF updater class?
IMU::IMU(IMU::Params params)
: Sensor(params.name)
{
  ImuState imuState;

  m_isBaseSensor = params.baseSensor;
  m_isIntrinsic = params.intrinsic;
  m_rate = params.rate;
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_accBias = params.accBias;
  m_omgBias = params.omgBias;
  m_accBiasStability = params.accBiasStability;
  m_omgBiasStability = params.omgBiasStability;

  imuState.position = m_posOffset;
  imuState.orientation = m_angOffset;
  imuState.accBias = m_accBias;
  imuState.omgBias = m_omgBias;

  Eigen::MatrixXd cov = minBoundVector(params.variance, 1e-6).asDiagonal();

  m_ekf->registerIMU(m_id, imuState, cov);
}

double IMU::getAccBiasStability()
{
  return m_accBiasStability;
}

double IMU::getOmgBiasStability()
{
  return m_omgBiasStability;
}

Eigen::VectorXd IMU::predictMeasurement()
{
  Eigen::VectorXd predictedMeasurement(6);

  if (m_isBaseSensor) {
    if (m_isIntrinsic) {
      predictedMeasurement.segment<3>(0) = m_bodyAcc + m_accBias;
      predictedMeasurement.segment<3>(3) = m_bodyAngVel + m_omgBias;
    } else {
      predictedMeasurement.segment<3>(0) = m_bodyAcc;
      predictedMeasurement.segment<3>(3) = m_bodyAngVel;
    }
  } else {
    // Transform acceleration to IMU location
    Eigen::Vector3d imuAcc = m_bodyAcc +
      m_bodyAngAcc.cross(m_posOffset) +
      m_bodyAngVel.cross((m_bodyAngVel.cross(m_posOffset)));

    // Rotate measurements in place
    Eigen::Vector3d imuAccRot = m_angOffset * imuAcc;
    Eigen::Vector3d imuOmgRot = m_angOffset * m_bodyAngVel;

    // Add bias
    if (m_isIntrinsic) {
      imuAccRot += m_accBias;
      imuOmgRot += m_omgBias;
    }

    predictedMeasurement.segment<3>(0) = imuAccRot;
    predictedMeasurement.segment<3>(3) = imuOmgRot;
  }

  return predictedMeasurement;
}

Eigen::MatrixXd IMU::getMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(6, 12 + 18);
  measurementJacobian.setZero();

  if (m_isBaseSensor) {
    // Base Acceleration
    measurementJacobian.block<3, 3>(0, 6) = Eigen::MatrixXd::Identity(3, 3);

    // Base Angular Velocity
    measurementJacobian.block<3, 3>(3, 12) = Eigen::MatrixXd::Identity(3, 3);

    if (m_isIntrinsic) {
      // IMU Accelerometer Bias
      measurementJacobian.block<3, 3>(0, 18) = Eigen::MatrixXd::Identity(3, 3);

      // IMU Gyroscope Bias
      measurementJacobian.block<3, 3>(3, 21) = Eigen::MatrixXd::Identity(3, 3);
    }
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
    measurementJacobian.block<3, 3>(0, 15) = m_angOffset * skewSymmetric(
      m_posOffset);

    // IMU Positional Offset
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
    measurementJacobian.block<3, 3>(0, 18) =
      m_angOffset * skewSymmetric(m_bodyAngAcc) + temp;

    // IMU Angular Offset
    Eigen::Vector3d imu_acc = m_bodyAcc +
      m_bodyAngAcc.cross(m_posOffset) +
      m_bodyAngVel.cross(m_bodyAngVel.cross(m_posOffset));

    measurementJacobian.block<3, 3>(0, 21) =
      -(m_angOffset * skewSymmetric(imu_acc));

    // IMU Body Angular Velocity
    measurementJacobian.block<3, 3>(3, 12) = m_angOffset.toRotationMatrix();

    // IMU Angular Offset
    measurementJacobian.block<3, 3>(3, 21) =
      -(m_angOffset * skewSymmetric(m_bodyAngVel));

    if (m_isIntrinsic) {
      // IMU Accelerometer Bias
      measurementJacobian.block<3, 3>(0, 24) = Eigen::MatrixXd::Identity(3, 3);

      // IMU Gyroscope Bias
      measurementJacobian.block<3, 3>(3, 27) = Eigen::MatrixXd::Identity(3, 3);
    }
  }
  return measurementJacobian;
}

/// @todo Move to a EKF updater class
void IMU::callback(
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance)
{
  m_logger->log(LogLevel::DEBUG, "IMU \"" + m_name + "\" callback at time " + std::to_string(time));

  m_ekf->processModel(time);

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angularRate;

  Eigen::VectorXd z_pred = predictMeasurement();
  Eigen::VectorXd resid = z - z_pred;

  unsigned int stateSize = m_ekf->getState().getStateSize();
  unsigned int stateStartIndex = m_ekf->getImuStateStartIndex(m_id);
  Eigen::MatrixXd subH = getMeasurementJacobian();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, stateSize);
  H.block<6, 18>(0, 0) = subH.block<6, 18>(0, 0);
  H.block(0, stateStartIndex, 6, 12) = subH.block<0, 18>(6, 12);

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = minBoundDiagonal(accelerationCovariance, 1e-3);
  R.block<3, 3>(3, 3) = minBoundDiagonal(angularRateCovariance, 1e-2);

  Eigen::MatrixXd S = H * m_ekf->getCov() * H.transpose() + R;
  Eigen::MatrixXd K = m_ekf->getCov() * H.transpose() * S.inverse();

  Eigen::VectorXd update = K * resid;
  m_ekf->getState() += update;
  m_ekf->getCov() = (Eigen::MatrixXd::Identity(stateSize, stateSize) - K * H) * m_ekf->getCov();
}
