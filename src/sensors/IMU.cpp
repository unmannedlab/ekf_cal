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

#include "sensors/Sensor.hpp"
#include "utility/MathHelper.hpp"
#include "utility/TypeHelper.hpp"

/// @todo Consider moving EKF update equations into EKF updater class?
IMU::IMU(IMU::Params params)
: Sensor(params.name)
{
  if (params.baseSensor == true) {
    if (params.intrinsic == true) {
      m_stateSize = 6U;
    } else {
      m_stateSize = 0U;
    }
  } else {
    if (params.intrinsic == true) {
      m_stateSize = 12U;
    } else {
      m_stateSize = 6U;
    }
  }

  m_isBaseSensor = params.baseSensor;
  m_isIntrinsic = params.intrinsic;
  m_rate = params.rate;
  m_posOffset = params.posOffset;
  m_angOffset = params.angOffset;
  m_accBias = params.accBias;
  m_omgBias = params.omgBias;
  m_accBiasStability = params.accBiasStability;
  m_omgBiasStability = params.omgBiasStability;

  if (m_stateSize) {
    Eigen::VectorXd sensorState = GetState();
    Eigen::VectorXd varVec = params.variance.segment(0, m_stateSize);
    Eigen::MatrixXd cov = MathHelper::MinBoundVector(varVec, 1e-6).asDiagonal();
    m_ekf->ExtendState(m_stateSize, sensorState, cov);
  }
}

double IMU::GetAccBiasStability()
{
  return m_accBiasStability;
}

double IMU::GetOmgBiasStability()
{
  return m_omgBiasStability;
}

bool IMU::IsBaseSensor()
{
  return m_isBaseSensor;
}

bool IMU::IsIntrinsic()
{
  return m_isIntrinsic;
}

Eigen::VectorXd IMU::PredictMeasurement()
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

Eigen::MatrixXd IMU::GetMeasurementJacobian()
{
  Eigen::MatrixXd measurementJacobian(6, m_stateSize + 18);
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
    measurementJacobian.block<3, 3>(0, 15) = m_angOffset * MathHelper::CrossProductMatrix(
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
      m_angOffset * MathHelper::CrossProductMatrix(m_bodyAngAcc) + temp;

    // IMU Angular Offset
    Eigen::Vector3d imu_acc = m_bodyAcc +
      m_bodyAngAcc.cross(m_posOffset) +
      m_bodyAngVel.cross(m_bodyAngVel.cross(m_posOffset));

    measurementJacobian.block<3, 3>(0, 21) =
      -(m_angOffset * MathHelper::CrossProductMatrix(imu_acc));

    // IMU Body Angular Velocity
    measurementJacobian.block<3, 3>(3, 12) = m_angOffset.toRotationMatrix();

    // IMU Angular Offset
    measurementJacobian.block<3, 3>(3, 21) =
      -(m_angOffset * MathHelper::CrossProductMatrix(m_bodyAngVel));

    if (m_isIntrinsic) {
      // IMU Accelerometer Bias
      measurementJacobian.block<3, 3>(0, 24) = Eigen::MatrixXd::Identity(3, 3);

      // IMU Gyroscope Bias
      measurementJacobian.block<3, 3>(3, 27) = Eigen::MatrixXd::Identity(3, 3);
    }
  }
  return measurementJacobian;
}

Eigen::VectorXd IMU::GetState()
{
  Eigen::VectorXd stateVec(m_stateSize);

  if (m_isBaseSensor) {
    if (m_isIntrinsic) {
      stateVec.segment<3>(0) = m_accBias;
      stateVec.segment<3>(3) = m_omgBias;
    } else {
      m_logger->log(LogLevel::WARN, "Base IMU has no state to get");
    }
  } else {
    Eigen::AngleAxisd angAxis{m_angOffset};
    Eigen::Vector3d angBiasRotVec = angAxis.axis() * angAxis.angle();
    stateVec.segment<3>(0) = m_posOffset;
    stateVec.segment<3>(3) = angBiasRotVec;
    if (m_isIntrinsic) {
      stateVec.segment<3>(6) = m_accBias;
      stateVec.segment<3>(9) = m_omgBias;
    }
  }

  return stateVec;
}

void IMU::SetState()
{
  Eigen::VectorXd state = m_ekf->GetState();

  if (m_isBaseSensor) {
    if (m_isIntrinsic) {
      m_accBias = state.segment<3>(m_stateStartIndex);
      m_omgBias = state.segment<3>(m_stateStartIndex + 3);
    } else {
      // Do nothing if zero-sized state
    }
  } else {
    m_posOffset = state.segment<3>(m_stateStartIndex);
    Eigen::Vector3d rotVec = state.segment<3>(m_stateStartIndex + 3);

    double angle = rotVec.norm();
    Eigen::Vector3d axis = rotVec / angle;

    m_angOffset = Eigen::AngleAxisd(angle, axis);

    if (m_isIntrinsic) {
      m_accBias = state.segment<3>(m_stateStartIndex + 6);
      m_omgBias = state.segment<3>(m_stateStartIndex + 9);
    }
  }
}

void IMU::Callback(
  double time, Eigen::Vector3d acceleration,
  Eigen::Matrix3d accelerationCovariance, Eigen::Vector3d angularRate,
  Eigen::Matrix3d angularRateCovariance)
{
  m_logger->log(LogLevel::DEBUG, "IMU \"" + m_name + "\" callback at time " + std::to_string(time));

  m_ekf->Predict(time);
  SetState();

  Eigen::VectorXd z(acceleration.size() + angularRate.size());
  z.segment<3>(0) = acceleration;
  z.segment<3>(3) = angularRate;

  Eigen::VectorXd z_pred = PredictMeasurement();
  Eigen::VectorXd resid = z - z_pred;

  unsigned int stateSize = m_ekf->GetStateSize();
  unsigned int stateStartIndex = GetStateStartIndex();
  Eigen::MatrixXd subH = GetMeasurementJacobian();
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, stateSize);
  H.block<6, 18>(0, 0) = subH.block<6, 18>(0, 0);
  if (m_stateSize) {
    H.block(0, stateStartIndex, 6, m_stateSize) = subH.block<0, 18>(6, m_stateSize);
  }

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
  R.block<3, 3>(0, 0) = MathHelper::MinBoundDiagonal(accelerationCovariance, 1e-3);
  R.block<3, 3>(3, 3) = MathHelper::MinBoundDiagonal(angularRateCovariance, 1e-2);

  Eigen::MatrixXd S = H * m_ekf->GetCov() * H.transpose() + R;
  Eigen::MatrixXd K = m_ekf->GetCov() * H.transpose() * S.inverse();

  m_ekf->GetState() = m_ekf->GetState() + K * resid;
  m_ekf->GetCov() = (Eigen::MatrixXd::Identity(stateSize, stateSize) - K * H) * m_ekf->GetCov();
  SetState();
}
