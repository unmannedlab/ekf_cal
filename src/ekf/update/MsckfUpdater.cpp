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

#include "ekf/update/MsckfUpdater.hpp"
#include "utility/MathHelper.hpp"
#include "sensors/IMU.hpp"

MsckfUpdater::MsckfUpdater(unsigned int camID)
: Updater(camID) {}

AugmentedState MsckfUpdater::matchState(
  unsigned int frameID)
{
  AugmentedState augStateMatch;

  for (auto & augState : m_augStates) {
    if (augState.frameID == frameID) {
      augStateMatch = augState;
      break;
    }
  }

  return augStateMatch;
}

void MsckfUpdater::updateEKF(unsigned int cameraID, FeatureTracks featureTracks)
{
  m_logger->log(
    LogLevel::DEBUG,
    "Called update_msckf for camera ID: " + std::to_string(cameraID));

  // Calculate the max possible measurement size
  unsigned int max_meas_size = 0;
  for (unsigned int i = 0; i < featureTracks.size(); ++i) {
    max_meas_size += 2 * featureTracks[i].size();
  }

  unsigned int ct_meas = 0;
  unsigned int camStateStart = m_ekf->getCamStateStartIndex(cameraID);
  unsigned int stateSize = m_ekf->getState().getStateSize();
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, stateSize);

  // MSCKF Update
  for (auto & featureTrack : featureTracks) {
    m_logger->log(
      LogLevel::DEBUG,
      "Feature Track size: " + std::to_string(featureTrack.size()));

    AugmentedState augStateMatch = matchState(featureTrack[0].frameID);

    // 3D Cartesian Triangulation
    /// @todo Move into separate function
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    unsigned int total_meas = 1;
    unsigned int total_hx = 6;

    const Eigen::Matrix<double, 3, 3> & R_GtoA = augStateMatch.orientation.toRotationMatrix();
    const Eigen::Matrix<double, 3, 1> & p_AinG = augStateMatch.position;

    for (unsigned int i = 1; i < featureTrack.size(); ++i) {
      augStateMatch = matchState(featureTrack[i].frameID);

      const Eigen::Matrix<double, 3, 3> R_GtoCi = augStateMatch.orientation.toRotationMatrix();
      const Eigen::Vector3d p_CIinG = augStateMatch.position;

      // Convert current position relative to anchor
      Eigen::Matrix3d R_AtoCi = R_GtoCi * R_GtoA.transpose();
      Eigen::Vector3d p_CIinA = R_GtoA * (p_CIinG - p_AinG);

      // Get the UV coordinate normal
      Eigen::Vector3d b_i;
      b_i << featureTrack[i].keyPoint.pt.x, featureTrack[i].keyPoint.pt.y, 1;
      b_i = R_AtoCi.transpose() * b_i;
      b_i = b_i / b_i.norm();
      Eigen::Matrix3d bSkew = skewSymmetric(b_i);
      Eigen::Matrix3d Ai = bSkew.transpose() * bSkew;
      A += Ai;
      b += Ai * p_CIinA;

      total_meas += 1;
      total_hx += 6;
      std::stringstream msg;
      msg << "p_CIinG: " << p_CIinG.transpose() << std::endl;
      msg << "p_AinG: " << p_AinG.transpose() << std::endl;
      msg << "p_CIinA: " << p_CIinA.transpose() << std::endl;
      m_logger->log(LogLevel::DEBUG, msg.str());
    }

    // Solve linear triangulation for 3D cartesian estimate of feature position
    Eigen::Vector3d p_FinA = A.colPivHouseholderQr().solve(b);
    std::stringstream msg;
    msg << "MSCKF Triangulation: " << std::endl
        << "A: " << A << std::endl
        << "b: " << b.transpose() << std::endl;
    m_logger->log(LogLevel::DEBUG, msg.str());
    if (p_FinA == Eigen::Vector3d::Zero()) {
      continue;
    }
    /// @todo Additional non-linear optimization

    /// @todo Get Jacobian
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;

    // Calculate the position of this feature in the global frame
    // Get calibration for our anchor camera
    Eigen::Matrix3d R_ItoC = m_augStates[0].orientation.toRotationMatrix();
    Eigen::Vector3d p_IinC = m_augStates[0].position;
    // Anchor pose orientation and position
    Eigen::Matrix3d R_GtoI = m_augStates[0].imuOrientation.toRotationMatrix();
    Eigen::Vector3d p_IinG = m_augStates[0].imuPosition;
    // Feature in the global frame
    Eigen::Vector3d p_FinG = R_GtoI.transpose() * R_ItoC.transpose() * (p_FinA - p_IinC) +
      p_IinG;

    // Allocate our residual and Jacobians
    int c = 0;
    int jacobSize = 3;
    res = Eigen::VectorXd::Zero(2 * total_meas);
    H_f = Eigen::MatrixXd::Zero(2 * total_meas, jacobSize);
    H_x = Eigen::MatrixXd::Zero(2 * total_meas, total_hx);

    // Derivative of p_FinG in respect to feature representation.
    // This only needs to be computed once and thus we pull it out of the loop
    Eigen::MatrixXd dPFG_dLambda;
    Eigen::Matrix3d R_CtoG = R_GtoI.transpose() * R_ItoC.transpose();

    // Jacobian for our anchor pose
    Eigen::Matrix<double, 3, 6> H_anc;
    H_anc.block(
      0, 0, 3,
      3).noalias() = -R_GtoI.transpose() * skewSymmetric(R_ItoC.transpose() * (p_FinA - p_IinC));
    H_anc.block(0, 3, 3, 3).setIdentity();

    // Add anchor Jacobians to our return vector

    // Get calibration Jacobians (for anchor clone)
    Eigen::Matrix<double, 3, 6> H_calib;
    H_calib.block(0, 0, 3, 3).noalias() = -R_CtoG * skewSymmetric(p_FinA - p_IinC);
    H_calib.block(0, 3, 3, 3) = -R_CtoG;

    dPFG_dLambda = R_CtoG;

    // Get the Jacobian for this feature
    // Loop through each camera for this feature
    if (featureTrack.size() == 0) {
      return;
    }
    for (unsigned int i = 1; i < featureTrack.size(); ++i) {
      augStateMatch = matchState(featureTrack[i].frameID);

      // Our calibration between the IMU and CAMi frames
      Eigen::Matrix3d R_ItoC = augStateMatch.orientation.toRotationMatrix();
      Eigen::Vector3d p_IinC = augStateMatch.position;

      // Get current IMU clone state
      Eigen::Matrix3d R_GtoIi = augStateMatch.imuOrientation.toRotationMatrix();
      Eigen::Vector3d p_Ii_inG = augStateMatch.imuPosition;

      // Get current feature in the IMU
      Eigen::Vector3d p_FinIi = R_GtoIi * (p_FinG - p_Ii_inG);

      // Project the current feature into the current frame of reference
      Eigen::Vector3d p_FinCi = R_ItoC * p_FinIi + p_IinC;
      Eigen::Vector2d uv_norm;
      uv_norm << p_FinCi(0) / p_FinCi(2), p_FinCi(1) / p_FinCi(2);

      // Normalized coordinates in respect to projection function
      Eigen::MatrixXd dzn_dPFC = Eigen::MatrixXd::Zero(2, 3);
      dzn_dPFC << 1 / p_FinCi(2), 0, -p_FinCi(0) / (p_FinCi(2) * p_FinCi(2)), 0, 1 / p_FinCi(2),
        -p_FinCi(1) / (p_FinCi(2) * p_FinCi(2));

      // Derivative of p_FinCi in respect to p_FinIi
      Eigen::MatrixXd dPFC_dPFG = R_ItoC * R_GtoIi;

      // Derivative of p_FinCi in respect to camera clone state
      Eigen::MatrixXd dPFC_dClone = Eigen::MatrixXd::Zero(3, 6);
      dPFC_dClone.block(0, 0, 3, 3) = R_ItoC * skewSymmetric(p_FinIi);
      dPFC_dClone.block(0, 3, 3, 3) = -dPFC_dPFG;

      unsigned int augStateStart = m_ekf->getAugStateStartIndex(cameraID, featureTrack[i].frameID);

      // Precompute some matrices
      Eigen::MatrixXd dz_dPFC = dzn_dPFC;
      Eigen::MatrixXd dz_dPFG = dPFC_dPFG;

      // Chain Rule: get the total feature Jacobian
      H_f.block(2 * c, 0, 2, H_f.cols()) = dz_dPFG * dPFG_dLambda;

      // Chain Rule: get state clone Jacobian
      H_x.block(2 * c, camStateStart, 2, 6) = dz_dPFC * dPFC_dClone;

      // Chain Rule: loop through all extra states and add their
      // NOTE: we add the Jacobian here as we might be in the anchoring pose for this measurement
      H_x.block(2 * c, augStateStart, 2, 6) += dz_dPFG * H_anc;
      H_x.block(2 * c, augStateStart + 6, 2, 6) += dz_dPFG * H_calib;

      // Derivative of p_FinCi in respect to camera calibration (R_ItoC, p_IinC)

      // Calculate the Jacobian
      Eigen::MatrixXd dPFC_dCalib = Eigen::MatrixXd::Zero(3, 6);
      dPFC_dCalib.block(0, 0, 3, 3) = skewSymmetric(p_FinCi - p_IinC);
      dPFC_dCalib.block(0, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();

      // Chain Rule it and add it to the big jacobian
      H_x.block(2 * c, camStateStart + 6, 2, 6) += dz_dPFC * dPFC_dCalib;

      // Move the Jacobian and residual index forward
      c++;
    }

    // Apply the left nullspace of H_f to all variables
    // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
    // See page 252, Algorithm 5.2.4 for how these two loops work
    // They use "matlab" index notation, thus we need to subtract 1 from all index
    Eigen::JacobiRotation<double> tempHo_GR;
    for (int n = 0; n < H_f.cols(); ++n) {
      for (int m = (int)H_f.rows() - 1; m > n; m--) {
        // Givens matrix G
        tempHo_GR.makeGivens(H_f(m - 1, n), H_f(m, n));
        // Multiply G to the corresponding lines (m-1,m) in each matrix
        // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
        //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
        (H_f.block(m - 1, n, 2, H_f.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        (H_x.block(m - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
        (res.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      }
    }

    // The H_f jacobian max rank is 3 if it is a 3d position, thus size of the left nullspace is Hf.rows()-3
    // NOTE: need to eigen3 eval here since this experiences aliasing!
    // H_f = H_f.block(H_f.cols(),0,H_f.rows()-H_f.cols(),H_f.cols()).eval();
    H_x = H_x.block(H_f.cols(), 0, H_x.rows() - H_f.cols(), H_x.cols()).eval();
    res = res.block(H_f.cols(), 0, res.rows() - H_f.cols(), res.cols()).eval();

    /// @todo Chi2 distance check

    // Append our jacobian and residual
    Hx_big.block(ct_meas, camStateStart, H_x.rows(), H_x.cols()) = H_x;
    res_big.block(ct_meas, 0, res.rows(), 1) = res;

    ct_meas += total_hx;
  }

  // 5. Perform measurement compression
  // Return if Hx_big is a fat matrix (there is no need to compress in this case)
  if (Hx_big.rows() <= Hx_big.cols()) {
    return;
  }

  // Do measurement compression through givens rotations
  // Based on "Matrix Computations 4th Edition by Golub and Van Loan"
  // See page 252, Algorithm 5.2.4 for how these two loops work
  // They use "matlab" index notation, thus we need to subtract 1 from all index
  Eigen::JacobiRotation<double> tempHo_GR;
  for (int n = 0; n < Hx_big.cols(); n++) {
    for (int m = (int)Hx_big.rows() - 1; m > n; m--) {
      // Givens matrix G
      tempHo_GR.makeGivens(Hx_big(m - 1, n), Hx_big(m, n));
      // Multiply G to the corresponding lines (m-1,m) in each matrix
      // Note: we only apply G to the nonzero cols [n:Ho.cols()-n-1], while
      //       it is equivalent to applying G to the entire cols [0:Ho.cols()-1].
      (Hx_big.block(m - 1, n, 2, Hx_big.cols() - n)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
      (res_big.block(m - 1, 0, 2, 1)).applyOnTheLeft(0, 1, tempHo_GR.adjoint());
    }
  }

  // If H is a fat matrix, then use the rows
  // Else it should be same size as our state
  int r = std::min(Hx_big.rows(), Hx_big.cols());
  if (r == 1) {
    return;
  }

  // Construct the smaller jacobian and residual after measurement compression
  Hx_big.conservativeResize(r, Hx_big.cols());
  res_big.conservativeResize(r, res_big.cols());


  unsigned int SIGMA_PIX {1U};
  // Our noise is isotropic, so make it here after our compression
  Eigen::MatrixXd R_big = SIGMA_PIX * SIGMA_PIX * Eigen::MatrixXd::Identity(
    res_big.rows(), res_big.rows());

  // // 6. With all good features update the state
  Eigen::MatrixXd S = Hx_big *
    m_ekf->getCov().block(0, 0, stateSize, stateSize) * Hx_big.transpose() + R_big;
  Eigen::MatrixXd K =
    m_ekf->getCov().block(0, 0, stateSize, stateSize) * Hx_big.transpose() * S.inverse();

  unsigned int imuStateSize = m_ekf->getImuCount() * IMU::IMU_STATE_SIZE;
  unsigned int camStateSize = stateSize - EKF::BODY_STATE_SIZE - imuStateSize;

  Eigen::VectorXd update = K * res_big;
  Eigen::VectorXd bodyUpdate = update.segment<EKF::BODY_STATE_SIZE>(0);
  Eigen::VectorXd imuUpdate = update.segment(EKF::BODY_STATE_SIZE, imuStateSize);
  Eigen::VectorXd camUpdate = update.segment(EKF::BODY_STATE_SIZE + imuStateSize, camStateSize);


  m_ekf->getState().bodyState += bodyUpdate;
  m_ekf->getState().imuStates += imuUpdate;
  m_ekf->getState().camStates += camUpdate;

  m_ekf->getCov() =
    (Eigen::MatrixXd::Identity(stateSize, stateSize) - K * Hx_big) * m_ekf->getCov();
}

void MsckfUpdater::RefreshStates()
{
  BodyState bodyState = m_ekf->getBodyState();
  m_bodyPos = bodyState.position;
  m_bodyVel = bodyState.velocity;
  m_bodyAcc = bodyState.acceleration;
  m_bodyAngPos = bodyState.orientation;
  m_bodyAngVel = bodyState.angularVelocity;
  m_bodyAngAcc = bodyState.angularAcceleration;

  CamState camState = m_ekf->getCamState(m_id);
  m_posOffset = camState.position;
  m_angOffset = camState.orientation;
  m_augStates = camState.augmentedStates;
}
