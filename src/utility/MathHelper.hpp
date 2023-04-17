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

#ifndef UTILITY__MATHHELPER_HPP_
#define UTILITY__MATHHELPER_HPP_

#include <eigen3/Eigen/Eigen>

///
/// @brief Produces a cross product matrix
/// @param inVec Input vector with which to find the left hand size cross
/// product matrix
/// @return Cross product matrix
///
inline Eigen::Matrix3d skewSymmetric(Eigen::Vector3d inVec)
{
  Eigen::Matrix3d outMat = Eigen::Matrix3d::Zero();

  outMat(0U, 1U) = -inVec(2U);
  outMat(0U, 2U) = inVec(1U);
  outMat(1U, 2U) = -inVec(0U);
  outMat(1U, 0U) = inVec(2U);
  outMat(2U, 0U) = -inVec(1U);
  outMat(2U, 1U) = inVec(0U);

  return outMat;
}

///
/// @brief Bound matrix diagonal by a minimum value
/// @param inMat Input matrix to be bound
/// @param minBound Bounding value
/// @return
///
inline Eigen::MatrixXd minBoundDiagonal(Eigen::MatrixXd inMat, double minBound)
{
  Eigen::MatrixXd outMat = inMat;
  for (unsigned int i = 0; i < inMat.rows() && i < inMat.cols(); ++i) {
    if (outMat(i, i) < minBound) {
      outMat(i, i) = minBound;
    }
  }

  return outMat;
}

///
/// @brief Bound vector by a minimum value
/// @param inVec Input vector to be bound
/// @param minBound Bounding value
/// @return
///
inline Eigen::VectorXd minBoundVector(Eigen::VectorXd inVec, double minBound)
{
  Eigen::VectorXd outVec = inVec;
  for (unsigned int i = 0; i < outVec.size(); ++i) {
    if (outVec(i) < minBound) {
      outVec(i) = minBound;
    }
  }

  return outVec;
}

inline Eigen::MatrixXd insertInMatrix(
  Eigen::MatrixXd subMat, Eigen::MatrixXd inMat, unsigned int row,
  unsigned int col)
{
  unsigned int iRows = inMat.rows();
  unsigned int iCols = inMat.cols();
  unsigned int sRows = subMat.rows();
  unsigned int sCols = subMat.cols();
  unsigned int oRows = inMat.rows() + subMat.rows();
  unsigned int oCols = inMat.cols() + subMat.cols();

  Eigen::MatrixXd outMat = Eigen::MatrixXd::Zero(oRows, oCols);

  outMat.block(0, 0, row, col) = inMat.block(0, 0, row, col);
  outMat.block(row, col, sRows, sCols) = subMat;
  outMat.block(row + sRows, col + sCols, iRows - row, iCols - col) =
    inMat.block(row, col, iRows - row, iCols - col);

  return outMat;
}

inline Eigen::MatrixXd removeFromMatrix(
  Eigen::MatrixXd inMat, unsigned int row,
  unsigned int col, unsigned int size)
{
  unsigned int iRows = inMat.rows();
  unsigned int iCols = inMat.cols();

  Eigen::MatrixXd outMat = Eigen::MatrixXd::Zero(iRows - size, iCols - size);

  outMat.block(0, 0, row, col) = inMat.block(0, 0, row, col);

  outMat.block(row, col, iRows - row - size, iCols - col - size) =
    inMat.block(row + size, col + size, iRows - row - size, iCols - col - size);

  outMat.block(row, 0, iRows - row - size, col) =
    inMat.block(row + size, 0, iRows - row - size, col);

  outMat.block(0, col, row, iCols - col - size) =
    inMat.block(0, col + size, row, iCols - col - size);

  return outMat;
}

#endif  // UTILITY__MATHHELPER_HPP_
