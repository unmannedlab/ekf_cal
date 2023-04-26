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

#ifndef UTILITY__MATH_HELPER_HPP_
#define UTILITY__MATH_HELPER_HPP_

#include <eigen3/Eigen/Eigen>

///
/// @brief Produces a cross product matrix
/// @param in_vec Input vector with which to find the left hand size cross
/// product matrix
/// @return Cross product matrix
///
inline Eigen::Matrix3d SkewSymmetric(Eigen::Vector3d in_vec)
{
  Eigen::Matrix3d out_mat = Eigen::Matrix3d::Zero();

  out_mat(0U, 1U) = -in_vec(2U);
  out_mat(0U, 2U) = in_vec(1U);
  out_mat(1U, 2U) = -in_vec(0U);
  out_mat(1U, 0U) = in_vec(2U);
  out_mat(2U, 0U) = -in_vec(1U);
  out_mat(2U, 1U) = in_vec(0U);

  return out_mat;
}

///
/// @brief Bound matrix diagonal by a minimum value
/// @param in_mat Input matrix to be bound
/// @param min_bound Bounding value
/// @return
///
inline Eigen::MatrixXd MinBoundDiagonal(Eigen::MatrixXd in_mat, double min_bound)
{
  Eigen::MatrixXd out_mat = in_mat;
  for (unsigned int i = 0; i < in_mat.rows() && i < in_mat.cols(); ++i) {
    if (out_mat(i, i) < min_bound) {
      out_mat(i, i) = min_bound;
    }
  }

  return out_mat;
}

///
/// @brief Bound vector by a minimum value
/// @param in_vec Input vector to be bound
/// @param min_bound Bounding value
/// @return
///
inline Eigen::VectorXd MinBoundVector(Eigen::VectorXd in_vec, double min_bound)
{
  Eigen::VectorXd out_vec = in_vec;
  for (unsigned int i = 0; i < out_vec.size(); ++i) {
    if (out_vec(i) < min_bound) {
      out_vec(i) = min_bound;
    }
  }

  return out_vec;
}

///
/// @brief Insert a diagonal matrix block into another matrix
/// @param sub_mat Sub matrix to insert
/// @param in_mat Input matrix
/// @param row Insertion row
/// @param col Insertion column
/// @return
///
inline Eigen::MatrixXd InsertInMatrix(
  Eigen::MatrixXd sub_mat, Eigen::MatrixXd in_mat, unsigned int row,
  unsigned int col)
{
  unsigned int in_rows = in_mat.rows();
  unsigned int in_cols = in_mat.cols();
  unsigned int sub_rows = sub_mat.rows();
  unsigned int sub_cols = sub_mat.cols();
  unsigned int out_rows = in_mat.rows() + sub_mat.rows();
  unsigned int out_cols = in_mat.cols() + sub_mat.cols();

  Eigen::MatrixXd out_mat = Eigen::MatrixXd::Zero(out_rows, out_cols);

  out_mat.block(0, 0, row, col) = in_mat.block(0, 0, row, col);
  out_mat.block(row, col, sub_rows, sub_cols) = sub_mat;
  out_mat.block(row + sub_rows, col + sub_cols, in_rows - row, in_cols - col) =
    in_mat.block(row, col, in_rows - row, in_cols - col);

  return out_mat;
}

///
/// @brief Remove rows and columns from a matrix
/// @param in_mat Input matrix
/// @param row Starting row
/// @param col Starting col
/// @param size Size of rows and columns to be removed
/// @return Matrix with rows and columns removed
///
inline Eigen::MatrixXd RemoveFromMatrix(
  Eigen::MatrixXd in_mat, unsigned int row,
  unsigned int col, unsigned int size)
{
  unsigned int in_rows = in_mat.rows();
  unsigned int in_cols = in_mat.cols();

  Eigen::MatrixXd out_mat = Eigen::MatrixXd::Zero(in_rows - size, in_cols - size);

  out_mat.block(0, 0, row, col) = in_mat.block(0, 0, row, col);

  out_mat.block(row, col, in_rows - row - size, in_cols - col - size) =
    in_mat.block(row + size, col + size, in_rows - row - size, in_cols - col - size);

  out_mat.block(row, 0, in_rows - row - size, col) =
    in_mat.block(row + size, 0, in_rows - row - size, col);

  out_mat.block(0, col, row, in_cols - col - size) =
    in_mat.block(0, col + size, row, in_cols - col - size);

  return out_mat;
}

#endif  // UTILITY__MATH_HELPER_HPP_
