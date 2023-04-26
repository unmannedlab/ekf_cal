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
/// @param in_vec Input vector with which to find the left hand size cross product matrix
/// @return Cross product matrix
///
Eigen::Matrix3d SkewSymmetric(Eigen::Vector3d in_vec);

///
/// @brief Bound matrix diagonal by a minimum value
/// @param in_mat Input matrix to be bound
/// @param min_bound Bounding value
/// @return
///
Eigen::MatrixXd MinBoundDiagonal(Eigen::MatrixXd in_mat, double min_bound);

///
/// @brief Bound vector by a minimum value
/// @param in_vec Input vector to be bound
/// @param min_bound Bounding value
/// @return
///
Eigen::VectorXd MinBoundVector(Eigen::VectorXd in_vec, double min_bound);

///
/// @brief Insert a diagonal matrix block into another matrix
/// @param sub_mat Sub matrix to insert
/// @param in_mat Input matrix
/// @param row Insertion row
/// @param col Insertion column
/// @return
///
Eigen::MatrixXd InsertInMatrix(
  Eigen::MatrixXd sub_mat, Eigen::MatrixXd in_mat, unsigned int row, unsigned int col);

///
/// @brief Remove rows and columns from a matrix
/// @param in_mat Input matrix
/// @param row Starting row
/// @param col Starting col
/// @param size Size of rows and columns to be removed
/// @return Matrix with rows and columns removed
///
Eigen::MatrixXd RemoveFromMatrix(
  Eigen::MatrixXd in_mat, unsigned int row, unsigned int col, unsigned int size);

#endif  // UTILITY__MATH_HELPER_HPP_
