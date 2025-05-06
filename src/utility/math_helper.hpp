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

#include <vector>

///
/// @brief Produces a cross product matrix
/// @param in_vec Input vector with which to find the left hand size cross product matrix
/// @return Cross product matrix
///
Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d & in_vec);

///
/// @brief Bound vector by a minimum value
/// @param in_vec Input vector to be bound
/// @param min_bound Bounding value
///
void MinBoundVector(Eigen::VectorXd & in_vec, double min_bound);

///
/// @brief Insert a diagonal matrix block into another matrix
/// @param sub_mat Sub matrix to insert
/// @param in_mat Input matrix
/// @param row Insertion row
/// @param col Insertion column
/// @return Matrix with sub-matrix added
///
Eigen::MatrixXd InsertInMatrix(
  const Eigen::MatrixXd & sub_mat,
  const Eigen::MatrixXd & in_mat,
  unsigned int row,
  unsigned int col);

///
/// @brief Remove rows and columns from a matrix
/// @param in_mat Input matrix
/// @param row Starting row
/// @param col Starting col
/// @param size Size of rows and columns to be removed
/// @return Matrix with rows and columns removed
///
Eigen::MatrixXd RemoveFromMatrix(
  const Eigen::MatrixXd & in_mat, unsigned int row, unsigned int col, unsigned int size);

///
/// @brief Apply left nullspace to update matrices
/// @param H_f Feature Jacobian
/// @param H_x Track Jacobian
/// @param res Update residual
///
void ApplyLeftNullspace(const Eigen::MatrixXd & H_f, Eigen::MatrixXd & H_x, Eigen::VectorXd & res);

///
/// @brief Perform measurement compression
/// @param jacobian Measurement Jacobian
/// @param residual Measurement residual
///
void CompressMeasurements(Eigen::MatrixXd & jacobian, Eigen::VectorXd & residual);

///
/// @brief Find average of multiple vectors
/// @param vectors Vector of vectors to average
/// @return Average vector
///
Eigen::Vector3d average_vectors(const std::vector<Eigen::Vector3d> & vectors);

///
/// @brief Find average of multiple vectors
/// @param vectors Vector of vectors to average
/// @param weights Vector of weights
/// @return Average vector
///
Eigen::Vector3d average_vectors(
  const std::vector<Eigen::Vector3d> & vectors,
  const std::vector<double> & weights);

///
/// @brief Calculate jacobian of quaternion with respect to a rotation measurement
/// @param quat Input quaternion
/// @return Jacobian matrix
///
Eigen::MatrixXd QuaternionJacobian(const Eigen::Quaterniond & quat);

///
/// @brief Function to determine sign of a double
/// @param val Value to consider
/// @return The sign of the input value as a +/- 1 double
///
double sign(const double val);

///
/// @brief Create a horizontal 2xN matrix from a list of vectors
/// @param input_vectors List of vectors
/// @return 2xN Matrix
///
Eigen::MatrixXd matrix2d_from_vectors3d(const std::vector<Eigen::Vector3d> & input_vectors);

///
/// @brief Get angle of affine transformation
/// @param transform Affine transformation
/// @return Rotation angle
///
double affine_angle(const Eigen::Affine3d & transform);

///
/// @brief Find in-plane transformation between two sets of points using the Kabsch algorithm
/// @param points_tgt Points in target frame
/// @param points_src Points in source frame
/// @param transform Resultant transform
/// @param pos_stddev Standard deviation of the position error
/// @param ang_stddev Standard deviation of the angle error
/// @return Was function successful
///
bool kabsch_2d(
  const std::vector<Eigen::Vector3d> & points_tgt,
  const std::vector<Eigen::Vector3d> & points_src,
  Eigen::Affine3d & transform,
  double & pos_stddev,
  double & ang_stddev);

///
/// @brief Find maximum distance between any two points in list
/// @param eigen_points List of points to find baseline
/// @return Maximum distance found between points
///
double maximum_distance(const std::vector<Eigen::Vector3d> & eigen_points);

///
/// @brief Calculate standard deviation of the mean of a vector of vectors
/// @param input_vectors Vector of vectors to calculate standard deviation
/// @return Standard deviation of the mean
///
double mean_standard_deviation(const std::vector<Eigen::Vector3d> & input_vectors);

///
/// @brief Vertically concatenate two matrices and get R of QR decomposition
/// @param A Top matrix
/// @param B Bottom matrix
/// @return R matrix from QR decomposition
///
Eigen::MatrixXd QR_r(const Eigen::MatrixXd & A, const Eigen::MatrixXd & B);

#endif  // UTILITY__MATH_HELPER_HPP_
