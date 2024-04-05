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

#include "utility/math_helper.hpp"

#include <eigen3/Eigen/Eigen>

#include <algorithm>
#include <vector>

#include "utility/type_helper.hpp"

Eigen::Matrix3d SkewSymmetric(Eigen::Vector3d in_vec)
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

Eigen::MatrixXd MinBoundDiagonal(Eigen::MatrixXd in_mat, double min_bound)
{
  Eigen::MatrixXd out_mat = in_mat;
  for (unsigned int i = 0; i < in_mat.rows() && i < in_mat.cols(); ++i) {
    if (out_mat(i, i) < min_bound) {
      out_mat(i, i) = min_bound;
    }
  }

  return out_mat;
}

Eigen::VectorXd MinBoundVector(Eigen::VectorXd in_vec, double min_bound)
{
  Eigen::VectorXd out_vec = in_vec;
  for (unsigned int i = 0; i < out_vec.size(); ++i) {
    if (out_vec(i) < min_bound) {
      out_vec(i) = min_bound;
    }
  }

  return out_vec;
}

Eigen::MatrixXd MinBoundMatrix(Eigen::MatrixXd in_mat, double min_bound)
{
  Eigen::MatrixXd out_mat = in_mat;
  for (unsigned int i = 0; i < out_mat.rows(); ++i) {
    for (unsigned int j = 0; j < out_mat.cols(); ++j) {
      if (out_mat(i, j) < min_bound) {
        out_mat(i, j) = min_bound;
      }
    }
  }
  return out_mat;
}

Eigen::MatrixXd MaxBoundMatrix(Eigen::MatrixXd in_mat, double max_bound)
{
  Eigen::MatrixXd out_mat = in_mat;
  for (unsigned int i = 0; i < out_mat.rows(); ++i) {
    for (unsigned int j = 0; j < out_mat.cols(); ++j) {
      if (out_mat(i, j) > max_bound) {
        out_mat(i, j) = max_bound;
      }
    }
  }
  return out_mat;
}

/// @todo Do in place
Eigen::MatrixXd InsertInMatrix(
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

/// @todo Do in place
Eigen::MatrixXd RemoveFromMatrix(
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

void ApplyLeftNullspace(Eigen::MatrixXd & H_f, Eigen::MatrixXd & H_x, Eigen::VectorXd & res)
{
  unsigned int m = H_f.rows();
  unsigned int n = H_f.cols();

  // Apply the left nullspace of H_f to the jacobians and the residual
  Eigen::JacobiRotation<double> givens;
  for (unsigned int j = 0; j < n; ++j) {
    for (unsigned int i = m - 1; i > j; --i) {
      // Givens matrix G
      givens.makeGivens(H_f(i - 1, j), H_f(i, j));

      // Apply nullspace
      (H_f.block(i - 1, j, 2, n - j)).applyOnTheLeft(0, 1, givens.adjoint());
      (H_x.block(i - 1, 0, 2, H_x.cols())).applyOnTheLeft(0, 1, givens.adjoint());
      (res.segment<2>(i - 1)).applyOnTheLeft(0, 1, givens.adjoint());
    }
  }

  H_x = H_x.block(n, 0, H_x.rows() - n, H_x.cols()).eval();
  res = res.block(n, 0, res.rows() - n, res.cols()).eval();
}

void CompressMeasurements(Eigen::MatrixXd & jacobian, Eigen::VectorXd & residual)
{
  unsigned int m = jacobian.rows();
  unsigned int n = jacobian.cols();

  // Cannot compress fat matrices
  if (m >= n) {
    Eigen::JacobiRotation<double> givens;
    for (unsigned int j = 0; j < n; j++) {
      for (unsigned int i = m - 1; i > j; --i) {
        // Givens matrix
        givens.makeGivens(jacobian(i - 1, j), jacobian(i, j));

        // Compress measurements
        (jacobian.block(i - 1, j, 2, n - j)).applyOnTheLeft(0, 1, givens.adjoint());
        (residual.segment<2>(i - 1)).applyOnTheLeft(0, 1, givens.adjoint());
      }
    }

    // Count non-zero rows after compression
    unsigned int r = (jacobian.array().abs() > 1e-9).rowwise().any().cast<unsigned int>().sum();

    // Construct the smaller jacobian and residual after measurement compression
    jacobian.conservativeResize(r, jacobian.cols());
    residual.conservativeResize(r);
  }
}


Eigen::Quaterniond average_quaternions(
  std::vector<Eigen::Quaterniond> quaternions,
  std::vector<double> weights)
{
  Eigen::Quaterniond average_quaternion{1.0, 0.0, 0.0, 0.0};
  Eigen::MatrixXd accum_matrix(4, quaternions.size());

  /// @todo(jhartzer): Check that the vectors are equally sized. Log warning otherwise
  for (unsigned int i = 0; i < quaternions.size(); ++i) {
    if (quaternions[i].w() < 0) {
      weights[i] *= -1;
    }
    accum_matrix(0, i) = weights[i] * quaternions[i].w();
    accum_matrix(1, i) = weights[i] * quaternions[i].x();
    accum_matrix(2, i) = weights[i] * quaternions[i].y();
    accum_matrix(3, i) = weights[i] * quaternions[i].z();
  }

  Eigen::Matrix4d A_matrix = accum_matrix * accum_matrix.transpose();

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigen_solver(A_matrix);

  if (eigen_solver.info() == Eigen::Success) {
    Eigen::Vector4d eigen_values = eigen_solver.eigenvalues();
    Eigen::Matrix4d eigen_vectors = eigen_solver.eigenvectors();
    unsigned int max_eigen_index;
    eigen_values.maxCoeff(&max_eigen_index);

    average_quaternion.w() = eigen_vectors(0, max_eigen_index);
    average_quaternion.x() = eigen_vectors(1, max_eigen_index);
    average_quaternion.y() = eigen_vectors(2, max_eigen_index);
    average_quaternion.z() = eigen_vectors(3, max_eigen_index);
    average_quaternion.normalize();
  }

  return average_quaternion;
}


Eigen::Vector3d average_vectors(
  std::vector<Eigen::Vector3d> vectors, std::vector<double> weights)
{
  Eigen::Vector3d average_vector {0.0, 0.0, 0.0};
  double weights_sum {0.0};

  for (unsigned int i = 0; i < vectors.size(); ++i) {
    average_vector += weights[i] * vectors[i];
    weights_sum += weights[i];
  }
  return average_vector / weights_sum;
}

Eigen::MatrixXd quaternion_jacobian(Eigen::Quaterniond quat)
{
  Eigen::Vector3d rot_vec = QuatToRotVec(quat);
  Eigen::Matrix3d skew_mat = SkewSymmetric(rot_vec);
  double vec_norm = std::max(rot_vec.norm(), 1e-9);
  double coeff_one = (1 - std::cos(vec_norm)) / std::pow(vec_norm, 2);
  double coeff_two = (vec_norm - std::sin(vec_norm)) / std::pow(vec_norm, 3);
  Eigen::Matrix3d jacobian =
    Eigen::Matrix3d::Identity() -
    coeff_one * skew_mat +
    coeff_two * skew_mat * skew_mat;
  return jacobian;
}

Eigen::MatrixXd quaternion_jacobian_inv(Eigen::Quaterniond quat)
{
  Eigen::Vector3d rot_vec = QuatToRotVec(quat);
  Eigen::Matrix3d skew_mat = SkewSymmetric(rot_vec);
  double vec_norm = std::max(rot_vec.norm(), 1e-9);
  double coeff_one = 0.5;
  double coeff_two =
    std::pow(vec_norm, -2) - (1 + std::cos(vec_norm)) / (2 * vec_norm * std::sin(vec_norm));
  Eigen::Matrix3d jacobian =
    Eigen::Matrix3d::Identity() +
    coeff_one * skew_mat +
    coeff_two * skew_mat * skew_mat;
  return jacobian;
}

void align_points(
  const std::vector<Eigen::Vector3d> & tgt_points,
  const std::vector<Eigen::Vector3d> & src_points,
  Eigen::Affine3d & transformation,
  Eigen::Vector3d & singular_values)
{
  if ((tgt_points.size() != src_points.size()) || (tgt_points.size() < 4)) {
    return;
  }

  size_t n = tgt_points.size();
  Eigen::MatrixXd src(3, n), tgt(3, n);
  for (unsigned int i = 0; i < n; i++) {
    tgt.col(i) = tgt_points[i];
    src.col(i) = src_points[i];
  }

  Eigen::Vector3d centroid_src(0, 0, 0), centroid_tgt(0, 0, 0);
  for (unsigned int i = 0; i < n; i++) {
    centroid_src += src.col(i);
    centroid_tgt += tgt.col(i);
  }
  centroid_src /= n;
  centroid_tgt /= n;
  for (unsigned int i = 0; i < n; i++) {
    src.col(i) -= centroid_src;
    tgt.col(i) -= centroid_tgt;
  }

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();

  Eigen::MatrixXd cov = src * tgt.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0) {d = 1.0;} else {d = -1.0;}
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
  I(2, 2) = d;
  rotation = svd.matrixV() * I * svd.matrixU().transpose();

  singular_values = svd.singularValues();
  transformation.setIdentity();
  transformation.linear() = rotation;
  transformation.translation() = (centroid_tgt - rotation * centroid_src);
}
