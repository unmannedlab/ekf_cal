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

#include <opencv2/opencv.hpp>

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

void MinBoundVector(Eigen::VectorXd & in_vec, double min_bound)
{
  for (unsigned int i = 0; i < in_vec.size(); ++i) {
    if (in_vec(i) < min_bound) {
      in_vec(i) = min_bound;
    }
  }
}

Eigen::MatrixXd InsertInMatrix(
  Eigen::MatrixXd sub_mat, Eigen::MatrixXd in_mat, unsigned int row, unsigned int col)
{
  unsigned int in_rows = in_mat.rows();
  unsigned int in_cols = in_mat.cols();
  unsigned int sub_rows = sub_mat.rows();
  unsigned int sub_cols = sub_mat.cols();
  unsigned int out_rows = in_mat.rows() + sub_mat.rows();
  unsigned int out_cols = in_mat.cols() + sub_mat.cols();

  Eigen::MatrixXd out_mat = Eigen::MatrixXd::Zero(out_rows, out_cols);

  // Top Left
  out_mat.block(0, 0, row, col) = in_mat.block(0, 0, row, col);

  // Top Right
  out_mat.block(0, col + sub_cols, row, in_cols - col) = in_mat.block(0, col, row, in_cols - col);

  // Bottom Left
  out_mat.block(row + sub_rows, 0, in_rows - row, col) = in_mat.block(row, 0, in_rows - row, col);

  // Bottom Right
  out_mat.block(row + sub_rows, col + sub_cols, in_rows - row, in_cols - col) =
    in_mat.block(row, col, in_rows - row, in_cols - col);

  // Center
  out_mat.block(row, col, sub_rows, sub_cols) = sub_mat;

  return out_mat;
}

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
  Eigen::HouseholderQR<Eigen::MatrixXd> QR(H_f);
  Eigen::MatrixXd Q = QR.householderQ();
  Eigen::MatrixXd Q_null = Q.block(H_f.cols(), 0, H_f.rows() - H_f.cols(), H_f.rows());
  H_x = Q_null * H_x;
  res = Q_null * res;
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

Eigen::Vector3d average_vectors(
  std::vector<Eigen::Vector3d> vectors)
{
  std::vector<double> weights;
  for (unsigned int i = 0; i < vectors.size(); ++i) {
    weights.push_back(1.0);
  }
  return average_vectors(vectors, weights);
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
    Eigen::Matrix3d::Identity(3, 3) -
    coeff_one * skew_mat +
    coeff_two * skew_mat * skew_mat;
  return jacobian;
}

double sign(double val)
{
  return (0.0 < val) - (val < 0.0);
}

Eigen::MatrixXd matrix2d_from_vectors3d(const std::vector<Eigen::Vector3d> & input_vectors)
{
  Eigen::MatrixXd matrix_out(2, input_vectors.size());
  for (unsigned int i = 0; i < input_vectors.size(); i++) {
    matrix_out(0, i) = input_vectors[i][0];
    matrix_out(1, i) = input_vectors[i][1];
  }
  return matrix_out;
}

double affine_angle(const Eigen::Affine3d & transform)
{
  return std::acos((transform.linear().trace() - 1) / 2);
}

bool kabsch_2d(
  const std::vector<Eigen::Vector3d> & points_tgt,
  const std::vector<Eigen::Vector3d> & points_src,
  Eigen::Affine3d & transform,
  double & pos_stddev,
  double & ang_stddev)
{
  if ((points_tgt.size() != points_src.size()) || (points_tgt.size() < 4)) {
    return false;
  }

  Eigen::Vector3d centroid_src = average_vectors(points_src);
  Eigen::Vector3d centroid_tgt = average_vectors(points_tgt);

  std::vector<Eigen::Vector3d> centered_points_tgt = points_src - centroid_src;
  std::vector<Eigen::Vector3d> centered_points_src = points_tgt - centroid_tgt;

  Eigen::MatrixXd mat_src = matrix2d_from_vectors3d(centered_points_tgt);
  Eigen::MatrixXd mat_tgt = matrix2d_from_vectors3d(centered_points_src);

  Eigen::Matrix2d rotation_2d = Eigen::Matrix2d::Identity();

  Eigen::MatrixXd cov = mat_src * mat_tgt.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
  double det = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  I(1, 1) = sign(det);
  rotation_2d = svd.matrixV() * I * svd.matrixU().transpose();

  Eigen::Matrix3d rotation_3d = Eigen::Matrix3d::Identity(3, 3);
  rotation_3d.block(0, 0, 2, 2) = rotation_2d;

  transform.setIdentity();
  transform.linear() = rotation_3d;
  transform.translation() = centroid_tgt - rotation_3d * centroid_src;

  std::vector<Eigen::Vector3d> pos_errors;
  double sum_square_slopes {0.0};
  double sum_count{0};
  for (unsigned int i = 0; i < points_tgt.size(); ++i) {
    Eigen::Vector3d pos_error = points_tgt[i] - transform * points_src[i];
    double baseline_dist = (points_tgt[i] - centroid_src).norm();
    pos_errors.push_back(pos_error);
    if (baseline_dist > pos_error.norm()) {
      sum_square_slopes += std::pow(pos_error.norm() / baseline_dist, 2);
      ++sum_count;
    }
  }

  pos_stddev = mean_standard_deviation(pos_errors);
  if (sum_count) {
    ang_stddev = std::sqrt(sum_square_slopes) / sum_count;
  }

  return true;
}

double maximum_distance(const std::vector<Eigen::Vector3d> & eigen_points)
{
  std::vector<cv::Point> points, hull;
  for (auto eigen_point : eigen_points) {
    cv::Point cv_point;
    cv_point.x = eigen_point.x();
    cv_point.y = eigen_point.y();
    points.push_back(cv_point);
  }
  cv::convexHull(points, hull);

  double max_distance {0.0};
  for (unsigned int i = 0; i < hull.size(); ++i) {
    for (unsigned int j = 0; j < hull.size(); ++j) {
      if (i != j) {
        double norm_dist = cv::norm(hull[i] - hull[j]);
        if (norm_dist > max_distance) {
          max_distance = norm_dist;
        }
      }
    }
  }
  return max_distance;
}

double mean_standard_deviation(const std::vector<Eigen::Vector3d> & input_vectors)
{
  double square_sum_of_difference{0.0};
  Eigen::Vector3d mean_var = average_vectors(input_vectors);

  for (auto & vector : input_vectors) {
    double diff = (vector - mean_var).norm();
    square_sum_of_difference += diff * diff;
  }

  return std::sqrt(square_sum_of_difference) / input_vectors.size();
}

Eigen::MatrixXd QR_r(Eigen::MatrixXd A, Eigen::MatrixXd B)
{
  Eigen::MatrixXd vert_cat(A.rows() + B.rows(), A.cols());

  vert_cat << A, B;
  Eigen::HouseholderQR<Eigen::MatrixXd> QR(vert_cat);
  Eigen::MatrixXd R = QR.matrixQR().block(0, 0, A.cols(), A.cols()).triangularView<Eigen::Upper>();

  return R;
}
