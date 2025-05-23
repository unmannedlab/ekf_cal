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

#include "utility/type_helper.hpp"

#include <eigen3/Eigen/Eigen>

#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"
#include "infrastructure/debug_logger.hpp"


Eigen::VectorXd StdToEigVec(std::vector<double> const & std_vec)
{
  assert(std_vec.size() == 3);
  Eigen::VectorXd eig_vec(std_vec.size());
  for (unsigned int i = 0; i < std_vec.size(); ++i) {
    eig_vec(i) = std_vec[i];
  }
  return eig_vec;
}

Eigen::Quaterniond StdToEigQuat(std::vector<double> const & std_quat)
{
  if (std_quat.size() == 4) {
    Eigen::Quaterniond quat{std_quat[0], std_quat[1], std_quat[2], std_quat[3]};
    quat.normalize();
    return quat;
  } else {
    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  }
}

Eigen::Quaterniond RotVecToQuat(const Eigen::Vector3d & rot_vec)
{
  double angle = rot_vec.norm();
  Eigen::Quaterniond quat;
  if (angle > 1e-9) {
    Eigen::Vector3d axis = rot_vec / rot_vec.norm();
    Eigen::AngleAxisd ang_axis{angle, axis};
    quat = Eigen::Quaterniond(ang_axis);
  } else {
    quat = Eigen::Quaterniond{1, 0, 0, 0};
  }
  return quat;
}

Eigen::Vector3d QuatToRotVec(const Eigen::Quaterniond & quat)
{
  Eigen::AngleAxisd ang_axis{quat};
  Eigen::Vector3d rot_vec = ang_axis.axis() * ang_axis.angle();
  return rot_vec;
}

Eigen::Quaterniond EigVecToQuat(const Eigen::Vector3d & euler_angles)
{
  Eigen::Quaterniond quaternion =
    Eigen::AngleAxisd(euler_angles(2), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(euler_angles(1), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(euler_angles(0), Eigen::Vector3d::UnitX());
  return quaternion;
}

void EigenMatrixToCv(const Eigen::Matrix3d & matrix_eigen, cv::Mat & matrix_cv)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      matrix_cv.at<double>(i, j) = matrix_eigen(i, j);
    }
  }
}

void CvMatrixToEigen(const cv::Mat matrix_cv, Eigen::Matrix3d & matrix_eigen)
{
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      matrix_eigen(i, j) = matrix_cv.at<double>(i, j);
    }
  }
}

cv::Vec3d QuatToRodrigues(const Eigen::Quaterniond & quat)
{
  Eigen::Matrix3d rotation_matrix_eigen = quat.toRotationMatrix();
  cv::Mat rotation_matrix_cv(3, 3, cv::DataType<double>::type);
  EigenMatrixToCv(rotation_matrix_eigen, rotation_matrix_cv);
  cv::Vec3d rodrigues_vector(3, 1, cv::DataType<double>::type);
  cv::Rodrigues(rotation_matrix_cv, rodrigues_vector);

  return rodrigues_vector;
}

Eigen::Quaterniond RodriguesToQuat(cv::Vec3d & rodrigues_vector)
{
  cv::Mat rotation_matrix_cv(3, 3, cv::DataType<double>::type);
  cv::Rodrigues(rodrigues_vector, rotation_matrix_cv);
  Eigen::Matrix3d rotation_matrix_eigen;
  CvMatrixToEigen(rotation_matrix_cv, rotation_matrix_eigen);
  Eigen::Quaterniond quat(rotation_matrix_eigen);

  return quat;
}

Eigen::Vector3d CvVectorToEigen(cv::Vec3d & vector_cv)
{
  return Eigen::Vector3d{vector_cv[0], vector_cv[1], vector_cv[2]};
}

std::vector<Eigen::Vector3d> operator-(
  const std::vector<Eigen::Vector3d> & vector_of_vectors,
  const Eigen::Vector3d & vector_to_subtract)
{
  std::vector<Eigen::Vector3d> vectors_out;
  for (unsigned int i = 0; i < vector_of_vectors.size(); ++i) {
    vectors_out.push_back(vector_of_vectors[i] - vector_to_subtract);
  }
  return vectors_out;
}
