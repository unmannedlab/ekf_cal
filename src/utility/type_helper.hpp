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

#ifndef UTILITY__TYPE_HELPER_HPP_
#define UTILITY__TYPE_HELPER_HPP_

#include <eigen3/Eigen/Eigen>

#include <vector>

#include <opencv2/opencv.hpp>

#include "ekf/types.hpp"

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Vector3
///
Eigen::VectorXd StdToEigVec(std::vector<double> const & in);

///
/// @brief Converts std::vector into Eigen Quaternion
/// @param in Input std::vector
/// @return Output Eigen Quaternion
///
Eigen::Quaterniond StdToEigQuat(std::vector<double> const & in);

///
/// @brief Convert rotation vector to quaternion
/// @param rot_vec Input rotation vector
/// @return Rotation quaternion
///
Eigen::Quaterniond RotVecToQuat(const Eigen::Vector3d & rot_vec);

///
/// @brief Convert quaternion to rotation vector
/// @param quat Input rotation quaternion
/// @return Rotation vector
///
Eigen::Vector3d QuatToRotVec(const Eigen::Quaterniond & quat);

///
/// @brief Convert Euler angles to quaternion
/// @param euler_angles Input Euler angles
/// @return Resulting quaternion
///
Eigen::Quaterniond EigVecToQuat(const Eigen::Vector3d & euler_angles);

///
/// @brief Convert Eigen matrix to cv matrix
/// @param matrix_eigen Input Eigen matrix
/// @param matrix_cv Output CV matrix
///
void EigenMatrixToCv(const Eigen::Matrix3d & matrix_eigen, cv::Mat & matrix_cv);

///
/// @brief Convert CV matrix to Eigen matrix
/// @param matrix_cv Input CV matrix
/// @param matrix_eigen Output Eigen matrix
///
void CvMatrixToEigen(const cv::Mat matrix_cv, Eigen::Matrix3d & matrix_eigen);

///
/// @brief Convert quaternion to Rodrigues rotation vector
/// @param quat Input rotation quaternion
/// @return Rodrigues rotation vector
///
cv::Vec3d QuatToRodrigues(const Eigen::Quaterniond & quat);

///
/// @brief Convert Rodrigues rotation vector to quaternion
/// @param rodrigues_vector Input rodrigues vector
/// @return rotation quaternion
///
Eigen::Quaterniond RodriguesToQuat(cv::Vec3d rodrigues_vector);

///
/// @brief Convert CV vector to Eigen vector
/// @param vector_cv Input CV vector
/// @return Output Eigen vector
///
Eigen::Vector3d CvVectorToEigen(cv::Vec3d & vector_cv);

///
/// @brief Telegraph subtract vector from list of vectors
/// @param vector_of_vectors vector of vectors
/// @param vector_to_subtract vector to subtract
/// @return Vector of vectors result from telegraphed subtraction
///
std::vector<Eigen::Vector3d> operator-(
  const std::vector<Eigen::Vector3d> & vector_of_vectors,
  const Eigen::Vector3d & vector_to_subtract);

#endif  // UTILITY__TYPE_HELPER_HPP_
