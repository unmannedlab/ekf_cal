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

#include "utility/constants.hpp"
#include "infrastructure/debug_logger.hpp"


Eigen::VectorXd StdToEigVec(std::vector<double> const & in)
{
  Eigen::VectorXd out(in.size());
  for (unsigned int i = 0; i < in.size(); ++i) {
    out(i) = in[i];
  }
  return out;
}

Eigen::Quaterniond StdToEigQuat(std::vector<double> const & in)
{
  if (in.size() == 4U) {
    Eigen::Quaterniond quat{in[0U], in[1U], in[2U], in[3U]};
    quat.normalize();
    return quat;
  } else {
    DebugLogger::GetInstance()->Log(LogLevel::WARN, "Vector incorrect size for Eigen conversion");
    return Eigen::Quaterniond{1.0, 0.0, 0.0, 0.0};
  }
}

Eigen::Quaterniond RotVecToQuat(Eigen::Vector3d rot_vec)
{
  double angle = rot_vec.norm();
  if (angle > 1e-9) {
    Eigen::Vector3d axis = rot_vec / rot_vec.norm();
    Eigen::AngleAxisd ang_axis{angle, axis};
    return Eigen::Quaterniond(ang_axis);
  } else {
    return Eigen::Quaterniond{1, 0, 0, 0};
  }
}

Eigen::Vector3d QuatToRotVec(Eigen::Quaterniond quat)
{
  Eigen::AngleAxisd ang_axis{quat};
  Eigen::Vector3d rot_vec = ang_axis.axis() * ang_axis.angle();
  return rot_vec;
}
