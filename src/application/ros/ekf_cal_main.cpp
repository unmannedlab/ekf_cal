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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "application/ros/node/ekf_cal_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto ekf_cal_node = std::make_shared<EkfCalNode>();
  ekf_cal_node->Initialize();
  ekf_cal_node->DeclareSensors();
  ekf_cal_node->LoadSensors();
  rclcpp::spin(ekf_cal_node);
  rclcpp::shutdown();

  return 0;
}
