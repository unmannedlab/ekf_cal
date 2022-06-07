# Copyright 2022 Jacob Hartzer
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    this_dir = get_package_share_directory("ekf_cal")

    start_ekf_cal_node_cmd = Node(
        package="ekf_cal",
        executable="ekf_cal",
        output="screen",
        parameters=[os.path.join(this_dir, "config", "multi-imu.yaml")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(start_ekf_cal_node_cmd)

    return ld
