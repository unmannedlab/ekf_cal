import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    this_dir = get_package_share_directory("ekf-cal")

    start_ekf_cal_node_cmd = Node(
        package="ekf-cal",
        executable="ekf-cal",
        output="screen",
        parameters=[os.path.join(this_dir, "config", "multi-imu.yaml")],
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(start_ekf_cal_node_cmd)

    return ld
