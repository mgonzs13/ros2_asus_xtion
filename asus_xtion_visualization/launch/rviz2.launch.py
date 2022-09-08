import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_path = get_package_share_directory("asus_xtion_visualization")

    rviz_config = os.path.join(
        pkg_path,
        "rviz",
        "default.rviz")

    rviz_cmd = Node(
        name="rviz",
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config]
    )

    ld = LaunchDescription()

    ld.add_action(rviz_cmd)

    return ld
