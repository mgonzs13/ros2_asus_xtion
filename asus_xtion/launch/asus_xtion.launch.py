import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_description_path = get_package_share_directory(
        "asus_xtion_description")
    pkg_openni2_ros = get_package_share_directory("openni2_camera")

    ### LAUNCHS ###
    description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description_path, "launch",
                         "robot_state_publisher.launch.py")
        )
    )

    openni2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_openni2_ros, "launch",
                         "camera_with_cloud.launch.py")
        )
    )

    ld = LaunchDescription()

    ld.add_action(description_cmd)
    ld.add_action(openni2_cmd)

    return ld
