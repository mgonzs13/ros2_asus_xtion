# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


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
