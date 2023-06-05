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
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
import xacro


def robot_state_publisher(context: LaunchContext):

    ### XACRO ###
    xacro_file = os.path.join(get_package_share_directory(
        "asus_xtion_description"), "robots/asus_xtion.urdf.xacro")
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)

    params = {"robot_description": doc.toxml(), "use_sim_time": True}

    ### NODES ###
    robot_state_publisher_cmd = Node(
        name="robot_state_publisher",
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[params]
    )

    return [robot_state_publisher_cmd]


def generate_launch_description():

    prepare_xacro_cmd = OpaqueFunction(function=robot_state_publisher)

    ld = LaunchDescription()
    ld.add_action(prepare_xacro_cmd)

    return ld
