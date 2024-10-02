#
# Copyright (c) 2024 University of York and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
# 
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   * Pedro Ribeiro - initial implementation
#
# This file contains an example of a launch file that includes two other
# launch files from the packages 'turtlebot3_gazebo' and 'turtlebot3_bringup'.
# It also launches the node 'turtlebot3_drive' from the package 'turtlebot3_gazebo'
 
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the paths for where the launch files are located according to package names.
    turtlebot3_gazebo = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    turtlebot3_bringup = os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch')

    # Use the facilities of ROS to import the launch files.
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo, 'turtlebot3_world.launch.py')
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup, 'rviz2.launch.py')
        )
    )

    # Define the node to be run
    turtlebot3_teleop = Node(package='turtlebot3_gazebo', executable='turtlebot3_drive')

    # Construct the launch description to be returned
    return LaunchDescription([
        turtlebot3_world,
        turtlebot3_teleop,
        rviz
    ])

