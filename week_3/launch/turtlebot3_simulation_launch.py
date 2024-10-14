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
#   * Alan Millard - initial implementation
#   * Pedro Ribeiro - revised implementation
# 
# This launch file incorporates elements from the following two launch files:
#
# https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/tb3_simulation_launch.py
# https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/launch/turtlebot3_world.launch.py
#
# In comparison with 'turtlebot3_world.launch.py' it provides finer control over starting pose of the robot,
# see the function spawn_entity below, and the ability to select different worlds as a parameter, eg.
# by changing the 'world' launch configuration as follows:
#
# ros2 launch week_3 turtlebot3_simulation_launch.py world:=turtlebot3_empty
#
# Note that the default is 'turtlebot3_world'.
#
# You can check all supported launch arguments by inspecting this code or using the terminal by, eg.
# issuing the following command:
#
# ros2 launch week_3 turtlebot3_simulation_launch.py --show-args
#

import os
from math import radians

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def gazebo_server(context : LaunchContext,
                  world : LaunchConfiguration,
                  launch_dir : str,
                  turtlebot3_gazebo_package_dir : str):
    
    world_name = context.launch_configurations['world']
    context.launch_configurations['world'] = os.path.join(turtlebot3_gazebo_package_dir, 'worlds', world_name + '.world')
    
    execute_process = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')
    
    return [execute_process]

def spawn_entity(context: LaunchContext,
                 robot_name : LaunchConfiguration,
                 robot_sdf : LaunchConfiguration,
                 x : LaunchConfiguration,
                 y : LaunchConfiguration,
                 z : LaunchConfiguration,
                 R : LaunchConfiguration,
                 P : LaunchConfiguration,
                 Y : LaunchConfiguration):

    roll = radians(float(context.launch_configurations['R']))
    context.launch_configurations['R'] = str(roll)

    pitch = radians(float(context.launch_configurations['P']))
    context.launch_configurations['P'] = str(pitch)

    yaw = radians(float(context.launch_configurations['Y']))
    context.launch_configurations['Y'] = str(yaw)

    node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-x', x, '-y', y, '-z', z,
            '-R', R, '-P', P, '-Y', Y])
    
    return [node]

def generate_launch_description():
    # Get the launch directory
    local_package_dir = get_package_share_directory('week_3')
    launch_dir = os.path.join(local_package_dir, 'launch')

    # Create variables that point to ROBOTIS TurtleBot3 files
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_name = 'turtlebot3_' + TURTLEBOT3_MODEL
    turtlebot3_gazebo_package_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_bringup_package_dir = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_description_package_dir = get_package_share_directory('turtlebot3_description')
    
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_marker = LaunchConfiguration('use_rviz_marker')
    run_controller = LaunchConfiguration('run_controller')

    # Launch configuration variables specific to simulation
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    robot_controller = LaunchConfiguration('robot_controller')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            turtlebot3_description_package_dir, 'rviz', 'model.rviz'),
        description='Full path to the RViz config file to use')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')
    
    declare_use_rviz_marker_cmd = DeclareLaunchArgument(
        'use_rviz_marker',
        default_value='True',
        description='Whether to start RViz marker node')
    
    declare_run_controller_cmd = DeclareLaunchArgument(
        'run_controller',
        default_value='True',
        description='Run robot controller if true')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='Full path to world model file to load')
    
    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value='-2.00',
        description='x coordinate of robot')
    
    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value='-0.50',
        description='y coordinate of robot')
    
    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value='0.01',
        description='z coordinate of robot')
    
    declare_R_cmd = DeclareLaunchArgument(
        'R',
        default_value='0.00',
        description='roll angle of robot (radians)')
    
    declare_P_cmd = DeclareLaunchArgument(
        'P',
        default_value='0.00',
        description='pitch angle of robot (radians)')
    
    declare_Y_cmd = DeclareLaunchArgument(
        'Y',
        default_value='0.00',
        description='yaw angle of robot (radians)')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value=model_name,
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(turtlebot3_gazebo_package_dir, 'models', model_name, 'model.sdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')
    
    declare_robot_controller_cmd = DeclareLaunchArgument(
        'robot_controller',
        default_value='turtlebot3_random_walk',
        description='robot controller node name')

    # END of launch arguments #
    
    # Specify the actions
    start_gazebo_server_cmd = OpaqueFunction(
        function=gazebo_server, args=[world, launch_dir, turtlebot3_gazebo_package_dir])

    # Gazebo simulator client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='screen')
    
    # URDF model for the robot
    urdf = os.path.join(turtlebot3_gazebo_package_dir, 'urdf', model_name + '.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # ROS2 package that allows publishing the coordinate frames of the robot to 'tf', ie. its joints and so on.
    # See https://github.com/ros/robot_state_publisher for further background.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}])
    
    # Starts gazebo with precise configuration for the robot model 'robot_sdf' using the
    # pose defined by position [x, y, z] and orientation [R, P, Y].
    start_gazebo_spawner_cmd = OpaqueFunction(
        function=spawn_entity,
        args=[robot_name, robot_sdf, x, y, z, R, P, Y])
    
    # Launch file for RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_package_dir, 'launch', 'rviz2.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'rviz_config': rviz_config_file}.items())
    
    # Node that provides pose/state information for rviz.
    rviz_marker_cmd = Node(
        package='week_3',
        executable='rviz_text_marker',
        condition=IfCondition(use_rviz_marker))

    # Actual node that controls the robot
    start_robot_controller_cmd = Node(
        package='week_3',
        executable=robot_controller,
        condition=IfCondition(run_controller))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_rviz_marker_cmd)
    ld.add_action(declare_run_controller_cmd)

    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_R_cmd)
    ld.add_action(declare_P_cmd)
    ld.add_action(declare_Y_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_robot_controller_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(rviz_marker_cmd)

    ld.add_action(start_robot_controller_cmd)

    return ld