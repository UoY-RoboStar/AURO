# Based on:
# https://github.com/ROBOTIS-GIT/turtlebot3_simulations/blob/humble-devel/turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py
# https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/tb3_simulation_launch.py

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

package_name = 'assessment'
package_dir = FindPackageShare(package_name)
launch_dir = os.path.join(get_package_share_directory('assessment'), 'launch')

def bringup_actions(context : LaunchContext):

    urdf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_waffle_pi' + '.urdf')

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}])

    wait_for_items = eval(context.launch_configurations['wait_for_items'].lower().capitalize())

    if wait_for_items == True:
        wait_entity = 'ready'
    else:
        wait_entity = ''

    robot_name = context.launch_configurations['ros_namespace'].strip('/')

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',        
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-wait', wait_entity,
            '-entity', robot_name,
            '-file', context.launch_configurations['robot_sdf'],
            '-robot_namespace', robot_name,
            '-x', context.launch_configurations['x_pose'],
            '-y', context.launch_configurations['y_pose'],
            '-z', context.launch_configurations['z_pose'],
            '-R', context.launch_configurations['roll'],
            '-P', context.launch_configurations['pitch'],
            '-Y', context.launch_configurations['yaw']])
    
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        condition=IfCondition(context.launch_configurations['use_nav2']),
        launch_arguments={'namespace': robot_name,
                          'map': context.launch_configurations['map'],
                          'use_sim_time': 'True',
                          'params_file': context.launch_configurations['params_file'],
                          'autostart': 'True',
                          'use_composition': 'True',
                          'use_respawn': 'True'}.items())
    
    return [start_robot_state_publisher_cmd, start_gazebo_spawner_cmd, bringup_cmd]


def generate_launch_description():

    use_nav2 = LaunchConfiguration('use_nav2')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')
    robot_sdf = LaunchConfiguration('robot_sdf')
    wait_for_items = LaunchConfiguration('wait_for_items')

    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial pose: x')
    
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial pose: y')
    
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Initial pose: z')
    
    declare_roll_cmd = DeclareLaunchArgument(
        'roll',
        default_value='0.0',
        description='Initial pose: roll')
    
    declare_pitch_cmd = DeclareLaunchArgument(
        'pitch',
        default_value='0.0',
        description='Initial pose: pitch')
    
    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial pose: yaw')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=PathJoinSubstitution([package_dir, 'models', 'waffle_pi', 'model.sdf']),
        description='Full path to robot SDF file to spawn the robot in Gazebo')
    
    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Whether to use the navigation stack (Nav2)')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([package_dir, 'maps', 'assessment_world.yaml']),
        description='Full path to map file to load')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory(package_name), 'params', 'nav2_params_namespaced.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_wait_for_items_cmd = DeclareLaunchArgument(
        'wait_for_items',
        default_value='False',
        description='Whether to wait for every item to spawn before spawning any robots')
    
    bringup_cmd = OpaqueFunction(function=bringup_actions)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_z_pose_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_wait_for_items_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd)

    return ld
