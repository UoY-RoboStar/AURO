# Based on:
# https://raw.githubusercontent.com/ros-planning/navigation2/humble/nav2_bringup/launch/rviz_launch.py
# https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_bringup/launch/rviz2.launch.py

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def rviz_action(context : LaunchContext):

    namespace = context.launch_configurations['ros_namespace']
    rviz_config_file = context.launch_configurations['rviz_config']

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': namespace})
    
    namespaced_rviz_config_file = ReplaceString(
            source_file=namespaced_rviz_config_file,
            replacements={'<window_x>': context.launch_configurations['window_x'],
                          '<window_y>': context.launch_configurations['window_y'],
                          '<window_width>': context.launch_configurations['window_width'],
                          '<window_height>': context.launch_configurations['window_height']})

    start_namespaced_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', namespaced_rviz_config_file],
        output='screen')

    exit_event_handler_namespaced = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    return [start_namespaced_rviz_cmd, exit_event_handler_namespaced]


def generate_launch_description():
    package_name = 'assessment'

    # Create the launch configuration variables
    rviz_config_file = LaunchConfiguration('rviz_config')
    window_x = LaunchConfiguration('window_x')
    window_y = LaunchConfiguration('window_y')
    window_width = LaunchConfiguration('window_width')
    window_height = LaunchConfiguration('window_height')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'namespaced.rviz']),
        description='Full path to the RViz config file to use')
    
    declare_window_x_cmd = DeclareLaunchArgument(
        'window_x',
        default_value='0',
        description='RViz window X offset from top-left corner of screen')
    
    declare_window_y_cmd = DeclareLaunchArgument(
        'window_x',
        default_value='0',
        description='RViz window Y offset from top-left corner of screen')
    
    declare_window_width_cmd = DeclareLaunchArgument(
        'window_width',
        default_value='1280',
        description='RViz window width')
    
    declare_window_height_cmd = DeclareLaunchArgument(
        'window_height',
        default_value='720',
        description='RViz window height')    
    
    rviz_cmd = OpaqueFunction(function=rviz_action)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_window_x_cmd)
    ld.add_action(declare_window_y_cmd)
    ld.add_action(declare_window_width_cmd)
    ld.add_action(declare_window_height_cmd)

    ld.add_action(rviz_cmd)

    return ld
