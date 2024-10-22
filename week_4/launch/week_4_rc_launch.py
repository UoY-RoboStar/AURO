import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'week_4'
    launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    turtlebot3_gazebo_package_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_bringup_package_dir = get_package_share_directory('turtlebot3_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'))

    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'week_4_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_package_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )  

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_package_dir, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_package_dir, 'launch', 'rviz2.launch.py')),
        launch_arguments={'rviz_config': rviz_config_file}.items())

    ld = LaunchDescription()

    rc_driving = Node(
        package='week_4',
        executable='rc_driving',
        output='screen'
    )

    rc_obstacle_detector = Node(
        package='week_4',
        executable='rc_obstacle_detector',
        output='screen'
    )

    rviz_text_marker = Node(
        package='week_3',
        executable='rviz_text_marker',
        output='screen'
    )

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(robot_controller)
    ld.add_action(rc_driving)
    ld.add_action(rc_obstacle_detector)
    ld.add_action(rviz_text_marker)
    ld.add_action(rviz_cmd)

    return ld
