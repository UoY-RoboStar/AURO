# Based on: https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():
    package_name = 'week_6'
    launch_file_dir = os.path.join(get_package_share_directory(package_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    turtlebot3_gazebo_package_dir = get_package_share_directory('turtlebot3_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub', default="True")

    world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'week_6_world.world'
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

    robots_list = {}

    robots_list["robot1"] = {}
    robots_list["robot1"]["x"] = 1.0
    robots_list["robot1"]["y"] = 1.0
    robots_list["robot1"]["yaw"] = 0.0

    robots_list["robot2"] = {}
    robots_list["robot2"]["x"] = -1.0
    robots_list["robot2"]["y"] = -1.0
    robots_list["robot2"]["yaw"] = 0.0

    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction([
            LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'rviz_launch.py')),
                launch_arguments={'namespace': TextSubstitution(text=robot_name)}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_file_dir,
                                                           'spawn_robot_launch.py')),
                launch_arguments={'namespace': robot_name,
                                  'use_sim_time': 'True',
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub,
                                  'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                  'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                  'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                  'robot_name':TextSubstitution(text=robot_name), }.items())
        ])

        bringup_cmd_group.append(group)

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    for cmd in bringup_cmd_group:
        ld.add_action(cmd)

    return ld
