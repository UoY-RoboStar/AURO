import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import yaml

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, Shutdown, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace, RosTimer


def robot_controller_actions(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
        
    yaml_path = os.path.join(get_package_share_directory('assessment'), 'config', 'initial_poses.yaml')

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    actions = []

    for robot_number in range(1, num_robots + 1):

        robot_name = 'robot' + str(robot_number)

        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            Node(
                package='solution',
                executable='robot_controller',
                # prefix=['xfce4-terminal --tab --execute'], # Opens in new tab
                # prefix=['xfce4-terminal --execute'], # Opens in new window
                # prefix=['gnome-terminal --tab --execute'], # Opens in new tab
                # prefix=['gnome-terminal --window --execute'], # Opens in new window
                # prefix=['wt.exe --window 0 new-tab wsl.exe -e bash -ic'], # Opens in new tab
                # prefix=['wt.exe wsl.exe -e bash -ic'], # Opens in new window
                output='screen',
                parameters=[initial_poses[robot_name]]),

            # Node(
            #     package='turtlebot3_gazebo',
            #     executable='turtlebot3_drive',
            #     output='screen'),

        ])

        actions.append(group)

    return actions

def generate_launch_description():

    package_name = 'solution'

    num_robots = LaunchConfiguration('num_robots')
    random_seed = LaunchConfiguration('random_seed')
    experiment_duration = LaunchConfiguration('experiment_duration')
    data_log_path = LaunchConfiguration('data_log_path')
    data_log_filename = LaunchConfiguration('data_log_filename')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for item manager')
    
    declare_experiment_duration_cmd = DeclareLaunchArgument(
        'experiment_duration',
        default_value='300.0',
        description='Experiment duration in seconds')
    
    declare_data_log_path_cmd = DeclareLaunchArgument(
        'data_log_path',
        default_value = os.path.join(get_package_prefix(package_name), '../../'),
        description='Full path to directory where data logs will be saved')
    
    declare_data_log_filename_cmd = DeclareLaunchArgument(
        'data_log_filename',
        default_value='data_log',
        description='Filename prefix to use for data logs')

    rviz_config = PathJoinSubstitution([FindPackageShare('assessment'), 'rviz', 'namespaced_nav2.rviz'])
    rviz_windows = PathJoinSubstitution([FindPackageShare('assessment'), 'config', 'rviz_windows.yaml'])
    # rviz_windows = PathJoinSubstitution([FindPackageShare(package_name), 'config', 'custom_rviz_windows.yaml'])
    map = PathJoinSubstitution([FindPackageShare('assessment'), 'maps', 'assessment_world.yaml'])
    params = PathJoinSubstitution([FindPackageShare('assessment'), 'params', 'nav2_params_namespaced.yaml'])
    # params = PathJoinSubstitution([FindPackageShare(package_name), 'params', 'custom_nav2_params_namespaced.yaml'])

    assessment_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('assessment'),
                'launch',
                'assessment_launch.py'
                ])
        ),
        launch_arguments={'num_robots': num_robots,
                          'visualise_sensors': 'false',
                          'odometry_source': 'ENCODER',
                          'sensor_noise': 'false',
                          'use_rviz': 'true',
                          'rviz_config': rviz_config,
                          'rviz_windows': rviz_windows,
                          'obstacles': 'true',
                          'item_manager': 'true',
                          'random_seed': random_seed,
                          'use_nav2': 'True',
                          'map': map,
                          'params_file': params,
                          'headless': 'false',
                          'limit_real_time_factor': 'true',
                          'wait_for_items': 'false',
                          # 'extra_gazebo_args': '--verbose',
                          }.items()
    )

    robot_controller_cmd = OpaqueFunction(function=robot_controller_actions)

    data_logger_cmd = Node(
        package='solution',
        executable='data_logger',
        output='screen',
        arguments=['--path', data_log_path,
                   '--filename', data_log_filename,
                   '--random_seed', random_seed])

    timeout_cmd = RosTimer(                                         
            period = experiment_duration,
            actions = [                                                       
                Shutdown(reason="Experiment timeout reached")     
            ],
        )

    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_experiment_duration_cmd)
    ld.add_action(declare_data_log_path_cmd)
    ld.add_action(declare_data_log_filename_cmd)

    ld.add_action(assessment_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(data_logger_cmd)
    ld.add_action(timeout_cmd)

    return ld
