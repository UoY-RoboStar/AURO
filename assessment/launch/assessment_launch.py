# Based on: https://github.com/ros-planning/navigation2/blob/humble/nav2_bringup/launch/cloned_multi_tb3_simulation_launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter, SetRemap, PushRosNamespace

import xml.etree.ElementTree as ET
import yaml

package_name = 'assessment'
launch_file_dir = PathJoinSubstitution([FindPackageShare(package_name), 'launch'])
pkg_gazebo_ros = FindPackageShare('gazebo_ros')

def gazebo_world(context : LaunchContext):

    obstacles = eval(context.launch_configurations['obstacles'].lower().capitalize())
    limit_real_time_factor = eval(context.launch_configurations['limit_real_time_factor'].lower().capitalize())
    zone_top_left = eval(context.launch_configurations['zone_top_left'].lower().capitalize())
    zone_top_right = eval(context.launch_configurations['zone_top_right'].lower().capitalize())
    zone_bottom_left = eval(context.launch_configurations['zone_bottom_left'].lower().capitalize())
    zone_bottom_right = eval(context.launch_configurations['zone_bottom_right'].lower().capitalize())

    world_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'assessment_world.world')
    tree = ET.parse(world_path)
    root = tree.getroot()

    if obstacles == False:
        for world in root.findall('.//world'):
            for model in world.findall('.//model'):
                if "box" in model.attrib['name'] or "cylinder" in model.attrib['name']:
                    world.remove(model)

    if limit_real_time_factor == False:
        for node in root.iter("real_time_update_rate"):
            for element in node.iter():
                element.text = "0.0"

    if zone_top_left == False:
        for visual in root.findall(".//world/model/link/visual"):
            if "top_left" in visual.attrib['name']:
                if "floor" in visual.attrib['name']:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "grey"
                else:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "yellow"

    if zone_top_right == False:
        for visual in root.findall(".//world/model/link/visual"):
            if "top_right" in visual.attrib['name']:
                if "floor" in visual.attrib['name']:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "grey"
                else:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "yellow"

    if zone_bottom_left == False:
        for visual in root.findall(".//world/model/link/visual"):
            if "bottom_left" in visual.attrib['name']:
                if "floor" in visual.attrib['name']:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "grey"
                else:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "yellow"

    if zone_bottom_right == False:
        for visual in root.findall(".//world/model/link/visual"):
            if "bottom_right" in visual.attrib['name']:
                if "floor" in visual.attrib['name']:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "grey"
                else:
                    for colour_name in visual.findall(".//material/script/name"):
                        colour_name.text = "yellow"

    world = os.path.join(get_package_share_directory(package_name), 'worlds', 'simulation_world.world')

    with open(world, 'w') as f:
        tree.write(f, encoding='unicode')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world, 'force_system': 'False'}.items()
    )

    return [gzserver_cmd]

def group_action(context : LaunchContext):

    num_robots = int(context.launch_configurations['num_robots'])
    visualise_sensors = context.launch_configurations['visualise_sensors'].lower()
    odometry_source = context.launch_configurations['odometry_source']
    sensor_noise = eval(context.launch_configurations['sensor_noise'].lower().capitalize())
    initial_pose_package = context.launch_configurations['initial_pose_package']
    initial_pose_file = context.launch_configurations['initial_pose_file']

    sdf_path = os.path.join(get_package_share_directory(package_name), 'models', 'waffle_pi', 'model.sdf')
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    for node in root.iter("visualize"):
        for element in node.iter():
            element.text = visualise_sensors

    for node in root.iter("odometry_source"):
        for element in node.iter():
            if odometry_source == "ENCODER":
                element.text = "0"
            elif odometry_source == "WORLD":
                element.text = "1"

    if sensor_noise == False:

        for sensor in root.findall('.//sensor'):
            for imu in sensor.findall('.//imu'):
                sensor.remove(imu)

        for ray in root.findall('.//ray'):
            for noise in ray.findall('.//noise'):
                ray.remove(noise)

        for camera in root.findall('.//camera'):
            for noise in camera.findall('.//noise'):
                camera.remove(noise)

    robot_sdf = os.path.join(get_package_share_directory(package_name), 'models', 'robot.sdf')

    with open(robot_sdf, 'w') as f:
        tree.write(f, encoding='unicode')

    yaml_path = os.path.join(get_package_share_directory(initial_pose_package), initial_pose_file)

    print('pose: ' + yaml_path)

    with open(yaml_path, 'r') as f:
        configuration = yaml.safe_load(f)

    initial_poses = configuration[num_robots]

    with open(context.launch_configurations['rviz_windows'], 'r') as f:
        configuration = yaml.safe_load(f)

    rviz_windows = configuration[num_robots]

    bringup_cmd_group = []

    for robot_name, init_pose in initial_poses.items():
        group = GroupAction([

            PushRosNamespace(robot_name),
            SetRemap('/tf', 'tf'),
            SetRemap('/tf_static', 'tf_static'),

            LogInfo(msg=['Launching namespace=', robot_name, ' init_pose=', str(init_pose)]),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_file_dir, 'rviz_launch.py'])),
                condition=IfCondition(context.launch_configurations['use_rviz']),
                launch_arguments={'rviz_config': context.launch_configurations['rviz_config'],
                                  'window_x': str(rviz_windows[robot_name]['window_x']),
                                  'window_y': str(rviz_windows[robot_name]['window_y']),
                                  'window_width': str(rviz_windows[robot_name]['window_width']),
                                  'window_height': str(rviz_windows[robot_name]['window_height'])}.items()),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    launch_file_dir,
                    'spawn_robot_launch.py'])),
                launch_arguments={'use_nav2': context.launch_configurations['use_nav2'],
                                  'map': context.launch_configurations['map'],
                                  'params_file': context.launch_configurations['params_file'],
                                  'x_pose': TextSubstitution(text=str(init_pose['x'])),
                                  'y_pose': TextSubstitution(text=str(init_pose['y'])),
                                  'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                                  'robot_sdf': robot_sdf}.items()),
            Node(
                package=package_name,
                executable='item_sensor',
                output='screen'),
            Node(
                package=package_name,
                executable='zone_sensor',
                output='screen'),
            Node(
                package=package_name,
                executable='robot_sensor',
                output='screen')
        ])
    
        bringup_cmd_group.append(group)

    return bringup_cmd_group

def generate_launch_description():

    num_robots = LaunchConfiguration('num_robots')
    visualise_sensors = LaunchConfiguration('visualise_sensors')
    odometry_source = LaunchConfiguration('odometry_source')
    sensor_noise = LaunchConfiguration('sensor_noise')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config')
    rviz_windows = LaunchConfiguration('rviz_windows')
    obstacles = LaunchConfiguration('obstacles')
    item_manager = LaunchConfiguration('item_manager')
    random_seed = LaunchConfiguration('random_seed')
    use_nav2 = LaunchConfiguration('use_nav2')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    headless = LaunchConfiguration('headless')
    limit_real_time_factor = LaunchConfiguration('limit_real_time_factor')
    wait_for_items = LaunchConfiguration('wait_for_items')

    # Zone configuration
    zone_top_left = LaunchConfiguration('zone_top_left')
    zone_top_right = LaunchConfiguration('zone_top_right')
    zone_bottom_left = LaunchConfiguration('zone_bottom_left')
    zone_bottom_right = LaunchConfiguration('zone_bottom_right')

    # Initial position configuration
    initial_pose_package = LaunchConfiguration('initial_pose_package')
    initial_pose_file = LaunchConfiguration('initial_pose_file')

    declare_initial_pose_file = DeclareLaunchArgument(
        'initial_pose_file',
        default_value='config/initial_poses.yaml',
        description="Location of initial pose yaml file relative to the package in 'initial_pose_package'"
    )

    declare_initial_pose_package = DeclareLaunchArgument(
        'initial_pose_package',
        default_value='assessment',
        description="Package name for finding the file 'config/initial_poses.yaml'"
        )

    declare_zone_top_left = DeclareLaunchArgument(
        'zone_top_left',
        default_value='true',
        description='Whether to enable the top left zone'
        )
    
    declare_zone_top_right = DeclareLaunchArgument(
        'zone_top_right',
        default_value='true',
        description='Whether to enable the top right zone'
        )

    declare_zone_bottom_left = DeclareLaunchArgument(
        'zone_bottom_left',
        default_value='true',
        description='Whether to enable the bottom left zone'
        )
    
    declare_zone_bottom_right = DeclareLaunchArgument(
        'zone_bottom_right',
        default_value='true',
        description='Whether to enable the bottom right zone'
        )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('assessment'), 'maps', 'assessment_world.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('assessment'), 'params', 'nav2_params_namespaced.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_num_robots_cmd = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn')
    
    declare_visualise_sensors_cmd = DeclareLaunchArgument(
        'visualise_sensors',
        default_value='false',
        description='Whether to visualise sensors in Gazebo')
    
    declare_odometry_source_cmd = DeclareLaunchArgument(
        'odometry_source',
        default_value='ENCODER',
        description='Odometry source - ENCODER or WORLD')
    
    declare_sensor_noise_cmd = DeclareLaunchArgument(
        'sensor_noise',
        default_value='false',
        description='Whether to enable sensor noise (applies to camera, LiDAR, and IMU)')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RViz')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'namespaced.rviz']),
        description='Full path to the RViz config file to use')
    
    declare_rviz_windows_cmd = DeclareLaunchArgument(
        'rviz_windows',
        default_value=PathJoinSubstitution([FindPackageShare(package_name), 'config', 'rviz_windows.yaml']),
        description='Full path to the RViz windows YAML file to use')        
    
    declare_obstacles_cmd = DeclareLaunchArgument(
        'obstacles',
        default_value='true',
        description='Whether the world contains obstacles')

    declare_item_manager_cmd = DeclareLaunchArgument(
        'item_manager',
        default_value='true',
        description='Whether to start the item manager')
    
    declare_random_seed_cmd = DeclareLaunchArgument(
        'random_seed',
        default_value='0',
        description='Random number seed for item manager')
    
    declare_use_nav2_cmd = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Whether to use the navigation stack (Nav2)')
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to run the Gazebo GUI')
    
    declare_limit_real_time_factor_cmd = DeclareLaunchArgument(
        'limit_real_time_factor',
        default_value='True',
        description='Whether to limit the Gazebo real-time factor to 1.0')
    
    declare_wait_for_items_cmd = DeclareLaunchArgument(
        'wait_for_items',
        default_value='False',
        description='Whether to wait for every item to spawn before spawning any robots')
    
    gzserver_cmd = OpaqueFunction(function=gazebo_world)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
        condition=UnlessCondition(headless)
    )

    start_tf_relay_cmd = Node(
        package='tf_relay',
        executable='relay',
        output='screen',
        arguments=['robot', num_robots])

    start_item_manager_cmd = Node(
        package=package_name,
        executable='item_manager',
        output='screen',
        condition=IfCondition(item_manager),
        arguments=['--random_seed', random_seed],
        parameters=[{'zone_top_left': zone_top_left, 'zone_top_right': zone_top_right, 'zone_bottom_left': zone_bottom_left, 'zone_bottom_right':zone_bottom_right}])

    bringup_cmd_group = OpaqueFunction(function=group_action)
        
    ld = LaunchDescription()

    ld.add_action(SetParameter(name='use_sim_time', value=True))

    # Declare the launch options
    ld.add_action(declare_num_robots_cmd)
    ld.add_action(declare_visualise_sensors_cmd)
    ld.add_action(declare_odometry_source_cmd)
    ld.add_action(declare_sensor_noise_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_rviz_windows_cmd)
    ld.add_action(declare_obstacles_cmd)
    ld.add_action(declare_item_manager_cmd)
    ld.add_action(declare_random_seed_cmd)
    ld.add_action(declare_use_nav2_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_limit_real_time_factor_cmd)
    ld.add_action(declare_wait_for_items_cmd)    

    # Zone launch options
    ld.add_action(declare_zone_top_left)
    ld.add_action(declare_zone_top_right)
    ld.add_action(declare_zone_bottom_left)
    ld.add_action(declare_zone_bottom_right)

    # Add launch initial pose option
    ld.add_action(declare_initial_pose_package)
    ld.add_action(declare_initial_pose_file)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    ld.add_action(start_tf_relay_cmd)
    ld.add_action(start_item_manager_cmd)

    ld.add_action(bringup_cmd_group)

    return ld
