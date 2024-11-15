import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'week_8'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-1.0')
    y_pose = LaunchConfiguration('y_pose', default=' 0.0')

    world = os.path.join(
        get_package_share_directory('week_8'),
        'worlds',
        'turtlebot3_world_empty.world'
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

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        'turtlebot3_waffle_pi.urdf')
    
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': robot_description}],)

    true_urdf_path = os.path.join(
        get_package_share_directory('week_8'),
        'urdf',
        'true_turtlebot3_waffle_pi.urdf')
    
    with open(true_urdf_path, 'r') as infp:
        true_robot_description = infp.read()        

    start_true_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='true_robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'robot_description': true_robot_description}],
        remappings = [('robot_description', 'true_robot_description')])
    
    start_joint_state_republisher_cmd = Node(
        package=package_name,
        executable='joint_state_republisher',
        output='screen')
    
    start_path_publisher_cmd = Node(
        package=package_name,
        executable='path_publisher',
        output='screen',
        arguments=['-x', x_pose,
                   '-y', y_pose])

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'robot',
            '-file', PathJoinSubstitution([FindPackageShare(package_name), 'models', 'turtlebot3_waffle_pi_odom', 'model.sdf']),
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'])

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([FindPackageShare(package_name), 'rviz', 'paths.rviz'])],
        output='screen')

    ld = LaunchDescription()

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(start_gazebo_spawner_cmd)    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_true_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_republisher_cmd)
    ld.add_action(start_path_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
