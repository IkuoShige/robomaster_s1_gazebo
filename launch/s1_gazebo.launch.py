"""
RoboMaster S1 Gazebo Launch File

Launches RoboMaster S1 robot in Gazebo with:
- ISCAS Museum world environment
- Livox MID-360 LiDAR sensor
- Mecanum wheel omnidirectional control via planar_move plugin
- Camera and gimbal system

Robot control:
  - Subscribe: /cmd_vel (geometry_msgs/Twist)
  - Publish: /odom (nav_msgs/Odometry)
  - Publish: /livox/lidar (sensor_msgs/PointCloud2)

Usage:
  ros2 launch robomaster_s1_gazebo s1_gazebo.launch.py [x_pose:=0.0] [y_pose:=0.0] [z_pose:=0.1]
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_robomaster_s1_gazebo = get_package_share_directory('robomaster_s1_gazebo')
    pkg_robomaster_description = get_package_share_directory('robomaster_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_ros2_livox_simulation = get_package_share_directory('ros2_livox_simulation')

    # Paths
    world_file = os.path.join(pkg_robomaster_s1_gazebo, 'worlds', 'iscas_museum.world')
    urdf_file = os.path.join(pkg_robomaster_description, 'urdf', 'robomaster_s1_gazebo.urdf.xacro')

    # Set GAZEBO_MODEL_PATH to include robomaster_description and ros2_livox_simulation packages
    gazebo_model_path = os.path.join(pkg_robomaster_description, '..') + ':' + os.path.join(pkg_ros2_livox_simulation, '..')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = gazebo_model_path + ':' + os.environ['GAZEBO_MODEL_PATH']

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    verbose = LaunchConfiguration('verbose', default='false')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': verbose,
            'gui': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }]
    )

    # Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'robomaster_s1',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    return LaunchDescription([
        # Set DISPLAY for GUI
        SetEnvironmentVariable('DISPLAY', ':0'),
        # Disable Gazebo online model database to speed up startup
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        # Set GAZEBO_MODEL_PATH to find robomaster_description meshes
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('verbose', default_value='false', description='Set to true for verbose Gazebo output'),
        DeclareLaunchArgument('x_pose', default_value='0.0', description='X position of robot'),
        DeclareLaunchArgument('y_pose', default_value='0.0', description='Y position of robot'),
        DeclareLaunchArgument('z_pose', default_value='0.1', description='Z position of robot'),
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
