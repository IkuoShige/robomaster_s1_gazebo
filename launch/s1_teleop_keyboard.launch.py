from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_robomaster_s1_gazebo = get_package_share_directory('robomaster_s1_gazebo')

    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robomaster_s1_gazebo, 'launch', 's1_gazebo.launch.py')
        )
    )

    # Mecanum wheel teleop keyboard node (custom for omnidirectional control)
    teleop_keyboard = Node(
        package='robomaster_s1_gazebo',
        executable='mecanum_teleop_keyboard.py',
        name='mecanum_teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal window
        remappings=[
            ('cmd_vel', 'cmd_vel')  # Publish to /cmd_vel for planar_move plugin
        ]
    )

    return LaunchDescription([
        gazebo,
        teleop_keyboard,
    ])
