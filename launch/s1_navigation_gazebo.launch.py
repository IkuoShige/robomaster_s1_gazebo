#!/usr/bin/env python3
"""
RoboMaster S1 Navigation in Gazebo Simulation Launch File
Launches Gazebo simulation and navigation stack together
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get package directories
    gazebo_pkg_dir = get_package_share_directory('robomaster_s1_gazebo')
    nav_pkg_dir = get_package_share_directory('robomaster_s1_tvvf_navigation')

    # Launch arguments
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map_file')

    # Default map file path
    default_map_file = os.path.join(nav_pkg_dir, 'maps', 'maps.yaml')

    # Declare launch arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='museum',
        description='World name (museum, empty, etc.)'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    # Launch Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_dir, 'launch', 's1_gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
        }.items()
    )

    # Launch navigation stack
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_dir, 'launch', 'waypoint_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',  # Use Gazebo clock
            'map_file': map_file,     # Map file to load
            'rviz': rviz,
        }.items()
    )

    # Create launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_world)
    ld.add_action(declare_rviz)
    ld.add_action(declare_map_file)

    # Add launch includes
    ld.add_action(gazebo_launch)
    ld.add_action(navigation_launch)

    return ld
