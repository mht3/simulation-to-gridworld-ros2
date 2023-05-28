#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'ctf_package'
    pkg = get_package_share_directory(package_name)

    gazebo_models_path = os.path.join(pkg, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    # Start World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'empty_world.launch.py'),
        )
    )

    spawn_robot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'spawn_multiple_turtlebots.launch.py'),
        )
    )     

    create_broadcaster = Node(
            package='ctf_package',
            executable='tf_broadcaster',
            parameters=[
                {'turtlename': 'turtlebot_0'}
            ]
    )

    return LaunchDescription([
        start_world,
        spawn_robot_world,
        # create_broadcaster
    ])