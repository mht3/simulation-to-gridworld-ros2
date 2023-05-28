#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

TURTLEBOT3_MODEL = 'burger'

def gen_robot_list(number_of_robots):

    robots = []

    for i in range(number_of_robots):
        robot_name = "turtlebot_"+str(i)

        if number_of_robots <= 2:
            x_pos = 0.
        else:
            #TODO Fix if we have more than 2 robots
            x_pos = float(i)

        y_pos = 2.
        if i % 2 == 0:
            y_pos = -y_pos

        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': y_pos, 'z_pose': 0.1})


    return robots 

def get_num_robots(context: LaunchContext, num_robots: LaunchConfiguration):
    value = context.perform_substitution(num_robots)
    print(value)
    return Node(name=value)

def generate_launch_description():

    package = 'ctf_package'
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    sdf_file_name = 'model.sdf'
    robot_name = 'turtlebot3_' + TURTLEBOT3_MODEL
    urdf = os.path.join(get_package_share_directory(package), 'turtlebot3_description', 'urdf', urdf_file_name)
    sdf = os.path.join(get_package_share_directory(package), 'models/turtlebot3_burger', sdf_file_name)
    pkg_description = get_package_share_directory(package)
    assert os.path.exists(urdf), "{} doesnt exist in ".format(robot_name) + str(urdf)

    # Names and poses of the robots
    num_robots = 2
    robots = gen_robot_list(num_robots)

    # We create the list of spawn robots commands
    spawn_robots_cmds = []

    for robot in robots:
        spawn_robots_cmds.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_description, 'launch',
                                                           'spawn_turtlebot.launch.py')),
                launch_arguments={
                                  'robot_urdf': urdf,
                                  'robot_sdf' : sdf,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose'])),
                                  'robot_name': robot['name'],
                                  'robot_namespace': robot['name']
                                  }.items()))

    # Create the launch description and populate
    ld = LaunchDescription()
    # ld.add_action(OpaqueFunction(function=get_num_robots, args=[num_robots_arg]))
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld

