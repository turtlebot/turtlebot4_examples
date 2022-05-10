#!/usr/bin/env python3
# Copyright 2021 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='lite',
                          choices=['standard', 'lite'],
                          description='Turtlebot4 Model')
]

def generate_launch_description():
    pkg_turtlebot4_bringup = get_package_share_directory('turtlebot4_bringup')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_turtlebot4_cpp_examples = get_package_share_directory('turtlebot4_cpp_examples')

    param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution(
            [pkg_turtlebot4_bringup, 'config', 'turtlebot4.yaml']),
        description='Turtlebot4 Robot param file'
    )

    turtlebot4_param_yaml_file = LaunchConfiguration('param_file')

    # Launch files
    turtlebot4_robot_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'robot.launch.py'])
    joy_teleop_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_bringup, 'launch', 'joy_teleop.launch.py'])
    oakd_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_cpp_examples, 'launch', 'mobilenet_spatial_publisher.launch.py'])
    description_launch_file = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    )

    standard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([turtlebot4_robot_launch_file]),
        launch_arguments=[('model', LaunchConfiguration('model')),
                          ('param_file', turtlebot4_param_yaml_file)])
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([joy_teleop_launch_file]))
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([oakd_launch_file]),
        launch_arguments=[('tf_prefix', 'oakd_pro')])
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_file]),
        launch_arguments=[('model', LaunchConfiguration('model'))])

    followbot = Node(
        package='turtlebot4_python_examples',
        executable='followbot',
        output='screen'
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(param_file_cmd)
    ld.add_action(standard_launch)
    ld.add_action(teleop_launch)
    ld.add_action(oakd_launch)
    ld.add_action(description_launch)
    ld.add_action(followbot)
    return ld
