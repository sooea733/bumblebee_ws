#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # TURTLEBOT3_MODEL 환경 변수는 실제 로딩에는 영향 없음
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', '')

    # 런치 인자 선언
    namespace = ''#LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Xacro 파일을 Command로 실행
    robot_description = Command([
        FindExecutable(name = 'xacro'),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('bumblebee_MobileManipulator_ver30_description'),
            'urdf',
            'bumblebee_MobileManipulator_ver4.xacro'
        ])
    ])
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='""',
            description='Namespace for the robot'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': use_sim_time},
                {'frame_prefix': PythonExpression(['"', namespace, '/"'])}
            ]
        )
    ])

"""
def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    
    namespace = LaunchConfiguration('namespace')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'bumblebee_MobileManipulator_ver4.xacro'

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf = os.path.join(
        get_package_share_directory('bumblebee_MobileManipulator_ver30_description'),
        'urdf',
        urdf_file_name)

    rsp_params = {'robot_description': Command(
        [
                'xacro ',
    FindPackageShare('bumblebee_MobileManipulator_ver30_description'),
    '/urdf/bumblebee_MobileManipulator_ver4.xacro'  
    ])}

    # Major refactor of the robot_state_publisher
    # Reference page: https://github.com/ros2/demos/pull/426
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    # print (robot_desc) # Printing urdf information.

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[
                    rsp_params,
                    {'use_sim_time': use_sim_time},
                    {'frame_prefix': PythonExpression(['"', namespace, '/"'])}])
    ])
"""