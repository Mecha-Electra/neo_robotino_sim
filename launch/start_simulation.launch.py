# Copyright 2019 Open Source Robotics Foundation, Inc.
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_neo_robotino_sim = get_package_share_directory('neo_robotino_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r ', os.path.join(pkg_neo_robotino_sim, 'worlds', 'simple.sdf')
            ]
        }.items(),
    )
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
             '-d', os.path.join(pkg_neo_robotino_sim, 'rviz', 'robotino_only.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Robot description urdf parsing
    #using the command to avoid relative path errors
    robot_desc = Command(['xacro ', os.path.join(pkg_neo_robotino_sim, 'urdf', 'robotino_base.urdf.xacro')])

    # Robot state publisher
    robot_state_publisher_parameters=[
        #this following line is for loading without misinterpreting values as another file
        {'robot_description': launch_ros.descriptions.ParameterValue(robot_desc, value_type=str)},
        {'use_sim_time': True}
        ]

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=robot_state_publisher_parameters,
            arguments=[])

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
        output='screen'
    )

    #robot spawning
    spawn = Node(package='ros_gz_sim', executable='create',
        arguments=[
        '-name', 'Robotino2',
        '-x', '0',
        '-z', '1',
        '-Y', '0',
        '-topic', '/robot_description'],
        output='screen'
    )

    #controle
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[rviz, load_joint_trajectory_controller, bridge],
            )
        ),
        spawn
    ])