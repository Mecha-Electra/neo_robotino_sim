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
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
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
            # '-d', os.path.join(pkg_ros_gz_sim_demos, 'rviz', 'rgbd_camera_bridge.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    robot_desc = Command(['xacro ', os.path.join(pkg_neo_robotino_sim, 'urdf', 'robotino_base.urdf.xacro')])
    robot_state_publisher_parameters=[
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
        # Topics to be connected (gazebo ros bridge)
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan/'
        ],
        output='screen',
        # topics that should be remapped (from, to)
        remappings=[
            ('/camera', '/camera/image'),
            ('/camera_info', '/camera/camera_info')
        ],
    )

    spawn = Node(package='ros_gz_sim', executable='create',
        arguments=[
        '-name', 'Robotino2',
        '-x', '0',
        '-z', '1',
        '-Y', '0',
        '-topic', '/robot_description'],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        spawn,
        bridge,
        rviz,
    ])