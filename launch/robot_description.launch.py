import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions
from launch_ros.actions import Node


def generate_launch_description():

    # pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_neo_robotino_sim = get_package_share_directory('neo_robotino_sim')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for simulation, false for real robot)'
    )

    # Robot description urdf parsing
    #using the command to avoid relative path errors
    robot_desc = Command(['xacro ', os.path.join(pkg_neo_robotino_sim, 'urdf', 'robotino_base.urdf.xacro')])

    # Robot state publisher
    robot_state_publisher_parameters=[
        #this following line is for loading without misinterpreting values as another file
        {'robot_description': launch_ros.descriptions.ParameterValue(robot_desc, value_type=str)},
        {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=robot_state_publisher_parameters,
            arguments=[])

    return LaunchDescription([
        robot_state_publisher,
        use_sim_time_arg
    ])