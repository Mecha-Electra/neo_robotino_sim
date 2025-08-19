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
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    set_res_path = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.abspath(os.path.join(pkg_neo_robotino_sim, os.pardir))) #vai pelo pai de pkg_neo_robotino_sim

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r ', os.path.join(pkg_neo_robotino_sim, 'worlds', 'simple.sdf'),
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

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_neo_robotino_sim, 'launch', 'robot_description.launch.py')),
        launch_arguments={
            'use_sim_time': 'true'
        }.items(),
    )
    
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_0@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_0/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_1@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_2@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_2/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_3@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_3/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_4@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_4/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_5@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_5/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_6@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_6/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_7@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_7/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
                   'dist_sensor_8@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   '/dist_sensor_8/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked'],
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
    robot_controllers = os.path.join(
            pkg_neo_robotino_sim,
            'config',
            'motor_velocity_controller.yaml',
    )

    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'velocity_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    robotino_node = Node(
        package='neo_robotino_sim',
        executable='robotino_node',
        arguments=[],
    )

    delayed_joint_state_broad = TimerAction(
        period=20.0,  # Tempo de delay em segundos
        actions=[joint_state_broadcaster_spawner]  # Nenhuma ação executada durante o delay
    )

    return LaunchDescription([
        set_res_path,
        gz_sim,
        robot_description,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[rviz, delayed_joint_state_broad, bridge, robotino_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[velocity_controller_spawner],
            )
        ),
        TimerAction(
            period=30.0,  # Tempo de delay em segundos
            actions=[spawn]  # Nenhuma ação executada durante o delay
        )
    ])
