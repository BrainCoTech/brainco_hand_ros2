#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('model', default_value='dual_revo2_hand',
                              description='Robot model name.'),
        DeclareLaunchArgument('world', default_value='empty_world',
                              description='Gz sim World'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='Launch RViz visualization'),
    ]

    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')

    # Package paths
    demo_package_path = get_package_share_directory('brainco_gazebo')
    revo2_description_path = get_package_share_directory('revo2_description')

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(demo_package_path, 'worlds'), ':' +
            str(Path(revo2_description_path).parent.resolve())
        ]
    )

    # Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch'),
            '/gz_sim.launch.py'
        ]),
        launch_arguments=[
            ('gz_args', [
                world,
                '.sdf',
                ' -v 1',
                ' -r'
            ])
        ]
    )

    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('brainco_gazebo'),
            'config',
            'dual_revo2_gazebo_description.urdf.xacro',
        ]),
        ' ',
        'model:=', model,
        ' ',
        'use_sim:=true',
    ])

    robot_description = {'robot_description': robot_description_content}

    # Robot state publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # Spawn entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', model,
            '-allow_renaming', 'true',
        ],
    )

    # Controllers
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('brainco_gazebo'),
        'config',
        'dual_revo2_controllers.yaml',
    ])

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    left_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_revo2_hand_controller', '--param-file', controllers_yaml],
        output='screen'
    )

    right_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_revo2_hand_controller', '--param-file', controllers_yaml],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # RViz (optional)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('brainco_gazebo'),
        'rviz',
        'dual_revo2_hand.rviz',
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=LaunchConfigurationEquals('use_rviz', 'true'),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        *declared_arguments,
        # Sequential controller loading
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_hand_controller_spawner, right_hand_controller_spawner],
            )
        ),
        # Core nodes
        bridge,
        gazebo_resource_path,
        gazebo,
        robot_state_pub_node,
        gz_spawn_entity,
        rviz_node,
    ])
