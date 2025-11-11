#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup",
        ),
        DeclareLaunchArgument(
            "ctrl_param_duration_ms",
            default_value="20",
            description="运动时间参数，单位ms，1ms为以最快速度运动",
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="left_revo2_hand_controller",
            description="Robot controller to start.",
        ),
        DeclareLaunchArgument(
            "hand_type",
            default_value="right",
            description="Hand type: left or right",
            choices=["left", "right"]
        ),
    ]

    # Initialize Arguments
    prefix = LaunchConfiguration("prefix")
    ctrl_param_duration_ms = LaunchConfiguration("ctrl_param_duration_ms")
    robot_controller = LaunchConfiguration("robot_controller")
    hand_type = LaunchConfiguration("hand_type")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
            [
                FindPackageShare("brainco_hand_ethercat_driver"),
                "config",
                ["revo2_", hand_type, ".urdf.xacro"],
            ]
            ),
            " ",
            "prefix:=",
            prefix,
            " ",
            "ctrl_param_duration_ms:=",
            ctrl_param_duration_ms,
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_ethercat_driver"),
            "config",
            ["revo2_", hand_type, "_controllers.yaml"],
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[[hand_type, "_revo2_hand_controller"], "-c", "/controller_manager"],
        output="both",
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
