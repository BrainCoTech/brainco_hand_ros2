#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    IfElseSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "hand_type",
            default_value="right",
            description="选择控制的手型：left 或 right",
            choices=["left", "right"],
        ),
        DeclareLaunchArgument(
            "protocol",
            default_value="modbus",
            description="通信协议：modbus 或 canfd",
            choices=["modbus", "canfd"],
        ),
        DeclareLaunchArgument(
            "protocol_config_file",
            default_value="",
            description="协议配置文件（YAML），留空则使用包内默认配置",
        ),
    ]

    hand_type = LaunchConfiguration("hand_type")
    protocol = LaunchConfiguration("protocol")
    protocol_config_override = LaunchConfiguration("protocol_config_file")

    # 根据 hand_type 动态生成控制器名称
    robot_controller = [hand_type, "_revo2_hand_controller"]

    protocol_config_default = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
            "config",
            PythonExpression([
                "'protocol_canfd_left.yaml' if ('",
                hand_type,
                "'.lower() == 'left' and '",
                protocol,
                "'.lower() == 'canfd') else ('protocol_modbus_left.yaml' if '",
                hand_type,
                "'.lower() == 'left' else ('protocol_canfd.yaml' if '",
                protocol,
                "'.lower() == 'canfd' else 'protocol_modbus.yaml'))",
            ]),
        ]
    )

    protocol_config_path = IfElseSubstitution(
        PythonExpression(["'", protocol_config_override, "' != ''"]),
        protocol_config_override,
        protocol_config_default,
    )

    xacro_command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("brainco_hand_driver"),
                "config",
                ["revo2_", hand_type, ".urdf.xacro"],
            ]
        ),
        " ",
        "protocol_config_file:=",
        protocol_config_path,
    ]

    robot_description_content = Command(xacro_command)
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
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
        arguments=[robot_controller, "-c", "/controller_manager"],
        output="both",
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)


