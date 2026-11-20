#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="revo2_description",
            description="Use the public Revo2 model or a package with urdf/revo2.single.system.xacro.",
        ),
        DeclareLaunchArgument(
            "left_protocol",
            default_value="modbus",
            description="左手协议：modbus 或 canfd",
            choices=["modbus", "canfd"],
        ),
        DeclareLaunchArgument(
            "right_protocol",
            default_value="modbus",
            description="右手协议：modbus 或 canfd",
            choices=["modbus", "canfd"],
        ),
        DeclareLaunchArgument(
            "left_protocol_config_file",
            default_value="",
            description="左手协议配置文件（YAML），留空则使用默认配置。",
        ),
        DeclareLaunchArgument(
            "right_protocol_config_file",
            default_value="",
            description="右手协议配置文件（YAML），留空则使用默认配置。",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="",
            description="双手共用初始位置配置文件（YAML），留空则各自使用单手默认配置。",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="",
            description="控制器模板文件（YAML），留空则使用 revo2_controllers.yaml。",
        ),
        DeclareLaunchArgument(
            "use_namespace",
            default_value="true",
            description="是否为左右手分别使用命名空间。",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "sdk_file_logging",
            default_value="",
            description="Override SDK file logging for both hands; empty uses protocol YAML.",
            choices=["", "true", "false"],
        ),
        DeclareLaunchArgument(
            "if_sim",
            default_value="false",
            description="使用 mock_components/GenericSystem 模拟硬件",
        ),
        DeclareLaunchArgument(
            "launch_rsp",
            default_value="true",
            description="是否启动左右手各自的 robot_state_publisher",
        ),
    ]

    single_launch = PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("revo2_driver"),
            "launch",
            "revo2_system.launch.py",
        ])
    )

    left_hand_launch = IncludeLaunchDescription(
        single_launch,
        launch_arguments={
            "description_package": LaunchConfiguration("description_package"),
            "hand_side": "left",
            "sdk_file_logging": LaunchConfiguration("sdk_file_logging"),
            "protocol": LaunchConfiguration("left_protocol"),
            "protocol_config_file": LaunchConfiguration("left_protocol_config_file"),
            "initial_positions_file": LaunchConfiguration("initial_positions_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "if_sim": LaunchConfiguration("if_sim"),
            "launch_rsp": LaunchConfiguration("launch_rsp"),
        }.items(),
    )

    right_hand_launch = IncludeLaunchDescription(
        single_launch,
        launch_arguments={
            "description_package": LaunchConfiguration("description_package"),
            "hand_side": "right",
            "sdk_file_logging": LaunchConfiguration("sdk_file_logging"),
            "protocol": LaunchConfiguration("right_protocol"),
            "protocol_config_file": LaunchConfiguration("right_protocol_config_file"),
            "initial_positions_file": LaunchConfiguration("initial_positions_file"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "use_namespace": LaunchConfiguration("use_namespace"),
            "if_sim": LaunchConfiguration("if_sim"),
            "launch_rsp": LaunchConfiguration("launch_rsp"),
        }.items(),
    )

    return LaunchDescription(declared_arguments + [left_hand_launch, right_hand_launch])
