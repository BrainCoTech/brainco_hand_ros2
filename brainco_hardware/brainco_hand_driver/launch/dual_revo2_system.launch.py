#!/usr/bin/env python3
"""
双手系统启动文件 - Modbus 模式
同时控制左右两只 Revo2 灵巧手
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
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
            "left_protocol_config_file",
            default_value="",
            description="左手协议配置文件（YAML），留空则使用包内默认配置 protocol_modbus_left.yaml",
        ),
        DeclareLaunchArgument(
            "right_protocol_config_file",
            default_value="",
            description="右手协议配置文件（YAML），留空则使用包内默认配置 protocol_modbus.yaml",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="",
            description="初始位置配置文件（YAML），留空则使用包内默认配置 dual_revo2_initial_positions.yaml",
        ),
    ]

    left_protocol_config_override = LaunchConfiguration("left_protocol_config_file")
    right_protocol_config_override = LaunchConfiguration("right_protocol_config_file")
    initial_positions_override = LaunchConfiguration("initial_positions_file")

    # 默认配置文件路径
    left_protocol_config_default = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
            "config",
            "protocol_modbus_left.yaml",
        ]
    )

    right_protocol_config_default = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
            "config",
            "protocol_modbus.yaml",
        ]
    )

    initial_positions_default = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
            "config",
            "dual_revo2_initial_positions.yaml",
        ]
    )

    # 处理相对路径：如果用户提供的是相对路径（不以 / 开头），转换为 $(find package) 格式
    # xacro 支持 $(find package) 格式，这样可以正确解析相对路径
    # 使用 PythonExpression 检查路径是否以 / 开头，如果不是，添加 $(find brainco_hand_driver)/config/ 前缀
    
    # 对于相对路径，我们需要添加包路径前缀
    # 如果路径不以 / 开头，假设它是相对于包的 config 目录的
    left_protocol_config_relative = PathJoinSubstitution([
        FindPackageShare("brainco_hand_driver"),
        "config",
        left_protocol_config_override,
    ])
    
    right_protocol_config_relative = PathJoinSubstitution([
        FindPackageShare("brainco_hand_driver"),
        "config",
        right_protocol_config_override,
    ])
    
    initial_positions_relative = PathJoinSubstitution([
        FindPackageShare("brainco_hand_driver"),
        "config",
        initial_positions_override,
    ])

    # 使用覆盖值或默认值
    # 如果用户提供了路径，使用它（如果是相对路径，会自动添加包路径）
    # 否则使用默认路径
    left_protocol_config = IfElseSubstitution(
        PythonExpression(["'", left_protocol_config_override, "' != ''"]),
        # 如果路径以 / 开头，使用绝对路径；否则使用相对路径（添加包路径）
        IfElseSubstitution(
            PythonExpression(["len('", left_protocol_config_override, "') > 0 and '", left_protocol_config_override, "'[0] == '/'"]),
            left_protocol_config_override,  # 绝对路径
            left_protocol_config_relative,  # 相对路径，添加包路径
        ),
        left_protocol_config_default,
    )

    right_protocol_config = IfElseSubstitution(
        PythonExpression(["'", right_protocol_config_override, "' != ''"]),
        IfElseSubstitution(
            PythonExpression(["len('", right_protocol_config_override, "') > 0 and '", right_protocol_config_override, "'[0] == '/'"]),
            right_protocol_config_override,
            right_protocol_config_relative,
        ),
        right_protocol_config_default,
    )

    initial_positions = IfElseSubstitution(
        PythonExpression(["'", initial_positions_override, "' != ''"]),
        IfElseSubstitution(
            PythonExpression(["len('", initial_positions_override, "') > 0 and '", initial_positions_override, "'[0] == '/'"]),
            initial_positions_override,
            initial_positions_relative,
        ),
        initial_positions_default,
    )

    # 构建 xacro 命令
    # 注意：xacro 参数需要使用 $(find package) 格式，所以需要将路径转换为 find 格式
    xacro_command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution(
            [
                FindPackageShare("brainco_hand_driver"),
                "config",
                "dual_revo2.urdf.xacro",
            ]
        ),
        " ",
        "left_protocol_config_file:=",
        left_protocol_config,
        " ",
        "right_protocol_config_file:=",
        right_protocol_config,
        " ",
        "initial_positions_file:=",
        initial_positions,
    ]

    robot_description_content = Command(xacro_command)
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 控制器配置文件
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("brainco_hand_driver"),
            "config",
            "dual_revo2_controllers.yaml",
        ]
    )

    # ros2_control 节点
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    # Left Hand Controller
    left_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_revo2_hand_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # Right Hand Controller
    right_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_revo2_hand_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # 使用事件处理器确保控制器按顺序启动
    ld = LaunchDescription(declared_arguments)
    ld.add_action(control_node)
    ld.add_action(robot_state_pub_node)
    ld.add_action(joint_state_broadcaster_spawner)

    # 等待 joint_state_broadcaster 启动后再启动左右手控制器
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_hand_controller_spawner, right_hand_controller_spawner],
            )
        )
    )

    return ld

