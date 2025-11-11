"""
MoveIt MoveGroup 节点 launch 文件
启动 MoveIt 的核心 MoveGroup 节点，负责处理运动规划请求

MoveGroup 节点是 MoveIt 的核心组件，提供以下服务：
- 运动规划服务
- 执行轨迹服务
- 规划场景监控
- 各种 MoveIt 功能插件

Launch 参数：
- debug: 是否启用调试模式（默认: false）
- allow_trajectory_execution: 是否允许轨迹执行（默认: true）
- publish_monitored_planning_scene: 是否发布监控的规划场景（默认: true）
- capabilities: 要加载的 MoveGroup 能力（用空格分隔，默认来自配置）
- disable_capabilities: 要禁用的 MoveGroup 能力（用空格分隔，默认来自配置）
- monitor_dynamics: 是否监控动力学信息（默认: false）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明 launch 参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )

    # 加载非默认的 MoveGroup 能力插件（用空格分隔）
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    # 禁用指定的默认 MoveGroup 能力（用空格分隔）
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )

    # 是否从 /joint_states 复制动力学信息到内部机器人监控
    # 默认设为 false，因为 move_group 中很少使用此信息
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

    # 获取规划场景发布配置
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    # 配置 MoveGroup 节点的参数
    move_group_configuration = {
        # 发布机器人语义描述（SRDF）
        "publish_robot_description_semantic": True,
        # 是否允许轨迹执行
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # 注意：包装以下值是必要的，以便参数值可以是空字符串
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # 发布物理机器人的规划场景，以便 rviz 插件可以知道实际机器人状态
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,  # 此处硬编码为 False，可能需要检查源码确认
    }

    # 合并所有 MoveGroup 参数
    move_group_params = [
        moveit_config.to_dict(),  # 所有 MoveIt 配置参数
        move_group_configuration,  # MoveGroup 特定配置
    ]

    # 添加 MoveGroup 节点，支持调试模式
    # 如果启用调试，会使用 gdb 启动 move_group
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # 设置 DISPLAY 环境变量，以防 OpenGL 代码在内部使用
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    return ld
