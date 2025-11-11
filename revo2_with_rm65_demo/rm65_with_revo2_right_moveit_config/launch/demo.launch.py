"""
完整的 MoveIt demo launch 文件
启动 RM65 机械臂与 Revo2 右手集成系统的完整演示环境

包括以下组件：
- 机器人状态发布器 (robot_state_publisher)
- MoveIt MoveGroup 节点
- RViz 可视化界面
- ros2_control 控制器管理器
- 关节状态广播器和轨迹控制器
- 可选的静态虚拟关节 TF 发布器（如果配置了虚拟关节）
- 可选的 MoveIt 仓库数据库（用于存储查询）

Launch 参数：
- db: 是否启动 MoveIt 仓库数据库（默认: false）
- debug: 是否启用调试模式（默认: false）
- use_rviz: 是否启动 RViz 可视化界面（默认: true）
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg


def generate_launch_description():
    # 创建 MoveIt 配置构建器，指定机器人名称和包名
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 获取包路径，用于包含其他 launch 文件
    launch_package_path = moveit_config.package_path

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明 launch 参数
    # db: 是否启动数据库（默认不启动，因为数据库可能很大）
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    # debug: 是否启用调试模式（默认不启用）
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    # use_rviz: 是否启动 RViz 可视化界面（默认启动）
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # 如果存在虚拟关节，包含静态虚拟关节 TF 发布器
    # 虚拟关节用于将机器人基座固定在世界坐标系中
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # 包含机器人状态发布器 launch 文件
    # 机器人状态发布器根据关节状态发布机器人各连杆的 TF 变换
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    # 包含 MoveIt MoveGroup launch 文件
    # MoveGroup 是 MoveIt 的核心节点，处理运动规划请求
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # 如果启用 RViz，包含 MoveIt RViz launch 文件
    # RViz 提供可视化界面，用于查看机器人状态和规划结果
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 如果启用数据库，包含仓库数据库 launch 文件
    # 仓库数据库用于存储和查询运动规划结果
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # 启动 ros2_control 控制器管理器节点
    # 控制器管理器负责加载和管理各种控制器（如关节轨迹控制器、关节状态广播器）
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,  # 机器人描述（URDF）
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),  # 控制器配置文件
            ],
        )
    )

    # 包含控制器生成器 launch 文件
    # 生成器会启动所有配置的控制器，包括：
    # - joint_state_broadcaster: 广播关节状态
    # - rm65_controller: RM65 机械臂的关节轨迹控制器
    # - revo2_right_index_controller: Revo2 右手食指控制器
    # - revo2_right_thumb_controller: Revo2 右手拇指控制器
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld
