"""
完整的 MoveIt demo launch 文件 - Dual Revo2 Hands
启动双 Revo2 灵巧手的完整 MoveIt 演示环境（使用 FakeSystem）

包括以下组件：
- 静态虚拟关节 TF 发布器（如果配置了虚拟关节）
- 机器人状态发布器 (robot_state_publisher)
- MoveIt MoveGroup 节点（含所有参数配置）
- RViz 可视化界面（条件启动）
- MoveIt 仓库数据库（条件启动）
- ros2_control 控制器管理器
- 关节状态广播器和轨迹控制器生成器

Launch 参数：
- use_rviz: 是否启动 RViz 可视化界面（默认: true）
- publish_frequency: TF 变换发布的频率（Hz，默认: 15.0）
- allow_trajectory_execution: 是否允许轨迹执行（默认: true）
- publish_monitored_planning_scene: 是否发布监控的规划场景（默认: true）
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from srdfdom.srdf import SRDF


def generate_launch_description():
    # 创建 MoveIt 配置构建器，指定机器人名称和包名
    moveit_config = (
        MoveItConfigsBuilder("dual_revo2", package_name="brainco_moveit_config")
        .robot_description(file_path="config/dual_revo2.urdf.xacro")
        .robot_description_semantic(file_path="config/dual_revo2.srdf")
        .trajectory_execution(file_path="config/dual_revo2_moveit_controllers.yaml")
        .joint_limits(file_path="config/dual_revo2_joint_limits.yaml")
        .robot_description_kinematics(file_path="config/dual_revo2_kinematics.yaml")
        .to_moveit_configs()
    )

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # ===== LAUNCH 参数声明 =====
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    ld.add_action(DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True))
    ld.add_action(
        DeclareLaunchArgument(
            "capabilities",
            default_value=moveit_config.move_group_capabilities["capabilities"],
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "disable_capabilities",
            default_value=moveit_config.move_group_capabilities["disable_capabilities"],
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # ===== 静态虚拟关节 TF 发布器 =====
    name_counter = 0
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=[
                        "--frame-id",
                        vj.parent_frame,
                        "--child-frame-id",
                        vj.child_link,
                    ],
                )
            )
            name_counter += 1

    # ===== 机器人状态发布器 =====
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    # ===== MoveIt MoveGroup 节点 =====
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    ld.add_action(move_group_node)

    # ===== RViz 可视化界面 =====
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )
    ld.add_action(rviz_node)

    # ===== ros2_control 控制器管理器 =====
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/dual_revo2_controllers.yaml"),
            ],
        )
    )

    # ===== 控制器生成器 =====
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )

    return ld

