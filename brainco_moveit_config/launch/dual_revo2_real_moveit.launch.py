#!/usr/bin/env python3
"""Revo2 双手灵巧手真机 + MoveIt 启动文件（Modbus 驱动）。"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
from srdfdom.srdf import SRDF


def build_xacro_command() -> list:
    """构建 xacro 命令，用于生成双手 URDF。"""
    command = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([
            FindPackageShare("brainco_hand_driver"),
            "config/dual_revo2.urdf.xacro",
        ]),
        " ",
        "left_protocol_config_file:=", LaunchConfiguration("left_protocol_config_file"),
        " ",
        "right_protocol_config_file:=", LaunchConfiguration("right_protocol_config_file"),
    ]
    return command


def generate_moveit_nodes(context, *args, **kwargs):  # noqa: D401
    """生成 MoveIt 相关节点。"""
    use_rviz = LaunchConfiguration("use_rviz")
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")

    robot_name = "dual_revo2"
    srdf_file = "config/dual_revo2.srdf"
    trajectory_file = "config/dual_revo2_moveit_controllers.yaml"
    joint_limits_file = "config/dual_revo2_joint_limits.yaml"
    kinematics_file = "config/dual_revo2_kinematics.yaml"

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="brainco_moveit_config")
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path=trajectory_file)
        .joint_limits(file_path=joint_limits_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .to_moveit_configs()
    )

    robot_description_content = Command(build_xacro_command())
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    pkg_path = get_package_share_directory("brainco_moveit_config")
    default_rviz_config = str(Path(pkg_path) / "config" / "moveit.rviz")

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": False,
    }

    move_group_params = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.joint_limits,
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    rviz_parameters = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"use_sim_time": False},
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", default_rviz_config],
        parameters=rviz_parameters,
        condition=IfCondition(use_rviz),
    )

    return [move_group_node, rviz_node]


def generate_launch_description():  # noqa: D401
    """生成 Revo2 双手 Modbus 真机 + MoveIt 的启动描述。"""

    declared_arguments = [
        DeclareBooleanLaunchArg(
            "use_rviz",
            default_value=True,
            description="是否启动 RViz 可视化",
        ),
        DeclareBooleanLaunchArg(
            "publish_monitored_planning_scene",
            default_value=True,
            description="是否发布监控的规划场景",
        ),
        DeclareLaunchArgument(
            "left_protocol_config_file",
            default_value="",
            description="左手协议配置文件（YAML），留空则使用包内默认 Modbus 配置",
        ),
        DeclareLaunchArgument(
            "right_protocol_config_file",
            default_value="",
            description="右手协议配置文件（YAML），留空则使用包内默认 Modbus 配置",
        ),
    ]

    robot_description_content = Command(build_xacro_command())
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare("brainco_moveit_config"),
        "config/dual_revo2_controllers.yaml",
    ])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    left_revo2_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_revo2_hand_controller", "-c", "/controller_manager"],
        output="screen",
    )

    right_revo2_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_revo2_hand_controller", "-c", "/controller_manager"],
        output="screen",
    )

    # 使用临时配置来解析 SRDF 以获取虚拟关节
    moveit_config_temp = (
        MoveItConfigsBuilder("dual_revo2", package_name="brainco_moveit_config")
        .robot_description_semantic(file_path="config/dual_revo2.srdf")
        .trajectory_execution(file_path="config/dual_revo2_moveit_controllers.yaml")
        .joint_limits(file_path="config/dual_revo2_joint_limits.yaml")
        .robot_description_kinematics(file_path="config/dual_revo2_kinematics.yaml")
        .to_moveit_configs()
    )

    static_tf_nodes = []
    name_counter = 0
    for _, xml_contents in moveit_config_temp.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            if vj.parent_frame == vj.child_link:
                continue
            static_tf_nodes.append(
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

    ld = LaunchDescription()
    for arg in declared_arguments:
        ld.add_action(arg)

    ld.add_action(robot_state_pub_node)
    ld.add_action(control_node)

    for node in static_tf_nodes:
        ld.add_action(node)

    ld.add_action(joint_state_broadcaster_spawner)

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[left_revo2_hand_controller_spawner],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=left_revo2_hand_controller_spawner,
                on_exit=[right_revo2_hand_controller_spawner],
            )
        )
    )

    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=right_revo2_hand_controller_spawner,
                on_exit=[OpaqueFunction(function=generate_moveit_nodes)],
            )
        )
    )

    return ld

