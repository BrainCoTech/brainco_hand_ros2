#!/usr/bin/env python3
"""
Real Hardware-MoveIt 整合 Launch 文件 - Revo2 单手 (EtherCAT)
启动 Revo2 灵巧手的真机控制 + MoveIt 运动规划系统

此 launch 文件整合了：
1. ros2_control 硬件接口（使用 EtherCAT driver）
2. MoveIt 运动规划功能
3. RViz 可视化界面

主要特点：
- 使用 ros2_control 的 robot description（包含 mimic 关节和硬件插件）
- 通过 controller_manager 管理控制器
- 支持 MoveIt 的所有规划和执行功能
- 支持左手和右手配置

Launch 参数：
- hand_type: 手的类型（left/right，默认: right）
- ctrl_param_duration_ms: 运动时间参数，单位毫秒（默认: "10"）
- use_rviz: 是否启动 RViz（默认: true）
- publish_monitored_planning_scene: 是否发布监控的规划场景（默认: true）
"""

import os
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


def generate_moveit_nodes(context, *args, **kwargs):
    """根据 hand_type 参数动态生成 MoveIt 节点"""
    hand_type_value = LaunchConfiguration('hand_type').perform(context)
    ctrl_param_duration_ms_value = LaunchConfiguration('ctrl_param_duration_ms').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    
    # 包路径
    pkg_path = get_package_share_directory('brainco_moveit_config')
    
    # 根据 hand_type 加载对应的 MoveIt 配置
    if hand_type_value == 'left':
        robot_name = "revo2_left"
        srdf_file = "config/revo2_left.srdf"
        trajectory_file = "config/revo2_left_moveit_controllers.yaml"
        joint_limits_file = "config/revo2_left_joint_limits.yaml"
        kinematics_file = "config/revo2_left_kinematics.yaml"
        urdf_file = "config/revo2_left.urdf.xacro"
    else:  # right
        robot_name = "revo2_right"
        srdf_file = "config/revo2_right.srdf"
        trajectory_file = "config/revo2_right_moveit_controllers.yaml"
        joint_limits_file = "config/revo2_right_joint_limits.yaml"
        kinematics_file = "config/revo2_right_kinematics.yaml"
        urdf_file = "config/revo2_right.urdf.xacro"

    # 加载 MoveIt 配置（使用 brainco_moveit_config）
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="brainco_moveit_config")
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path=trajectory_file)
        .joint_limits(file_path=joint_limits_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .to_moveit_configs()
    )
    
    # Robot Description（真机配置，使用 EtherCAT ros2_control）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('brainco_hand_ethercat_driver'),
            [urdf_file],
        ]),
        ' ',
        'ctrl_param_duration_ms:=',
        ctrl_param_duration_ms_value,
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # MoveGroup 配置
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": False,  # 真机使用实际时间
    }
    
    # 合并 MoveIt 参数
    move_group_params = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.joint_limits,
        move_group_configuration,
    ]
    
    # MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )
    
    # RViz 节点
    default_rviz_config = str(Path(pkg_path) / 'config' / 'moveit.rviz')
    
    rviz_parameters = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {'use_sim_time': False},
    ]
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", default_rviz_config],
        parameters=rviz_parameters,
        condition=IfCondition(use_rviz),
    )
    
    return [move_group_node, rviz_node]


def generate_launch_description():
    # ===== 参数声明 =====
    declared_arguments = [
        DeclareLaunchArgument(
            'hand_type',
            default_value='right',
            description='手的类型: left 或 right',
            choices=['left', 'right']
        ),
        DeclareLaunchArgument(
            'ctrl_param_duration_ms',
            default_value='10',
            description='运动时间参数，单位毫秒'
        ),
        DeclareBooleanLaunchArg(
            'use_rviz',
            default_value=True,
            description='是否启动 RViz 可视化'
        ),
        DeclareBooleanLaunchArg(
            'publish_monitored_planning_scene',
            default_value=True,
            description='是否发布监控的规划场景'
        ),
    ]

    hand_type = LaunchConfiguration('hand_type')
    ctrl_param_duration_ms = LaunchConfiguration('ctrl_param_duration_ms')

    # ===== Robot Description =====
    # 使用 EtherCAT driver 的 URDF 配置（包含 ros2_control）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('brainco_hand_ethercat_driver'),
            'config',
            ['revo2_', hand_type, '.urdf.xacro'],
        ]),
        ' ',
        'ctrl_param_duration_ms:=',
        ctrl_param_duration_ms,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ===== Robot State Publisher =====
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # ===== 控制器配置 =====
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('brainco_hand_ethercat_driver'),
        'config',
        ['revo2_', hand_type, '_controllers.yaml'],
    ])

    # ros2_control 节点
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers_yaml],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen"
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen'
    )

    # Revo2 Hand Controller
    revo2_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[[hand_type, '_revo2_hand_controller'], '-c', '/controller_manager'],
        output='screen'
    )

    # ===== 静态虚拟关节 TF 发布器 =====
    # 读取虚拟关节信息用于发布静态 TF （左右手虚拟关节相同）
    moveit_config_temp = (
        MoveItConfigsBuilder("revo2_right", package_name="brainco_moveit_config")
        .robot_description_semantic(file_path="config/revo2_right.srdf")
        .trajectory_execution(file_path="config/revo2_right_moveit_controllers.yaml")
        .joint_limits(file_path="config/revo2_right_joint_limits.yaml")
        .robot_description_kinematics(file_path="config/revo2_right_kinematics.yaml")
        .to_moveit_configs()
    )
    
    static_tf_nodes = []
    name_counter = 0
    for key, xml_contents in moveit_config_temp.robot_description_semantic.items():
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

    # ===== Launch Description =====
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
                on_exit=[revo2_hand_controller_spawner],
            )
        )
    )
    
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=revo2_hand_controller_spawner,
                on_exit=[OpaqueFunction(function=generate_moveit_nodes)],
            )
        )
    )
    
    return ld

