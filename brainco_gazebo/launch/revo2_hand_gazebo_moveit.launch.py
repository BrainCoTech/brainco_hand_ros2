#!/usr/bin/env python3
"""
Gazebo-MoveIt 整合 Launch 文件 - Revo2 单手
启动 Revo2 灵巧手的 Gazebo 仿真 + MoveIt 运动规划系统

此 launch 文件整合了：
1. Gazebo 仿真环境（使用 GazeboSimSystem）
2. MoveIt 运动规划功能
3. RViz 可视化界面

主要特点：
- 使用 Gazebo 的 robot description（包含 mimic 关节和 gz_ros2_control 插件）
- 通过 gz_ros2_control 插件管理控制器
- 支持 MoveIt 的所有规划和执行功能
- 支持左手和右手配置

Launch 参数：
- hand_type: 手的类型（left/right，默认: right）
- world: Gazebo 世界文件名（默认: empty_world）
- use_rviz: 是否启动 RViz（默认: true）
- publish_monitored_planning_scene: 是否发布监控的规划场景（默认: true）
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import SetEnvironmentVariable, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    use_rviz = LaunchConfiguration('use_rviz')
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    capabilities = LaunchConfiguration('capabilities')
    disable_capabilities = LaunchConfiguration('disable_capabilities')
    
    # 包路径
    moveit_config_package_path = get_package_share_directory('brainco_moveit_config')
    
    # 根据 hand_type 加载对应的 MoveIt 配置
    if hand_type_value == 'left':
        robot_name = "revo2_left"
        srdf_file = "config/revo2_left.srdf"
        trajectory_file = "config/revo2_left_moveit_controllers.yaml"
        joint_limits_file = "config/revo2_left_joint_limits.yaml"
        kinematics_file = "config/revo2_left_kinematics.yaml"
    else:  # right
        robot_name = "revo2_right"
        srdf_file = "config/revo2_right.srdf"
        trajectory_file = "config/revo2_right_moveit_controllers.yaml"
        joint_limits_file = "config/revo2_right_joint_limits.yaml"
        kinematics_file = "config/revo2_right_kinematics.yaml"
    
    # 加载 MoveIt 配置
    # MoveIt 会使用包配置目录中的默认 planning pipeline 配置
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name="brainco_moveit_config")
        .robot_description_semantic(file_path=srdf_file)
        .trajectory_execution(file_path=trajectory_file)
        .joint_limits(file_path=joint_limits_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .to_moveit_configs()
    )
    
    # Robot Description（从 Gazebo）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('brainco_gazebo'),
            'config',
            'revo2_gazebo_description.urdf.xacro',
        ]),
        ' ',
        f'hand_type:={hand_type_value}',
        ' ',
        'model:=revo2_hand',
        ' ',
        'use_sim:=true',
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # MoveGroup 配置
    capabilities_default = moveit_config.move_group_capabilities.get('capabilities', '')
    disable_capabilities_default = moveit_config.move_group_capabilities.get('disable_capabilities', '')
    
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": ParameterValue(capabilities, value_type=str),
        "disable_capabilities": ParameterValue(disable_capabilities, value_type=str),
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
        "use_sim_time": True,
    }
    
    # 合并 MoveIt 参数
    move_group_params = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.moveit_cpp,
        moveit_config.joint_limits,
        move_group_configuration,
    ]
    
    # MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    
    # RViz 节点
    default_rviz_config = str(Path(moveit_config_package_path) / 'config' / 'moveit.rviz')
    
    rviz_parameters = [
        robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {'use_sim_time': True},
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
            'world', 
            default_value='empty_world',
            description='Gazebo 世界文件名'
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
        DeclareLaunchArgument(
            'capabilities',
            default_value='',
            description='MoveGroup 能力列表（空格分隔）'
        ),
        DeclareLaunchArgument(
            'disable_capabilities',
            default_value='',
            description='要禁用的 MoveGroup 能力（空格分隔）'
        ),
    ]

    hand_type = LaunchConfiguration('hand_type')
    world = LaunchConfiguration('world')
    
    # ===== 包路径 =====
    gazebo_package_path = get_package_share_directory('brainco_gazebo')
    revo2_description_path = get_package_share_directory('revo2_description')
    moveit_config_package_path = get_package_share_directory('brainco_moveit_config')
    
    # ===== Gazebo 环境设置 =====
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(gazebo_package_path, 'worlds'), ':' +
            str(Path(revo2_description_path).parent.resolve())
        ]
    )

    # ===== Gazebo 仿真 =====
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

    # ===== Robot Description =====
    # 使用 Gazebo 的 URDF 配置（包含 mimic 关节和 GazeboSimSystem）
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([
            FindPackageShare('brainco_gazebo'),
            'config',
            'revo2_gazebo_description.urdf.xacro',
        ]),
        ' ',
        'hand_type:=', hand_type,
        ' ',
        'model:=revo2_hand',
        ' ',
        'use_sim:=true',
    ])

    robot_description = {'robot_description': robot_description_content}

    # ===== Robot State Publisher =====
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # ===== 在 Gazebo 中生成机器人实体 =====
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
            '-name', 'revo2_hand',
            '-allow_renaming', 'true',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ===== 控制器配置 =====
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('brainco_gazebo'),
        'config',
        ['revo2_', hand_type, '_controllers.yaml'],
    ])

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Revo2 Hand Controller
    revo2_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[[hand_type, '_revo2_hand_controller'], '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
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
    
    # 解析虚拟关节并创建静态 TF 发布器
    static_tf_nodes = []
    name_counter = 0
    for key, xml_contents in moveit_config_temp.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
            # 跳过无效的虚拟关节（parent_frame 和 child_link 相同会导致错误）
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
                    parameters=[{'use_sim_time': True}],
                )
            )
            name_counter += 1

    # ===== Clock Bridge =====
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ===== Launch Description =====
    ld = LaunchDescription()
    
    # 添加参数声明
    for arg in declared_arguments:
        ld.add_action(arg)
    
    # 添加环境变量和基础节点
    ld.add_action(gazebo_resource_path)
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(robot_state_pub_node)
    ld.add_action(gz_spawn_entity)
    
    # 添加静态 TF 发布器
    for node in static_tf_nodes:
        ld.add_action(node)
    
    # 按顺序启动控制器
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[revo2_hand_controller_spawner],
            )
        )
    )
    
    # 在控制器启动后，动态生成并启动 MoveIt 节点
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=revo2_hand_controller_spawner,
                on_exit=[OpaqueFunction(function=generate_moveit_nodes)],
            )
        )
    )
    
    return ld
