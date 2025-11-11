#!/usr/bin/env python3
"""
Gazebo-MoveIt 集成 Launch 文件
启动 RM65 机械臂与 Revo2 右手的 Gazebo 仿真 + MoveIt 运动规划系统

此 launch 文件整合了：
1. Gazebo 仿真环境（使用 GazeboSimSystem）
2. MoveIt 运动规划功能
3. RViz 可视化界面

主要特点：
- 使用 Gazebo 的 robot description（包含 mimic 关节）
- 通过 gz_ros2_control 插件管理控制器
- 支持 MoveIt 的所有规划和执行功能

Launch 参数：
- world: Gazebo 世界文件名（默认: empty_world）
- use_rviz: 是否启动 RViz（默认: true）
- rviz_config: RViz 配置文件（默认使用 MoveIt 配置）
- db: 是否启动 MoveIt 数据库（默认: false）
- publish_monitored_planning_scene: 是否发布监控的规划场景（默认: true）
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import SetEnvironmentVariable
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


def generate_launch_description():
    # ===== 参数声明 =====
    declared_arguments = [
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
        DeclareLaunchArgument(
            'rviz_config',
            default_value='',
            description='RViz 配置文件路径（为空则使用 MoveIt 默认配置）'
        ),
        DeclareBooleanLaunchArg(
            'db',
            default_value=False,
            description='是否启动 MoveIt 数据库'
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
        DeclareLaunchArgument(
            'moveit_warehouse_database_path',
            default_value='',
            description='MoveIt 数据库路径'
        ),
        DeclareBooleanLaunchArg(
            'reset',
            default_value=False,
            description='是否重置数据库'
        ),
        DeclareLaunchArgument(
            'moveit_warehouse_port',
            default_value='33829',
            description='MongoDB 端口'
        ),
        DeclareLaunchArgument(
            'moveit_warehouse_host',
            default_value='localhost',
            description='MongoDB 主机'
        ),
    ]

    world = LaunchConfiguration('world')
    use_rviz = LaunchConfiguration('use_rviz')
    db = LaunchConfiguration('db')
    
    # ===== 包路径 =====
    demo_package_path = get_package_share_directory('gazebo_rm_65_6f_with_revo2_demo')
    revo2_description_path = get_package_share_directory('revo2_description')
    moveit_config_package_path = get_package_share_directory('rm65_with_revo2_right_moveit_config')
    
    # 加载 MoveIt 配置
    moveit_config = MoveItConfigsBuilder(
        "rm_65_revo2_right", 
        package_name="rm65_with_revo2_right_moveit_config"
    ).to_moveit_configs()

    # ===== Gazebo 环境设置 =====
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(demo_package_path, 'worlds'), ':' +
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
            FindPackageShare('gazebo_rm_65_6f_with_revo2_demo'),
            'config',
            'gazebo_65_6f_revo2_description_gz.urdf.xacro',
        ]),
        ' ',
        'model:=rm_65_revo2',
        ' ',
        'use_sim:=true',
    ])

    robot_description = {'robot_description': robot_description_content}

    # ===== Robot State Publisher =====
    # 注意：使用 sim_time
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
            '-name', 'rm_65_revo2',
            '-allow_renaming', 'true',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ===== 控制器配置 =====
    # 使用 Gazebo 的控制器配置（与 gz_ros2_control 插件中的配置一致）
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('gazebo_rm_65_6f_with_revo2_demo'),
        'config',
        'revo2_controllers.yaml',
    ])

    # 注意：控制器由 Gazebo 的 gz_ros2_control 插件管理，不需要单独启动 ros2_control_node
    # 但我们需要按顺序生成控制器

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # RM65 Arm Controller
    rm_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rm_group_controller', '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Revo2 Hand Controller
    revo2_hand_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['revo2_hand_controller', '--param-file', controllers_yaml],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ===== 静态虚拟关节 TF 发布器 =====
    # 从 MoveIt 配置中读取虚拟关节信息
    static_tf_nodes = []
    name_counter = 0
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(xml_contents)
        for vj in srdf.virtual_joints:
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

    # ===== MoveIt MoveGroup 节点 =====
    should_publish = LaunchConfiguration("publish_monitored_planning_scene")
    
    # 确定 capabilities 参数
    capabilities = LaunchConfiguration('capabilities')
    disable_capabilities = LaunchConfiguration('disable_capabilities')
    
    # 如果未指定，使用 MoveIt 配置中的默认值
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
        "use_sim_time": True,  # 关键：使用仿真时间
    }

    # 合并 robot_description（从 Gazebo）和其他 MoveIt 配置
    move_group_params = [
        robot_description,  # 使用 Gazebo 的 robot description
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.trajectory_execution,
        moveit_config.moveit_cpp,
        moveit_config.joint_limits,
        move_group_configuration,
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )

    # ===== RViz 可视化 =====
    # 确定 RViz 配置文件路径
    rviz_config_arg = LaunchConfiguration('rviz_config')
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

    # ===== MoveIt 数据库（可选） =====
    warehouse_db_path = LaunchConfiguration('moveit_warehouse_database_path')
    default_warehouse_path = str(Path(moveit_config_package_path) / 'default_warehouse_mongo_db')
    
    db_parameters = [
        {
            "overwrite": False,
            "database_path": default_warehouse_path,
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            "warehouse_exec": "mongod",
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
            "use_sim_time": True,
        },
    ]

    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=db_parameters,
        condition=IfCondition(db),
    )

    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration("reset")),
    )

    # ===== Clock Bridge =====
    # 桥接 Gazebo 时钟到 ROS 2
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
    
    # 按顺序启动控制器（使用事件处理器）
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
                on_exit=[rm_group_controller_spawner],
            )
        )
    )
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=rm_group_controller_spawner,
                on_exit=[revo2_hand_controller_spawner],
            )
        )
    )
    
    # 在所有控制器启动后，启动 MoveGroup
    ld.add_action(
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=revo2_hand_controller_spawner,
                on_exit=[move_group_node],
            )
        )
    )
    
    # 添加可视化和数据库节点
    ld.add_action(rviz_node)
    ld.add_action(db_node)
    ld.add_action(reset_node)
    
    return ld

