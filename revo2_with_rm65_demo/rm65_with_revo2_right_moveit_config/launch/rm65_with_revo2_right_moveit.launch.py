"""
完整的 MoveIt demo launch 文件 - 展开版
启动 RM65 机械臂与 Revo2 右手集成系统的完整演示环境

所有被包含的 launch 文件都已展开到此单一文件中，便于维护和调试。

包括以下组件：
- 静态虚拟关节 TF 发布器（如果配置了虚拟关节）
- 机器人状态发布器 (robot_state_publisher)
- MoveIt MoveGroup 节点（含所有参数配置）
- RViz 可视化界面（条件启动）
- MoveIt 仓库数据库（条件启动）
- ros2_control 控制器管理器
- 关节状态广播器和轨迹控制器生成器

Launch 参数：
- db: 是否启动 MoveIt 仓库数据库（默认: false）
- use_rviz: 是否启动 RViz 可视化界面（默认: true）
- publish_frequency: TF 变换发布的频率（Hz，默认: 15.0）
- allow_trajectory_execution: 是否允许轨迹执行（默认: true）
- publish_monitored_planning_scene: 是否发布监控的规划场景（默认: true）
- capabilities: 要加载的 MoveGroup 能力（用空格分隔，默认来自配置）
- disable_capabilities: 要禁用的 MoveGroup 能力（用空格分隔，默认来自配置）
- monitor_dynamics: 是否监控动力学信息（默认: false）
- rviz_config: RViz 配置文件路径（默认: 包中的 config/moveit.rviz）
- moveit_warehouse_database_path: 数据库文件存储路径（默认: 包中的 default_warehouse_mongo_db）
- reset: 是否重置数据库（删除现有数据，默认: false）
- moveit_warehouse_port: MongoDB 服务端口（默认: 33829）
- moveit_warehouse_host: MongoDB 服务主机（默认: localhost）
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
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # ===== LAUNCH 参数声明 =====

    # 主控制参数
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # Robot State Publisher 参数
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # MoveGroup 参数
    ld.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    ld.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
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

    # RViz 参数
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # Warehouse DB 参数
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("reset", default_value=False))
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_port", default_value="33829")
    )
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost")
    )

    # ===== 静态虚拟关节 TF 发布器 =====
    # 虚拟关节用于将机器人基座固定在世界坐标系中
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
    # 根据关节状态发布机器人各连杆的 TF 变换
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
    # MoveIt 的核心节点，负责处理运动规划请求
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

    # 创建 MoveGroup 节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        additional_env={"DISPLAY": os.environ.get("DISPLAY", "")},
    )
    ld.add_action(move_group_node)

    # ===== RViz 可视化界面 =====
    # 条件启动：只有当 use_rviz 为 true 时才启动
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    # 创建 RViz 节点
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

    # ===== MoveIt 仓库数据库 =====
    # 条件启动：只有当 db 为 true 时才启动
    db_parameters = [
        {
            "overwrite": False,
            "database_path": LaunchConfiguration("moveit_warehouse_database_path"),
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            "warehouse_exec": "mongod",
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        },
    ]

    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=db_parameters,
        condition=IfCondition(LaunchConfiguration("db")),
    )
    ld.add_action(db_node)

    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset")),
    )
    ld.add_action(reset_node)

    # ===== ros2_control 控制器管理器 =====
    # 负责加载和管理各种控制器
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )

    # ===== 控制器生成器 =====
    # 启动所有配置的控制器
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
