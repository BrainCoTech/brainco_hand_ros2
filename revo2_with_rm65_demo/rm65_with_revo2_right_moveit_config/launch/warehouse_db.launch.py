"""
MoveIt 仓库数据库 launch 文件
启动 MoveIt 仓库数据库服务，用于存储和查询运动规划结果

仓库数据库功能：
- 存储成功的运动规划查询结果
- 允许快速查询已知的规划结果
- 支持规划结果的重用和缓存
- 使用 MongoDB 作为后端存储

Launch 参数：
- moveit_warehouse_database_path: 数据库文件存储路径（默认: 包中的 default_warehouse_mongo_db）
- reset: 是否重置数据库（删除现有数据，默认: false）
- moveit_warehouse_port: MongoDB 服务端口（默认: 33829，避免与系统 MongoDB 冲突）
- moveit_warehouse_host: MongoDB 服务主机（默认: localhost）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明数据库路径参数
    ld.add_action(
        DeclareLaunchArgument(
            "moveit_warehouse_database_path",
            default_value=str(
                moveit_config.package_path / "default_warehouse_mongo_db"
            ),
        )
    )

    # 声明重置参数
    from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg
    ld.add_action(DeclareBooleanLaunchArg("reset", default_value=False))

    # 声明数据库端口参数
    # 使用非标准端口避免与系统 MongoDB 冲突
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_port", default_value="33829")
    )

    # 声明数据库主机参数
    ld.add_action(
        DeclareLaunchArgument("moveit_warehouse_host", default_value="localhost")
    )

    # 配置数据库参数
    db_parameters = [
        {
            # 不覆盖现有数据
            "overwrite": False,
            # 数据库文件路径
            "database_path": LaunchConfiguration("moveit_warehouse_database_path"),
            # 数据库端口
            "warehouse_port": LaunchConfiguration("moveit_warehouse_port"),
            # 数据库主机
            "warehouse_host": LaunchConfiguration("moveit_warehouse_host"),
            # MongoDB 执行程序
            "warehouse_exec": "mongod",
            # 数据库插件
            "warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection",
        },
    ]

    # 启动数据库服务器节点
    db_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        # TODO(dlu): 确认是否需要在特定目录中运行
        # （ROS 1 版本设置了工作目录为 ROS_HOME）
        parameters=db_parameters,
    )
    ld.add_action(db_node)

    # 如果启用重置，运行数据库初始化节点
    reset_node = Node(
        package="moveit_ros_warehouse",
        executable="moveit_init_demo_warehouse",
        output="screen",  # 输出到屏幕，查看重置进度
        condition=IfCondition(LaunchConfiguration("reset")),
    )
    ld.add_action(reset_node)

    return ld
