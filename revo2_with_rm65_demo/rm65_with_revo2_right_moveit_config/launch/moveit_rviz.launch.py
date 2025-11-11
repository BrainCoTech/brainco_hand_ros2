"""
MoveIt RViz 可视化界面 launch 文件
启动 RViz 以及 MoveIt RViz 插件，用于可视化机器人状态和运动规划

RViz 功能：
- 显示机器人 3D 模型
- 显示规划的运动轨迹
- 显示规划场景（障碍物、碰撞检测等）
- 提供交互式运动规划界面
- 显示传感器数据和点云

Launch 参数：
- debug: 是否启用调试模式（默认: false）
- rviz_config: RViz 配置文件路径（默认: 包中的 config/moveit.rviz）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明 launch 参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    # 声明 RViz 配置文件路径参数
    # 默认为包中预配置的 moveit.rviz 文件
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )

    # 配置 RViz 节点的参数
    # 这些参数会被 MoveIt RViz 插件使用
    rviz_parameters = [
        # 规划管道配置（规划器参数）
        moveit_config.planning_pipelines,
        # 机器人描述的运动学参数（IK 求解器参数）
        moveit_config.robot_description_kinematics,
        # 关节限制参数
        moveit_config.joint_limits,
    ]

    # 添加 RViz 节点，支持调试模式
    # 如果启用调试，会使用 gdb 启动 rviz2
    add_debuggable_node(
        ld,
        package="rviz2",
        executable="rviz2",
        output="log",  # 输出到日志文件，避免干扰控制台
        respawn=False,  # RViz 不需要重启
        arguments=["-d", LaunchConfiguration("rviz_config")],  # 指定配置文件
        parameters=rviz_parameters,  # 传递 MoveIt 配置参数
    )

    return ld
