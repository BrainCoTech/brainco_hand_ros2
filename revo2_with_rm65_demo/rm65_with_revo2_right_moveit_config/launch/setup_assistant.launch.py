"""
MoveIt Setup Assistant launch 文件
启动 MoveIt Setup Assistant 图形界面工具，用于配置 MoveIt

MoveIt Setup Assistant 功能：
- 加载和编辑机器人 URDF/SRDF 文件
- 配置自碰撞检测矩阵
- 定义运动规划组
- 配置末端执行器
- 配置虚拟关节
- 生成 MoveIt 配置文件

Launch 参数：
- debug: 是否启用调试模式（默认: false）
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import DeclareBooleanLaunchArg, add_debuggable_node
from launch import LaunchDescription


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明 launch 参数
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    # 添加 MoveIt Setup Assistant 节点，支持调试模式
    # 如果启用调试，会使用 gdb 启动 moveit_setup_assistant
    add_debuggable_node(
        ld,
        package="moveit_setup_assistant",
        executable="moveit_setup_assistant",
        # 传递配置包路径参数，让工具知道加载哪个配置包
        arguments=[["--config_pkg=", str(moveit_config.package_path)]],
    )

    return ld
