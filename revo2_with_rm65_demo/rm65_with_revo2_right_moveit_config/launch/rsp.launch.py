"""
机器人状态发布器 (Robot State Publisher) launch 文件
启动 robot_state_publisher 节点，根据关节状态发布机器人各连杆的 TF 变换

robot_state_publisher 的功能：
- 订阅 /joint_states 话题获取关节状态
- 根据 URDF 描述计算各连杆之间的 TF 变换
- 发布所有连杆的 TF 变换到 /tf 话题
- 同时发布机器人描述参数到参数服务器

Launch 参数：
- publish_frequency: TF 变换发布的频率（Hz，默认: 15.0）
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 声明发布频率参数
    # robot_state_publisher 会以此频率发布 TF 变换
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))

    # 创建 robot_state_publisher 节点
    # 该节点根据发布的关节状态，发布机器人各连杆的 TF 变换
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # 设置为 respawn=True，确保节点崩溃时自动重启
        respawn=True,
        # 输出到屏幕，便于调试
        output="screen",
        parameters=[
            # 机器人描述参数（URDF），包含机器人结构信息
            moveit_config.robot_description,
            {
                # TF 变换发布频率
                "publish_frequency": LaunchConfiguration("publish_frequency"),
            },
        ],
    )
    ld.add_action(rsp_node)

    return ld
