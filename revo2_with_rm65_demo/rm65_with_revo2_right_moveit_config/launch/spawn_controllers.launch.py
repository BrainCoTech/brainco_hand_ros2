"""
控制器生成器 launch 文件
启动所有配置的 ros2_control 控制器

启动的控制器包括：
- joint_state_broadcaster: 广播关节状态到 /joint_states 话题
- 所有在轨迹执行管理器中配置的控制器：
  - rm65_controller: RM65 机械臂的关节轨迹控制器
  - revo2_right_index_controller: Revo2 右手食指控制器
  - revo2_right_thumb_controller: Revo2 右手拇指控制器

这些控制器由 controller_manager 管理，通过 /controller_manager 话题通信。
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 获取控制器名称列表
    # 从 moveit_simple_controller_manager 配置中提取控制器名称
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    # 为每个控制器创建 spawner 节点
    # 同时包含 joint_state_broadcaster，它始终需要启动
    for controller in controller_names + ["joint_state_broadcaster"]:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",  # 输出到屏幕，便于查看控制器启动状态
            )
        )

    return ld
