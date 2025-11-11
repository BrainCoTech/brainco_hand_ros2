"""
静态虚拟关节 TF 发布器 launch 文件
发布虚拟关节的静态 TF 变换

虚拟关节用于将机器人基座固定在世界坐标系中。在 SRDF 中定义的虚拟关节
会被转换为静态 TF 变换发布器，持续广播从虚拟关节父坐标系到子坐标系的变换。

在本配置中，虚拟关节定义：
- 名称: virtual_joint
- 类型: fixed (固定关节)
- 父坐标系: world
- 子连杆: base_link

这会发布从 "world" 到 "base_link" 的恒定变换，将机器人基座固定在世界原点。
"""

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from srdfdom.srdf import SRDF


def generate_launch_description():
    # 创建 MoveIt 配置构建器
    moveit_config = MoveItConfigsBuilder("rm_65_revo2_right", package_name="rm65_with_revo2_right_moveit_config").to_moveit_configs()

    # 创建 LaunchDescription 对象
    ld = LaunchDescription()

    # 用于为每个 TF 发布器生成唯一名称的计数器
    name_counter = 0

    # 遍历所有机器人描述语义配置（SRDF 文件）
    for key, xml_contents in moveit_config.robot_description_semantic.items():
        # 从 XML 字符串解析 SRDF
        srdf = SRDF.from_xml_string(xml_contents)

        # 为每个虚拟关节创建静态 TF 发布器
        for vj in srdf.virtual_joints:
            ld.add_action(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    # 为每个发布器生成唯一名称
                    name=f"static_transform_publisher{name_counter}",
                    output="log",  # 输出到日志，避免干扰控制台
                    arguments=[
                        # TF 变换参数：从父坐标系到子坐标系
                        "--frame-id",      # 父坐标系 ID
                        vj.parent_frame,   # 虚拟关节的父坐标系
                        "--child-frame-id", # 子坐标系 ID
                        vj.child_link,     # 虚拟关节的子连杆
                        # 对于固定关节，不需要指定变换（默认为单位变换）
                        # 如果需要非单位变换，可以在这里添加 x y z roll pitch yaw 参数
                    ],
                )
            )
            name_counter += 1

    return ld
