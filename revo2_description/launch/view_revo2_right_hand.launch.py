from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="revo2_description",
            description="Description package containing URDF and meshes.",
        )
    )

    def _spawn_nodes(context: LaunchContext):
        pkg = context.launch_configurations.get("description_package", "revo2_description")
        share_dir = get_package_share_directory(pkg)
        xacro_path = os.path.join(share_dir, "urdf", "revo2_right_hand.urdf.xacro")
        urdf_text = xacro.process_file(xacro_path).toprettyxml(indent="  ")

        robot_description = {"robot_description": urdf_text}
        rviz_config_file = os.path.join(share_dir, "rviz", "revo2_right_hand.rviz")

        joint_state_publisher_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        )

        robot_state_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )

        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_right_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "world", "right_hand_base_link"],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        )

        return [
            joint_state_publisher_node,
            robot_state_publisher_node,
            static_tf,
            rviz_node,
        ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_spawn_nodes)])


