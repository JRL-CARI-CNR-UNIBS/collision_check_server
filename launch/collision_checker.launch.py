"""Launch file for the collision_checker_server node.

Starts the `ik_collision_checker` executable, which serves the
`/collision_check` service. It expects a MoveIt stack to already be running
(providing `robot_description` and the `/get_planning_scene` service).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("collision_checker_server")
    default_params_file = os.path.join(
        pkg_share, "config", "collision_checker_params.yaml"
    )

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params_file,
        description="Full path to the YAML file with collision checker "
        "parameters (group_name, padding, scaling).",
    )

    node_name_arg = DeclareLaunchArgument(
        "node_name",
        default_value="ik_collision_check",
        description="Name of the collision checker node. Must match the "
        "node name used as the top-level key in params_file.",
    )

    collision_checker_node = Node(
        package="collision_checker_server",
        executable="ik_collision_checker",
        name=LaunchConfiguration("node_name"),
        output="screen",
        parameters=[LaunchConfiguration("params_file")],
    )

    return LaunchDescription(
        [
            params_file_arg,
            node_name_arg,
            collision_checker_node,
        ]
    )
