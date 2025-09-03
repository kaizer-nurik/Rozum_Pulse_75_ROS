"""
A launch file for running the motion planning python api tutorial
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    
    moveit_config = (
        MoveItConfigsBuilder(
            "rozum75_with_gripper", package_name="new_moveit"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path = "config/moveit_cpp.yaml"
        )
        .to_moveit_configs()
    )

    moveit_py_node = Node(
        name="demo",
        package="simple_demo",
        executable="demo",
        output="both",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([moveit_py_node])