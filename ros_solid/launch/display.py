#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # get package share directory
    pkg_ros_solid = get_package_share_directory('ros_solid')

    # full paths to your URDF and RVIZ config
    urdf_file = os.path.join(pkg_ros_solid, 'urdf', 'ros_solid.urdf')
    rviz_config = os.path.join(pkg_ros_solid, 'urdf.rviz')

    # read URDF so we can pass its contents on the 'robot_description' parameter
    with open(urdf_file, 'r') as inf:
        robot_description_content = inf.read()

    return LaunchDescription([

        # joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_content
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
    ])
