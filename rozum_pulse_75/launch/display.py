#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
import xacro

def generate_launch_description():
    # get package share directory
    pkg_rozum_pulse_75 = get_package_share_directory('rozum_pulse_75')

    # full paths to your URDF and RVIZ config
    rviz_config = os.path.join(pkg_rozum_pulse_75, 'urdf.rviz')


    xacro_file = get_package_share_path('rozum_pulse_75') / 'urdf' / 'rozum_pulse_75.urdf'
    robot_description_config = xacro.process_file(str(xacro_file)).toxml()
    return LaunchDescription([
        # joint_state_publisher_gui
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'robot_description': robot_description_config
            }],
            
        ),

        # robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_config
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
