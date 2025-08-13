from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    desc_pkg = 'rozum_pulse_75'  
    xacro_path = os.path.join(get_package_share_directory(desc_pkg), 'urdf', 'rozum_pulse_75.xacro')

    base_url = os.environ.get('ROZUM_BASE_URL', 'http://10.10.10.20:8081')
    robot_description = xacro.process(xacro_path, mappings={'rozum_base_url': base_url})

    controllers = os.path.join(get_package_share_directory('rozum_pulse_75_bringup'), 'config', 'controllers.yaml')

    control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controllers],
        output='screen'
    )

    js = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([control, js])