from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue  # <-- ВАЖНО

def generate_launch_description():
    desc_pkg    = 'rozum_pulse_75'
    bringup_pkg = 'rozum_pulse_75_bringup'

    base_url_arg = DeclareLaunchArgument(
        'rozum_base_url', default_value='http://10.10.10.20:8081'
    )

    xacro_file = PathJoinSubstitution([
        FindPackageShare(desc_pkg), 'urdf', 'rozum_pulse_75.xacro'
    ])

    robot_description = {
        'robot_description': ParameterValue(  # <-- ВАЖНО
            Command([
                FindExecutable(name='xacro'), ' ',
                xacro_file, ' ',
                'rozum_base_url:=', LaunchConfiguration('rozum_base_url')
            ]),
            value_type=str
        )
    }

    controllers_yaml = PathJoinSubstitution([
        FindPackageShare(bringup_pkg), 'config', 'controllers.yaml'
    ])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    js = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([base_url_arg, robot_state_pub, control, js])