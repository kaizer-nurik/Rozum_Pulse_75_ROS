from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _strip_quotes(value: str) -> str:
    if not isinstance(value, str):
        return value
    v = value.strip()
    if (v.startswith("\"") and v.endswith("\"")) or (v.startswith("'") and v.endswith("'")):
        return v[1:-1]
    return v


def generate_launch_description() -> LaunchDescription:
    # Locate params YAML packaged with realsense_to_scene_transform
    pkg_share = get_package_share_directory("realsense_to_scene_transform")
    yaml_path = os.path.join(pkg_share, "config", "d435_d455_aruco_params.yaml")

    with open(yaml_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    params = cfg.get("/aruco_node", {}).get("ros__parameters", {})
    # Extract and sanitize string fields (they may come with extra quotes in YAML)
    d455_serial = _strip_quotes(params.get("d455_serial_no", ""))
    d435_serial = _strip_quotes(params.get("d435_serial_no", ""))
    robot_name = _strip_quotes(params.get("robot_name", "rozum"))
    d455_cam_name = _strip_quotes(params.get("d455_camera_name", "D455_1"))
    d435_cam_name = _strip_quotes(params.get("d435_camera_name", "D435_1"))

    # Path to the single-camera ArUco launch file in the aruco_pose_estimation package
    aruco_launch_path = PathJoinSubstitution([
        FindPackageShare("aruco_pose_estimation"),
        "launch",
        "aruco_pose_estimation.launch.py",
    ])

    # Include ArUco launch for D455 (depth-enabled)
    d455_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_launch_path),
        launch_arguments={
            "serial_no": "'"+str(d455_serial)+"'",
            "robot_name": str(robot_name),
            "camera_name": str(d455_cam_name),
            "rviz": "false",
            # use_depth_input is read from default config; override if you want:
            "use_depth_input": "true",
        }.items(),
    )

    # Include ArUco launch for D435 (depth-enabled)
    d435_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_launch_path),
        launch_arguments={
            "serial_no": "'"+str(d435_serial)+"'",
            "robot_name": str(robot_name),
            "camera_name": str(d435_cam_name),
            "rviz": "false",
            "use_depth_input": "true",
        }.items(),
    )

    # Static TF publisher node (reads its own static_tf.yaml by default)
    static_tf = Node(
        package="realsense_to_scene_transform",
        executable="static_tf_yaml",
        name="static_tf_yaml",
        output="screen",
        parameters=[{
            "aruco_poses_topic": f"/{str(robot_name)}/{str(d455_cam_name)}/aruco/markers",
            "calibration_parent_frame": f"{str(d455_cam_name)}_color_optical_frame",
            "calibration_child_frame": "world",
        }],
    )

    rviz_file = PathJoinSubstitution([
        FindPackageShare('realsense_to_scene_transform'),
        'rviz',
        'cam_detect.rviz'
    ])

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        d455_aruco,
        # d435_aruco,
        static_tf,
        rviz2_node,
    ])


