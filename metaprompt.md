Write detailed prompt for ChatGPT to work as ai agent, that contol robot via mcp, connected to ROS2 jazzy. Here is MCP server's code: https://github.com/robotmcp/ros-mcp-server

Write detailed instructions on each mcp tool

Write detailed description of each ros2 topic and service

Write detailed description of tf2 tree structure

Write detailed description of ROS2 nodes. Here is projects git: https://github.com/kaizer-nurik/Rozum_Pulse_75_ROS 

Write detailed description of the setup: Rozum 75 pulse arm + dh95 gripper + d455 depth camera, that located in front of the manipulator on static tripod.

Write about XYZ system of the setup, so agent could understand where objects are located on the camera.

Write detailed instructions on how to interact with user

Write several detailed examples of interaction.

Here is ros2 topics:

/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/dh_ag95_gripper_controller/transition_event
/diagnostics
/dynamic_joint_states
/gripper_joint_state_broadcaster/transition_event
/joint_states
/move_arm_position
/parameter_events
/robot_description
/rosout
/rozum/D455_1/aligned_depth_to_color/camera_info
/rozum/D455_1/aligned_depth_to_color/image_raw
/rozum/D455_1/aruco/poses
/rozum/D455_1/color/image_raw
/rozum_75_joint_state_broadcaster/transition_event
/rozum_joint_controller/controller_state
/rozum_joint_controller/transition_event
/tf
/tf_static
/yolo/dbg_image

Here is ros2 topic nodes Give detailed desccription of everything

/aruco_node
/controller_manager
/dh_ag95_gripper
/dh_ag95_gripper_controller
/gripper_joint_state_broadcaster
/interactive_marker_display_97808596494624
/move_group
/move_group/moveit
/move_group_private_106590076013968
/moveit_3533013067
/moveit_4277956656
/moveit_simple_controller_manager
/robot_state_publisher
/rozum/D455_1
/rozum75_ros2_control
/rozum_75_joint_state_broadcaster
/rozum_joint_controller
/rviz
/rviz
/rviz_private_126672008528112
/static_tf_yaml
/transform_listener_impl_58c78956af90
/transform_listener_impl_58f4d64b8360
/transform_listener_impl_58f4d65a35b0
/transform_listener_impl_60f17010a980
/transform_listener_impl_733520039700
/yolo/debug_node
/yolo/detect_3d_node
/yolo/tracking_node
/yolo/yolo_node

Here is service list. Give detailed desccription of everything
/apply_planning_scene
/aruco_node/describe_parameters
/aruco_node/get_parameter_types
/aruco_node/get_parameters
/aruco_node/get_type_description
/aruco_node/list_parameters
/aruco_node/set_parameters
/aruco_node/set_parameters_atomically
/calibrate_realsense_position
/check_state_validity
/clear_octomap
/compute_cartesian_path
/compute_fk
/compute_ik
/controller_manager/configure_controller
/controller_manager/describe_parameters
/controller_manager/get_logger_levels
/controller_manager/get_parameter_types
/controller_manager/get_parameters
/controller_manager/get_type_description
/controller_manager/list_controller_types
/controller_manager/list_controllers
/controller_manager/list_hardware_components
/controller_manager/list_hardware_interfaces
/controller_manager/list_parameters
/controller_manager/load_controller
/controller_manager/reload_controller_libraries
/controller_manager/set_hardware_component_state
/controller_manager/set_logger_levels
/controller_manager/set_parameters
/controller_manager/set_parameters_atomically
/controller_manager/switch_controller
/controller_manager/unload_controller
/dh_ag95_gripper/describe_parameters
/dh_ag95_gripper/get_parameter_types
/dh_ag95_gripper/get_parameters
/dh_ag95_gripper/get_type_description
/dh_ag95_gripper/list_parameters
/dh_ag95_gripper/set_parameters
/dh_ag95_gripper/set_parameters_atomically
/dh_ag95_gripper_controller/describe_parameters
/dh_ag95_gripper_controller/get_logger_levels
/dh_ag95_gripper_controller/get_parameter_types
/dh_ag95_gripper_controller/get_parameters
/dh_ag95_gripper_controller/get_type_description
/dh_ag95_gripper_controller/list_parameters
/dh_ag95_gripper_controller/set_logger_levels
/dh_ag95_gripper_controller/set_parameters
/dh_ag95_gripper_controller/set_parameters_atomically
/get_planner_params
/get_planning_scene
/get_urdf
/gripper_joint_state_broadcaster/describe_parameters
/gripper_joint_state_broadcaster/get_logger_levels
/gripper_joint_state_broadcaster/get_parameter_types
/gripper_joint_state_broadcaster/get_parameters
/gripper_joint_state_broadcaster/get_type_description
/gripper_joint_state_broadcaster/list_parameters
/gripper_joint_state_broadcaster/set_logger_levels
/gripper_joint_state_broadcaster/set_parameters
/gripper_joint_state_broadcaster/set_parameters_atomically
/interactive_marker_display_97808596494624/describe_parameters
/interactive_marker_display_97808596494624/get_parameter_types
/interactive_marker_display_97808596494624/get_parameters
/interactive_marker_display_97808596494624/get_type_description
/interactive_marker_display_97808596494624/list_parameters
/interactive_marker_display_97808596494624/set_parameters
/interactive_marker_display_97808596494624/set_parameters_atomically
/load_geometry_from_file
/load_map
/move_group/describe_parameters
/move_group/get_parameter_types
/move_group/get_parameters
/move_group/get_type_description
/move_group/list_parameters
/move_group/moveit/describe_parameters
/move_group/moveit/get_parameter_types
/move_group/moveit/get_parameters
/move_group/moveit/get_type_description
/move_group/moveit/list_parameters
/move_group/moveit/set_parameters
/move_group/moveit/set_parameters_atomically
/move_group/set_parameters
/move_group/set_parameters_atomically
/move_group_private_106590076013968/describe_parameters
/move_group_private_106590076013968/get_parameter_types
/move_group_private_106590076013968/get_parameters
/move_group_private_106590076013968/get_type_description
/move_group_private_106590076013968/list_parameters
/move_group_private_106590076013968/set_parameters
/move_group_private_106590076013968/set_parameters_atomically
/moveit_3533013067/describe_parameters
/moveit_3533013067/get_parameter_types
/moveit_3533013067/get_parameters
/moveit_3533013067/get_type_description
/moveit_3533013067/list_parameters
/moveit_3533013067/set_parameters
/moveit_3533013067/set_parameters_atomically
/moveit_4277956656/describe_parameters
/moveit_4277956656/get_parameter_types
/moveit_4277956656/get_parameters
/moveit_4277956656/get_type_description
/moveit_4277956656/list_parameters
/moveit_4277956656/set_parameters
/moveit_4277956656/set_parameters_atomically
/moveit_simple_controller_manager/describe_parameters
/moveit_simple_controller_manager/get_parameter_types
/moveit_simple_controller_manager/get_parameters
/moveit_simple_controller_manager/get_type_description
/moveit_simple_controller_manager/list_parameters
/moveit_simple_controller_manager/set_parameters
/moveit_simple_controller_manager/set_parameters_atomically
/plan_kinematic_path
/plan_sequence_path
/query_planner_interface
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/get_type_description
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/rozum/D455_1/calib_config_read
/rozum/D455_1/calib_config_write
/rozum/D455_1/describe_parameters
/rozum/D455_1/device_info
/rozum/D455_1/get_parameter_types
/rozum/D455_1/get_parameters
/rozum/D455_1/get_type_description
/rozum/D455_1/hw_reset
/rozum/D455_1/list_parameters
/rozum/D455_1/set_parameters
/rozum/D455_1/set_parameters_atomically
/rozum75_ros2_control/describe_parameters
/rozum75_ros2_control/get_parameter_types
/rozum75_ros2_control/get_parameters
/rozum75_ros2_control/get_type_description
/rozum75_ros2_control/list_parameters
/rozum75_ros2_control/set_parameters
/rozum75_ros2_control/set_parameters_atomically
/rozum_75_joint_state_broadcaster/describe_parameters
/rozum_75_joint_state_broadcaster/get_logger_levels
/rozum_75_joint_state_broadcaster/get_parameter_types
/rozum_75_joint_state_broadcaster/get_parameters
/rozum_75_joint_state_broadcaster/get_type_description
/rozum_75_joint_state_broadcaster/list_parameters
/rozum_75_joint_state_broadcaster/set_logger_levels
/rozum_75_joint_state_broadcaster/set_parameters
/rozum_75_joint_state_broadcaster/set_parameters_atomically
/rozum_joint_controller/describe_parameters
/rozum_joint_controller/get_logger_levels
/rozum_joint_controller/get_parameter_types
/rozum_joint_controller/get_parameters
/rozum_joint_controller/get_type_description
/rozum_joint_controller/list_parameters
/rozum_joint_controller/query_state
/rozum_joint_controller/set_logger_levels
/rozum_joint_controller/set_parameters
/rozum_joint_controller/set_parameters_atomically
/rviz/describe_parameters
/rviz/get_parameter_types
/rviz/get_parameters
/rviz/get_type_description
/rviz/list_parameters
/rviz/reset_time
/rviz/set_parameters
/rviz/set_parameters_atomically
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/get_interactive_markers
/rviz_private_126672008528112/describe_parameters
/rviz_private_126672008528112/get_parameter_types
/rviz_private_126672008528112/get_parameters
/rviz_private_126672008528112/get_type_description
/rviz_private_126672008528112/list_parameters
/rviz_private_126672008528112/set_parameters
/rviz_private_126672008528112/set_parameters_atomically
/save_geometry_to_file
/save_map
/set_planner_params
/static_tf_yaml/describe_parameters
/static_tf_yaml/get_parameter_types
/static_tf_yaml/get_parameters
/static_tf_yaml/get_type_description
/static_tf_yaml/list_parameters
/static_tf_yaml/set_parameters
/static_tf_yaml/set_parameters_atomically
/transform_listener_impl_58c78956af90/get_type_description
/transform_listener_impl_58f4d64b8360/get_type_description
/transform_listener_impl_58f4d65a35b0/get_type_description
/transform_listener_impl_60f17010a980/get_type_description
/transform_listener_impl_733520039700/get_type_description
/yolo/debug_node/change_state
/yolo/debug_node/describe_parameters
/yolo/debug_node/get_available_states
/yolo/debug_node/get_available_transitions
/yolo/debug_node/get_parameter_types
/yolo/debug_node/get_parameters
/yolo/debug_node/get_state
/yolo/debug_node/get_transition_graph
/yolo/debug_node/get_type_description
/yolo/debug_node/list_parameters
/yolo/debug_node/set_parameters
/yolo/debug_node/set_parameters_atomically
/yolo/detect_3d_node/change_state
/yolo/detect_3d_node/describe_parameters
/yolo/detect_3d_node/get_available_states
/yolo/detect_3d_node/get_available_transitions
/yolo/detect_3d_node/get_parameter_types
/yolo/detect_3d_node/get_parameters
/yolo/detect_3d_node/get_state
/yolo/detect_3d_node/get_transition_graph
/yolo/detect_3d_node/get_type_description
/yolo/detect_3d_node/list_parameters
/yolo/detect_3d_node/set_parameters
/yolo/detect_3d_node/set_parameters_atomically
/yolo/enable
/yolo/tracking_node/change_state
/yolo/tracking_node/describe_parameters
/yolo/tracking_node/get_available_states
/yolo/tracking_node/get_available_transitions
/yolo/tracking_node/get_parameter_types
/yolo/tracking_node/get_parameters
/yolo/tracking_node/get_state
/yolo/tracking_node/get_transition_graph
/yolo/tracking_node/get_type_description
/yolo/tracking_node/list_parameters
/yolo/tracking_node/set_parameters
/yolo/tracking_node/set_parameters_atomically
/yolo/yolo_node/change_state
/yolo/yolo_node/describe_parameters
/yolo/yolo_node/get_available_states
/yolo/yolo_node/get_available_transitions
/yolo/yolo_node/get_parameter_types
/yolo/yolo_node/get_parameters
/yolo/yolo_node/get_state
/yolo/yolo_node/get_transition_graph
/yolo/yolo_node/get_type_description
/yolo/yolo_node/list_parameters
/yolo/yolo_node/set_parameters
/yolo/yolo_node/set_parameters_atomically
/manipulator/get_objects
/manipulator/grab_object
/manipulator/move_to
/manipulator/open_gripper
/manipulator/close_gripper
/manipulator/go_home
/manipulator/freeze
/manipulator/relax
/manipulator/stop



Here is action list
/dh_ag95_gripper_controller/gripper_cmd
/execute_trajectory
/move_action
/rozum/D455_1/triggered_calibration
/rozum_joint_controller/follow_joint_trajectory
/sequence_move_group


I also attach rqt_graph and tf2 frames and d455 camera view of the setup

Here is the whole system overview:

I have Rozum 75 pulse manipulator, connected to pc with ethernet cable. I use custom ros controller to send the whole trajectory to the robot. 

D455 realsense depth camera is located on static tripod. To calibrate camera's tf with the setup, i use 3 aruco markers and ros2-aruco-pose-estimation package. It is guaranteed, that camera is always calibrated before agent use.

Yolo tracker is used with fixed library:
{0: 'robot', 1: 'manipulator', 2: 'tennis ball', 3: 'blue container', 4: 'teddy bear'}

teddy, tennis ball, and blue container are objects, that are going to be manipulated.

There are also important services:

/manipulator/get_objects - returns list of detected by yolo objects and their center points

/manipulator/grab_object - give ID of an object, and manipulator will grab it

/manipulator/move_to - give x y z roll pitch yaw and manipulator will go there
/manipulator/open_gripper - opens gripper
/manipulator/close_gripper - closes gripper
/manipulator/go_home - sends manipulator to its home position
/manipulator/freeze - freezes the manipulator
/manipulator/relax - relaxes the manipulator. Warning! it turns off manipulator motors
/manipulator/stop - emergency stop of the manipulator

breef example of interactions with the user (You will also need to add some examples):

1) chatting with user ( about weather, news, etc). Agent must be friendly
2) detailed describing, what agent sees throungh camera. Also, it must check depth channels
3) execute basic commands ('freeze',stop,relax,open gripper etc) when prompted
4) execute complex commands (place tennis ball in the container)
