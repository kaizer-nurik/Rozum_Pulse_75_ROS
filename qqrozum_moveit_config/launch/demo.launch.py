from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    planning_scene_monitor_parameters={
         "publish_robot_description":True,
 "publish_robot_description_semantic":True,
    }
    moveit_config = MoveItConfigsBuilder("rozum_pulse_75", package_name="rozum_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
