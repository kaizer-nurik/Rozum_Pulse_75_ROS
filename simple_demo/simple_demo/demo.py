#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped

# MoveIt 2 modern Python API
from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent, PlanningSceneMonitor

try:
    from tf_transformations import quaternion_from_euler
except ImportError:
    def quaternion_from_euler(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)


class ToolTipPoseMoveItPy(Node):
    """
    Move a manipulator to a desired tool-tip pose given as x y z roll pitch yaw.
    Uses MoveItPy (ROS 2 Jazzy+).
    """

    def __init__(self):
        super().__init__("tool_tip_pose_moveitpy")

        # Parameters
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_link", "")

        # Optional one-shot target
        for p in ["x", "y", "z", "roll", "pitch", "yaw"]:
            self.declare_parameter(p, None)

        # Init MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        group_name = "arm"
        self.planning_component = self.moveit.get_planning_component(group_name)

        # Start planning scene monitor (for current state updates)
        # self.scene_monitor = PlanningSceneMonitor("planning_scene_monitor")
        # self.scene_monitor.start_state_monitor()
        # self.scene_monitor.start_scene_monitor()

        # Subscriber for streaming commands
        self.sub = self.create_subscription(
            PoseStamped, "move_arm_position", self.on_target_msg, 10
        )

        # One-shot target via params
        vals = [self._get_optional_float(p) for p in ["x", "y", "z", "roll", "pitch", "yaw"]]
        if all(v is not None for v in vals):
            self.get_logger().info("Received one-shot target via parameters.")
            self.move_to_rpy(vals)

        self.get_logger().info("tool_tip_pose_moveitpy ready.")

    def _get_optional_float(self, name: str) -> Optional[float]:
        val = self.get_parameter(name).value
        if val is None:
            return None
        return float(val)

    def on_target_msg(self, msg: PoseStamped):
        self.move_to_rpy(msg)

    def move_to_rpy(self, target_pose: PoseStamped):
        # x, y, z, roll, pitch, yaw = [float(v) for v in target]
        # qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        # pose = PoseStamped()
        # pose.header.frame_id = "base_link"

        # pose.pose.position.x = x
        # pose.pose.position.y = y
        # pose.pose.position.z = z
        # pose.pose.orientation.x = qx
        # pose.pose.orientation.y = qy
        # pose.pose.orientation.z = qz
        # pose.pose.orientation.w = qw        x, y, z, roll, pitch, yaw = [float(v) for v in target]
        # qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)



        # Set goal
        self.planning_component.set_goal_state(pose_stamped_msg=target_pose,pose_link="link6_dummy")
        # Plan
        plan_result = self.planning_component.plan()

        if plan_result:
            self.get_logger().info("Executing plan...")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().warn("Planning failed.")


def main():
    rclpy.init()
    node = ToolTipPoseMoveItPy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
