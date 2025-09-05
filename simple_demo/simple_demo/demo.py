#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped

from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent, PlanningSceneMonitor

import os
import yaml

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


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

        default_config_path = os.path.join(
            get_package_share_directory("simple_demo"),
            "config",
            "scene_objects.yaml",
        )

        self.declare_parameter(
            name="scene_yaml_path",
            value=default_config_path,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Path to YAML file containing static transforms",
            )
        )

        # Optional one-shot target
        for p in ["x", "y", "z", "roll", "pitch", "yaw"]:
            self.declare_parameter(p, None)

        # Init MoveItPy
        self.moveit = MoveItPy(node_name="moveit_py")
        group_name = "arm"
        self.planning_component = self.moveit.get_planning_component(group_name)

        # Start planning scene monitor (for current state updates)
        self.scene_monitor = self.moveit.get_planning_scene_monitor()


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
        self.load_scene()


    def load_scene(self):
        """
        Load collision objects from a YAML file and apply them to the planning scene.
        YAML schema is shown below in the example. Supports BOX, CYLINDER, and SPHERE.
        """
        yaml_path = self.get_parameter("scene_yaml_path").value or ""
        if not yaml_path:
            self.get_logger().warn("No 'scene_yaml_path' set; skipping scene load.")
            return

        # Resolve path (allow relative paths like 'config/scene_objects.yaml')
        yaml_path = os.path.expanduser(yaml_path)
        # if not os.path.isabs(yaml_path):
        #     yaml_path = os.path.join(os.getcwd(), yaml_path)

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Scene YAML not found: {yaml_path}")
            return

        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"Failed to read YAML '{yaml_path}': {e}")
            return

        objects = data.get("objects", [])
        if not isinstance(objects, list) or not objects:
            self.get_logger().warn(f"No collision 'objects' found in {yaml_path}.")
            return

        added = 0
        with self.scene_monitor.read_write() as scene:
            for spec in objects:
                try:
                    co = self._make_collision_object_from_spec(spec)
                    scene.apply_collision_object(co)
                    added += 1
                except Exception as e:
                    self.get_logger().error(f"Skipping object due to error: {e}")

            # ensure kinematic state reflects new objects
            scene.current_state.update()

        self.get_logger().info(f"Loaded {added}/{len(objects)} collision objects from {yaml_path}.")

    def _make_collision_object_from_spec(self, spec: dict) -> CollisionObject:
        """
        Convert a single YAML object spec to a CollisionObject.
        Expected keys per object:
          id: string (required)
          type: BOX|CYLINDER|SPHERE (required)
          frame_id: string (optional; defaults to base_frame param)
          dimensions: list[float] (BOX: [x,y,z], CYLINDER: [height,radius], SPHERE: [radius]) (required)
          position: {x,y,z} (optional; default 0)
          rpy: {roll,pitch,yaw} in radians (optional) OR
          quaternion: {x,y,z,w} (optional). If neither provided, identity orientation is used.
        """
        if "id" not in spec:
            raise ValueError("Missing 'id' in object spec")
        if "type" not in spec:
            raise ValueError("Missing 'type' in object spec")
        if "dimensions" not in spec:
            raise ValueError("Missing 'dimensions' in object spec")

        co = CollisionObject()
        co.id = str(spec["id"])
        co.header.frame_id = spec.get("frame_id", self.get_parameter("base_frame").value)

        typ = str(spec["type"]).upper()
        dims = [float(v) for v in spec["dimensions"]]

        prim = SolidPrimitive()
        if typ == "BOX":
            if len(dims) != 3:
                raise ValueError(f"BOX requires 3 dimensions [x,y,z], got {dims}")
            prim.type = SolidPrimitive.BOX
            prim.dimensions = dims
        elif typ == "CYLINDER":
            # ROS SolidPrimitive expects [height, radius] in that order.
            if len(dims) != 2:
                raise ValueError(f"CYLINDER requires 2 dimensions [height,radius], got {dims}")
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = dims
        elif typ == "SPHERE":
            if len(dims) != 1:
                raise ValueError(f"SPHERE requires 1 dimension [radius], got {dims}")
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = dims
        else:
            raise ValueError(f"Unsupported primitive type '{typ}'")

        # Build pose
        pose = Pose()
        pos = spec.get("position", {})
        pose.position.x = float(pos.get("x", 0.0))
        pose.position.y = float(pos.get("y", 0.0))
        pose.position.z = float(pos.get("z", 0.0))

        if "rpy" in spec:
            rpy = spec["rpy"]
            qx, qy, qz, qw = quaternion_from_euler(
                float(rpy.get("roll", 0.0)),
                float(rpy.get("pitch", 0.0)),
                float(rpy.get("yaw", 0.0)),
            )
        elif "quaternion" in spec:
            q = spec["quaternion"]
            qx, qy, qz, qw = (
                float(q.get("x", 0.0)),
                float(q.get("y", 0.0)),
                float(q.get("z", 0.0)),
                float(q.get("w", 1.0)),
            )
        else:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        return co


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

        

        # # Set goal
        # self.planning_component.set_goal_state(pose_stamped_msg=target_pose,pose_link="link6_dummy")
        # # Plan
        # plan_result = self.planning_component.plan()

        # if plan_result:
        #     self.get_logger().info("Executing plan...")
        #     robot_trajectory = plan_result.trajectory
        #     self.moveit.execute(robot_trajectory, controllers=[])
        # else:
        #     self.get_logger().warn("Planning failed.")

        q = target_pose.pose.orientation
        roll = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y)) 
        self.plan_xy_zrange_roll(target_pose.pose.position.x,target_pose.pose.position.y,roll = roll,)



    def plan_xy_zrange_roll(
    self,
    x: float,
    y: float,
    z_min: float = 0.3,
    z_max: float = 0.5,
    roll: float = 0.0,                  # radians
    xy_tol: float = 0.002,              # half-width in x,y (m)
    roll_tol: float = math.radians(2),  # +/- roll tolerance (rad)
    ):
        frame = self.get_parameter("base_frame").value
        link  = "link6_dummy"

        # --- Position constraint: thin box at (x,y), spanning z in [z_min, z_max]
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = link

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0 * xy_tol, 2.0 * xy_tol, (z_max - z_min)]  # BOX_X, BOX_Y, BOX_Z

        center = Pose()
        center.orientation.w = 1.0
        center.position.x = x
        center.position.y = y
        center.position.z = 0.5 * (z_min + z_max)

        bv = BoundingVolume()
        bv.primitives = [box]
        bv.primitive_poses = [center]
        pc.constraint_region = bv
        pc.weight = 1.0

        # --- Orientation constraint: fix roll only; free pitch/yaw
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        qx, qy, qz, qw = quaternion_from_euler(roll, 0.0, 0.0)
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = qx, qy, qz, qw
        oc.absolute_x_axis_tolerance = roll_tol      # constrain roll
        oc.absolute_y_axis_tolerance = math.pi       # free pitch
        oc.absolute_z_axis_tolerance = math.pi       # free yaw
        oc.weight = 1.0

        goal = Constraints()
        goal.position_constraints = [pc]
        goal.orientation_constraints = [oc]

        # Plan & execute
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(motion_plan_constraints=[goal])
        plan_result = self.planning_component.plan()
        if plan_result:
            self.get_logger().info("Executing plan...")
            self.moveit.execute(plan_result.trajectory, controllers=[])
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
