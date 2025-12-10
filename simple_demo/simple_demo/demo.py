#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import copy
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

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

from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import Point

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
        self.arm_group_name = group_name
        self.eef_link_name = self.get_parameter("eef_link").value or "grasp_link"

        # Start planning scene monitor (for current state updates)
        self.scene_monitor = self.moveit.get_planning_scene_monitor()


        # Subscriber for streaming commands
        latest_qos = qos_profile_sensor_data
        latest_qos.depth = 1
        self.sub = self.create_subscription(
            PoseStamped, "move_arm_position", self.on_target_msg, latest_qos
        )
        

        # One-shot target via params
        vals = [self._get_optional_float(p) for p in ["x", "y", "z", "roll", "pitch", "yaw"]]
        if all(v is not None for v in vals):
            self.get_logger().info("Received one-shot target via parameters.")
            self.move_to_rpy(vals)

        self.get_logger().info("tool_tip_pose_moveitpy ready.")
        self.load_scene()

        # --- Gripper services (open/close) ---
        # Parameters to configure gripper command values
        self.declare_parameter("gripper_open_position", 0.0)   # unit depends on controller mapping
        self.declare_parameter("gripper_close_position", 0.92)
        self.declare_parameter("gripper_max_effort", 40.0)
        self.declare_parameter("gripper_action_timeout_sec", 20.0)
        self.declare_parameter("gripper_wait_server_sec", 5.0)
        self.declare_parameter("gripper_action_name", "/dh_ag95_gripper_controller/gripper_cmd")

        # Action client to existing GripperActionController
        gripper_action_name = str(self.get_parameter("gripper_action_name").value)
        self.get_logger().info(f"Connecting to gripper action: {gripper_action_name}")
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            gripper_action_name,
        )

        # Expose simple Trigger services
        self.open_srv = self.create_service(Trigger, "open_gripper", self._on_open_gripper)
        self.close_srv = self.create_service(Trigger, "close_gripper", self._on_close_gripper)

        # --- YOLO 3D detections subscription and pick-and-place service ---
        self.declare_parameter("yolo_detections_topic", "/yolo/detections_3d")
        self.declare_parameter("pick_object_name", "teddy bear")
        self.declare_parameter("place_container_name", "blue container")
        self.declare_parameter("object_freshness_sec", 2.0)

        self._last_seen_objects = {}
        topic = self.get_parameter("yolo_detections_topic").value
        self._yolo_sub = self.create_subscription(
            DetectionArray, topic, self._on_yolo_detections, 10
        )
        self.pick_place_srv = self.create_service(
            Trigger, "pick_place_teddy_to_container", self._on_pick_and_place
        )


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

    def _on_yolo_detections(self, msg: DetectionArray):
        now = self.get_clock().now().nanoseconds
        for det in msg.detections:
            name = (det.class_name or "").strip().lower()
            if not name:
                continue
            if name in (self.get_parameter("pick_object_name").value.lower(),
                        self.get_parameter("place_container_name").value.lower()):
                p = det.bbox3d.center.position
                self._last_seen_objects[name] = {
                    "position": Point(x=p.x, y=p.y, z=p.z),
                    "stamp_ns": now,
                }

    def _wait_for_objects(self, required_names: list, timeout_sec: Optional[float]) -> Optional[dict]:
        start = self.get_clock().now().nanoseconds
        freshness_ns = int(float(self.get_parameter("object_freshness_sec").value) * 1e9)
        required_lower = [n.lower() for n in required_names]
        while True:
            now_ns = self.get_clock().now().nanoseconds
            found = {}
            for name in required_lower:
                info = self._last_seen_objects.get(name)
                if info and (now_ns - info["stamp_ns"]) <= freshness_ns:
                    found[name] = info["position"]
            if len(found) == len(required_lower):
                return found
            if timeout_sec is not None and (now_ns - start) > int(timeout_sec * 1e9):
                return None
            # brief sleep to yield CPU; MultiThreadedExecutor will keep callbacks flowing
            import time
            time.sleep(0.05)

    def _on_pick_and_place(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        pick_name = self.get_parameter("pick_object_name").value
        place_name = self.get_parameter("place_container_name").value
        self.get_logger().info(f"Waiting for '{pick_name}' and '{place_name}' detections...")
        positions = self._wait_for_objects([pick_name, place_name], timeout_sec=3)
        if positions is None:
            response.success = False
            response.message = "timeout waiting detections"
            return response

        bear_p = positions[pick_name.lower()]
        cont_p = positions[place_name.lower()]

        # Move above teddy bear and grasp
        self.get_logger().info(
            f"Moving to teddy bear at ({bear_p.x:.3f}, {bear_p.y:.3f}, {bear_p.z:.3f})"
        )

        if not self._send_gripper_goal(float(self.get_parameter("gripper_open_position").value), wait_result=False):
            response.success = False
            response.message = "failed to open gripper"
            return response

        import time
        time.sleep(3)

        ok = self.plan_xy_zrange_yaw(
            bear_p.x-0.13,
            bear_p.y+0.05,
            z_min=0.15,
            z_max=bear_p.z + 0.2,
        )
        time.sleep(2)
        for dz in range(8):
            ok = self.plan_xy_zrange_yaw(
            bear_p.x-0.13,
            bear_p.y+0.05,
            z_min=-0.08+dz/100,
            z_max=bear_p.z + 0.04,
            )
            self.get_logger().info(str(dz))
            if ok:
                break
        time.sleep(2)
        
        if not ok:
            response.success = False
            response.message = "failed to reach teddy bear"
            return response

        if not self._send_gripper_goal(float(self.get_parameter("gripper_close_position").value), wait_result=False):
            response.success = False
            response.message = "failed to close gripper"
            return response
        time.sleep(1)

        ok = self.plan_xy_zrange_yaw(
            bear_p.x-0.13,
            bear_p.y+0.05,
            z_min=0.15,
            z_max=bear_p.z + 0.2,
        )
        time.sleep(1)
        

        # Move above container and release
        self.get_logger().info(
            f"Moving above container at ({cont_p.x:.3f}, {cont_p.y:.3f}, {cont_p.z:.3f})"
        )

        ok = self.plan_xy_zrange_yaw(
            cont_p.x-0.05,
            cont_p.y,
            z_min=0.1,
            z_max=cont_p.z + 0.5,
            container =True
        )
        time.sleep(1)

        ok = self.plan_xy_zrange_yaw(
            cont_p.x-0.05,
            cont_p.y,
            z_min=0.02,
            z_max=cont_p.z + 0.5,
            container =True
        )
        time.sleep(1)

        if not ok:
            response.success = False
            response.message = "failed to reach container"
            return response

        if not self._send_gripper_goal(float(self.get_parameter("gripper_open_position").value), wait_result=False):
            response.success = False
            response.message = "failed to open gripper"
            return response
        time.sleep(1)
        

        response.success = True
        response.message = "pick and place done"
        return response
    
    
    def _send_gripper_goal(self, position: float, wait_result: bool = False, timeout_sec: Optional[float] = None) -> bool:
        """Send a gripper goal to the configured GripperCommand action.
        If wait_result is True, synchronously wait for acceptance and result with a timeout.
        Returns True on success, False otherwise.
        """
        # Resolve timeouts
        try:
            server_timeout = float(self.get_parameter("gripper_wait_server_sec").value)
        except Exception:
            server_timeout = 5.0
        if timeout_sec is None:
            try:
                timeout_sec = float(self.get_parameter("gripper_action_timeout_sec").value)
            except Exception:
                timeout_sec = 20.0

        # Ensure server is up
        if not self.gripper_client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("Gripper action server not available. Verify gripper_action_name and namespaces.")
            return False

        # Build goal
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        try:
            goal.command.max_effort = float(self.get_parameter("gripper_max_effort").value)
        except Exception:
            goal.command.max_effort = 40.0

        # Send goal
        try:
            send_future = self.gripper_client.send_goal_async(goal)
        except Exception as e:
            self.get_logger().error(f"Failed to send gripper goal: {e}")
            return False

        if not wait_result:
            # Non-blocking: attach logging callbacks
            def _on_goal_sent(fut):
                try:
                    handle = fut.result()
                    if not handle.accepted:
                        self.get_logger().warn("Gripper goal rejected by controller")
                        return
                    result_future = handle.get_result_async()

                    def _on_result(_):
                        try:
                            _ = result_future.result()
                            self.get_logger().info(
                                f"Gripper action finished (pos={goal.command.position:.3f}, effort={goal.command.max_effort:.1f})"
                            )
                        except Exception as err:
                            self.get_logger().error(f"Gripper result error: {err}")

                    result_future.add_done_callback(_on_result)
                except Exception as err:
                    self.get_logger().error(f"Error after goal dispatch: {err}")

            send_future.add_done_callback(_on_goal_sent)
            return True

        # Blocking path: wait for acceptance and result
        try:
            from rclpy import spin_until_future_complete
        except Exception:
            spin_until_future_complete = None

        # Wait for acceptance
        if spin_until_future_complete is None:
            spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        else:
            start_ns = self.get_clock().now().nanoseconds
            while not send_future.done():
                if (self.get_clock().now().nanoseconds - start_ns) > int(timeout_sec * 1e9):
                    break
                rclpy.spin_once(self, timeout_sec=0.05)

        if not send_future.done():
            self.get_logger().error("Timeout waiting for gripper goal acceptance")
            return False

        try:
            handle = send_future.result()
        except Exception as e:
            self.get_logger().error(f"Failed to obtain goal handle: {e}")
            return False

        if not handle.accepted:
            self.get_logger().warn("Gripper goal rejected by controller")
            return False

        # Wait for result
        result_future = handle.get_result_async()
        if spin_until_future_complete is not None:
            spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        else:
            start_ns = self.get_clock().now().nanoseconds
            while not result_future.done():
                if (self.get_clock().now().nanoseconds - start_ns) > int(timeout_sec * 1e9):
                    break
                rclpy.spin_once(self, timeout_sec=0.05)

        if not result_future.done():
            self.get_logger().error("Timeout waiting for gripper action result")
            return False

        try:
            _ = result_future.result()
            return True
        except Exception as e:
            self.get_logger().error(f"Error retrieving gripper result: {e}")
            return False

    
    def _on_open_gripper(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        open_pos = float(self.get_parameter("gripper_open_position").value)
        ok = self._send_gripper_goal(open_pos)
        response.success = bool(ok)
        response.message = "open command sent" if ok else "failed to send open command"
        return response

    def _on_close_gripper(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        close_pos = float(self.get_parameter("gripper_close_position").value)
        ok = self._send_gripper_goal(close_pos)
        response.success = bool(ok)
        response.message = "close command sent" if ok else "failed to send close command"
        return response


    def _get_optional_float(self, name: str) -> Optional[float]:
        val = self.get_parameter(name).value
        if val is None:
            return None
        return float(val)

    def on_target_msg(self, msg: PoseStamped):
        self.move_to_rpy(msg)

    def is_manipulator_moving(self, tolerance: float = 1e-3) -> bool:
        """
        Returns True if the manipulator's joint positions are changing (i.e., it's moving),
        otherwise returns False.
        Uses MoveIt's current robot state via the planning scene monitor.
        """
        try:
            # Get first joint state snapshot
            state1 = self.scene_monitor.get_current_state()
            if state1 is None:
                self.get_logger().warn("Unable to retrieve current robot state.")
                return False

            joints1 = state1.get_joint_group_positions(self.arm_group_name)

            # Wait briefly and get second snapshot
            import time
            time.sleep(0.1)

            state2 = self.scene_monitor.get_current_state()
            if state2 is None:
                return False

            joints2 = state2.get_joint_group_positions(self.arm_group_name)

            # Compare joint positions
            diffs = [abs(a - b) for a, b in zip(joints1, joints2)]
            moving = any(d > tolerance for d in diffs)

            return moving
        except Exception as e:
            self.get_logger().error(f"Failed to determine manipulator motion: {e}")
            return False

    def wait_until_manipulator_stopped(self, tolerance: float = 1e-3, stable_time: float = 0.5, timeout: float = 30.0) -> bool:
        """
        Blocks until the manipulator stops moving.
        Returns True if the manipulator becomes still before timeout, False otherwise.

        Args:
            tolerance: Minimum joint change to be considered movement.
            stable_time: How long (in seconds) the manipulator must remain still.
            timeout: Max time (in seconds) to wait before giving up.
        """
        import time

        start_time = time.time()
        last_moving = True
        still_since = None

        while time.time() - start_time < timeout:
            moving = self.is_manipulator_moving(tolerance=tolerance)

            if moving:
                last_moving = True
                still_since = None
            else:
                if last_moving:
                    # just stopped moving, start counting still time
                    still_since = time.time()
                    last_moving = False
                elif still_since is not None and (time.time() - still_since) >= stable_time:
                    # remained still for enough time
                    self.get_logger().info("Manipulator has stopped moving.")
                    return True

            time.sleep(0.1)

        self.get_logger().warn("Timeout while waiting for manipulator to stop.")
        return False

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
        # self.planning_component.set_goal_state(pose_stamped_msg=target_pose,pose_link="grasp_link")
        # # Plan
        # plan_result = self.planning_component.plan()

        # if plan_result:
        #     self.get_logger().info("Executing plan...")
        #     robot_trajectory = plan_result.trajectory
        #     self.moveit.execute(robot_trajectory, controllers=[])
        # else:
        #     self.get_logger().warn("Planning failed.")

        q = target_pose.pose.orientation
        roll  = math.atan2(2*(q.w*q.x + q.y*q.z), 1 - 2*(q.x*q.x + q.y*q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x))))
        yaw   = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)) + math.pi*60/180
        self.get_logger().info(f"{roll} {pitch} {yaw}")
        self.plan_xy_zrange_yaw(target_pose.pose.position.x,target_pose.pose.position.y,z_min = target_pose.pose.position.z,z_max = target_pose.pose.position.z+0.05,yaw = yaw,)



    def plan_xy_zrange_yaw(
        self,
        x: float,
        y: float,
        z_min: float = 0.12,
        z_max: float = 0.2,
        yaw: float = 0.0,
        xy_tol: float = 0.01,
        yaw_tol: float = math.radians(2),  # unused, retained for API compatibility
        container: bool = False,
    ) -> bool:
        """Plan to a Cartesian pose by solving IK and executing a joint trajectory."""



        group_name = self.arm_group_name
        base_frame = "base_link"
        eef_link =  "grasp_link"

        self.planning_component.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        quat = quaternion_from_euler(yaw, 1.512, 2.619)
        pose_goal.pose.orientation.w = quat[3]
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z_min+0.1
        self.planning_component.set_goal_state(pose_stamped_msg=pose_goal, pose_link="grasp_link")

        # current_state = self.planning_component.get_start_state()
        # if current_state is None:
        #     self.get_logger().error("Unable to fetch current robot state.")
        #     return False

        # try:
        #     self.scene_monitor.update_frame_transforms()
        # except RuntimeError as exc:
        #     self.get_logger().debug(f"Failed to refresh frame transforms before IK search: {exc}")

        # try:
        #     current_positions = list(current_state.get_joint_group_positions(group_name))
        # except RuntimeError as exc:
        #     self.get_logger().error(f"Failed to read current joint positions: {exc}")
        #     return False

        # # Prepare orientation candidates (primary orientation first).
        # base_roll = -1.553
        # base_pitch = -0.024
        # roll_offsets = [0.0, -0.25, 0.25]
        # pitch_offsets = [0.0, -0.25, 0.25]
        # yaw_offsets = [0.0, -0.2, 0.2]
        # if container:
        #     roll_offsets = [0.0, -0.4, 0.4]
        #     pitch_offsets = [0.0, -0.6, 0.6]
        #     yaw_offsets = [0.0, -0.5, 0.5]

        # orientation_candidates: List[tuple] = [quaternion_from_euler(yaw, 1.512, 2.619)]
        # # orientation_candidates: List[tuple] = []
        # # for yaw_delta in yaw_offsets:
        # #     for pitch_delta in pitch_offsets:
        # #         for roll_delta in roll_offsets:
        # #             orientation_candidates.append(
        # #                 quaternion_from_euler(
        # #                     base_roll + roll_delta,
        # #                     base_pitch + pitch_delta,
        # #                     yaw + yaw_delta,
        # #                 )
        # #             )

        # # Deduplicate while preserving order.
        # # seen_orientations = set()
        # # unique_orientations: List[tuple] = []
        # # for quat in orientation_candidates:
        # #     key = tuple(round(v, 6) for v in quat)
        # #     if key not in seen_orientations:
        # #         unique_orientations.append(quat)
        # #         seen_orientations.add(key)
        # # orientation_candidates = unique_orientations

        # # Sample z within the admissible range.
        # if math.isclose(z_min, z_max, abs_tol=1e-4):
        #     z_samples = [z_min]
        # else:
        #     steps = max(3, min(9, int(abs(z_max - z_min) / 0.01) + 1))
        #     step = (z_max - z_min) / (steps - 1)
        #     z_samples = [z_min + i * step for i in range(steps)]
        # z_samples = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,]
        # # Generate xy offsets to search for a feasible IK solution near the target.
        # offsets = [(0.0, 0.0)]
        # seen_offsets = {(0.0, 0.0)}
        # xy_step = max(0.01, xy_tol if xy_tol > 0.0 else 0.01)
        # max_xy_radius = max(0.01, xy_tol * 1.0)
        # radius = xy_step
        # while radius <= max_xy_radius + 1e-9:
        #     samples = [-radius, 0.0, radius]
        #     new_offsets = []
        #     for dx in samples:
        #         for dy in samples:
        #             if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        #                 continue
        #             key = (round(dx, 5), round(dy, 5))
        #             if key in seen_offsets:
        #                 continue
        #             seen_offsets.add(key)
        #             new_offsets.append((dx, dy))
        #     new_offsets.sort(key=lambda item: (abs(item[0]) + abs(item[1]), -item[1]))
        #     offsets.extend(new_offsets)
        #     radius += xy_step
        # self.get_logger().info(f'offsets initialised, {len(offsets)}')
        # best_joint_positions: Optional[List[float]] = None
        # best_pose: Optional[Pose] = None
        # best_metrics = (float("inf"), float("inf"), float("inf"), float("inf"))
        # best_colliding: Optional[tuple] = None
        # target_z_preference = 0.5 * (z_min + z_max)

        # for orientation_idx, quat in enumerate(orientation_candidates):
        #     for z in z_samples:
        #         self.get_logger().info(f'z sample {z}')

        #         for dx, dy in offsets:
                    
        #             candidate_pose = Pose()
        #             candidate_pose.position.x = x + dx
        #             candidate_pose.position.y = y + dy
        #             candidate_pose.position.z = z
        #             candidate_pose.orientation.x = quat[0]
        #             candidate_pose.orientation.y = quat[1]
        #             candidate_pose.orientation.z = quat[2]
        #             candidate_pose.orientation.w = quat[3]

        #             seed_state = copy.deepcopy(current_state)
        #             try:
        #                 success = seed_state.set_from_ik(
        #                     group_name,
        #                     candidate_pose,
        #                     tip_name=eef_link,
        #                     timeout=0.2,
        #                 )
        #             except RuntimeError as exc:
        #                 self.get_logger().debug(f"IK call failed: {exc}")
        #                 success = False

        #             if not success:
        #                 continue

        #             seed_state.update()

        #             try:
        #                 joint_positions = list(
        #                     seed_state.get_joint_group_positions(group_name)
        #                 )
        #             except RuntimeError as exc:
        #                 self.get_logger().warn(f"Failed to read IK solution joints: {exc}")
        #                 continue

        #             joint_distance = math.sqrt(
        #                 sum(
        #                     (jp - cp) ** 2
        #                     for jp, cp in zip(joint_positions, current_positions)
        #                 )
        #             )
        #             xy_error = math.sqrt(dx * dx + dy * dy)
        #             z_error = abs(z - target_z_preference)
        #             metrics = (
        #                 joint_distance,
        #                 xy_error,
        #                 z_error,
        #                 float(orientation_idx),
        #             )

        #             in_collision = False
        #             with self.scene_monitor.read_only() as locked_scene:
        #                 try:
        #                     in_collision = locked_scene.is_state_colliding(
        #                         seed_state, group_name, False
        #                     )
        #                 except RuntimeError as exc:
        #                     self.get_logger().debug(f"Collision check failed: {exc}")
        #                     in_collision = False

        #             if in_collision:
        #                 if best_colliding is None or metrics < best_colliding[0]:
        #                     best_colliding = (metrics, joint_positions, candidate_pose)
        #                 continue
                    

        #             if metrics < best_metrics:
        #                 best_metrics = metrics
        #                 best_joint_positions = joint_positions
        #                 best_pose = candidate_pose
                

        # if best_joint_positions is None:
        #     if best_colliding is not None:
        #         coll_metrics, coll_joints, coll_pose = best_colliding
        #         joint_str = ", ".join(f"{v:.4f}" for v in coll_joints)
        #         if coll_pose is not None:
        #             self.get_logger().warn(str([
        #                 "Best IK solution remains in collision (Δq=%.4f, xy=%.4f, zΔ=%.4f) "
        #                 "at pose (%.3f, %.3f, %.3f); joints [%s]",
        #                 coll_metrics[0],
        #                 coll_metrics[1],
        #                 coll_metrics[2],
        #                 coll_pose.position.x,
        #                 coll_pose.position.y,
        #                 coll_pose.position.z,
        #                 joint_str,
        #             ])
        #             )
        #         else:
        #             self.get_logger().warn(
        #                 str([
        #                 "Best IK solution remains in collision (Δq=%.4f, xy=%.4f, zΔ=%.4f); joints [%s]",
        #                 coll_metrics[0],
        #                 coll_metrics[1],
        #                 coll_metrics[2],
        #                 joint_str,
        #                 ])
        #             )
        #     self.get_logger().error(
        #         "Unable to find an IK solution near the requested pose; keeping current configuration."
        #     )
        #     candidate_pose = Pose()
        #     candidate_pose.position.x = x 
        #     candidate_pose.position.y = y 
        #     candidate_pose.position.z = 0.2
        #     candidate_pose.orientation.x = orientation_candidates[0][0]
        #     candidate_pose.orientation.y = orientation_candidates[0][1]
        #     candidate_pose.orientation.z = orientation_candidates[0][2]
        #     candidate_pose.orientation.w = orientation_candidates[0][3]

        #     seed_state = copy.deepcopy(current_state)
        #     try:
        #         success = seed_state.set_from_ik(
        #             group_name,
        #             candidate_pose,
        #             tip_name=eef_link,
        #             timeout=0.2,
        #         )
        #     except RuntimeError as exc:
        #         self.get_logger().debug(f"IK call failed: {exc}")
        #         success = False

        
        #     seed_state.update()

        #     try:
        #         joint_positions = list(
        #             seed_state.get_joint_group_positions(group_name)
        #         )
        #     except RuntimeError as exc:
        #         self.get_logger().warn(f"Failed to read IK solution joints: {exc}")
        #     best_metrics = [joint_positions,0,0]

        # best_joint_positions=joint_positions
        # joint_distance = best_metrics[0]
        # pos_error = best_metrics[1]
        # z_error = best_metrics[2]
        # if pos_error > 1e-3:
        #     self.get_logger().warn(
        #         f"Using approximate IK solution; xy deviation {pos_error:.3f} m."
        #     )
        # # if z_error > 1e-3 and best_pose is not None:
        # #     self.get_logger().debug(
        # #         "Chosen IK solution deviates along Z by %.3f m (target range %.3f-%.3f m).",
        # #         z_error,
        # #         z_min,
        # #         z_max,
        # #     )

        # # if joint_distance < 1e-6:
        # #     self.get_logger().info("Already at the closest joint configuration; no motion required.")
        # #     return True

        # target_state = copy.deepcopy(current_state)
        # try:
        #     target_state.set_joint_group_positions(group_name, best_joint_positions)
        # except RuntimeError as exc:
        #     self.get_logger().error(f"Failed to apply target joint positions: {exc}")
        #     return False
        # target_state.update()

        # self.planning_component.set_start_state_to_current_state()
        # self.planning_component.set_goal_state(robot_state=target_state)

        # joint_str = ", ".join(f"{v:.4f}" for v in best_joint_positions)
       

        plan_result = self.planning_component.plan()
        if not plan_result:
            self.get_logger().warn("Planning failed for the computed joint target.")
            return False


        try:
            execution_result = self.moveit.execute(plan_result.trajectory, controllers=[])
        except RuntimeError as exc:
            self.get_logger().error(f"Trajectory execution threw an exception: {exc}")
            return False

        if isinstance(execution_result, bool) and not execution_result:
            self.get_logger().error("MoveIt reported failure when starting execution.")
            return False
        self.wait_until_manipulator_stopped()
        manager = self.moveit.get_trajectory_execution_manager()
        if manager is not None:
            try:
                wait_ok = manager.wait_for_execution()
                if isinstance(wait_ok, bool) and not wait_ok:
                    self.get_logger().warn("Trajectory execution did not complete cleanly.")
            except RuntimeError as exc:
                self.get_logger().warn(f"Failed to wait for execution completion: {exc}")

        return True



def main():
    rclpy.init()
    node = ToolTipPoseMoveItPy()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
