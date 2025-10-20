#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

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

from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

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


class HighLevelInterface(Node):
    """
    Move a manipulator to a desired tool-tip pose given as x y z roll pitch yaw.
    Uses MoveItPy (ROS 2 Jazzy+).
    """

    def __init__(self):
        super().__init__("high_level_manipulator_interface")

        # Parameters
        self.declare_parameter("planning_group", "manipulator")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("eef_link", "grasp_link")

        default_config_path = os.path.join(
            get_package_share_directory("high_level_manipulator_interface"),
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
        self.open_srv = self.create_service(Trigger, "/manipulator/open_gripper", self._on_open_gripper)
        self.close_srv = self.create_service(Trigger, "/manipulator/close_gripper", self._on_close_gripper)
        self.close_srv = self.create_service(Trigger, "/manipulator/go_home", self._on_go_home)
        self.close_srv = self.create_service(Trigger, "/manipulator/relax", self._on_relax)
        self.close_srv = self.create_service(Trigger, "/manipulator/freeze", self._on_freeze)
        self.close_srv = self.create_service(Trigger, "/manipulator/stop", self._on_stop)

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
            bear_p.x-0.07,
            bear_p.y,
            z_min=bear_p.z+0.1,
            z_max=bear_p.z + 0.2,
        )
        time.sleep(3)

        ok = self.plan_xy_zrange_yaw(
            bear_p.x-0.07,
            bear_p.y,
            z_min=bear_p.z,
            z_max=bear_p.z + 0.04,
        )
        time.sleep(4)
        if not ok:
            response.success = False
            response.message = "failed to reach teddy bear"
            return response

        if not self._send_gripper_goal(float(self.get_parameter("gripper_close_position").value), wait_result=False):
            response.success = False
            response.message = "failed to close gripper"
            return response
        time.sleep(3)
        

        # Move above container and release
        self.get_logger().info(
            f"Moving above container at ({cont_p.x:.3f}, {cont_p.y:.3f}, {cont_p.z:.3f})"
        )
        ok = self.plan_xy_zrange_yaw(
            bear_p.x-0.07,
            bear_p.y,
            z_min=bear_p.z+0.8,
            z_max=bear_p.z + 0.5,
            container =True
        )
        ok = self.plan_xy_zrange_yaw(
            cont_p.x-0.05,
            cont_p.y,
            z_min=cont_p.z ,
            z_max=cont_p.z + 0.5,
            container =True
        )
        time.sleep(3)

        if not ok:
            response.success = False
            response.message = "failed to reach container"
            return response

        if not self._send_gripper_goal(float(self.get_parameter("gripper_open_position").value), wait_result=False):
            response.success = False
            response.message = "failed to open gripper"
            return response
        time.sleep(3)
        

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
    yaw: float = 0.0,                  # radians
    xy_tol: float = 0.01,              # half-width in x,y (m)
    yaw_tol: float = math.radians(2),  # +/- yaw tolerance (rad)
    container =False
    ):
        frame = self.get_parameter("base_frame").value
        link  = "grasp_link"

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

        # --- Orientation constraint: fix yaw only; free pitch/yaw
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = link
        qx, qy, qz, qw = quaternion_from_euler( yaw, 1.512, 2.619 )
        oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w = qx, qy, qz, qw
        oc.absolute_x_axis_tolerance = 0.5       # free roll
        oc.absolute_y_axis_tolerance = 0.5       # free pitch
        oc.absolute_z_axis_tolerance = 0.5      # constrain yaw
        oc.weight = 1.0
        if container:
            oc.absolute_x_axis_tolerance = 3.14       # free roll
            oc.absolute_y_axis_tolerance = 1.4       # free pitch
            oc.absolute_z_axis_tolerance = 3.15      # constrain yaw
        else:
            oc.absolute_x_axis_tolerance = 0.5       # free roll
            oc.absolute_y_axis_tolerance = 0.5       # free pitch
            oc.absolute_z_axis_tolerance = 0.5      # constrain yaw
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
            return True
        else:
            self.get_logger().warn("Planning failed.")
            return False



def main():
    rclpy.init()
    node = HighLevelInterface()
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
