#!/usr/bin/env python3
# SPDX-License-Identifier: MIT

import copy
import math
import time
from typing import List, Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose, PoseStamped

from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent, PlanningSceneMonitor, PlanRequestParameters

import os
import yaml

from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from yolo_msgs.msg import Detection, DetectionArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import Image as ImageMsg, CameraInfo

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

# ──────────────────────────────────────────────────────────────────────────────
# Gripper orientation constants (tuned for this robot; pitch ≈ π/2 = pointing
# down, roll_fixed is a wrist alignment offset).
_GRASP_PITCH = 1.512
_GRASP_ROLL_FIXED = 2.619
# ──────────────────────────────────────────────────────────────────────────────


class ToolTipPoseMoveItPy(Node):
    """
    Manipulator demo node.

    Uses YOLO segmentation masks to build an object point cloud, generates
    top-down grasp hypotheses with multiple yaw orientations, validates each
    candidate with IK + collision checking via MoveIt, and executes the best
    valid grasp for the /pick_place_teddy_to_container service.
    """

    def __init__(self):
        super().__init__("tool_tip_pose_moveitpy")

        # ── Core parameters ──────────────────────────────────────────────
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
                description="Path to YAML file containing static collision objects",
            ),
        )

        for p in ["x", "y", "z", "roll", "pitch", "yaw"]:
            self.declare_parameter(p, None)

        # ── MoveItPy ─────────────────────────────────────────────────────
        self.moveit = MoveItPy(node_name="moveit_py")
        group_name = "arm"
        self.planning_component = self.moveit.get_planning_component(group_name)
        self.arm_group_name = group_name
        self.eef_link_name = "grasp_link"
        self.scene_monitor = self.moveit.get_planning_scene_monitor()

        # ── Streaming pose commands ───────────────────────────────────────
        latest_qos = qos_profile_sensor_data
        latest_qos.depth = 1
        self.sub = self.create_subscription(
            PoseStamped, "move_arm_position", self.on_target_msg, latest_qos
        )

        # One-shot via params
        vals = [self._get_optional_float(p) for p in ["x", "y", "z", "roll", "pitch", "yaw"]]
        if all(v is not None for v in vals):
            self.get_logger().info("Received one-shot target via parameters.")
            self.move_to_rpy(vals)

        self.get_logger().info("tool_tip_pose_moveitpy ready.")
        self.load_scene()

        # ── Gripper ───────────────────────────────────────────────────────
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", 0.92)
        self.declare_parameter("gripper_max_effort", 40.0)
        self.declare_parameter("gripper_action_timeout_sec", 20.0)
        self.declare_parameter("gripper_wait_server_sec", 5.0)
        self.declare_parameter(
            "gripper_action_name", "/dh_ag95_gripper_controller/gripper_cmd"
        )

        gripper_action_name = str(self.get_parameter("gripper_action_name").value)
        self.get_logger().info(f"Connecting to gripper action: {gripper_action_name}")
        self.gripper_client = ActionClient(self, GripperCommand, gripper_action_name)

        self.open_srv = self.create_service(Trigger, "open_gripper", self._on_open_gripper)
        self.close_srv = self.create_service(
            Trigger, "close_gripper", self._on_close_gripper
        )

        # ── YOLO detections ───────────────────────────────────────────────
        self.declare_parameter("yolo_detections_topic", "/yolo/detections_3d")
        self.declare_parameter("pick_object_name", "teddy bear")
        self.declare_parameter("place_container_name", "blue container")
        self.declare_parameter("object_freshness_sec", 2.0)

        # Grasp tuning parameters
        self.declare_parameter("approach_height", 0.18)   # metres above object top
        self.declare_parameter("grasp_z_offset", -0.02)     # fine-tune grasp depth
        self.declare_parameter(
            "depth_image_topic",
            "/rozum/D455_1/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter(
            "depth_info_topic",
            "/rozum/D455_1/aligned_depth_to_color/camera_info",
        )

        # Internal state
        self._last_seen_objects: dict = {}

        topic = self.get_parameter("yolo_detections_topic").value
        self._yolo_sub = self.create_subscription(
            DetectionArray, topic, self._on_yolo_detections, 10
        )

        self.pick_place_srv = self.create_service(
            Trigger, "pick_place_teddy_to_container", self._on_pick_and_place
        )

        self.declare_parameter(
            "home_joint_positions",
            [0.0, math.radians(90.0), 0.0, 0.0, 0.0, 0.0],
        )
        self.home_srv = self.create_service(Trigger, "go_home", self._on_go_home)

        # ── Depth + camera-info subscriptions ─────────────────────────────
        self._cv_bridge = CvBridge()
        self._latest_depth_image: Optional[np.ndarray] = None
        self._latest_cam_info: Optional[CameraInfo] = None

        _sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._depth_sub = self.create_subscription(
            ImageMsg,
            self.get_parameter("depth_image_topic").value,
            self._on_depth_image,
            _sensor_qos,
        )
        self._cam_info_sub = self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_topic").value,
            self._on_cam_info,
            _sensor_qos,
        )

        # ── TF listener (camera → base_link) ─────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    # ═════════════════════════════════════════════════════════════════════
    # Sensor callbacks
    # ═════════════════════════════════════════════════════════════════════

    def _on_depth_image(self, msg: ImageMsg) -> None:
        try:
            self._latest_depth_image = self._cv_bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
        except Exception as e:
            self.get_logger().warn(f"Depth image conversion failed: {e}")

    def _on_cam_info(self, msg: CameraInfo) -> None:
        self._latest_cam_info = msg

    def _on_yolo_detections(self, msg: DetectionArray) -> None:
        now = self.get_clock().now().nanoseconds
        pick_name = self.get_parameter("pick_object_name").value.lower()
        place_name = self.get_parameter("place_container_name").value.lower()
        for det in msg.detections:
            name = (det.class_name or "").strip().lower()
            if name not in (pick_name, place_name):
                continue
            p = det.bbox3d.center.position
            self._last_seen_objects[name] = {
                "position": Point(x=p.x, y=p.y, z=p.z),
                "detection": det,
                "stamp_ns": now,
            }

    # ═════════════════════════════════════════════════════════════════════
    # Point cloud from segmentation mask
    # ═════════════════════════════════════════════════════════════════════
    def _build_pointcloud_base_link(
        self, detection: Detection
    ) -> Optional[np.ndarray]:

        depth_image = self._latest_depth_image
        cam_info = self._latest_cam_info

        if depth_image is None or cam_info is None:
            self.get_logger().warn("No depth / camera_info available for point cloud")
            return None

        if not detection.mask.data:
            self.get_logger().warn("Detection has no segmentation mask; skipping point cloud")
            return None

        H, W = depth_image.shape[:2]
        k = cam_info.k
        fx, fy, cx, cy = k[0], k[4], k[2], k[5]

        # --- Build mask ---
        boundary = np.array(
            [[int(pt.x), int(pt.y)] for pt in detection.mask.data],
            dtype=np.int32
        )
        poly_mask = np.zeros((H, W), dtype=np.uint8)
        cv2.fillPoly(poly_mask, [boundary], 255)

        ys, xs = np.where(poly_mask > 0)
        if len(xs) == 0:
            self.get_logger().warn("Mask yielded no pixels")
            return None

        # --- Depth handling ---
        z = depth_image[ys, xs].astype(np.float64)

        # Detect units automatically (RealSense in ROS2 is usually meters)
        if z.max() > 100:  # likely mm
            z = z / 1000.0

        valid = np.isfinite(z) & (z > 0.01)
        xs, ys, z = xs[valid], ys[valid], z[valid]

        if len(xs) == 0:
            self.get_logger().warn("No valid depth pixels under mask")
            return None

        # --- Back-project ---
        X = (xs - cx) * z / fx
        Y = (ys - cy) * z / fy
        pts_cam = np.column_stack((X, Y, z))  # (N, 3)

        # --- TF lookup (FIX: use correct timestamp) ---
        try:
            tf_st = self._tf_buffer.lookup_transform(
                "base_link",
                cam_info.header.frame_id,
                cam_info.header.stamp,  # ✅ IMPORTANT FIX
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as e:
            self.get_logger().error(
                f"TF lookup failed ({cam_info.header.frame_id} → base_link): {e}"
            )
            return None

        # --- Use TF matrix instead of manual quaternion ---
        import tf_transformations as tft

        trans = tf_st.transform.translation
        rot = tf_st.transform.rotation

        T = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        T[0:3, 3] = [trans.x, trans.y, trans.z]

        # --- Transform points ---
        pts_h = np.hstack((pts_cam, np.ones((pts_cam.shape[0], 1))))
        pts_base = (T @ pts_h.T).T[:, :3]

        self.get_logger().info(
            f"Point cloud built: {len(pts_base)} points "
            f"(frame: {cam_info.header.frame_id} → base_link)"
        )

        return pts_base

    @staticmethod
    def _rotate_points(pts: np.ndarray, q: np.ndarray) -> np.ndarray:
        """Rotate (N,3) array by quaternion q=[w,x,y,z] using rotation matrix."""
        w, x, y, z = q
        R = np.array([
            [1 - 2*(y**2 + z**2),  2*(x*y - w*z),      2*(x*z + w*y)     ],
            [2*(x*y + w*z),         1 - 2*(x**2 + z**2), 2*(y*z - w*x)    ],
            [2*(x*z - w*y),         2*(y*z + w*x),       1 - 2*(x**2 + y**2)],
        ])
        return pts @ R.T

    # ═════════════════════════════════════════════════════════════════════
    # Grasp hypothesis generation
    # ═════════════════════════════════════════════════════════════════════

    def _generate_grasp_hypotheses(
        self, pts: np.ndarray, bbox3d
    ) -> List[Tuple[PoseStamped, PoseStamped]]:
        """
        Generate top-down grasp pose pairs (pre_grasp, grasp) from the
        object point cloud.  Twelve yaw orientations are sampled at 30°
        increments.

        pre_grasp: approach clearance above object top
        grasp:     gripper centre at object point-cloud centroid Z
        """
        approach_height = float(self.get_parameter("approach_height").value)
        grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)

        # Remove Z outliers (keep within 2.5σ of median)
        z = pts[:, 2]
        z_med = np.median(z)
        z_std = max(float(np.std(z)), 1e-4)
        pts_clean = pts[np.abs(z - z_med) < 2.5 * z_std]
        if len(pts_clean) == 0:
            pts_clean = pts

        cx = float(np.mean(pts_clean[:, 0]))
        cy = float(np.mean(pts_clean[:, 1]))
        z_top = float(np.percentile(pts_clean[:, 2], 90))
        z_center = float(np.mean(pts_clean[:, 2]))

        pre_z = z_top + approach_height
        grp_z = z_center + grasp_z_offset

        self.get_logger().info(
            f"Object centroid: ({cx:.3f}, {cy:.3f}), "
            f"z_center={z_center:.3f}, z_top={z_top:.3f}, "
            f"pre_grasp_z={pre_z:.3f}, grasp_z={grp_z:.3f}"
        )

        return self._build_pose_pairs(cx, cy, pre_z, grp_z, n_yaw=36)

    def _generate_grasp_hypotheses_from_bbox3d(
        self, pos, size
    ) -> List[Tuple[PoseStamped, PoseStamped]]:
        """Fallback: hypotheses from bbox3d centre/size when no mask is available."""
        approach_height = float(self.get_parameter("approach_height").value)
        grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)

        cx, cy = pos.x, pos.y
        z_top = pos.z + size.z / 2.0
        grp_z = pos.z + grasp_z_offset
        pre_z = z_top + approach_height

        self.get_logger().info(
            f"Bbox3d fallback: centroid ({cx:.3f}, {cy:.3f}), "
            f"grasp_z={grp_z:.3f}, pre_grasp_z={pre_z:.3f}"
        )
        return self._build_pose_pairs(cx, cy, pre_z, grp_z, n_yaw=12)

    @staticmethod
    def _build_pose_pairs(
        cx: float, cy: float, pre_z: float, grp_z: float, n_yaw: int = 12
    ) -> List[Tuple[PoseStamped, PoseStamped]]:
        """Build (pre_grasp, grasp) PoseStamped pairs for each yaw candidate."""
        hypotheses = []

        # Define ranges (radians)
        yaw_min = math.radians(150)
        yaw_max = math.radians(210)

        pitch_min = math.radians(60)
        pitch_max = math.radians(120)

        for i in range(5):
            for j in range(5):

                # Interpolate within range
                yaw = yaw_min + (yaw_max - yaw_min) * i / (5 - 1)
                pitch = pitch_min + (pitch_max - pitch_min) * j / (5 - 1)

                q = quaternion_from_euler(yaw, pitch, _GRASP_ROLL_FIXED)

                def _make_pose(z):
                    ps = PoseStamped()
                    ps.header.frame_id = "base_link"
                    ps.pose.position.x = cx
                    ps.pose.position.y = cy
                    ps.pose.position.z = z
                    ps.pose.orientation.x = q[0]
                    ps.pose.orientation.y = q[1]
                    ps.pose.orientation.z = q[2]
                    ps.pose.orientation.w = q[3]
                    return ps

                hypotheses.append((_make_pose(pre_z), _make_pose(grp_z)))

        return hypotheses

    # ═════════════════════════════════════════════════════════════════════
    # IK + collision validation
    # ═════════════════════════════════════════════════════════════════════

    def _check_sequential_ik(
        self, pre_grasp_pose: Pose, grasp_pose: Pose
    ) -> bool:
        """
        Check that:
          1. pre_grasp_pose has a collision-free IK solution from the current state.
          2. grasp_pose has a collision-free IK solution seeded from the pre-grasp
             joint configuration (sequential reachability).
        Returns True only if both checks pass.
        """
        group = self.arm_group_name
        eef = self.eef_link_name
        try:
            with self.scene_monitor.read_only() as scene:
                current = copy.deepcopy(scene.current_state)
            current.update()

            # ── Check pre-grasp ──────────────────────────────────────────
            pre_state = copy.deepcopy(current)
            if not pre_state.set_from_ik(group, pre_grasp_pose, tip_name=eef, timeout=0.2):
                return False
            pre_state.update()
            with self.scene_monitor.read_only() as scene:
                if scene.is_state_colliding(pre_state, group, False):
                    return False

            # ── Check grasp seeded from pre-grasp state ──────────────────
            grp_state = copy.deepcopy(pre_state)
            if not grp_state.set_from_ik(group, grasp_pose, tip_name=eef, timeout=0.2):
                return False
            grp_state.update()
            with self.scene_monitor.read_only() as scene:
                if scene.is_state_colliding(grp_state, group, False):
                    return False

            return True

        except Exception as e:
            self.get_logger().debug(f"Sequential IK check error: {e}")
            return False

    def _check_ik_collision_free(self, pose: Pose) -> bool:
        """Single-pose IK + collision check."""
        group = self.arm_group_name
        eef = self.eef_link_name
        try:
            with self.scene_monitor.read_only() as scene:
                current = copy.deepcopy(scene.current_state)
            current.update()
            test = copy.deepcopy(current)
            if not test.set_from_ik(group, pose, tip_name=eef, timeout=0.2):
                return False
            test.update()
            with self.scene_monitor.read_only() as scene:
                return not scene.is_state_colliding(test, group, False)
        except Exception as e:
            self.get_logger().debug(f"IK check error: {e}")
            return False

    def _rank_grasp_hypotheses(
        self,
        hypotheses: List[Tuple[PoseStamped, PoseStamped]],
    ) -> List[Tuple[int, PoseStamped, PoseStamped]]:
        """
        Sort hypotheses by IK+collision validity (fast, no planning calls).
        IK-valid ones come first; the rest are appended as fallback.
        Returns list of (original_idx, pre_grasp_pose, grasp_pose).
        """
        valid: List[Tuple[int, PoseStamped, PoseStamped]] = []
        fallback: List[Tuple[int, PoseStamped, PoseStamped]] = []
        for i, (pre, grp) in enumerate(hypotheses):
            if self._check_sequential_ik(pre.pose, grp.pose):
                valid.append((i, pre, grp))
            else:
                fallback.append((i, pre, grp))
        self.get_logger().info(
            f"IK pre-check: {len(valid)}/{len(hypotheses)} hypotheses passed"
        )
        return valid + fallback

    # ═════════════════════════════════════════════════════════════════════
    # Motion execution
    # ═════════════════════════════════════════════════════════════════════

    def _execute_trajectory(self, trajectory) -> bool:
        """Execute a planned trajectory. Returns True on success."""
        try:
            execution_result = self.moveit.execute(trajectory, controllers=[])
        except RuntimeError as exc:
            self.get_logger().error(f"Execution error: {exc}")
            return False
        if isinstance(execution_result, bool) and not execution_result:
            self.get_logger().error("Execution reported failure")
            return False
        self.wait_until_manipulator_stopped()
        manager = self.moveit.get_trajectory_execution_manager()
        if manager is not None:
            try:
                manager.wait_for_execution()
            except RuntimeError:
                pass
        return True

    def _diagnose_pose(self, pose_stamped: PoseStamped, label: str = "") -> None:
        """Solve IK for a pose, log joint angles, and check scene collision."""
        group = self.arm_group_name
        eef = self.eef_link_name
        p = pose_stamped.pose.position
        tag = f"[{label}] " if label else ""
        try:
            with self.scene_monitor.read_only() as scene:
                current = copy.deepcopy(scene.current_state)
            current.update()
            candidate = copy.deepcopy(current)
            ik_ok = candidate.set_from_ik(
                group, pose_stamped.pose, tip_name=eef, timeout=0.5
            )
            if not ik_ok:
                self.get_logger().warn(
                    f"{tag}IK FAILED — pose ({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
                    "has no reachable solution from current state"
                )
                return
            candidate.update()
            joints = candidate.get_joint_group_positions(group)
            joints_deg = [round(math.degrees(j), 2) for j in joints]
            self.get_logger().warn(
                f"{tag}IK OK  pose ({p.x:.3f},{p.y:.3f},{p.z:.3f}) → "
                f"joints (deg) {joints_deg}"
            )
            # verbose=True makes MoveIt print the colliding link pairs to stdout
            with self.scene_monitor.read_only() as scene:
                colliding = scene.is_state_colliding(candidate, group, True)
            if colliding:
                self.get_logger().warn(
                    f"{tag}IK solution IS IN COLLISION — see C++ collision "
                    "pair output above for details"
                )
            else:
                self.get_logger().warn(
                    f"{tag}IK solution is collision-free in Python scene check"
                )
        except Exception as exc:
            self.get_logger().error(f"{tag}Diagnosis error: {exc}")

    def _try_plan_and_execute_pilz_ptp(self, pose_stamped: PoseStamped) -> bool:
        """Plan with Pilz PTP (deterministic, no goal-tree sampling).

        Pilz PTP solves IK exactly once using the current robot state as a
        seed, then interpolates in joint space.  There is no random IK sampling
        and no 'goal tree', so it never hits the 'Unable to sample any valid
        states for goal tree' failure that OMPL encounters in constrained
        workspaces.
        """
        try:
            params = PlanRequestParameters(self.moveit, self.arm_group_name)
            params.planning_pipeline = "pilz_industrial_motion_planner"
            params.planner_id = "PTP"
            params.planning_time = 5.0
            params.planning_attempts = 1
            params.max_velocity_scaling_factor = 0.5
            params.max_acceleration_scaling_factor = 0.5

            self.planning_component.set_start_state_to_current_state()
            self.planning_component.set_goal_state(
                pose_stamped_msg=pose_stamped, pose_link=self.eef_link_name
            )
            plan_result = self.planning_component.plan(single_plan_parameters=params)
            if not plan_result:
                p = pose_stamped.pose.position
                self.get_logger().warn(
                    f"Pilz PTP: no plan found for ({p.x:.3f},{p.y:.3f},{p.z:.3f})"
                )
                self._diagnose_pose(pose_stamped, "Pilz-fail")
                return False
            return self._execute_trajectory(plan_result.trajectory)
        except Exception as exc:
            p = pose_stamped.pose.position
            self.get_logger().warn(
                f"Pilz PTP exception for ({p.x:.3f},{p.y:.3f},{p.z:.3f}): {exc}"
            )
            self._diagnose_pose(pose_stamped, "Pilz-exc")
            return False

    def _try_plan_and_execute_ompl(self, pose_stamped: PoseStamped) -> bool:
        """OMPL fallback with Cartesian pose goal."""
        self.planning_component.set_start_state_to_current_state()
        self.planning_component.set_goal_state(
            pose_stamped_msg=pose_stamped, pose_link=self.eef_link_name
        )
        plan_result = self.planning_component.plan()
        if not plan_result:
            p = pose_stamped.pose.position
            self.get_logger().warn(
                f"OMPL: planning failed for ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})"
            )
            return False
        return self._execute_trajectory(plan_result.trajectory)

    def _plan_and_execute_pose(self, pose_stamped: PoseStamped) -> bool:
        """Try Pilz PTP first; fall back to OMPL if it fails."""
        
        if self._try_plan_and_execute_pilz_ptp(pose_stamped):
            return True
        p = pose_stamped.pose.position
        self.get_logger().info(
            f"Pilz PTP failed for ({p.x:.3f},{p.y:.3f},{p.z:.3f}); trying OMPL"
        )
        return self._try_plan_and_execute_ompl(pose_stamped)

    # ═════════════════════════════════════════════════════════════════════
    # Object waiting helper
    # ═════════════════════════════════════════════════════════════════════

    def _wait_for_objects(
        self, required_names: list, timeout_sec: Optional[float]
    ) -> Optional[dict]:
        """
        Block until all required objects are freshly detected.
        Returns a dict {name_lower: {"position": Point, "detection": Detection,
        "stamp_ns": int}} or None on timeout.
        """
        start = self.get_clock().now().nanoseconds
        freshness_ns = int(
            float(self.get_parameter("object_freshness_sec").value) * 1e9
        )
        required_lower = [n.lower() for n in required_names]
        while True:
            now_ns = self.get_clock().now().nanoseconds
            found = {}
            for name in required_lower:
                info = self._last_seen_objects.get(name)
                if info and (now_ns - info["stamp_ns"]) <= freshness_ns:
                    found[name] = info
            if len(found) == len(required_lower):
                return found
            if timeout_sec is not None and (now_ns - start) > int(timeout_sec * 1e9):
                return None
            time.sleep(0.05)

    # ═════════════════════════════════════════════════════════════════════
    # Pick-and-place service
    # ═════════════════════════════════════════════════════════════════════

    def _on_pick_and_place(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        pick_name = self.get_parameter("pick_object_name").value
        place_name = self.get_parameter("place_container_name").value
        open_pos = float(self.get_parameter("gripper_open_position").value)
        close_pos = float(self.get_parameter("gripper_close_position").value)
        approach_h = float(self.get_parameter("approach_height").value)
        # Max planning attempts per phase — prevents runaway timeout spam
        MAX_GRASP_ATTEMPTS = 4
        MAX_PLACE_ATTEMPTS = 3

        # ── 1. Wait for fresh detections ──────────────────────────────────
        self.get_logger().info(
            f"Waiting for detections: '{pick_name}' and '{place_name}'..."
        )
        obj_data = self._wait_for_objects([pick_name, place_name], timeout_sec=10.0)
        if obj_data is None:
            response.success = False
            response.message = "Timeout waiting for YOLO detections"
            return response

        bear_det: Detection = obj_data[pick_name.lower()]["detection"]
        cont_det: Detection = obj_data[place_name.lower()]["detection"]

        # ── 2. Build point cloud from segmentation mask ───────────────────
        self.get_logger().info("Building point cloud from segmentation mask...")
        pts = self._build_pointcloud_base_link(bear_det)

        if pts is not None and len(pts) >= 10:
            hypotheses = self._generate_grasp_hypotheses(pts, bear_det.bbox3d)
        else:
            self.get_logger().warn("Insufficient point cloud; using bbox3d centroid")
            hypotheses = self._generate_grasp_hypotheses_from_bbox3d(
                bear_det.bbox3d.center.position, bear_det.bbox3d.size
            )

        # ── 3. Sort hypotheses (IK-valid first, no planning here) ─────────
        self.get_logger().info(
            f"Ranking {len(hypotheses)} hypotheses via fast IK check..."
        )
        ranked = self._rank_grasp_hypotheses(hypotheses)

        # ── 4. Open gripper once before starting hypothesis loop ──────────
        self.get_logger().info("Opening gripper...")
        self._send_gripper_goal(open_pos, wait_result=True)

        # ── 5. Hypothesis execution loop ─────────────────────────────────
        #   For each candidate: try plan→execute pre-grasp, then grasp.
        #   If planning fails for a candidate, skip to the next — do NOT
        #   abort the whole operation.  Gripper only closes on success.
        grasped = False
        used_pre_pose: Optional[PoseStamped] = None

        for attempt, (idx, pre, grp) in enumerate(ranked):
            if attempt >= MAX_GRASP_ATTEMPTS:
                self.get_logger().warn(
                    f"Reached max grasp attempts ({MAX_GRASP_ATTEMPTS}); aborting"
                )
                break

            self.get_logger().info(
                f"Grasp attempt {attempt + 1}/{MAX_GRASP_ATTEMPTS} "
                f"(hypothesis {idx}, yaw={idx * 30}°)"
            )

            # ── pre-grasp ────────────────────────────────────────────────
            if not self._plan_and_execute_pose(pre):
                self.get_logger().warn(
                    f"Hypothesis {idx}: pre-grasp planning/execution failed, trying next"
                )
                continue

            # ── descend to grasp ─────────────────────────────────────────
            if not self._plan_and_execute_pose(grp):
                self.get_logger().warn(
                    f"Hypothesis {idx}: grasp planning/execution failed; "
                    "returning to pre-grasp height"
                )
                self._plan_and_execute_pose(pre)   # safe retreat (non-fatal)
                continue

            grasped = True
            used_pre_pose = pre
            self.get_logger().info(f"Grasp pose reached (hypothesis {idx})")
            break

        if not grasped:
            response.success = False
            response.message = (
                f"Could not execute any of {min(len(ranked), MAX_GRASP_ATTEMPTS)} "
                "grasp hypotheses"
            )
            return response

        # ── 6. Close gripper ──────────────────────────────────────────────
        self.get_logger().info("Closing gripper...")
        self._send_gripper_goal(close_pos, wait_result=True)

        # ── 7. Lift object ────────────────────────────────────────────────
        assert used_pre_pose is not None
        lift_pose = copy.deepcopy(used_pre_pose)
        lift_pose.pose.position.z = used_pre_pose.pose.position.z + 0.05
        self.get_logger().info("Lifting object...")
        if not self._plan_and_execute_pose(lift_pose):
            self._plan_and_execute_pose(used_pre_pose)  # fall back to pre-grasp height

        # ── 8. Compute + rank placement hypotheses (hover above container) ─
        self.get_logger().info("Computing placement pose above container...")
        cont_pts = self._build_pointcloud_base_link(cont_det)
        if cont_pts is not None and len(cont_pts) >= 10:
            place_hypotheses = self._generate_grasp_hypotheses(
                cont_pts, cont_det.bbox3d
            )
        else:
            place_hypotheses = self._generate_grasp_hypotheses_from_bbox3d(
                cont_det.bbox3d.center.position, cont_det.bbox3d.size
            )

        cont_top_z = (
            cont_det.bbox3d.center.position.z + cont_det.bbox3d.size.z / 2.0
        )
        for pre_place, _ in place_hypotheses:
            pre_place.pose.position.z = cont_top_z + approach_h

        ranked_place = self._rank_grasp_hypotheses(place_hypotheses)

        # ── 9. Placement execution loop (hover) ───────────────────────────
        placed = False
        for attempt, (p_idx, pre_place, _) in enumerate(ranked_place):
            if attempt >= MAX_PLACE_ATTEMPTS:
                self.get_logger().warn(
                    f"Reached max placement attempts ({MAX_PLACE_ATTEMPTS})"
                )
                break

            self.get_logger().info(
                f"Placement attempt {attempt + 1}/{MAX_PLACE_ATTEMPTS} "
                f"(hypothesis {p_idx})"
            )
            if self._plan_and_execute_pose(pre_place):
                placed = True
                break
            self.get_logger().warn(
                f"Placement hypothesis {p_idx}: planning/execution failed"
            )

        if not placed:
            response.success = False
            response.message = "Could not reach any container placement pose"
            return response

        # ── 10. Descend to place (at container top + 2 cm) ────────────────
        self.get_logger().info("Descending to place pose...")
        cont_pts2 = self._build_pointcloud_base_link(cont_det)
        if cont_pts2 is not None and len(cont_pts2) >= 10:
            place_hypotheses2 = self._generate_grasp_hypotheses(
                cont_pts2, cont_det.bbox3d
            )
        else:
            place_hypotheses2 = self._generate_grasp_hypotheses_from_bbox3d(
                cont_det.bbox3d.center.position, cont_det.bbox3d.size
            )

        cont_top_z2 = (
            cont_det.bbox3d.center.position.z + cont_det.bbox3d.size.z / 2.0 + 0.02
        )
        for pre_place2, _ in place_hypotheses2:
            pre_place2.pose.position.z = cont_top_z2

        ranked_place2 = self._rank_grasp_hypotheses(place_hypotheses2)

        placed2 = False
        for attempt, (p_idx, pre_place2, _) in enumerate(ranked_place2):
            if attempt >= MAX_PLACE_ATTEMPTS:
                self.get_logger().warn(
                    f"Reached max placement attempts ({MAX_PLACE_ATTEMPTS})"
                )
                break

            self.get_logger().info(
                f"Placement attempt {attempt + 1}/{MAX_PLACE_ATTEMPTS} "
                f"(hypothesis {p_idx})"
            )
            if self._plan_and_execute_pose(pre_place2):
                placed2 = True
                break
            self.get_logger().warn(
                f"Placement hypothesis {p_idx}: planning/execution failed"
            )

        if not placed2:
            response.success = False
            response.message = "Could not reach any container placement pose"
            return response

        # ── 11. Release object ────────────────────────────────────────────
        self.get_logger().info("Releasing object...")
        self._send_gripper_goal(open_pos, wait_result=True)
        time.sleep(0.4)
        self.go_home()

        response.success = True
        response.message = "Pick and place completed successfully"
        return response

    # ═════════════════════════════════════════════════════════════════════
    # Home position
    # ═════════════════════════════════════════════════════════════════════

    def go_home(self) -> bool:
        """Move the arm to the home joint configuration (default: 0,90,0,0,0,0 deg).

        Uses Pilz PTP planner for a smooth, deterministic joint-space motion.
        Falls back to OMPL if Pilz fails.  Blocks until execution completes.
        Returns True on success.
        """
        home_joints = list(self.get_parameter("home_joint_positions").value)
        self.get_logger().info(
            f"go_home: target joints (rad) {[round(j, 4) for j in home_joints]}"
        )

        try:
            with self.scene_monitor.read_only() as scene:
                goal_state = copy.deepcopy(scene.current_state)
            goal_state.set_joint_group_positions(self.arm_group_name, home_joints)
            goal_state.update()
        except Exception as e:
            self.get_logger().error(f"go_home: failed to build goal state: {e}")
            return False

        # ── Try Pilz PTP ─────────────────────────────────────────────────
        try:
            params = PlanRequestParameters(self.moveit, self.arm_group_name)
            params.planning_pipeline = "pilz_industrial_motion_planner"
            params.planner_id = "PTP"
            params.planning_time = 5.0
            params.planning_attempts = 1
            params.max_velocity_scaling_factor = 0.5
            params.max_acceleration_scaling_factor = 0.5

            self.planning_component.set_start_state_to_current_state()
            self.planning_component.set_goal_state(robot_state=goal_state)
            plan_result = self.planning_component.plan(single_plan_parameters=params)
            if plan_result:
                return self._execute_trajectory(plan_result.trajectory)
            self.get_logger().warn("go_home: Pilz PTP failed, trying OMPL")
        except Exception as e:
            self.get_logger().warn(f"go_home: Pilz PTP exception: {e}, trying OMPL")

        # ── OMPL fallback ─────────────────────────────────────────────────
        try:
            self.planning_component.set_start_state_to_current_state()
            self.planning_component.set_goal_state(robot_state=goal_state)
            plan_result = self.planning_component.plan()
            if plan_result:
                return self._execute_trajectory(plan_result.trajectory)
            self.get_logger().error("go_home: OMPL planning also failed")
            return False
        except Exception as e:
            self.get_logger().error(f"go_home: OMPL exception: {e}")
            return False

    def _on_go_home(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        ok = self.go_home()
        response.success = ok
        response.message = "Home reached" if ok else "Failed to reach home position"
        return response

    # ═════════════════════════════════════════════════════════════════════
    # Scene loading
    # ═════════════════════════════════════════════════════════════════════

    def load_scene(self):
        """Load collision objects from YAML and apply to the planning scene."""
        yaml_path = self.get_parameter("scene_yaml_path").value or ""
        if not yaml_path:
            self.get_logger().warn("No 'scene_yaml_path' set; skipping scene load.")
            return

        yaml_path = os.path.expanduser(yaml_path)
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
            scene.current_state.update()

        self.get_logger().info(
            f"Loaded {added}/{len(objects)} collision objects from {yaml_path}."
        )

    def _make_collision_object_from_spec(self, spec: dict) -> CollisionObject:
        """Convert a YAML spec dict to a CollisionObject message."""
        if "id" not in spec:
            raise ValueError("Missing 'id' in object spec")
        if "type" not in spec:
            raise ValueError("Missing 'type' in object spec")
        if "dimensions" not in spec:
            raise ValueError("Missing 'dimensions' in object spec")

        co = CollisionObject()
        co.id = str(spec["id"])
        co.header.frame_id = spec.get(
            "frame_id", self.get_parameter("base_frame").value
        )

        typ = str(spec["type"]).upper()
        dims = [float(v) for v in spec["dimensions"]]

        prim = SolidPrimitive()
        if typ == "BOX":
            if len(dims) != 3:
                raise ValueError(f"BOX requires 3 dimensions, got {dims}")
            prim.type = SolidPrimitive.BOX
            prim.dimensions = dims
        elif typ == "CYLINDER":
            if len(dims) != 2:
                raise ValueError(f"CYLINDER requires 2 dimensions, got {dims}")
            prim.type = SolidPrimitive.CYLINDER
            prim.dimensions = dims
        elif typ == "SPHERE":
            if len(dims) != 1:
                raise ValueError(f"SPHERE requires 1 dimension, got {dims}")
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = dims
        else:
            raise ValueError(f"Unsupported primitive type '{typ}'")

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
            qx = float(q.get("x", 0.0))
            qy = float(q.get("y", 0.0))
            qz = float(q.get("z", 0.0))
            qw = float(q.get("w", 1.0))
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

    # ═════════════════════════════════════════════════════════════════════
    # Gripper helpers
    # ═════════════════════════════════════════════════════════════════════

    def _send_gripper_goal(
        self,
        position: float,
        wait_result: bool = False,
        timeout_sec: Optional[float] = None,
    ) -> bool:
        try:
            server_timeout = float(self.get_parameter("gripper_wait_server_sec").value)
        except Exception:
            server_timeout = 5.0
        if timeout_sec is None:
            try:
                timeout_sec = float(
                    self.get_parameter("gripper_action_timeout_sec").value
                )
            except Exception:
                timeout_sec = 20.0

        if not self.gripper_client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("Gripper action server not available.")
            return False

        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        try:
            goal.command.max_effort = float(
                self.get_parameter("gripper_max_effort").value
            )
        except Exception:
            goal.command.max_effort = 40.0

        try:
            send_future = self.gripper_client.send_goal_async(goal)
        except Exception as e:
            self.get_logger().error(f"Failed to send gripper goal: {e}")
            return False

        if not wait_result:
            def _on_goal_sent(fut):
                try:
                    handle = fut.result()
                    if not handle.accepted:
                        self.get_logger().warn("Gripper goal rejected")
                        return
                    result_future = handle.get_result_async()

                    def _on_result(_):
                        try:
                            result_future.result()
                            self.get_logger().info(
                                f"Gripper finished (pos={goal.command.position:.3f})"
                            )
                        except Exception as err:
                            self.get_logger().error(f"Gripper result error: {err}")

                    result_future.add_done_callback(_on_result)
                except Exception as err:
                    self.get_logger().error(f"Error after goal dispatch: {err}")

            send_future.add_done_callback(_on_goal_sent)
            return True

        # Blocking path
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
            self.get_logger().warn("Gripper goal rejected")
            return False

        result_future = handle.get_result_async()
        start_ns = self.get_clock().now().nanoseconds
        while not result_future.done():
            if (self.get_clock().now().nanoseconds - start_ns) > int(timeout_sec * 1e9):
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        if not result_future.done():
            self.get_logger().error("Timeout waiting for gripper result")
            return False
        try:
            result_future.result()
            return True
        except Exception as e:
            self.get_logger().error(f"Gripper result error: {e}")
            return False

    def _on_open_gripper(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        pos = float(self.get_parameter("gripper_open_position").value)
        ok = self._send_gripper_goal(pos)
        response.success = bool(ok)
        response.message = "open command sent" if ok else "failed to send open command"
        return response

    def _on_close_gripper(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        pos = float(self.get_parameter("gripper_close_position").value)
        ok = self._send_gripper_goal(pos)
        response.success = bool(ok)
        response.message = "close command sent" if ok else "failed to send close command"
        return response

    # ═════════════════════════════════════════════════════════════════════
    # Arm motion helpers (kept for move_arm_position subscription)
    # ═════════════════════════════════════════════════════════════════════

    def _get_optional_float(self, name: str) -> Optional[float]:
        val = self.get_parameter(name).value
        return None if val is None else float(val)

    def on_target_msg(self, msg: PoseStamped):
        self.move_to_rpy(msg)

    def is_manipulator_moving(self, tolerance: float = 1e-3) -> bool:
        try:
            with self.scene_monitor.read_only() as scene:
                joints1 = list(scene.current_state.get_joint_group_positions(self.arm_group_name))
            time.sleep(0.1)
            with self.scene_monitor.read_only() as scene:
                joints2 = list(scene.current_state.get_joint_group_positions(self.arm_group_name))
            return any(abs(a - b) > tolerance for a, b in zip(joints1, joints2))
        except Exception as e:
            self.get_logger().error(f"Failed to determine motion: {e}")
            return False

    def wait_until_manipulator_stopped(
        self,
        tolerance: float = 1e-3,
        stable_time: float = 0.5,
        timeout: float = 30.0,
    ) -> bool:
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
                    still_since = time.time()
                    last_moving = False
                elif still_since is not None and (time.time() - still_since) >= stable_time:
                    self.get_logger().info("Manipulator stopped.")
                    return True
            time.sleep(0.1)
        self.get_logger().warn("Timeout waiting for manipulator to stop.")
        return False

    def move_to_rpy(self, target_pose: PoseStamped):
        q = target_pose.pose.orientation
        roll = math.atan2(
            2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2)
        )
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = (
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
            + math.pi * 60 / 180
        )
        self.get_logger().info(f"move_to_rpy: roll={roll:.3f} pitch={pitch:.3f} yaw={yaw:.3f}")
        self.plan_xy_zrange_yaw(
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            z_min=target_pose.pose.position.z,
            z_max=target_pose.pose.position.z + 0.05,
            yaw=yaw,
        )

    def plan_xy_zrange_yaw(
        self,
        x: float,
        y: float,
        z_min: float = 0.12,
        z_max: float = 0.2,
        yaw: float = 0.0,
        xy_tol: float = 0.01,
        yaw_tol: float = math.radians(2),
        container: bool = False,
    ) -> bool:
        """Plan to a Cartesian pose (kept for move_arm_position subscriber)."""
        self.planning_component.set_start_state_to_current_state()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        quat = quaternion_from_euler(yaw, _GRASP_PITCH, _GRASP_ROLL_FIXED)
        pose_goal.pose.orientation.w = quat[3]
        pose_goal.pose.orientation.x = quat[0]
        pose_goal.pose.orientation.y = quat[1]
        pose_goal.pose.orientation.z = quat[2]
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z_min + 0.1
        self.planning_component.set_goal_state(
            pose_stamped_msg=pose_goal, pose_link="grasp_link"
        )
        plan_result = self.planning_component.plan()
        if not plan_result:
            self.get_logger().warn("plan_xy_zrange_yaw: planning failed.")
            return False
        try:
            execution_result = self.moveit.execute(plan_result.trajectory, controllers=[])
        except RuntimeError as exc:
            self.get_logger().error(f"Execution error: {exc}")
            return False
        if isinstance(execution_result, bool) and not execution_result:
            return False
        self.wait_until_manipulator_stopped()
        manager = self.moveit.get_trajectory_execution_manager()
        if manager is not None:
            try:
                manager.wait_for_execution()
            except RuntimeError:
                pass
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
