#!/usr/bin/env python3

import os
import math
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from std_srvs.srv import Trigger
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import rclpy
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose, PoseStamped


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Compute quaternion (x, y, z, w) from roll, pitch, yaw in radians.

    Implemented locally to avoid external dependencies.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quaternion_from_rotation_matrix(R: np.ndarray):
    """Compute quaternion (x, y, z, w) from a proper 3x3 rotation matrix.

    The quaternion represents the orientation of the child frame's axes (columns of R)
    expressed in the parent frame.
    """
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif (m00 > m11) and (m00 > m22):
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return (x, y, z, w)

# Helper quaternion math (x, y, z, w)
def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )

def q_conj(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def q_normalize(q):
    import math
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    return (0.0, 0.0, 0.0, 1.0) if n < 1e-12 else (x/n, y/n, z/n, w/n)

def q_apply(q, v):
    # rotate vector v by quaternion q
    vx, vy, vz = v
    x, y, z, w = q
    # q * (v,0)
    vqx = w*vx + y*vz - z*vy
    vqy = w*vy + z*vx - x*vz
    vqz = w*vz + x*vy - y*vx
    vqw = -x*vx - y*vy - z*vz
    # (result) * q_conj
    cx, cy, cz, cw = -x, -y, -z, w
    rx = vqw*cx + vqx*cw + vqy*cz - vqz*cy
    ry = vqw*cy - vqx*cz + vqy*cw + vqz*cx
    rz = vqw*cz + vqx*cy - vqy*cx + vqz*cw
    return (rx, ry, rz)

class StaticTFYamlNode(Node):
    def __init__(self):
        super().__init__("static_tf_yaml")

        default_config_path = os.path.join(
            get_package_share_directory("realsense_to_scene_transform"),
            "config",
            "static_tf.yaml",
        )

        self.declare_parameter(
            name="config_file",
            value=default_config_path,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Path to YAML file containing static transforms",
            ),
        )

        self._broadcaster = StaticTransformBroadcaster(self)

        config_path = self.get_parameter("config_file").get_parameter_value().string_value

        try:
            with open(config_path, "r") as f:
                config = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().error(f"Failed to read YAML config: {config_path}: {exc}")
            raise

        self._config_path = config_path
        self._config_data = config

        # Parse auxiliary sections for convenience
        self._aruco_config = self._parse_aruco_section(self._config_data)
        self._arena_config = self._parse_arena_section(self._config_data)
        if self._aruco_config:
            self.get_logger().info(f"Loaded {len(self._aruco_config)} aruco entries from YAML")
        if self._arena_config:
            self.get_logger().info(f"Loaded arena config keys: {list(self._arena_config.keys())}")

        transforms = self._config_data.get("transforms", [])
        if not isinstance(transforms, list) or len(transforms) == 0:
            self.get_logger().warn("No transforms found in YAML; nothing to publish.")
            transforms = []

        to_send = []
        for idx, tf_cfg in enumerate(transforms):
            try:
                parent = tf_cfg["parent"]
                child = tf_cfg["child"]
            except KeyError as exc:
                self.get_logger().error(f"Transform #{idx} missing required key: {exc}")
                continue

            translation = tf_cfg.get("translation", [0.0, 0.0, 0.0])
            if not (isinstance(translation, (list, tuple)) and len(translation) == 3):
                self.get_logger().error(
                    f"Transform '{parent}->{child}' has invalid translation; expected 3-element list"
                )
                continue

            quat = None
            if "rotation_quat" in tf_cfg:
                q = tf_cfg.get("rotation_quat")
                if isinstance(q, (list, tuple)) and len(q) == 4:
                    quat = tuple(float(v) for v in q)
                else:
                    self.get_logger().error(
                        f"Transform '{parent}->{child}' rotation_quat must be [x, y, z, w]"
                    )
                    continue
            elif "rotation_rpy" in tf_cfg:
                rpy = tf_cfg.get("rotation_rpy")
                if isinstance(rpy, (list, tuple)) and len(rpy) == 3:
                    roll, pitch, yaw = [float(v) for v in rpy]
                    quat = quaternion_from_euler(roll, pitch, yaw)
                else:
                    self.get_logger().error(
                        f"Transform '{parent}->{child}' rotation_rpy must be [roll, pitch, yaw]"
                    )
                    continue
            else:
                # default: identity rotation
                quat = (0.0, 0.0, 0.0, 1.0)

            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = parent
            msg.child_frame_id = child
            msg.transform.translation.x = float(translation[0])
            msg.transform.translation.y = float(translation[1])
            msg.transform.translation.z = float(translation[2])
            msg.transform.rotation.x = quat[0]
            msg.transform.rotation.y = quat[1]
            msg.transform.rotation.z = quat[2]
            msg.transform.rotation.w = quat[3]

            to_send.append(msg)

        if to_send:
            self._broadcaster.sendTransform(to_send)
            self.get_logger().info(f"Published {len(to_send)} static transform(s) from YAML.")
        else:
            self.get_logger().warn("No valid static transforms to publish.")

        # Parameters and subscriptions for calibration
        self.declare_parameter(
            name="aruco_poses_topic",
            value="/aruco/markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to subscribe for ArUco poses (ArucoMarkers)",
            ),
        )
        self.declare_parameter(
            name="calibration_parent_frame",
            value="camera_color_optical_frame",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Parent frame for calibrated transform",
            ),
        )
        self.declare_parameter(
            name="calibration_child_frame",
            value="world",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Child frame for calibrated transform",
            ),
        )

        self._aruco_topic = self.get_parameter("aruco_poses_topic").get_parameter_value().string_value
        self._parent_frame = self.get_parameter("calibration_parent_frame").get_parameter_value().string_value
        self._child_frame = self.get_parameter("calibration_child_frame").get_parameter_value().string_value

        self._calibration_active = False
        self._poses_buffer = []
        self._target_count = 100
        latest_qos = qos_profile_sensor_data
        latest_qos.depth = 1
        self._aruco_sub = self.create_subscription(ArucoMarkers, self._aruco_topic, self._aruco_callback, latest_qos)
        

        # Service to trigger calibration
        self._srv = self.create_service(Trigger, "calibrate_realsense_position", self._on_calibrate)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
    
        self._dyn_broadcaster = TransformBroadcaster(self)

        self.robot_commands = self.create_publisher(PoseStamped,'/move_arm_position', qos_profile=latest_qos)

    def _on_calibrate(self, request, response):
        if self._calibration_active:
            response.success = False
            response.message = "Calibration already in progress"
            return response

        self._calibration_active = True
        self._poses_buffer = []
        response.success = True
        response.message = f"Calibration started. Collecting {self._target_count} poses from {self._aruco_topic}"
        self.get_logger().info(response.message)
        return response

    def _aruco_callback(self, msg: ArucoMarkers):

        # get marker 3 pose and publish tranform from world to marker3


        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'aruco_marker_3'



        aruco_3_found = False
        aruco_3_pose = None
        aruco_3_orient = None
        for marker, pose in zip(msg.marker_ids, msg.poses):
            if marker == 3:
                aruco_3_found = True
                aruco_3_pose = pose.position
                aruco_3_orient = pose.orientation
                break


        if aruco_3_found:

            # get transform from camera_link  to base_link
            # apply it to aruco_marker_3 pose and publish it 

            

        

            base_frame = "base_link"
            camera_frame = self._parent_frame  # e.g. "camera_color_optical_frame"
            ok = False
            try:
                tf_bc = self._tf_buffer.lookup_transform(
                    base_frame,             # target
                    camera_frame,           # source
                    rclpy.time.Time()       # latest available
                )
                ok = True
            except Exception as ex:
                self.get_logger().warn(
                    f"TF lookup failed {camera_frame} -> {base_frame}: {ex}"
                )
                

            
            if ok:
                # Extract T_base_camera
                t_bc = (
                    tf_bc.transform.translation.x,
                    tf_bc.transform.translation.y,
                    tf_bc.transform.translation.z,
                )
                q_bc = (
                    tf_bc.transform.rotation.x,
                    tf_bc.transform.rotation.y,
                    tf_bc.transform.rotation.z,
                    tf_bc.transform.rotation.w,
                )
                q_bc = q_normalize(q_bc)

                # Pose of marker 3 in camera frame
                p_cm = (aruco_3_pose.x, aruco_3_pose.y, aruco_3_pose.z)
                if aruco_3_orient is not None:
                    q_cm = (
                        aruco_3_orient.x,
                        aruco_3_orient.y,
                        aruco_3_orient.z,
                        aruco_3_orient.w,
                    )
                else:
                    q_cm = (0.0, 0.0, 0.0, 1.0)
                # Compose: T_base_marker = T_base_camera ∘ T_camera_marker
                p_bm = tuple(tb + rb for tb, rb in zip(t_bc, q_apply(q_bc, p_cm)))
                q_bm = q_normalize(q_mult(q_bc, q_cm))

                # Fill and publish transform (base_link -> aruco_marker_3)
                t.transform.translation.x = float(p_bm[0])
                t.transform.translation.y = float(p_bm[1])
                t.transform.translation.z = float(p_bm[2])
                t.transform.rotation.x = float(q_bm[0])
                t.transform.rotation.y = float(q_bm[1])
                t.transform.rotation.z = float(q_bm[2])
                t.transform.rotation.w = float(q_bm[3])

                self._dyn_broadcaster.sendTransform(t)

                pose_msg = PoseStamped()

                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'base_link'

                pose_msg.pose.position.x = float(p_bm[0])
                pose_msg.pose.position.y = float(p_bm[1])
                pose_msg.pose.position.z = 0.45

                
                pose_msg.pose.orientation.x = float(q_bm[0])
                pose_msg.pose.orientation.y = float(q_bm[1])
                pose_msg.pose.orientation.z = float(q_bm[2])
                pose_msg.pose.orientation.w = float(q_bm[3])

                self.robot_commands.publish(pose_msg)
                self.get_logger().info("Send command to the robot")



        if not self._calibration_active:
            return
        # Append all poses in the incoming array; if empty, ignore
        if len(msg.poses) == 0:
            return
        if len(self._poses_buffer) < self._target_count:
            self._poses_buffer.append(msg)
            if len(self._poses_buffer) >= self._target_count:
                self._poses_buffer = self._poses_buffer[: self._target_count]
                self._calibration_callback()
                

    def _calibration_callback(self):
        if not self._calibration_active:
            return
        
        if len(self._poses_buffer) < self._target_count:
            return

        # We have enough poses; run user-supplied derivation
        try:
            translation, quaternion = self._derive_transform_from_aruco(self._poses_buffer, self._config_data)
        except Exception as exc:
            self.get_logger().error(f"Calibration derivation failed: {exc}")
            self._calibration_active = False
            self._poses_buffer = []
            return

        if translation is None or quaternion is None:
            self.get_logger().warn("Calibration derivation returned None. Skipping update.")
            self._calibration_active = False
            self._poses_buffer = []
            return

        # Update YAML and re-publish
        self._update_yaml_transform(self._parent_frame, self._child_frame, translation, quaternion)
        self._publish_single_static(self._parent_frame, self._child_frame, translation, quaternion)

        self.get_logger().info("Calibration completed and static transform updated.")
        self._calibration_active = False
        self._poses_buffer = []

    def _publish_single_static(self, parent: str, child: str, translation, quaternion):
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.transform.translation.x = float(translation[0])
        msg.transform.translation.y = float(translation[1])
        msg.transform.translation.z = float(translation[2])
        msg.transform.rotation.x = float(quaternion[0])
        msg.transform.rotation.y = float(quaternion[1])
        msg.transform.rotation.z = float(quaternion[2])
        msg.transform.rotation.w = float(quaternion[3])
        self._broadcaster.sendTransform([msg])

    def _update_yaml_transform(self, parent: str, child: str, translation, quaternion):
        # Ensure transforms list exists
        transforms = self._config_data.get("transforms")
        if not isinstance(transforms, list):
            transforms = []
            self._config_data["transforms"] = transforms

        # Find existing transform with same parent/child
        idx = None
        for i, t in enumerate(transforms):
            if t.get("parent") == parent and t.get("child") == child:
                idx = i
                break

        new_tf = {
            "parent": parent,
            "child": child,
            "translation": [float(translation[0]), float(translation[1]), float(translation[2])],
            "rotation_quat": [float(quaternion[0]), float(quaternion[1]), float(quaternion[2]), float(quaternion[3])],
        }

        if idx is None:
            transforms.append(new_tf)
        else:
            transforms[idx] = new_tf

        # Write back to file
        try:
            with open(self._config_path, "w") as f:
                yaml.safe_dump(self._config_data, f, default_flow_style=False)
            self.get_logger().info(f"Updated YAML config: {self._config_path}")
        except Exception as exc:
            self.get_logger().error(f"Failed to write YAML config: {exc}")

    def _derive_transform_from_aruco(self, poses: list, yaml_data: dict):
        
        aruco_0_poses = []
        aruco_1_poses = []
        aruco_2_poses = []

        for pose in poses:
            for marker, pose in zip(pose.marker_ids, pose.poses):
                if marker == 0:
                    aruco_0_poses.append(pose.position)
                elif marker == 1:
                    aruco_1_poses.append(pose.position)
                elif marker == 2:
                    aruco_2_poses.append(pose.position)

        # Convert geometry_msgs/Point to numpy vectors
        def to_vec(points):
            if len(points) == 0:
                return None
            arr = np.array([[p.x, p.y, p.z] for p in points], dtype=float)
            return np.mean(arr, axis=0)

        p0 = to_vec(aruco_0_poses)
        p1 = to_vec(aruco_1_poses)
        p2 = to_vec(aruco_2_poses)

        if p0 is None or p1 is None or p2 is None:
            self.get_logger().warn("Not enough ArUco pose data to compute orientation; returning identity")
            return [0.0, 0.0, 0.0], (0.0, 0.0, 0.0, 1.0)

        # Axes definition:
        # X axis: from marker 2 to marker 0
        # Y axis: from marker 1 to marker 0
        # Z axis: normal to the plane (X x Y)
        x_axis = p0 - p2
        y_axis = p0 - p1

        # Normalize and orthogonalize via Gram-Schmidt to ensure a proper basis
        def normalize(v):
            n = np.linalg.norm(v)
            return v / n if n > 1e-9 else v

        x_axis = normalize(x_axis)
        # Remove any component of y along x, then normalize
        y_axis = y_axis - np.dot(y_axis, x_axis) * x_axis
        y_axis = normalize(y_axis)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = normalize(z_axis)

        # Re-orthogonalize Y to ensure exact orthonormality and right-handed frame
        y_axis = np.cross(z_axis, x_axis)

        # Build rotation matrix R with columns as the child axes in parent coordinates
        R = np.stack([x_axis, y_axis, z_axis], axis=1)

        q = quaternion_from_rotation_matrix(R)

        # For translation, choose origin at marker 0 mean position (can be changed as needed)
        p2_offset = np.array([-0.150,-0.4755,0.0])
        R_inv = np.linalg.inv(R)
        p2_offset = R.dot(p2_offset)
        t = (p2+p2_offset).tolist()
        self.get_logger().info(f"Final Translation: {t}")
        self.get_logger().info(f"Final Quaternion: {q}")
        return [float(t[0]), float(t[1]), float(t[2])], (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def _parse_aruco_section(self, data: dict):
        """Parse 'aruco' section which is a list of single-key maps.

        Example YAML:
        aruco:
          - aruco_marker_0:
              x_offset: -0.810
              y_offset: -0.4755
          - aruco_marker_1:
              x_offset: -0.810
              y_offset: +0.4755

        Returns a dict: { 'aruco_marker_0': {'x_offset': -0.810, 'y_offset': -0.4755}, ... }
        """
        result = {}
        aruco_list = data.get("aruco", [])
        if isinstance(aruco_list, list):
            for entry in aruco_list:
                if isinstance(entry, dict):
                    for name, cfg in entry.items():
                        if isinstance(cfg, dict):
                            x_off = float(cfg.get("x_offset", 0.0))
                            y_off = float(cfg.get("y_offset", 0.0))
                            result[str(name)] = {"x_offset": x_off, "y_offset": y_off}
        return result

    def _parse_arena_section(self, data: dict):
        """Parse 'arena' section which is a list of key/value maps into a flat dict."""
        arena = {}
        arena_list = data.get("arena", [])
        if isinstance(arena_list, list):
            for pair in arena_list:
                if isinstance(pair, dict):
                    for key, value in pair.items():
                        arena[str(key)] = value
        return arena


def main():
    rclpy.init()
    node = StaticTFYamlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()


