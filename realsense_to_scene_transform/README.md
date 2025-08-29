## realsense_to_scene_transform

### Overview
Python ROS 2 package that:
- Publishes static transforms from a YAML config;
- Exposes a calibration service that collects ArUco poses and updates the transform in the YAML, then republishes it (ACHTUNG! Hardcoded)

Node entry point: `static_tf_yaml`

### YAML config
Default config is installed at `share/realsense_to_scene_transform/config/static_tf.yaml`.

Example:
```yaml
transforms:
  - parent: "camera_link"
    child: "world"
    translation: [0.0, 0.0, 0.0]
    rotation_rpy: [0.0, 0.0, 0.0]

aruco:
  - aruco_marker_0:
      x_offset: -0.810
      y_offset: -0.4755
  - aruco_marker_1:
      x_offset: -0.810
      y_offset: +0.4755
  - aruco_marker_2:
      x_offset: -0.150
      y_offset: +0.4755

arena:
  - x: 810
  - y: 1100
```

Notes:
- Each transform accepts either `rotation_quat: [x, y, z, w]` or `rotation_rpy: [roll, pitch, yaw]`.
- The `aruco` and `arena` sections are available for calibration logic.

### Build
```bash
colcon build --packages-select realsense_to_scene_transform --symlink-install
. install/setup.bash
```

### Run
Default parameters:
```bash
ros2 run realsense_to_scene_transform static_tf_yaml
```

With explicit parameters:
```bash
ros2 run realsense_to_scene_transform static_tf_yaml \
  --ros-args \
  -p config_file:="./src/Rozum_Pulse_75_ROS/realsense_to_scene_transform/config/static_tf.yaml" \
  -p aruco_poses_topic:=/aruco/markers \
  -p calibration_parent_frame:=aruco_marker_2 \
  -p calibration_child_frame:=world
```

### Calibration service
The node subscribes to `ArucoMarkers` on `/aruco/markers` and buffers 100 poses when calibration is active.

- Service name: `/calibrate_realsense_position`
- Type: `std_srvs/srv/Trigger`

Invoke:
```bash
ros2 service call /calibrate_realsense_position std_srvs/srv/Trigger "{}"
```

On completion:
- The transform is computed (user logic inside the node),
- `static_tf.yaml` is updated,
- The updated static transform is republished.


