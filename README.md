# self_localization

ROS 2 Humble package for camera self-localization using multiple ArUco markers. It consumes a video (or webcam), detects markers, and publishes the camera pose (`map -> camera`) plus a path for visualization in RViz2.

## Features
- Multi-marker localization: uses any detected marker present in a known map (`config/aruco_map.yaml`) to compute `map -> camera`.
- Path publishing: publishes `camera_pose` and `camera_path` (Path).
- Debug images: `/aruco_detection` (detections) and `/coordinate_system` (axes).
- Dockerfile to build and run in a ROS 2 Humble container.
- Calibration helper: `tools/calibrate_chessboard.py` to produce `camera_calibration_params.npz`.

## Package Layout
- `self_localization/multi_aruco_localization.py` — main multi-marker node, publishes TF, Pose, Path.
- `self_localization/aruco_self_localization.py` — legacy single-marker node.
- `self_localization/image_publisher.py` — publishes frames from video or webcam to `video_frames`.
- `config/aruco_map.yaml` — marker map (positions and orientations) and marker size.
- `launch/multi_aruco.launch.py` — runs image publisher + multi-marker localization with params.
- `tools/calibrate_chessboard.py` — chessboard calibration script (npz output).
- `Dockerfile` — ROS 2 Humble desktop base with dependencies and colcon build.

## Prerequisites (host)
- ROS 2 Humble, or use the provided Dockerfile.
- OpenCV, tf2_ros, tf_transformations, nav_msgs, cv_bridge, numpy, yaml (declared in package.xml).
- A calibration file `camera_calibration_params.npz` with `camera_matrix` and `dist_coeffs`.
- ArUco markers printed with the IDs present in `config/aruco_map.yaml` (DICT_6X6_250, size = `marker_size` meters).

## Build & Run (Docker)
```bash
# From repo root
docker build -t self_localization:humble .

# Run with video and display (X11), mounting data
docker run -it --rm --net=host \
  -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/you/cas_data:/root/cas_data \
  self_localization:humble

# Inside container
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
ros2 launch self_localization multi_aruco.launch.py \
  video_path:=/root/cas_data/multi_aruco.mp4 \
  camera_calibration_file:=/root/cas_data/camera_calibration_params.npz \
  aruco_map_file:=/ros_ws/src/self_localization/config/aruco_map.yaml
```
For webcam, add `--device /dev/video0` to docker run and override `use_video:=False` in `image_publisher`.

## Build & Run (native ROS 2 Humble)
```bash
cd ~/ros_ws
colcon build --packages-select self_localization
source install/setup.bash
ros2 launch self_localization multi_aruco.launch.py \
  video_path:=/home/you/cas_data/multi_aruco.mp4 \
  camera_calibration_file:=/home/you/cas_data/camera_calibration_params.npz \
  aruco_map_file:=/home/you/ros_ws/src/self_localization/config/aruco_map.yaml
```

## Calibration
Use a printed chessboard (inner corners `pattern_cols` x `pattern_rows`, square size in meters):
```bash
python3 tools/calibrate_chessboard.py \
  --video 0 \
  --pattern_cols 7 --pattern_rows 5 \
  --square_size 0.023 \
  --output /home/you/cas_data/camera_calibration_params.npz
```
Supports video file (pass the path) or camera index (e.g., `--video 0`). Use `--no-gui` in headless mode.

## Map Configuration
`config/aruco_map.yaml` example:
```yaml
world_frame: "map"
marker_size: 0.04    # meters, edge length of the marker
aruco_markers:
  0: {x: 0.081, y: 0.138, z: 0.0, roll: 0.0, pitch: 0.0, yaw: -1.57}
  1: {x: 0.091, y: 0.069, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 1.57}
  2: {x: 0.061, y: 0.003, z: 0.146, roll: 0.0, pitch: 0.0, yaw: 0.0}
  4: {x: 0.191, y: 0.008, z: 0.048, roll: 0.0, pitch: 0.0, yaw: 0.0}
```
- `world_frame` is the global frame (set RViz Fixed Frame to this).
- `marker_size` must match the printed marker size in meters.
- `yaw/roll/pitch` in radians by default; set `map_angles_degrees:=true` in launch if you use degrees.

## RViz2
- Fixed Frame: `map`
- Displays: `TF` (to see `map -> camera`), `Path` on `/camera_path`, `Image` on `/aruco_detection` or `/coordinate_system`.

## Notes
- The node uses DICT_6X6_250; ensure printed markers match.
- At least one marker from the map must be visible to publish TF/Pose/Path.
- If multiple markers are detected, it currently uses the first valid one (can be extended to fuse poses).
